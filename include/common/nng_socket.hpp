#pragma once

#include <string>
#include <optional>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <nng/nng.h>
#include <nng/protocol/pubsub0/pub.h>
#include <nng/protocol/pubsub0/sub.h>

/**
 * @brief NNG-based pub/sub socket with raw struct serialization
 *
 * This class provides inter-process communication using nanomsg-next-gen (nng)
 * pub/sub sockets with raw C++ struct serialization.
 *
 * Template parameters:
 *   T - The C++ message type (must be trivially copyable POD struct)
 *
 * Usage:
 *   // Publisher side
 *   NngSocket<raw_accel_msg_t> pub_socket("ipc:///tmp/accel.ipc", true);
 *   pub_socket.send(accel_msg);
 *
 *   // Subscriber side
 *   NngSocket<raw_accel_msg_t> sub_socket("ipc:///tmp/accel.ipc", false);
 *   auto msg = sub_socket.receive();
 */
template <typename T>
class NngSocket {
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable for raw serialization");
    
public:
    /**
     * @brief Construct an NNG socket
     * @param url IPC URL (e.g., "ipc:///tmp/sensor.ipc")
     * @param is_publisher True for publisher, false for subscriber
     * @param dial_retry_ms Retry interval for subscriber dial attempts (default 50ms)
     * @param max_dial_retries Maximum number of dial retries for subscriber (default 100)
     */
    NngSocket(const std::string& url, bool is_publisher,
               int dial_retry_ms = 50, int max_dial_retries = 100)
        : url_(url), is_publisher_(is_publisher), closed_(false) {
        int rv;
        if (is_publisher_) {
            rv = nng_pub0_open(&socket_);
        } else {
            rv = nng_sub0_open(&socket_);
        }
        if (rv != 0) {
            throw std::runtime_error("Failed to open nng socket: " + std::string(nng_strerror(rv)));
        }

        if (is_publisher_) {
            rv = nng_listen(socket_, url_.c_str(), nullptr, 0);
            if (rv != 0) {
                nng_close(socket_);
                throw std::runtime_error("Failed to listen on nng socket: " + std::string(nng_strerror(rv)));
            }
        } else {
            // Subscribe to all messages (empty topic)
            rv = nng_socket_set(socket_, NNG_OPT_SUB_SUBSCRIBE, "", 0);
            if (rv != 0) {
                nng_close(socket_);
                throw std::runtime_error("Failed to set subscription: " + std::string(nng_strerror(rv)));
            }

            // Retry dial with backoff since publisher may not be ready yet
            int retries = 0;
            while (retries < max_dial_retries) {
                rv = nng_dial(socket_, url_.c_str(), nullptr, 0);
                if (rv == 0) {
                    break;
                }
                if (rv != NNG_ECONNREFUSED) {
                    nng_close(socket_);
                    throw std::runtime_error("Failed to dial nng socket: " + std::string(nng_strerror(rv)));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(dial_retry_ms));
                ++retries;
            }
            if (rv != 0) {
                nng_close(socket_);
                throw std::runtime_error("Failed to connect nng socket after retries: " + std::string(nng_strerror(rv)));
            }
        }
    }

    ~NngSocket() {
        close();
    }

    // Non-copyable
    NngSocket(const NngSocket&) = delete;
    NngSocket& operator=(const NngSocket&) = delete;

    // Moveable
    NngSocket(NngSocket&& other) noexcept
        : socket_(other.socket_), url_(std::move(other.url_)),
          is_publisher_(other.is_publisher_), closed_(other.closed_.load()) {
        other.socket_ = NNG_SOCKET_INITIALIZER;
        other.closed_ = true;
    }

    NngSocket& operator=(NngSocket&& other) noexcept {
        if (this != &other) {
            close();
            socket_ = other.socket_;
            url_ = std::move(other.url_);
            is_publisher_ = other.is_publisher_;
            closed_ = other.closed_.load();
            other.socket_ = NNG_SOCKET_INITIALIZER;
            other.closed_ = true;
        }
        return *this;
    }

    /**
     * @brief Send data through the channel (publisher only)
     * @param data The message to send
     * @return true on success, false if channel is closed or not a publisher
     */
    bool send(const T& data) {
        if (closed_ || !is_publisher_) {
            return false;
        }

        int rv = nng_send(socket_, static_cast<void*>(const_cast<T*>(&data)), sizeof(T), 0);
        return rv == 0;
    }

    /**
     * @brief Receive data from the channel (blocking, subscriber only)
     * @return The received message, or nullopt if channel is closed
     */
    std::optional<T> receive() {
        if (closed_ || is_publisher_) {
            return std::nullopt;
        }

        void* buf = nullptr;
        size_t sz = 0;

        int rv = nng_recv(socket_, &buf, &sz, NNG_FLAG_ALLOC);
        if (rv != 0) {
            return std::nullopt;
        }

        if (sz != sizeof(T)) {
            nng_free(buf, sz);
            return std::nullopt;
        }

        T data;
        std::memcpy(&data, buf, sizeof(T));
        nng_free(buf, sz);
        return data;
    }

    /**
     * @brief Try to receive data non-blocking (subscriber only)
     * @param data Output parameter for received data
     * @return true if data was received, false otherwise
     */
    bool try_receive(T& data) {
        if (closed_ || is_publisher_) {
            return false;
        }

        void* buf = nullptr;
        size_t sz = 0;

        int rv = nng_recv(socket_, &buf, &sz, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);
        if (rv != 0) {
            return false;
        }

        if (sz != sizeof(T)) {
            nng_free(buf, sz);
            return false;
        }

        std::memcpy(&data, buf, sizeof(T));
        nng_free(buf, sz);
        return true;
    }

    /**
     * @brief Close the channel
     */
    void close() {
        bool expected = false;
        if (closed_.compare_exchange_strong(expected, true)) {
            nng_close(socket_);
        }
    }

    /**
     * @brief Check if channel is closed
     */
    bool is_closed() const {
        return closed_.load();
    }

    /**
     * @brief Check if this is a publisher channel
     */
    bool is_publisher() const {
        return is_publisher_;
    }

    /**
     * @brief Get the IPC URL
     */
    const std::string& url() const {
        return url_;
    }

private:
    nng_socket socket_;
    std::string url_;
    bool is_publisher_;
    std::atomic<bool> closed_;
};
