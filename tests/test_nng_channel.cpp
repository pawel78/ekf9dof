#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cassert>
#include <cmath>

#include "imu/messages/nng_imu_channels.hpp"

// Simple test framework - using exit(1) on failure is acceptable for this minimal
// test suite since cleanup is handled by process termination
#define TEST(name) void name()
#define ASSERT_TRUE(x) do { if (!(x)) { std::cerr << "FAIL: " << #x << " at line " << __LINE__ << "\n"; exit(1); } } while(0)
#define ASSERT_FALSE(x) do { if (x) { std::cerr << "FAIL: NOT " << #x << " at line " << __LINE__ << "\n"; exit(1); } } while(0)
#define ASSERT_EQ(a, b) do { if ((a) != (b)) { std::cerr << "FAIL: " << #a << " != " << #b << " at line " << __LINE__ << "\n"; exit(1); } } while(0)
#define ASSERT_NEAR(a, b, eps) do { if (std::fabs((a) - (b)) > (eps)) { std::cerr << "FAIL: " << #a << " not near " << #b << " at line " << __LINE__ << "\n"; exit(1); } } while(0)

// Test IPC URL - /tmp is standard on Linux where tests run
constexpr const char* TEST_IPC_URL = "ipc:///tmp/test_nng_channel.ipc";

/**
 * Test basic pub/sub communication with raw accelerometer messages
 */
TEST(test_basic_pubsub) {
    std::cout << "Running test_basic_pubsub...\n";

    std::atomic<bool> publisher_ready{false};
    std::atomic<bool> subscriber_ready{false};
    messages::raw_accel_msg_t received_msg{};

    // Publisher thread - starts first and signals when ready
    std::thread pub_thread([&]() {
        imu::NngRawAccelChannel pub(TEST_IPC_URL, true);
        publisher_ready.store(true);

        // Wait for subscriber to connect
        while (!subscriber_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // Small delay to ensure subscriber socket is fully ready
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Send test message
        messages::raw_accel_msg_t msg{};
        msg.timestamp_ns = 123456789;
        msg.x = 1.5f;
        msg.y = -2.5f;
        msg.z = 9.8f;

        ASSERT_TRUE(pub.send(msg));
    });

    // Subscriber thread - waits for publisher to be ready first
    std::thread sub_thread([&]() {
        // Wait for publisher to be listening
        while (!publisher_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        imu::NngRawAccelChannel sub(TEST_IPC_URL, false);
        subscriber_ready.store(true);

        auto result = sub.receive();
        ASSERT_TRUE(result.has_value());
        received_msg = result.value();
    });

    pub_thread.join();
    sub_thread.join();

    // Verify received data
    ASSERT_EQ(received_msg.timestamp_ns, 123456789ULL);
    ASSERT_NEAR(received_msg.x, 1.5f, 0.001f);
    ASSERT_NEAR(received_msg.y, -2.5f, 0.001f);
    ASSERT_NEAR(received_msg.z, 9.8f, 0.001f);

    std::cout << "test_basic_pubsub PASSED\n";
}

/**
 * Test non-blocking try_receive
 */
TEST(test_try_receive) {
    std::cout << "Running test_try_receive...\n";

    constexpr const char* url = "ipc:///tmp/test_try_receive.ipc";

    std::atomic<bool> publisher_ready{false};
    std::atomic<bool> subscriber_ready{false};

    std::thread pub_thread([&]() {
        imu::NngRawGyroChannel pub(url, true);
        publisher_ready.store(true);

        while (!subscriber_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        messages::raw_gyro_msg_t msg{};
        msg.timestamp_ns = 987654321;
        msg.x = 0.1f;
        msg.y = 0.2f;
        msg.z = 0.3f;

        pub.send(msg);
    });

    std::thread sub_thread([&]() {
        while (!publisher_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        imu::NngRawGyroChannel sub(url, false);
        subscriber_ready.store(true);

        // try_receive should initially fail (no message yet)
        messages::raw_gyro_msg_t msg{};

        // Wait for the message (with timeout)
        int attempts = 0;
        while (!sub.try_receive(msg) && attempts < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            attempts++;
        }

        ASSERT_TRUE(attempts < 100);
        ASSERT_EQ(msg.timestamp_ns, 987654321ULL);
        ASSERT_NEAR(msg.x, 0.1f, 0.001f);
    });

    pub_thread.join();
    sub_thread.join();

    std::cout << "test_try_receive PASSED\n";
}

/**
 * Test channel close behavior
 */
TEST(test_close) {
    std::cout << "Running test_close...\n";

    constexpr const char* url = "ipc:///tmp/test_close.ipc";

    imu::NngRawMagChannel pub(url, true);
    ASSERT_FALSE(pub.is_closed());

    pub.close();
    ASSERT_TRUE(pub.is_closed());

    // Send should fail after close
    messages::raw_mag_msg_t msg{};
    ASSERT_FALSE(pub.send(msg));

    std::cout << "test_close PASSED\n";
}

/**
 * Test multiple message types
 */
TEST(test_multiple_message_types) {
    std::cout << "Running test_multiple_message_types...\n";

    // Test with temperature message (different structure)
    constexpr const char* url = "ipc:///tmp/test_temp.ipc";

    std::atomic<bool> publisher_ready{false};
    std::atomic<bool> subscriber_ready{false};

    std::thread pub([&]() {
        imu::NngRawTempChannel pub_ch(url, true);
        publisher_ready.store(true);

        while (!subscriber_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        messages::raw_temp_msg_t msg{};
        msg.timestamp_ns = 111222333;
        msg.temp_c = 25.5f;
        pub_ch.send(msg);
    });

    std::thread sub([&]() {
        while (!publisher_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        imu::NngRawTempChannel sub_ch(url, false);
        subscriber_ready.store(true);

        auto result = sub_ch.receive();
        ASSERT_TRUE(result.has_value());
        ASSERT_EQ(result->timestamp_ns, 111222333ULL);
        ASSERT_NEAR(result->temp_c, 25.5f, 0.001f);
    });

    pub.join();
    sub.join();

    std::cout << "test_multiple_message_types PASSED\n";
}

/**
 * Test processed message types
 */
TEST(test_processed_messages) {
    std::cout << "Running test_processed_messages...\n";

    constexpr const char* url = "ipc:///tmp/test_proc_accel.ipc";

    std::atomic<bool> publisher_ready{false};
    std::atomic<bool> subscriber_ready{false};

    std::thread pub([&]() {
        imu::NngProcAccelChannel pub_ch(url, true);
        publisher_ready.store(true);

        while (!subscriber_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        messages::proc_accel_msg_t msg{};
        msg.timestamp_ns = 444555666;
        msg.x = 0.01f;
        msg.y = 0.02f;
        msg.z = 1.0f;
        pub_ch.send(msg);
    });

    std::thread sub([&]() {
        while (!publisher_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        imu::NngProcAccelChannel sub_ch(url, false);
        subscriber_ready.store(true);

        auto result = sub_ch.receive();
        ASSERT_TRUE(result.has_value());
        ASSERT_EQ(result->timestamp_ns, 444555666ULL);
        ASSERT_NEAR(result->z, 1.0f, 0.001f);
    });

    pub.join();
    sub.join();

    std::cout << "test_processed_messages PASSED\n";
}

int main() {
    std::cout << "=== NNG Channel Tests ===\n\n";

    test_basic_pubsub();
    test_try_receive();
    test_close();
    test_multiple_message_types();
    test_processed_messages();

    std::cout << "\n=== All tests PASSED ===\n";
    return 0;
}
