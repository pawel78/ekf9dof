// Adafruit LSM9DS0 9-DOF Breakout Board PCB Model
// Based on Adafruit-LSM9DS0-PCB design files
// https://github.com/adafruit/Adafruit-LSM9DS0-PCB
//
// License: Creative Commons Attribution-ShareAlike 3.0
// Designed by Limor Fried/Ladyada for Adafruit Industries
// OpenSCAD model created for ekf9dof project

// Parameters - all dimensions from Adafruit Eagle PCB files
pcb_length = 33.02;      // X dimension from Eagle board file
pcb_width = 20.32;       // Y dimension from Eagle board file  
pcb_thickness = 1.6;     // Standard PCB thickness
hole_diameter = 2.5;     // Mounting hole drill diameter from .brd file
hole_positions = [
    [2.54, 2.54],        // Bottom left corner
    [30.48, 2.54],       // Bottom right corner
    [2.54, 17.78],       // Top left corner
    [30.48, 17.78]       // Top right corner
];

// Component dimensions (approximated from board layout)
lsm9ds0_chip_size = [4, 4, 1];  // LGA-24 4x4mm package
chip_position = [pcb_length/2, pcb_width/2, pcb_thickness];

// Connector positions (from board file - 1x9 power/IO header and 1x5 SPI header)
header_1x9_position = [16.51, 2.54, pcb_thickness];
header_1x5_position = [16.51, 17.78, pcb_thickness];
pin_size = [0.64, 0.64, 11.43];  // Square pin 0.64mm, 11.43mm height

// Module: Complete Adafruit LSM9DS0 PCB assembly
module adafruit_lsm9ds0_pcb(show_components = true, pcb_color = "#2196F3") {
    
    // PCB base
    difference() {
        union() {
            // Main PCB board
            color(pcb_color)
            linear_extrude(height = pcb_thickness)
            difference() {
                // PCB outline
                square([pcb_length, pcb_width]);
                
                // Mounting holes
                for (pos = hole_positions) {
                    translate(pos)
                    circle(d = hole_diameter, $fn = 32);
                }
            }
        }
    }
    
    if (show_components) {
        // LSM9DS0 IC chip (center of board)
        color("dimgray")
        translate(chip_position)
        cube(lsm9ds0_chip_size, center = true);
        
        // Pin 1 indicator dot
        color("white")
        translate([chip_position[0] - lsm9ds0_chip_size[0]/2 + 0.5, 
                   chip_position[1] - lsm9ds0_chip_size[1]/2 + 0.5, 
                   chip_position[2] + lsm9ds0_chip_size[2]])
        cylinder(d = 0.6, h = 0.1, $fn = 16);
        
        // 1x9 Pin header (bottom - power and GPIO)
        color("black")
        translate(header_1x9_position)
        for (i = [-4:4]) {
            translate([i * 2.54, 0, 0])
            cube(pin_size, center = true);
        }
        
        // 1x5 Pin header (top - SPI interface)
        color("black")
        translate(header_1x5_position)
        for (i = [-2:2]) {
            translate([i * 2.54, 0, 0])
            cube(pin_size, center = true);
        }
        
        // Voltage regulator (approximate position from layout)
        color("darkslategray")
        translate([8.89, 10.16, pcb_thickness])
        cube([2.9, 1.6, 1.2], center = true);
        
        // Level shifter MOSFETs (approximate positions)
        for (x_offset = [12.7, 15.24, 17.78]) {
            color("darkslategray")
            translate([x_offset, 6.858, pcb_thickness])
            cube([1.3, 2.5, 0.9], center = true);
        }
        
        // SMD capacitors and resistors (simplified representation)
        for (pos = [[6.35, 12.7], [24.892, 11.938], [12.192, 14.351]]) {
            color("sienna")
            translate([pos[0], pos[1], pcb_thickness])
            cube([2.0, 1.25, 0.6], center = true);
        }
    }
}

// Render the PCB
adafruit_lsm9ds0_pcb(show_components = true);

// Print board information
echo("=== Adafruit LSM9DS0 PCB ===");
echo(str("Board size: ", pcb_length, "mm x ", pcb_width, "mm x ", pcb_thickness, "mm"));
echo(str("Mounting holes: ", hole_diameter, "mm diameter"));
echo("Mounting hole positions (X, Y):");
for (i = [0:len(hole_positions)-1]) {
    echo(str("  Hole ", i+1, ": (", hole_positions[i][0], ", ", hole_positions[i][1], ")"));
}
echo("Sensor: LSM9DS0 9-DOF (3-axis accelerometer, magnetometer, gyroscope)");
echo("Headers: 1x9 pin (Power/GPIO), 1x5 pin (SPI)");
echo("License: CC-BY-SA 3.0 - Adafruit Industries");
