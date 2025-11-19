// Adafruit LSM9DS0 IMU Mounting Fixture
// Mounting system with standoffs and base plate
// Uses reusable hole pattern for chassis/wall mounting
//
// OpenSCAD model created for ekf9dof project

use <adafruit_lsm9ds0_pcb.scad>

// ===== MOUNTING PARAMETERS =====

// PCB dimensions (from adafruit_lsm9ds0_pcb.scad)
pcb_length = 33.02;
pcb_width = 20.32;
pcb_hole_diameter = 2.5;
pcb_hole_positions = [
    [2.54, 2.54],
    [30.48, 2.54],
    [2.54, 17.78],
    [30.48, 17.78]
];

// Standoff parameters
standoff_height = 8;              // Height above base plate
standoff_outer_diameter = 5;      // Outer diameter of standoff
standoff_inner_diameter = 2.65;   // Inner diameter for M2.5 screw (clearance)
standoff_top_diameter = 4.5;      // Top platform diameter for PCB mounting

// Base plate parameters
base_plate_length = 50;           // Base plate X dimension
base_plate_width = 40;            // Base plate Y dimension
base_plate_thickness = 3;         // Base plate thickness
base_plate_corner_radius = 2;     // Rounded corners

// Chassis mounting hole pattern (4-hole pattern on base plate)
chassis_hole_diameter = 2.65;     // M2.5 clearance holes
chassis_hole_spacing = 32;        // Distance between holes (center-to-center)
chassis_hole_inset = 4;           // Distance from edge to hole center

// Hardware tolerances
screw_tolerance = 0.15;           // Extra clearance for screws

// ===== REUSABLE MOUNTING HOLE PATTERN MODULE =====

// Module: Creates a 4-hole mounting pattern (rectangular)
// This pattern can be used to:
//   1. Create mounting holes in the base plate (positive)
//   2. Create matching holes in chassis/walls (negative)
//   3. Ensure perfect alignment between mount and chassis
//
// Parameters:
//   hole_d = hole diameter
//   spacing_x = horizontal spacing between holes
//   spacing_y = vertical spacing between holes
//   depth = hole depth (use large value for through-holes)
module mounting_hole_pattern_4(hole_d = 3.2, spacing_x = 32, spacing_y = 20, depth = 10) {
    // Calculate positions for 4-hole rectangular pattern
    x_offset = spacing_x / 2;
    y_offset = spacing_y / 2;
    
    positions = [
        [-x_offset, -y_offset],  // Bottom left
        [ x_offset, -y_offset],  // Bottom right
        [-x_offset,  y_offset],  // Top left
        [ x_offset,  y_offset]   // Top right
    ];
    
    for (pos = positions) {
        translate([pos[0], pos[1], 0])
        cylinder(d = hole_d, h = depth, center = true, $fn = 32);
    }
}

// ===== IMU MOUNT COMPONENTS =====

// Module: Single standoff for PCB mounting
module pcb_standoff() {
    difference() {
        union() {
            // Main standoff column
            cylinder(d = standoff_outer_diameter, h = standoff_height, $fn = 32);
            
            // Top platform for PCB support
            translate([0, 0, standoff_height])
            cylinder(d = standoff_top_diameter, h = 1.5, $fn = 32);
        }
        
        // Through-hole for M2.5 screw (bottom to top)
        translate([0, 0, -0.1])
        cylinder(d = standoff_inner_diameter, h = standoff_height + 2, $fn = 32);
    }
}

// Module: Base plate with standoff mounting points and chassis holes
module base_plate() {
    difference() {
        // Base plate body with rounded corners
        translate([0, 0, base_plate_thickness/2])
        linear_extrude(height = base_plate_thickness, center = true)
        offset(r = base_plate_corner_radius)
        offset(r = -base_plate_corner_radius)
        square([base_plate_length, base_plate_width], center = true);
        
        // Chassis mounting holes (4-hole pattern)
        translate([0, 0, base_plate_thickness/2])
        mounting_hole_pattern_4(
            hole_d = chassis_hole_diameter,
            spacing_x = chassis_hole_spacing,
            spacing_y = base_plate_width - 2 * chassis_hole_inset,
            depth = base_plate_thickness + 1
        );
        
        // PCB standoff mounting holes (countersunk for M2.5 screws from bottom)
        for (pos = pcb_hole_positions) {
            // Center the PCB hole pattern on the base plate
            translate([pos[0] - pcb_length/2, pos[1] - pcb_width/2, -0.1]) {
                // Through hole for screw shaft
                cylinder(d = standoff_inner_diameter, h = base_plate_thickness + 0.2, $fn = 32);
                
                // Countersink for screw head (from bottom)
                translate([0, 0, -0.1])
                cylinder(d1 = 5.5, d2 = standoff_inner_diameter, h = 2, $fn = 32);
            }
        }
    }
}

// ===== COMPLETE IMU MOUNT ASSEMBLY =====

// Module: Complete IMU mount with base plate and standoffs
module imu_mount_assembly(show_pcb = false, explode = 0) {
    
    // Base plate
    color("lightgray")
    base_plate();
    
    // PCB standoffs
    color("silver")
    for (pos = pcb_hole_positions) {
        translate([pos[0] - pcb_length/2, pos[1] - pcb_width/2, base_plate_thickness])
        pcb_standoff();
    }
    
    // Optional: Show PCB for fit check
    if (show_pcb) {
        translate([0, 0, base_plate_thickness + standoff_height + 1.5 + explode])
        translate([-pcb_length/2, -pcb_width/2, 0])
        adafruit_lsm9ds0_pcb(show_components = true);
    }
}

// ===== HELPER MODULE FOR CHASSIS CUTOUT =====

// Module: Generate matching hole pattern in chassis/wall
// Use this with difference() to cut mounting holes in your chassis
module chassis_mounting_pattern(wall_thickness = 3, hole_d = 2.65) {
    mounting_hole_pattern_4(
        hole_d = hole_d,
        spacing_x = chassis_hole_spacing,
        spacing_y = base_plate_width - 2 * chassis_hole_inset,
        depth = wall_thickness + 1
    );
}

// ===== EXAMPLE CHASSIS WALL WITH MOUNTING HOLES =====

// Module: Example chassis wall showing how to use the mounting pattern
module example_chassis_wall() {
    wall_thickness = 3;
    wall_size = [60, 50, wall_thickness];
    
    difference() {
        // Wall
        color("lightblue", 0.5)
        translate([0, 0, -wall_thickness])
        cube(wall_size, center = true);
        
        // Mounting holes using the pattern
        translate([0, 0, -wall_thickness/2])
        chassis_mounting_pattern(wall_thickness, chassis_hole_diameter);
    }
}

// ===== RENDER SELECTION =====

// Render options:
render_mode = "standoffs_only";  // Options: "assembly", "base_only", "standoffs_only", "with_chassis", "pattern_test"

if (render_mode == "assembly") {
    // IMU mount assembly with PCB
    imu_mount_assembly(show_pcb = true, explode = 0);
    
} else if (render_mode == "base_only") {
    // Base plate only (for 3D printing)
    base_plate();
    
} else if (render_mode == "standoffs_only") {
    // Standoffs only (for 3D printing as separate parts)
    color("silver")
    for (i = [0:len(pcb_hole_positions)-1]) {
        pos = pcb_hole_positions[i];
        // Arrange standoffs in a line for printing
        translate([i * (standoff_outer_diameter + 2), 0, 0])
        pcb_standoff();
    }
    
} else if (render_mode == "with_chassis") {
    // Show mount with example chassis wall
    imu_mount_assembly(show_pcb = true, explode = 10);
    example_chassis_wall();
    
} else if (render_mode == "pattern_test") {
    // Show just the hole patterns for verification
    color("red")
    mounting_hole_pattern_4(
        hole_d = chassis_hole_diameter,
        spacing_x = chassis_hole_spacing,
        spacing_y = base_plate_width - 2 * chassis_hole_inset,
        depth = 5
    );
}

// ===== DOCUMENTATION =====

echo("=== Adafruit LSM9DS0 IMU Mount ===");
echo(str("Base plate: ", base_plate_length, "mm x ", base_plate_width, "mm x ", base_plate_thickness, "mm"));
echo(str("Standoff height: ", standoff_height, "mm"));
echo(str("Total assembly height: ", base_plate_thickness + standoff_height + 1.5, "mm (without PCB)"));
echo("");
echo("=== Chassis Mounting Pattern ===");
echo(str("4-hole pattern: ", chassis_hole_spacing, "mm x ", base_plate_width - 2 * chassis_hole_inset, "mm"));
echo(str("Hole diameter: ", chassis_hole_diameter, "mm (M2.5 clearance)"));
echo(str("Use chassis_mounting_pattern() module to create matching holes in chassis"));
echo("");
echo("=== Hardware Required ===");
echo("4x M2.5 screws (length: ~15mm) - PCB to standoffs");
echo("4x M2.5 screws + nuts - Base plate to chassis");
