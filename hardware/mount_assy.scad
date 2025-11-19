//-----------------------------------------------------------------
// Jetson Orin Nano Dev Board Fixture
//-----------------------------------------------------------------
wall_thk = 3;
base_width = 25;
base_plate_geometry = [pcb_len + 2 * wall_thk, wall_thk, base_width];
base_plate_center = [-wall_thk, pcb_wid, -wall_thk];

module base_plase( geometry, offset) {
    color("#555555");
    translate( offset )
    cube( geometry, center = false);
}

rail_height = 15;
rail_width = 5;
rail_geometry = [rail_width, rail_height, base_width];
rail_center = [pcb_len - rail_width + wall_thk, pcb_wid - rail_height, -wall_thk];

// Define a 2D square in the XY plane…
size_xy = [6+0.2, pcb_thk+0.2];   // (x,y) of the 2D shapen + .02 for tolerance
extrude_len = 90;             // how far to extrude along +Y

module hollow() {
    // …then extrude along +Z and rotate it so +Z maps to +Y
    rotate([-90, 0, 0])                 // +Z → +Y
    translate([pcb_len - 3, -3, 0])
    linear_extrude(height = extrude_len)
    square(size_xy, center = true);  // your 2D profile
    
    rotate([-90, 0, 0])                 // +Z → +Y
    translate([3, -3, 0])
    linear_extrude(height = extrude_len)
    square(size_xy, center = true);  // your 2D profile
}

module orin_rail(geometry, offset) {
    color("#555555");
    translate( offset )
    cube( geometry, center = false);
    translate( [-3, pcb_wid-rail_height, -3] )
    cube( geometry, center = false);
}

/************* Scene *************/
jetson_orin_nano_dev();
base_plase(base_plate_geometry, base_plate_center);
difference() {
    orin_rail(rail_geometry, rail_center);
    hollow();
}