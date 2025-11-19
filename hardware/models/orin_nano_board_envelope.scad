//
// Jetson Orin Nano — Board Envelope (OpenSCAD)
// Simple parametric model: board plate + 40-pin header envelope + two CSI connector envelopes.
// Units: millimeters.
//
// How to use:
// 1) Open in OpenSCAD.
// 2) Tweak dimensions/positions below as needed.
// 3) F5 to preview, F6 to render, then File → Export → STL.
//
// Origin convention:
// - (0,0,0) at the board's lower-left corner on the bottom face.
// - +X to the right, +Y up, +Z out of the board (thickness direction).
//

/************* Parameters *************/

// Board
pcb_len = 103.0;    // X 
pcb_wid = 90.0;     // Y
pcb_thk = 6.5;      // Z
corner_rad = 0;     // set >0 for cosmetic rounded corners (visual only)

// 40-pin header (2x20) — envelope block (approximate)
hdr_len = 2.54 * 20; // length along pins
hdr_wid = 2.54 * 2;  // across rows
hdr_h   = 9.0;       // height above board
hdr_edge_margin = 3.2;
hdr_x = pcb_len - hdr_edge_margin - hdr_wid; // near right edge; width spans X
hdr_y = 22.25;                                // distance from bottom edge
hdr_z = pcb_thk;                             // sits on top surface

// CSI connector envelopes (approximate 22-pin FFC)
// Size of each connector block
mipi_len = 3.3;  // X span
mipi_wid = 16.0; // Y span
mipi_h   = 1.3;  // Z height

// Center positions [x, y] on the board's top face 
mipi_centers = [
  [6.0, 35.0], // J20 approx
  [6.0, 60.0]  // J21 approx
];

// Daughter card
daughter_card = [73, 49, 10];               // daughter card geometry
daughter_card_center = [13, 32, pcb_thk];   // daughter card center  

// Heat sink geometry
hsink = [58, 40, 19];   // heat sink + fan geometry 
hsink_center = [21, 41, pcb_thk + daughter_card[2] ];

// Remaining connectors
pwr_plg = [9, 14, 11];                  // power plug geometry (x, y, z) 
pwr_plg_center = [3, 0, pcb_thk];       // power plug center (x, y, z)
hdmi_plg = [18, 15, 7];                 // hdmi display connector geometry (x, y, z)
hdmi_plg_center = [15.5, 0, pcb_thk];   // hdmmi display center
usb2_twr_plg = [14, 18, 16];            // usb 2.0 tower plug geometry (x, y, z)
usb2_twr1_center = [37, 0, pcb_thk];    // usb 2.0 tower 1 plug center
usb2_twr2_center = [54, 0, pcb_thk];    // usb 2.0 tower 2 plug center
eth_plg = [16, 21, 14];                 // ethernet plug geometry (x, y, z)
eth_plg_center = [70.5, 0, pcb_thk];    // ethernet plug center (x, y, z)
usb3_plg = [9, 11, 3];                  // usb3 plug geometry (x, y, z)
usb3_plg_center = [89,  0, pcb_thk];    // usb3 plug center 

// hole pattern
hole_dia = 2.5;
hole_height = pcb_thk;
dd_hole_height = pcb_thk + daughter_card[2];

// list of holes: [x, y, dia, height, extra]
hole_pattern = [
    [6, 19, hole_dia, hole_height],     // left top corner mount of main PCB to plastic case
    [6, 78, hole_dia, hole_height],     // right top corner mount of main PCB to plastic case
    [92, 19, hole_dia, hole_height],    // left bottom corner mount of main PCB to plastic case
    [92, 77, hole_dia, hole_height],    // right bottom corner mount of main PCB to plastic case
    [97.5, 19, hole_dia, hole_height],  // left 40 pin header mount, open by default
    [97.5, 77, hole_dia, hole_height],  // right 40 pin header mount, open by default
    [18, 78, hole_dia, dd_hole_height], // top right corner daughter card mount to PCB
    [82, 78, hole_dia, dd_hole_height], // top right corner daughter card mount to PCB
];

// Visual quality
$fn = 64;

/************* Helpers *************/

module rounded_rect_xy(x, y, r, z) {
  if (r <= 0) {
    cube([x, y, z], center=false);
  } else {
    // Rounded rectangle prism via Minkowski
    minkowski() {
      cube([x-2*r, y-2*r, z], center=false);
      cylinder(r=r, h=0.001);
    }
  }
}

module box_at(origin=[0,0,0], size=[10,10,1]) {
  translate(origin) cube(size, center=false);
}

/************* Components *************/

module board_plate() {
  color("#177245") // green-ish
  rounded_rect_xy(pcb_len, pcb_wid, corner_rad, pcb_thk);
}

module header_envelope() {
  // Header runs along Y (long dimension = hdr_len), width across X (hdr_wid)
  color("#555555")
  box_at([hdr_x, hdr_y, hdr_z], [hdr_wid, hdr_len, hdr_h]);
}

module csi_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate([cx - mipi_len/2, cy - mipi_wid/2, pcb_thk])
    cube([mipi_len, mipi_wid, mipi_h], center=false);
}

module heat_sink_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#303030")
  translate(hsink_center)
    cube(hsink, center=false);
}

module daughter_card_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#228B22")
  translate(daughter_card_center)
    cube(daughter_card, center=false);
}

module pwr_plg_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate(pwr_plg_center)
    cube(pwr_plg, center=false);
}

module hdmi_plg_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate(hdmi_plg_center)
    cube(hdmi_plg, center=false);
}

module usb2_twr_plg_envelope( center ) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate(center)
    cube(usb2_twr_plg, center=false);
}

module eth_plg_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate(eth_plg_center)                
    cube(eth_plg, center=false);
}

module usb3_plg_envelope(cx, cy) {
  // Center the connector block at (cx, cy) on the top face
  color("#555555")
  translate(usb3_plg_center)
    cube(usb3_plg, center=false);
}

// hole pattern, drill along Z: center at (x,y), cut through height h
module drill_z(x, y, d, h){
    translate([x, y, 0-0.1]) cylinder(h = h+0.2, r = d/2);
}

module jetson_orin_nano_dev()
{
    difference() {
      union() {
          board_plate();
          header_envelope();
          for (c = mipi_centers) csi_envelope(c[0], c[1]);
          daughter_card_envelope();
          heat_sink_envelope();
          pwr_plg_envelope();
          hdmi_plg_envelope();
          usb2_twr_plg_envelope( usb2_twr1_center );
          usb2_twr_plg_envelope( usb2_twr2_center );
          eth_plg_envelope();
          usb3_plg_envelope();
          }
      for (h = hole_pattern)
          drill_z(h[0], h[1], h[2], h[3]);
      }
}

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