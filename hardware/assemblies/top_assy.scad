// assemblies/orin_stereo_fixture.scad
use <../models/slot_fixture.scad>

pcb = [103, 0, 6.5];

module orin_stereo_fixture(){
  // Board fixture
  slot_fixture(
    part=pcb,
    wall=3,
    base_thk=5,
    depth=76,
    rail_w=5,
    rail_h=15,
    slot_w=6,
    tol=0.2,
    slot_pad=3
  );
}

// assemblies/orin_stereo_fixture.scad
use <../models/imager_pcb.scad>

// Place one imager centered at [0,0] with PCB bottom at Z=10
module imager_pcb(){  
    translate([0, 8, 0])
        imager_pcb_with_lens(
            pcb=[38,38,1.75],
            hole_d=2.5,
            inset=[2.25,2.25],
            lens_d=32.5,
            lens_h=45
  );
    translate([0, 70-2, 0])
        imager_pcb_with_lens(
            pcb=[38,38,1.75],
            hole_d=2.5,
            inset=[2.25,2.25],
            lens_d=32.5,
            lens_h=45
  );
}

use <../models/upright_bracket.scad>

module imager_bracket(){
        upright_bracket(
            pcb=[38,38], L=80, H=40, t=3, D=40,
            inset=[2.25,2.25], hole_d=2.5,
            rid_b=40, rid_h=25, rid_d=3, rid_y=[-10, 13],
            cap_d=30, cap_h=5,
            feed_x=40, feed_w=15, feed_h=5,
            m = 1
        );
    
    translate([0,60,0])
        upright_bracket(
            pcb=[38,38], L=80, H=40, t=3, D=40,
            inset=[2.25,2.25], hole_d=2.5,
            rid_b=40, rid_h=25, rid_d=3, rid_y=[-10, 13],
            cap_d=30, cap_h=5,
            feed_x=40, feed_w=15, feed_h=5,
            m = 1
        );
}

use <../models/camarray_hat.scad>

module cam_hat() {
    hat_uc512(
        pcb=[65, 56, 1.75],
        hd=2.5,
        holes=[ [3.25,3.25], [58+3.25,3.25], [3.25,49.0+3.25] ],
        show_keepout=false,
        ko=[4,6]
 );
}
// assemble
h = -56;
d = 70;

module cam_hat_fixture(){
   //cube([65,56+19,4], center = false);
   upright_bracket(
   pcb=[56,65], L=80, H=30, t=5, D=65,
   inset=[3.25,3.25], hole_d=2.5,
   rid_b=20, rid_h=20, rid_d=3, rid_y=[-27.5, 30.5],
   cap_d=0, cap_h=0,
   feed_x=0, feed_w=0, feed_h=0,
   m = 0
);
}

use <../models/orin_dev_board_env.scad>

module orin_dev_board(){
    jetson_orin_nano_dev();
}

module top_assy(){
    rotate([0,180,0])
        translate([-103,0,0])
            orin_stereo_fixture();
    rotate([0,180,90])
        translate([-h, -90, d+3])
            imager_pcb();
    rotate([0,180,-90])
        translate([h,22,d])
            imager_bracket();
    rotate([0,180,0])
        translate([-47.5,-47,40])
            cam_hat();
    rotate([0,180,-90]) 
        translate([-47,47.5,32])
            cam_hat_fixture();
    rotate([0,0,0]) 
        translate([0,-90,-6.25])
            orin_dev_board();
}

// render
top_assy();