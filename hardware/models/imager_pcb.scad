// Parametric imager PCB with corner holes and a centered lens boss.
// Origin: PCB is centered in X/Y at [pos.x, pos.y]; PCB bottom sits at pos.z.

module imager_pcb_with_lens(
    pcb      = [38, 38, 1.6],   // [W, H, T] in mm
    hole_d   = 2.2,             // through-hole diameter
    inset    = [3, 3],          // [dx, dy] from each PCB corner to hole center
    lens_d   = 16,              // lens cylinder diameter
    lens_h   = 8,               // lens cylinder height
    pcb_col  = "#1a7f37",
    lens_col = "#999999",
    $fn      = 64               // smooth cylinders
){
    W = pcb[0]; H = pcb[1]; T = pcb[2];
    ix = inset[0]; iy = inset[1];

    // ---------- PCB w/ corner holes ----------
    difference() {
        // PCB centered in XY, bottom at pos.z
        color(pcb_col)
        translate([- W/2, - H/2, 0])
            cube([W, H, T], center=false);

        // Corner hole pattern (through PCB)
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx*(W/2 - ix),
                       sy*(H/2 - iy),
                       - 0.1])           // slight overshoot to ensure clean cut
                cylinder(d=hole_d, h=T + 0.3, $fn=$fn);
        }
    }

    // ---------- Lens boss (centered on PCB) ----------
    color(lens_col)
    translate([0, 0, 0 + T])
        cylinder(d=lens_d, h=lens_h, $fn=$fn);
}

//// example
//    imager_pcb_with_lens(
//    pcb=[38,38,1.75],
//    hole_d=2.5,
//    inset=[3,3],
//    lens_d=32.5,
//    lens_h=45
//  );