// Parametric L-stand with corner holes, support ridges, lens-cap boss, and MiPi slot.
// Origin: PCB center at (0,0); base plane at Z=0.

module upright_bracket(
    pcb    = [40, 40],    // [W, H] of the board in XY (mm)
    L      = 80,          // base length along +X (mm)
    H      = 40,          // upright height along +Z (mm)
    t      = 3,           // material thickness (mm)
    D      = 40,          // stand depth along +Y (mm)
    inset  = [3,3],       // corner-hole inset [dx, dy] from each PCB corner (mm)
    hole_d = 3.2,         // corner-hole diameter (mm)
    // Support ridge (triangular gusset) params
    rid_b  = 20,          // ridge base run along -X (mm)
    rid_h  = 25,          // ridge rise along +Z (mm)
    rid_d  = 3,           // ridge extrude depth along +Y (mm)
    rid_y  = [-10, 13],   // Y offsets for multiple ridges (array)
    // Lens-cap boss
    cap_d  = 30,          // lens-cap cylinder diameter (mm)
    cap_h  = 5,           // lens-cap cylinder height (mm)
    // MiPi cable feed slot
    feed_x = 40,          // slot X start position (mm)
    feed_w = 15,          // slot width (along Y) (mm)
    feed_h = 5,           // slot height (along Z) (mm)
    tol    = 0.2,         // small clearance for cut-throughs
    m      = 1,           // optional margin shift in Y for base/upright placement
    $fn    = 48           // cylinder smoothness
){
    W = pcb[0];
    Hh = pcb[1];          // PCB height in Y

    // --- Helpers ---
    module ridge_at(yoff){
        // Right-angle gusset in XZ, extruded along +Y
        // Triangle: joint corner at (0,0); run -rid_b on X; rise +rid_h on Z
        pts = [[0,0], [-rid_b,0], [0,rid_h]];
        translate([ L - t - W/2, yoff, 0 ])
            rotate([90,0,0])               // XY → XZ
                linear_extrude(height=rid_d)
                    polygon(points=pts);
    }

    module lens_cap_holder(){
        // Centered on origin, sits behind the PCB plane
        translate([0, 0, -cap_h])
            cylinder(d=cap_d, h=cap_h, $fn=$fn);
    }

    // --- Main shape ---
    difference(){
        union(){
            // Base plate
            translate([ -W/2, -Hh/2 - m, 0 ])
                cube([ L, D, t ], center=false);

            // Upright (shares edge at base’s far-right)
            translate([ L - t - W/2, -Hh/2 - m, 0 ])
                cube([ t, D, H ], center=false);

            // Ridges (one per y-offset)
            for(yoff = rid_y) ridge_at(yoff);

            // Lens-cap boss
            lens_cap_holder();
        }

        // Corner hole pattern (through base thickness)
        // Holes referenced to PCB corners at Z ~ 0 (base plane)
        dx = inset[0]; dy = inset[1];
        for (sx=[-1,1], sy=[-1,1]) {
            translate([ sx*(W/2 - dx), sy*(Hh/2 - dy), -0.1 ])
                cylinder(d=hole_d, h=t + 0.4, $fn=$fn);
        }

        // MiPi cable feed: a slot through the upright near its root (adjust as needed)
        translate([ feed_x, -feed_w/2, -0.1 ])
            cube([ t + tol, feed_w, feed_h + 0.2 ], center=false);
    }
}

 //--- Example usage (comment out in library file) ---
//upright_bracket(
//   pcb=[38,38], L=80, H=40, t=3, D=40,
//   inset=[2.25,2.25], hole_d=2.5,
//   rid_b=40, rid_h=25, rid_d=3, rid_y=[-10, 13],
//   cap_d=30, cap_h=5,
//   feed_x=40, feed_w=15, feed_h=5,
//   m = 1
//);