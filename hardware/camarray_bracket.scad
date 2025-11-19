// Arducam Camarray HAT UC-512 (parametric, 3-hole version)
// Origin: PCB centered at (0,0); bottom at Z=0.

module hat_uc512(
    pcb   = [65, 56, 1.75],     // [L, W, T] mm
    hd    = 2.5,                // hole diameter
    // Three hole centers given from LOWER-LEFT PCB corner (mm):
    // Default = triangular pattern: (ox,oy), (ox+px,oy), (ox,oy+py)
    holes = [ [2.25,2.25], [58,2.25], [2.25,49.0] ],
    show_keepout = false,       // draw keep-outs above PCB?
    ko    = [3.5, 5.0],         // [keep-out diameter, height]
    pcb_col = "#177245",
    $fn = 64
){
    // Unpack
    L = pcb[0]; W = pcb[1]; T = pcb[2];
    ko_d = ko[0]; ko_h = ko[1];

    // Convert lower-left based coords -> centered-origin coords
    function to_center(p) = [ p[0] - L/2, p[1] - W/2 ];

    // --- PCB plate ---
    module pcb_plate(){
        color(pcb_col)
        translate([-L/2, -W/2, 0])
            cube([L, W, T], center=false);
    }

    // --- Geometry ---
    difference(){
        pcb_plate();

        // Drill exactly 3 holes (robust to any 3-length list)
        for (i = [0 : min(2, len(holes)-1)]) {
            p = to_center(holes[i]);
            translate([p[0], p[1], -0.1])
                cylinder(d=hd, h=T+0.2, $fn=$fn);
        }
    }

    // Optional keep-outs
    if (show_keepout)
        for (i = [0 : min(2, len(holes)-1)]) {
            p = to_center(holes[i]);
            color([1,0,0,0.25])
            translate([p[0], p[1], T])
                cylinder(d=ko_d, h=ko_h, $fn=$fn);
        }
}

// --- Examples (comment out in library file) ---
 hat_uc512(
   pcb=[65, 56, 1.75],
   hd=2.5,
   holes=[ [2.25,2.25], [58,2.25], [2.25,49.0] ],
   show_keepout=true,
   ko=[4,6]
 );

