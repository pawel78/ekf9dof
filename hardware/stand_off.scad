// Stand-off: ID=2.5 mm, OD=5 mm, Length=5 mm
// Printable-friendly with optional tolerance & bevels.

id   = 2.5;   // inner diameter (hole)
od   = 5.0;   // outer diameter
len  = 5.0;   // length
$fn  = 96;    // smoothness

// Optional tweaks
tol        = 0.15;   // add e.g. 0.15 for easier screw fit

module standoff(id=2.5, od=5.0, len=5.0, tol=0.0){
    difference(){
        // main body
        cylinder(h=len, d=od, center=false);

        // through-hole (with optional tolerance)
        translate([0,0,-0.1])  // tiny offset to ensure clean cut
            cylinder(h=len+0.2, d=id+tol, center=false);
    }

}

// render it
standoff(id=id, od=od, len=len, tol=tol);
