// ---------- Params (mm) ----------
pcb_w = 38;
pcb_h = 38;
pcb_t = 1.75;
assy_dz = 3.0;

lens_d = 32.5;
lens_h = 45;

hole_d     = 2.5;  // hole diameter
edge_clear = 1.0;  // clearance from PCB edge to hole EDGE

$fn = 96;   // smoothness for the cylinder/holes

// ---------- Helpers ----------
inset = edge_clear + hole_d/2; // center inset from each edge

// ---------- Model ----------
module pcb_with_holes() {
    difference() {
        // PCB centered in X/Y, bottom at z=0
        translate([-pcb_w/2, -pcb_h/2, assy_dz])
            cube([pcb_w, pcb_h, pcb_t], center=false);

        // corner hole pattern (through holes)
        for (sx=[-1,1], sy=[-1,1]) {
            translate([sx*(pcb_w/2 - inset), sy*(pcb_h/2 - inset), -0.1 + assy_dz])
                cylinder(d=hole_d, h=pcb_t+0.4);
        }
    }
}

module lens() {
    // Lens cylinder sits on top face of PCB, centered
    translate([0, 0, pcb_t + assy_dz])
        cylinder(d=lens_d, h=lens_h);
}

// ----- Parameters (mm) -----
base_len = 80;   // bottom leg length
upright_h = 40;  // vertical leg height
thickness = 3;   // material thickness
width = 40;      // stand width (depth)
ridge_depth = 3; // ridge depth

// ----- Geometry -----

// lens cap holder
cap_h = 5;

module lens_cap_holder() {
    // place the lens cap holder directly behind the lens
    translate([0, 0, -cap_h])
    cylinder(d=30, h=cap_h);
}

// triangular gusset between base and upright
module support_ridge(base_extent=20, height_extent=25, y_offset) {
    // triangle defined in XZ plane (side view)
    pts = [
        [0, 0],                    // joint corner
        [-base_extent-20, 0],         // along base
        [0,  height_extent]        // up the upright
    ];

    // place it at the joint, rotate so it's in XZ, extrude along Y
    translate([base_len-thickness-pcb_w/2, y_offset, 0])
        rotate([90, 0, 0])              // XY→XZ
            linear_extrude(height = ridge_depth)
                polygon(points = pts);
}

module l_stand_with_holes() {
    difference() {
        union() {
            // Base
            translate([-pcb_w/2, -pcb_h/2 - 1.0, 0])
                cube([base_len, width, thickness], center=false);
            // Upright
            translate([base_len - thickness - pcb_w/2, -pcb_h/2 - 1.0, 0])
                cube([thickness, width, upright_h], center=false);
            
            support_ridge(20, 25, -10);    // tweak these numbers to adjust ridge size
            support_ridge(20, 25, 13);    // tweak these numbers to adjust ridge size
            lens_cap_holder();
        }
        // corner hole pattern (through holes)
        for (sx=[-1,1], sy=[-1,1]) {
            translate([sx*(pcb_w/2 - inset), sy*(pcb_h/2 - inset), -0.1])
                cylinder(d=hole_d, h=thickness+0.4);
        }
        // MiPi cable feed
        translate([40, -15/2, 0-0.1])
        cube([thickness+0.2, 15, 5], center=false);
    }
}

module assembly() {
    pcb_with_holes();
    lens();
    l_stand_with_holes();
}


assembly();
//l_stand_with_holes();