// Parametric slot fixture for Jetson Orin Nano–style boards.
// Call from your top assembly with: use <fixtures/slot_fixture.scad>; orin_slot_fixture(...);

module slot_fixture(
    part      = [100, 0, 6],    // [L, W, T] = board length X, width Y, thickness Z
    wall      = 3,              // wall thickness
    base_thk  = 5,              // base plate thickness
    depth     = 25,             // fixture depth along +Z
    rail_w    = 5,              // side rail width (X)
    rail_h    = 15,             // side rail height (Y); defaults to max(15, T+9)
    slot_w    = 6,              // slot opening height (X before rotate)
    tol       = 0.2,            // clearance on slot opening and thickness
    slot_pad  = 3,              // distance of each slot from left/right X edges
    body_col  = "#555555"       // color for visual sanity
){
    L = part[0];
    W = part[1];
    T = part[2];

    RH = is_undef(rail_h) ? max(15, T + 9) : rail_h;       // sensible default
    ext = RH + tol;                                         // extrude a touch longer to ensure clean cuts

    module base_plate_and_rails(){
        // --- Base plate (rear bumper behind PCB along +Z) ---
        color(body_col)
        translate([-wall, W, -wall])
            cube([L + 2*wall, 5/*wall*/, depth], center=false);

        // --- Side rails (two verticals near the top edge of PCB) ---
        color(body_col){
            // Left rail
            translate([-wall, W - RH, -wall])
                cube([rail_w, RH, depth], center=false);
            // Right rail
            translate([L + wall - rail_w, W - RH, -wall])
                cube([rail_w, RH, depth], center=false);
        }
    }
    // --- Dual slide slots (hollows) extruded along +Y ---
    // We build a 2D square in XY, rotate so +Z→+Y, then extrude along +Y.
    module slot_at(xpos){
        rotate([90, 0, 0])                 // +Z → +Y
        translate([xpos, wall, 0])         // start just behind the bumper
            linear_extrude(height=ext)
                square([slot_w + tol, T + tol], center=true);
    }

    difference(){
        // Everything above
        // Automatically drawn
        base_plate_and_rails();
        // Minus two mirrored slots: near left and right X edges
        union(){
            slot_at(slot_pad);           // left slot
            slot_at(L - slot_pad);       // right slot
        }
    }
}

//// example oard fixture
//  slot_fixture(
//    part=[103, 0, 6.5],
//    wall=3,
//    base_thk=5,
//    depth=25,
//    rail_w=5,
//    rail_h=15,
//    slot_w=6,
//    tol=0.2,
//    slot_pad=3
//  );