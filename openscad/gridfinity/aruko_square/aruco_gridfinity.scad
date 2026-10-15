// The Python build script supplies these values through OpenSCAD's -D option.
base_file = is_undef(base_file) ? "aruco_base.stl" : base_file;
base_scale = is_undef(base_scale) ? 1000 : base_scale;
base_scale_x = is_undef(base_scale_x) ? base_scale : base_scale_x;
base_scale_y = is_undef(base_scale_y) ? base_scale : base_scale_y;
base_scale_z = is_undef(base_scale_z) ? base_scale : base_scale_z;
marker_bits = is_undef(marker_bits) ? [[1]] : marker_bits;
marker_size_mm = is_undef(marker_size_mm) ? 32 : marker_size_mm;
marker_center_x = is_undef(marker_center_x) ? 0 : marker_center_x;
marker_center_y = is_undef(marker_center_y) ? 0 : marker_center_y;
top_z = is_undef(top_z) ? 6.76 : top_z;
engrave_depth_mm = is_undef(engrave_depth_mm) ? 0.6 : engrave_depth_mm;
mmu_depth_mm = is_undef(mmu_depth_mm) ? 0.4 : mmu_depth_mm;
output_part = is_undef(output_part) ? "engraved" : output_part;
cut_overlap_mm = 0.01; // Prevent coincident faces between neighboring cells.

module base() {
    // X/Y may be enlarged for fit while Z and marker_size_mm stay unchanged.
    scale([base_scale_x, base_scale_y, base_scale_z])
        import(base_file, convexity = 10);
}

module marker_volume(depth_mm) {
    rows = len(marker_bits);
    columns = len(marker_bits[0]);
    cell_x = marker_size_mm / columns;
    cell_y = marker_size_mm / rows;
    start_x = marker_center_x - marker_size_mm / 2;
    start_y = marker_center_y - marker_size_mm / 2;

    for (row = [0 : rows - 1])
        for (column = [0 : columns - 1])
            if (marker_bits[row][column] == 1)
                translate([
                    start_x + column * cell_x - cut_overlap_mm,
                    start_y + (rows - 1 - row) * cell_y - cut_overlap_mm,
                    top_z - depth_mm
                ])
                    cube([
                        cell_x + 2 * cut_overlap_mm,
                        cell_y + 2 * cut_overlap_mm,
                        depth_mm + 0.02
                    ]);
}

if (output_part == "engraved") {
    difference() {
        base();
        marker_volume(engrave_depth_mm);
    }
} else if (output_part == "mmu_white") {
    // Main body, with marker-shaped pockets spanning the requested top layers.
    difference() {
        base();
        marker_volume(mmu_depth_mm);
    }
} else if (output_part == "mmu_black") {
    // Black inlays share the original model's outer top surface exactly.
    intersection() {
        base();
        marker_volume(mmu_depth_mm);
    }
} else {
    assert(false, str("Unknown output_part: ", output_part));
}
