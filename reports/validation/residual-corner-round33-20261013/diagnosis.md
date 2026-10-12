# Round 33 residual corner diagnosis

The sole rejected residual fillet is the GND F.Cu bend at `(135.000, 98.600) mm` between UUIDs `529c7bcf-c81b-45d4-9a91-da693372059f` and `11df344f-4ec9-4510-9f46-9f3d0b641a51`. `make_circular_fillet` succeeds for the configured 1.0 mm radius and every halved backtracking radius.

The initial `324998 nm` versus `325000 nm` transaction report was **not a physical 2 nm intrusion**. `circular_arc_to_pad_copper_distance_nm` subtracted an unconditional `2 nm` from every arc-to-pad gap after evaluating the rounded pad core. The implicated foreign rectangular pad has an axis-aligned integral core, so rounding its vertices has zero displacement and the analytic/canonical core gap is `325000 nm`.

The geometry fix replaces that blanket deduction with `ceil(maximum vertex displacement)` for the actual rounded core. The maximum vertex displacement bounds the core's Hausdorff rounding error for its point, segment, or convex polygon representation; subtracting its ceiling stays conservative. Integral cores have a zero guard. A 45-degree 1 mm rectangular core has sub-nanometre fractional vertex displacement and therefore retains a 1 nm guard.

This does not change rounded-core radius handling or the original-core polygon-inside conservative test. It does not apply the final KiCad DRC epsilon to dynamic geometry.
