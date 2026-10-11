KRT pinned reference run
Source revision: 0db4b103f95a63995e8e2c2b2c8c0e3077a2388a
Input: tests/fixtures/routing-corpus/bm_svf.kicad_pcb (project sibling preserved separately)
Native build: python3 build_router.py --from-source
Route: python3 py_router/route.py INPUT OUTPUT --track-width 0.2 --clearance 0.2 --via-size 0.7 --via-drill 0.3 --escalation off --strict-sizes
No reduced rules or rule escalations requested. KRT emitted a warning that output project floor min_via_drill was written to 0.3 and copper-to-hole remains below original board 0.25; this is recorded as a comparability limitation.
