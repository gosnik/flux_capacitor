"""Measure final copper centreline length with the installed KiCad board API."""
import json
from pathlib import Path
import pcbnew
root = Path(__file__).resolve().parent
results = {'method': 'KiCad GetLength on track/arc copper plus net-assigned copper Bezier shapes; excludes via barrels', 'kicad_version': pcbnew.GetBuildVersion(), 'runs': {}}
for name in ('lazy-navmesh-round16-20261012', root.name):
    tiers = {}
    for tier in ('tier0', 'tier1'):
        source = next((root.parent/name/tier).glob('tier-*/*.kicad_pcb'))
        board = pcbnew.LoadBoard(str(source))
        tracks = [track for track in board.GetTracks() if not isinstance(track, pcbnew.PCB_VIA)]
        curves = [shape for shape in board.GetDrawings() if isinstance(shape, pcbnew.PCB_SHAPE)
                  and shape.GetShape() == pcbnew.SHAPE_T_BEZIER and shape.GetNetCode() > 0
                  and pcbnew.IsCopperLayer(shape.GetLayer())]
        tiers[tier] = {'length_mm': sum(float(item.GetLength()) for item in tracks + curves) / 1e6,
                       'tracks_and_arcs': len(tracks), 'net_assigned_copper_beziers': len(curves)}
    results['runs'][name] = tiers
(root/'copper-lengths.json').write_text(json.dumps(results, indent=2) + '\n')
print(json.dumps(results, indent=2))
