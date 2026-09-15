import os
import json
import math
import sys

# 1. Isolate and prioritize FreeCAD binary paths FIRST
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"

if freecad_bin_path not in os.environ.get("PATH", ""):
    os.environ["PATH"] = freecad_bin_path + os.pathsep + os.environ.get("PATH", "")

if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception:
        pass

if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)

freecad_site_packages = os.path.join(freecad_bin_path, "Lib", "site-packages")
sys.path = [p for p in sys.path if os.path.normpath(p) != os.path.normpath(freecad_site_packages)]

import FreeCAD as App
import Part
import Sketcher

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_JSON = os.path.join(PROJECT_DIR, "mechanism2_3.json")
OUTPUT_JSON = os.path.join(PROJECT_DIR, "mechanism2_3_revised.json")


def process_linkage_headless(json_in, json_out):
    print(f"Reading input JSON from: {json_in}")
    with open(json_in, 'r') as f:
        data = json.load(f)

    doc = App.newDocument("HeadlessLinkageDoc")
    sketch = doc.addObject("Sketcher::SketchObject", "LinkageSketch")

    nodes = {n["id"]: n for n in data["nodes"]}
    edges = data["edges"]

    # Map node IDs to sketch elements: node_id -> list of (geo_idx, pos_type)
    node_to_sketch_points = {n_id: [] for n_id in nodes}
    edge_geo_map = {}

    # 1. Add Line Geometries & Length Constraints
    for idx, edge in enumerate(edges):
        u_id, v_id = edge["nodeIds"]
        u, v = nodes[u_id], nodes[v_id]

        p1 = App.Vector(float(u["x"]), -float(u["y"]), 0.0)
        p2 = App.Vector(float(v["x"]), -float(v["y"]), 0.0)

        line_idx = sketch.addGeometry(Part.LineSegment(p1, p2), False)
        edge_geo_map[idx] = line_idx

        # Map start point (1) to u_id, end point (2) to v_id
        node_to_sketch_points[u_id].append((line_idx, 1))
        node_to_sketch_points[v_id].append((line_idx, 2))

        # Add Distance constraint between line end vertices (1 -> 2)
        length = math.hypot(p2.x - p1.x, p2.y - p1.y)
        sketch.addConstraint(Sketcher.Constraint("Distance", line_idx, 1, line_idx, 2, length))

    # 2. Add Coincident Constraints to join shared nodes
    for n_id, points in node_to_sketch_points.items():
        if len(points) > 1:
            ref_geo, ref_pos = points[0]
            for target_geo, target_pos in points[1:]:
                sketch.addConstraint(Sketcher.Constraint("Coincident", target_geo, target_pos, ref_geo, ref_pos))

    # 3. Lock Ground Node Coordinates
    for n_id, n in nodes.items():
        if n.get("isGround") and node_to_sketch_points[n_id]:
            geo_idx, pos_type = node_to_sketch_points[n_id][0]
            x_val = float(n["x"])
            y_val = -float(n["y"])

            sketch.addConstraint(Sketcher.Constraint("DistanceX", geo_idx, pos_type, x_val))
            sketch.addConstraint(Sketcher.Constraint("DistanceY", geo_idx, pos_type, y_val))

    # Solve assembly kinematics
    doc.recompute()

    # Correct property access for Degrees of Freedom in FreeCAD Python API
    dof = sketch.solve()
    print(f"Sketch assembly complete. Remaining Degrees of Freedom: {dof}")

    # Save output FCStd document
    fcstd_out = os.path.join(PROJECT_DIR, "LinkageSketchDoc.FCStd")
    doc.saveAs(fcstd_out)
    print(f"Saved FreeCAD Document to: {fcstd_out}")

    App.closeDocument("HeadlessLinkageDoc")


if __name__ == "__main__":
    process_linkage_headless(INPUT_JSON, OUTPUT_JSON)