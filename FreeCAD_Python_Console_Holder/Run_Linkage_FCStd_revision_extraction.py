import os
import json
import math
import sys

# 1. Environment & Path Setup for FreeCAD 1.1
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
FCSTD_INPUT = os.path.join(PROJECT_DIR, "LinkageSketchDoc.FCStd")
TEMPLATE_JSON = os.path.join(PROJECT_DIR, "mechanism2_3.json")
OUTPUT_JSON = os.path.join(PROJECT_DIR, "mechanism2_3_revised.json")


def extract_fcstd_to_json(fcstd_path, template_json_path, json_out_path):
    """
    Reads an edited FreeCAD document, extracts updated vertex positions from
    the Sketch, maps Y-coordinates back to screen space, and writes out revised JSON.
    """
    print(f"Opening FreeCAD Document: {fcstd_path}")
    doc = App.openDocument(fcstd_path)
    sketch = doc.getObject("LinkageSketch")

    if not sketch:
        raise ValueError("Target sketch 'LinkageSketch' not found in document.")

    # Recompute to ensure all sketch constraints and geometries are updated
    doc.recompute()

    # Read original JSON structure to preserve metadata and edge relationships
    with open(template_json_path, 'r') as f:
        revised_data = json.load(f)

    nodes = revised_data["nodes"]
    edges = revised_data["edges"]

    # Reconstruct dynamic node position mapping
    # Track node positions by aggregating line start (1) and end (2) vertex coordinates
    node_positions = {n["id"]: [] for n in nodes}

    for idx, edge in enumerate(edges):
        u_id, v_id = edge["nodeIds"]

        # Retrieve solved LineSegment geometry from FreeCAD sketcher
        geo = sketch.Geometry[idx]
        if isinstance(geo, Part.LineSegment):
            p1 = geo.StartPoint
            p2 = geo.EndPoint

            # FreeCAD Y (upward) -> Screen Y (downward)
            node_positions[u_id].append((round(p1.x, 2), round(-p1.y, 2)))
            node_positions[v_id].append((round(p2.x, 2), round(-p2.y, 2)))

    # Update node coordinates in revised dataset averaging coincidents to eliminate floating point precision drift
    for n in nodes:
        n_id = n["id"]
        coords = node_positions[n_id]
        if coords:
            avg_x = round(sum(c[0] for c in coords) / len(coords), 2)
            avg_y = round(sum(c[1] for c in coords) / len(coords), 2)
            n["x"] = avg_x
            n["y"] = avg_y

    # Save revised JSON output
    with open(json_out_path, 'w') as f:
        json.dump(revised_data, f, indent=2)

    print(f"Successfully extracted updated geometry to: {json_out_path}")
    App.closeDocument(doc.Name)


if __name__ == "__main__":
    extract_fcstd_to_json(FCSTD_INPUT, TEMPLATE_JSON, OUTPUT_JSON)