import os
import json
import FreeCAD as App
import Part
import Sketcher

PROJECT_DIR = r"C:\Users\jimsy\PycharmProjects\FreeCAD_As_FK_Engine\FreeCAD_Python_Console_Holder"
INPUT_JSON = os.path.join(PROJECT_DIR, "mechanism2_3.json")
OUTPUT_JSON = os.path.join(PROJECT_DIR, "mechanism2_3_revised.json")

def process_linkage_headless(json_in, json_out):
    print(f"Reading input JSON from: {json_in}")
    with open(json_in, 'r') as f:
        data = json.load(f)

    # 1. Create Document in memory (Headless mode)
    doc = App.newDocument("HeadlessLinkageDoc")
    sketch = doc.addObject("Sketcher::SketchObject", "LinkageSketch")

    nodes = {n["id"]: n for n in data["nodes"]}
    edges = data["edges"]
    edge_geometry_indices = []

    # 2. Build Lines
    for edge in edges:
        u_id, v_id = edge["nodeIds"]
        u, v = nodes[u_id], nodes[v_id]
        p1 = App.Vector(float(u["x"]), -float(u["y"]), 0.0)
        p2 = App.Vector(float(v["x"]), -float(v["y"]), 0.0)
        line_idx = sketch.addGeometry(Part.LineSegment(p1, p2), False)
        edge_geometry_indices.append((u_id, v_id, line_idx))

    # 3. Add Coincident Constraints
    node_vertex_map = {}
    for u_id, v_id, geo_idx in edge_geometry_indices:
        for node_id, v_type in [(u_id, 1), (v_id, 2)]:
            if node_id not in node_vertex_map:
                node_vertex_map[node_id] = (geo_idx, v_type)
            else:
                ref_geo, ref_v = node_vertex_map[node_id]
                sketch.addConstraint(Sketcher.Constraint("Coincident", geo_idx, v_type, ref_geo, ref_v))

    # 4. Lock Ground Nodes (Fix: Use DistanceX and DistanceY to fix coordinates)
    grounded_nodes = set()
    for n_id, n in nodes.items():
        if n.get("isGround") and n_id in node_vertex_map:
            geo_idx, v_type = node_vertex_map[n_id]
            x_val = float(n["x"])
            y_val = -float(n["y"])

            # Pin X and Y positions directly
            sketch.addConstraint(Sketcher.Constraint("DistanceX", geo_idx, v_type, x_val))
            sketch.addConstraint(Sketcher.Constraint("DistanceY", geo_idx, v_type, y_val))

            grounded_nodes.add((geo_idx, v_type))

    # Solve constraints programmatically
    doc.recompute()
    print("Sketch built and solved in memory successfully.")

    # 5. Save Document for GUI inspection (Optional)
    fcstd_out = os.path.join(PROJECT_DIR, "LinkageSketchDoc.FCStd")
    doc.saveAs(fcstd_out)
    print(f"Saved FreeCAD Document to: {fcstd_out}")

    # 6. Extract Vertices & Export Revised JSON
    nodes_dict = {}
    edges_list = []
    node_counter = 0

    def get_or_create_node_id(pos_vec, is_ground=False):
        nonlocal node_counter
        for n_id, d in nodes_dict.items():
            if abs(d["x"] - pos_vec.x) < 1e-3 and abs(-d["y"] - pos_vec.y) < 1e-3:
                if is_ground:
                    d["isGround"] = True
                return n_id

        nodes_dict[node_counter] = {
            "id": node_counter,
            "x": round(pos_vec.x, 2),
            "y": round(-pos_vec.y, 2),
            "isGround": is_ground,
            "isTarget": False
        }
        assigned = node_counter
        node_counter += 1
        return assigned

    grounded_vertices = set()
    for c in sketch.Constraints:
        if c.Type == "Block":
            grounded_vertices.add((c.First, c.FirstPos))

    for geo_idx, geo in enumerate(sketch.Geometry):
        if hasattr(geo, "StartPoint") and hasattr(geo, "EndPoint"):
            p1 = geo.StartPoint
            p2 = geo.EndPoint
            u_ground = (geo_idx, 1) in grounded_nodes
            v_ground = (geo_idx, 2) in grounded_nodes

            u_id = get_or_create_node_id(p1, is_ground=u_ground)
            v_id = get_or_create_node_id(p2, is_ground=v_ground)
            is_motor = (len(edges_list) == 0)

            edges_list.append({"nodeIds": [u_id, v_id], "isMotor": is_motor})

    nodes_array = list(nodes_dict.values())
    if len(nodes_array) > 4:
        nodes_array[4]["isTarget"] = True

    export_payload = {
        "nodes": nodes_array,
        "edges": edges_list,
        "nodeCount": len(nodes_array)
    }

    with open(json_out, 'w') as f:
        json.dump(export_payload, f, indent=2)

    print(f"Exported revised JSON to: {json_out}")
    App.closeDocument("HeadlessLinkageDoc")

if __name__ == "__main__":
    process_linkage_headless(INPUT_JSON, OUTPUT_JSON)