import os
import json
import FreeCAD as App
import Part
import Sketcher

# 1. Define Project Directory and Absolute Paths
PROJECT_DIR = r"C:\Users\jimsy\PycharmProjects\FreeCAD_As_FK_Engine\Src_Multi_Link"
input_json = os.path.join(PROJECT_DIR, "mechanism2_3.json")
output_json = os.path.join(PROJECT_DIR, "mechanism2_3_revised.json")


# 2. Custom Builder Function (Renamed to avoid namespace collision)
def my_build_sketch_from_json(json_path, doc_name="LinkageDoc", sketch_name="LinkageSketch"):
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Initialize Document & Sketch
    if doc_name not in App.listDocuments():
        doc = App.newDocument(doc_name)
    else:
        doc = App.getDocument(doc_name)

    sketch = doc.addObject('Sketcher::SketchObject', sketch_name)

    nodes = {n["id"]: n for n in data["nodes"]}
    edges = data["edges"]
    edge_geometry_indices = []

    # Draw Line Segments
    for edge in edges:
        u_id, v_id = edge["nodeIds"]
        u, v = nodes[u_id], nodes[v_id]
        p1 = App.Vector(float(u["x"]), -float(u["y"]), 0.0)
        p2 = App.Vector(float(v["x"]), -float(v["y"]), 0.0)
        line_idx = sketch.addGeometry(Part.LineSegment(p1, p2), False)
        edge_geometry_indices.append((u_id, v_id, line_idx))

    # Apply Coincident Constraints
    node_vertex_map = {}
    for u_id, v_id, geo_idx in edge_geometry_indices:
        for node_id, v_type in [(u_id, 1), (v_id, 2)]:
            if node_id not in node_vertex_map:
                node_vertex_map[node_id] = (geo_idx, v_type)
            else:
                ref_geo, ref_v = node_vertex_map[node_id]
                sketch.addConstraint(Sketcher.Constraint('Coincident', geo_idx, v_type, ref_geo, ref_v))

    # Lock Ground Pivots
    for n_id, n in nodes.items():
        if n.get("isGround") and n_id in node_vertex_map:
            geo_idx, v_type = node_vertex_map[n_id]
            sketch.addConstraint(Sketcher.Constraint('Block', geo_idx, v_type))

    doc.recompute()
    print(f"Successfully created '{sketch_name}' in document '{doc_name}'.")


# 3. Custom Exporter Function (Renamed to avoid namespace collision)
def my_export_sketch_to_json(doc_name="LinkageDoc", sketch_name="LinkageSketch", output_json="mechanism_out.json"):
    doc = App.getDocument(doc_name)
    sketch = doc.getObject(sketch_name)
    doc.recompute()

    nodes_dict = {}
    edges_list = []
    node_counter = 0

    def get_or_create_node_id(pos_vec, is_ground=False):
        nonlocal node_counter
        for n_id, data in nodes_dict.items():
            if abs(data["x"] - pos_vec.x) < 1e-3 and abs(-data["y"] - pos_vec.y) < 1e-3:
                if is_ground:
                    data["isGround"] = True
                return n_id

        nodes_dict[node_counter] = {
            "id": node_counter,
            "x": round(pos_vec.x, 2),
            "y": round(-pos_vec.y, 2),
            "isGround": is_ground,
            "isTarget": False
        }
        assigned_id = node_counter
        node_counter += 1
        return assigned_id

    grounded_vertices = set()
    for c in sketch.Constraints:
        if c.Type == 'Block':
            grounded_vertices.add((c.First, c.FirstPos))

    for geo_idx, geo in enumerate(sketch.Geometry):
        if hasattr(geo, "StartPoint") and hasattr(geo, "EndPoint"):
            p1 = geo.StartPoint
            p2 = geo.EndPoint
            u_ground = (geo_idx, 1) in grounded_vertices
            v_ground = (geo_idx, 2) in grounded_vertices

            u_id = get_or_create_node_id(p1, is_ground=u_ground)
            v_id = get_or_create_node_id(p2, is_ground=v_ground)
            is_motor = (len(edges_list) == 0)

            edges_list.append({
                "nodeIds": [u_id, v_id],
                "isMotor": is_motor
            })

    nodes_array = list(nodes_dict.values())
    if len(nodes_array) > 4:
        nodes_array[4]["isTarget"] = True

    export_payload = {
        "nodes": nodes_array,
        "edges": edges_list,
        "nodeCount": len(nodes_array)
    }

    with open(output_json, 'w') as f:
        json.dump(export_payload, f, indent=2)

    print(f"Successfully exported JSON to: {output_json}")


# 4. Execute Workflow
my_build_sketch_from_json(input_json)
my_export_sketch_to_json(doc_name="LinkageDoc", sketch_name="LinkageSketch", output_json=output_json)