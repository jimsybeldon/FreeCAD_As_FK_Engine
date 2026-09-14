import math
import sys
import os

# FreeCAD 1.1 Binary Path Linking
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"
if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)
if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception:
        pass

import FreeCAD as App
import Part


class PlanarKinematicChain2D:
    """
    2D Kinematic Chain operating strictly within a planar coordinate system (X, Y, Theta).
    """

    def __init__(self, link_lengths):
        self.lengths = link_lengths

    def compute_fk(self, angles_deg):
        assert len(angles_deg) == len(self.lengths), "Angle count must match link count."

        joint_positions = [(0.0, 0.0)]
        cumulative_theta = 0.0

        current_x = 0.0
        current_y = 0.0

        for length, angle in zip(self.lengths, angles_deg):
            cumulative_theta += math.radians(angle)

            current_x += length * math.cos(cumulative_theta)
            current_y += length * math.sin(cumulative_theta)

            joint_positions.append((current_x, current_y))

        end_effector = (current_x, current_y, math.degrees(cumulative_theta))
        return joint_positions, end_effector


def create_freecad_2d_sketch_geometry(joint_coords, end_effector, doc_name="Planar_2D_FK"):
    """
    Generates a 2D visual representation (wires/lines) directly on a 2D plane in FreeCAD.
    """
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # Construct 2D line segments across joint positions (Z is locked to 0)
    lines = []
    for i in range(len(joint_coords) - 1):
        p1 = App.Vector(joint_coords[i][0], joint_coords[i][1], 0)
        p2 = App.Vector(joint_coords[i + 1][0], joint_coords[i + 1][1], 0)
        lines.append(Part.makeLine(p1, p2))

    # Combine into a single planar wire shape
    arm_wire = Part.Wire(lines)
    sketch_obj = doc.addObject("Part::Feature", "Planar_Arm_Wire")
    sketch_obj.Shape = arm_wire

    doc.recompute()
    return end_effector


# --- EXECUTION ---
if __name__ == "__main__":
    arm = PlanarKinematicChain2D(link_lengths=[100.0, 80.0, 50.0])

    joint_points, end_effector = arm.compute_fk([30.0, 45.0, -15.0])

    # Pass both joint_points and end_effector
    create_freecad_2d_sketch_geometry(joint_points, end_effector)

    print("=" * 50)
    print("2D PLANAR FK EXECUTION SUCCESSFUL")
    print(f"Joint Points (X, Y): {[(round(x, 2), round(y, 2)) for x, y in joint_points]}")
    print(f"End Effector (X, Y, Heading°): ({end_effector[0]:.2f}, {end_effector[1]:.2f}, {end_effector[2]:.2f}°)")
    print("=" * 50)
