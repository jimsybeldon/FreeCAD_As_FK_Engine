import math
import sys
import os

# Ensure FreeCAD path is present
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"  # Update if 1.0 or installed elsewhere
if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)
if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception:
        pass

import FreeCAD as App
import Part


def build_fk_chain(j1_deg=0.0, j2_deg=0.0, j3_deg=-20.0):
    doc_name = "FK_Engine_POC"

    # Safely handle document creation in headless mode
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # Define Link Dimensions (mm)
    L1_len, L1_rad = 40.0, 8.0
    L2_len, L2_rad = 100.0, 5.0
    L3_len, L3_rad = 80.0, 3.0

    def make_link_shape(length, radius):
        cyl = Part.makeCylinder(radius, length)
        cyl.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), 90)
        return cyl

    link1 = doc.addObject("Part::Feature", "Base_Link")
    link1.Shape = make_link_shape(L1_len, L1_rad)

    link2 = doc.addObject("Part::Feature", "Arm_Link")
    link2.Shape = make_link_shape(L2_len, L2_rad)

    link3 = doc.addObject("Part::Feature", "Forearm_Link")
    link3.Shape = make_link_shape(L3_len, L3_rad)

    # FK Calculations
    T_j1 = App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(0, 0, 1), j1_deg))
    link1.Placement = T_j1

    offset_L1 = App.Placement(App.Vector(L1_len, 0, 0), App.Rotation())
    rot_j2 = App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(0, 1, 0), j2_deg))
    T_arm = T_j1.multiply(offset_L1).multiply(rot_j2)
    link2.Placement = T_arm

    offset_L2 = App.Placement(App.Vector(L2_len, 0, 0), App.Rotation())
    rot_j3 = App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(0, 1, 0), j3_deg))
    T_forearm = T_arm.multiply(offset_L2).multiply(rot_j3)
    link3.Placement = T_forearm

    offset_L3 = App.Placement(App.Vector(L3_len, 0, 0), App.Rotation())
    T_ee = T_forearm.multiply(offset_L3)
    ee_pos = T_ee.Base

    doc.recompute()
    return ee_pos


if __name__ == "__main__":
    end_effector_xyz = build_fk_chain(j1_deg=45.0, j2_deg=30.0, j3_deg=-20.0)
    print("=" * 50)
    print(f"FK EXECUTION SUCCESSFUL")
    print(
        f"End Effector Position -> X: {end_effector_xyz.x:.2f}, Y: {end_effector_xyz.y:.2f}, Z: {end_effector_xyz.z:.2f}")
    print("=" * 50)