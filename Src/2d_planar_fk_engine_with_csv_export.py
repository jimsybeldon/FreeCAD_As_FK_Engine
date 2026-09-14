import math
import sys
import os
import csv

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


class PlanarMechanism2D:
    """
    2D Kinematic Engine evaluating mechanism positions and coupler point trajectories
    over prescribed input joint rotations.
    """
    def __init__(self, l_crank: float, l_coupler: float, coupler_offset_len: float, coupler_offset_angle_deg: float):
        """
        l_crank: Length of crank link L1 (mm)
        l_coupler: Length of coupler link L2 (mm)
        coupler_offset_len: Distance from Pivot B to Coupler Point P (mm)
        coupler_offset_angle_deg: Angular offset of Coupler Point P relative to coupler centerline AB (degrees)
        """
        self.l1 = l_crank
        self.l2 = l_coupler
        self.r_p = coupler_offset_len
        self.alpha_p = coupler_offset_angle_deg

    def compute_coupler_point(self, theta1_deg: float, theta2_relative_deg: float):
        """
        Calculates Cartesian coordinates of Pivot A, Pivot B, and rigid Coupler Point P
        given input crank angle (theta1) and relative coupler angle (theta2).
        """
        t1_rad = math.radians(theta1_deg)
        t2_rel_rad = math.radians(theta2_relative_deg)

        # Ground Pivot O (0,0)
        x_o, y_o = 0.0, 0.0

        # Pivot A (Crank Tip)
        x_a = x_o + self.l1 * math.cos(t1_rad)
        y_a = y_o + self.l1 * math.sin(t1_rad)

        # Absolute orientation of Coupler Link (AB)
        theta_ab_rad = t1_rad + t2_rel_rad

        # Pivot B (Coupler Tip)
        x_b = x_a + self.l2 * math.cos(theta_ab_rad)
        y_b = y_a + self.l2 * math.sin(theta_ab_rad)

        # Coupler Point P (Offset relative to Pivot B and locked to link AB frame)
        theta_p_rad = theta_ab_rad + math.radians(self.alpha_p)
        x_p = x_b + self.r_p * math.cos(theta_p_rad)
        y_p = y_b + self.r_p * math.sin(theta_p_rad)

        return (x_a, y_a), (x_b, y_b), (x_p, y_p)


def generate_coupler_trajectory_csv(mechanism, output_filepath="coupler_trajectory.csv", steps=720, fixed_theta2_deg=45.0):
    """
    Sweeps crank angle over 360 degrees in specified steps and outputs trajectory to CSV.
    """
    trajectory_data = []

    for step in range(steps):
        # Calculate crank angle from 0.0 to 360.0 degrees across `steps` intervals
        crank_angle_deg = (step / steps) * 360.0

        # Compute forward kinematics
        pivot_a, pivot_b, coupler_p = mechanism.compute_coupler_point(
            theta1_deg=crank_angle_deg,
            theta2_relative_deg=fixed_theta2_deg
        )

        trajectory_data.append({
            "step": step,
            "crank_angle_deg": round(crank_angle_deg, 4),
            "coupler_x": round(coupler_p[0], 4),
            "coupler_y": round(coupler_p[1], 4),
            "pivot_a_x": round(pivot_a[0], 4),
            "pivot_a_y": round(pivot_a[1], 4),
            "pivot_b_x": round(pivot_b[0], 4),
            "pivot_b_y": round(pivot_b[1], 4)
        })

    # Write data to CSV using Python's standard csv module
    fieldnames = ["step", "crank_angle_deg", "coupler_x", "coupler_y", "pivot_a_x", "pivot_a_y", "pivot_b_x", "pivot_b_y"]
    
    with open(output_filepath, mode="w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(trajectory_data)

    return output_filepath, trajectory_data


def visualize_final_frame_in_freecad(joint_coords, doc_name="Planar_2D_FK"):
    """
    Generates a 2D Part::Feature wire shape of the mechanism in FreeCAD for the final frame.
    """
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    lines = []
    for i in range(len(joint_coords) - 1):
        p1 = App.Vector(joint_coords[i][0], joint_coords[i][1], 0.0)
        p2 = App.Vector(joint_coords[i+1][0], joint_coords[i+1][1], 0.0)
        lines.append(Part.makeLine(p1, p2))

    arm_wire = Part.Wire(lines)
    sketch_obj = doc.addObject("Part::Feature", "Mechanism_Coupler_Wire")
    sketch_obj.Shape = arm_wire

    doc.recompute()


if __name__ == "__main__":
    # Define mechanism dimensions (mm) and coupler point offset geometry
    mechanism = PlanarMechanism2D(
        l_crank=100.0,                 # Link 1 length
        l_coupler=80.0,                # Link 2 length
        coupler_offset_len=50.0,       # Coupler Point offset distance (r_p)
        coupler_offset_angle_deg=-15.0 # Coupler Point angular offset (alpha_p)
    )

    # Output CSV filename and sweep settings
    csv_filename = "coupler_trajectory.csv"
    total_steps = 720

    # Run kinematic sweep and save output to CSV
    saved_path, dataset = generate_coupler_trajectory_csv(
        mechanism=mechanism,
        output_filepath=csv_filename,
        steps=total_steps,
        fixed_theta2_deg=45.0
    )

    # Build FreeCAD 2D shape for the final evaluated step
    last_step = dataset[-1]
    final_coords = [
        (0.0, 0.0),
        (last_step["pivot_a_x"], last_step["pivot_a_y"]),
        (last_step["pivot_b_x"], last_step["pivot_b_y"]),
        (last_step["coupler_x"], last_step["coupler_y"])
    ]
    visualize_final_frame_in_freecad(final_coords)

    print("=" * 60)
    print("2D PLANAR KINEMATICS SWEEP COMPLETE")
    print(f"Total Steps Generated: {total_steps} (Step size: {360.0 / total_steps:.2f}°)")
    print(f"CSV Exported Successfully: {os.path.abspath(saved_path)}")
    print("Sample Trajectory Data (First 3 Steps & Final Step):")
    for row in dataset[:3] + [dataset[-1]]:
        print(f"  Step {row['step']:3d} | Crank: {row['crank_angle_deg']:7.2f}° | Coupler Point (X, Y): ({row['coupler_x']:8.2f}, {row['coupler_y']:8.2f})")
    print("=" * 60)