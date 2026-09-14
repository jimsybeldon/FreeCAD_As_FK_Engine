import sys
import os

# 1. Environment & DLL Isolation for Headless FreeCAD 1.1 + Matplotlib
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"
os.environ["PATH"] = freecad_bin_path + os.pathsep + os.environ.get("PATH", "")

if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception:
        pass

# Remove FreeCAD's internal site-packages to prevent NumPy C-extension collisions
sys.path = [p for p in sys.path if r"FreeCAD 1.1\bin\Lib\site-packages" not in p]
if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)

import math
import csv
import matplotlib.pyplot as plt
import FreeCAD as App
import Part


class FourBarKinematicEngine:
    """
    Analytical Closed-Loop 2D Planar Four-Bar Linkage Kinematic Engine.
    """
    def __init__(self, L1, L2, L3, d, LP, alpha_deg, O_A=(0.0, 0.0)):
        self.L1 = L1                  # Crank length (O_A -> A)
        self.L2 = L2                  # Coupler length (A -> B)
        self.L3 = L3                  # Rocker length (B -> O_B)
        self.d = d                    # Ground pivot spacing
        self.LP = LP                  # Coupler point offset distance from Pivot A
        self.alpha_deg = alpha_deg    # Coupler point offset angle relative to segment AB
        self.alpha = math.radians(alpha_deg)
        self.O_A = O_A                # Base Ground Pivot X, Y
        self.O_B = (O_A[0] + d, O_A[1])  # Follower Ground Pivot X, Y

    def solve_position(self, theta1_deg):
        theta1 = math.radians(theta1_deg)

        # 1. Crank Tip Position (Pivot A)
        xA = self.O_A[0] + self.L1 * math.cos(theta1)
        yA = self.O_A[1] + self.L1 * math.sin(theta1)

        # 2. Diagonal Vector from Pivot A to Ground Pivot O_B
        Kx = self.O_B[0] - xA
        Ky = self.O_B[1] - yA
        S = math.hypot(Kx, Ky)

        # 3. Geometric Boundary / Assembly Check
        if S > (self.L2 + self.L3) or S < abs(self.L2 - self.L3):
            return None  # Unresolvable loop closure (out of reach)

        # 4. Law of Cosines for Passive Coupler Angle (theta2) and Rocker Angle (theta3)
        phi1 = math.atan2(Ky, Kx)
        cos_mu = (self.L2**2 + S**2 - self.L3**2) / (2 * self.L2 * S)
        cos_mu = max(-1.0, min(1.0, cos_mu))
        mu = math.acos(cos_mu)

        # Coupler centerline orientation angle (AB)
        theta2 = phi1 + mu

        # 5. Pivot B Position (Coupler-Rocker Joint)
        xB = xA + self.L2 * math.cos(theta2)
        yB = yA + self.L2 * math.sin(theta2)

        # 6. Rocker Angle (theta3) relative to ground
        theta3 = math.atan2(yB - self.O_B[1], xB - self.O_B[0])

        # 7. Rigid Coupler Point P Position
        xP = xA + self.LP * math.cos(theta2 + self.alpha)
        yP = yA + self.LP * math.sin(theta2 + self.alpha)

        return {
            "theta1_deg": theta1_deg,
            "theta2_deg": math.degrees(theta2),
            "theta3_deg": math.degrees(theta3),
            "A": (xA, yA),
            "B": (xB, yB),
            "P": (xP, yP)
        }


def export_full_dataset(mechanism, filename="fourbar_coupler_path.csv", steps=720):
    trajectory_data = []
    step_size = 360.0 / steps

    for i in range(steps):
        crank_deg = i * step_size
        sol = mechanism.solve_position(crank_deg)
        if sol:
            trajectory_data.append({
                "step": i,
                "crank_angle_deg": round(sol["theta1_deg"], 2),
                "coupler_angle_deg": round(sol["theta2_deg"], 4),
                "rocker_angle_deg": round(sol["theta3_deg"], 4),
                "pivot_a_x": round(sol["A"][0], 4),
                "pivot_a_y": round(sol["A"][1], 4),
                "pivot_b_x": round(sol["B"][0], 4),
                "pivot_b_y": round(sol["B"][1], 4),
                "coupler_p_x": round(sol["P"][0], 4),
                "coupler_p_y": round(sol["P"][1], 4),
            })

    with open(filename, mode="w", newline="") as f:
        # --- METADATA HEADER BLOCK ---
        f.write("# ========================================================\n")
        f.write("# 4-BAR PLANAR KINEMATIC ENGINE DATASET EXPORT\n")
        f.write("# ========================================================\n")
        f.write(f"# Ground Pivot O_A (X, Y): ({mechanism.O_A[0]}, {mechanism.O_A[1]}) mm\n")
        f.write(f"# Ground Pivot O_B (X, Y): ({mechanism.O_B[0]}, {mechanism.O_B[1]}) mm\n")
        f.write(f"# Ground Distance (d): {mechanism.d} mm\n")
        f.write(f"# Link 1 (Crank Length L1): {mechanism.L1} mm\n")
        f.write(f"# Link 2 (Coupler Length L2): {mechanism.L2} mm\n")
        f.write(f"# Link 3 (Rocker Length L3): {mechanism.L3} mm\n")
        f.write(f"# Coupler Point Offset (LP): {mechanism.LP} mm\n")
        f.write(f"# Coupler Point Offset Angle (alpha): {mechanism.alpha_deg} deg (ref to segment AB)\n")
        f.write(f"# Resolution Steps: {steps} (Step size: {step_size:.2f} deg)\n")
        f.write("# ========================================================\n")

        # --- DATA TABLE ---
        fieldnames = [
            "step", "crank_angle_deg", "coupler_angle_deg", "rocker_angle_deg",
            "pivot_a_x", "pivot_a_y", "pivot_b_x", "pivot_b_y", "coupler_p_x", "coupler_p_y"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(trajectory_data)

    return trajectory_data


def plot_kinematics(mechanism, trajectory_data, j1_init=30.0):
    init_state = mechanism.solve_position(j1_init)

    path_x = [row["coupler_p_x"] for row in trajectory_data]
    path_y = [row["coupler_p_y"] for row in trajectory_data]

    plt.figure(figsize=(10, 8))

    # 1. Closed-Loop Coupler Path Trajectory
    plt.plot(path_x, path_y, 'r--', label='Coupler Point Path (P)', linewidth=2)

    # 2. Four-Bar Linkage Skeleton at initial step
    O_A = mechanism.O_A
    O_B = mechanism.O_B
    A = init_state["A"]
    B = init_state["B"]
    P = init_state["P"]

    # Ground Link
    plt.plot([O_A[0], O_B[0]], [O_A[1], O_B[1]], 'k--', linewidth=2, label='Ground Link (O_A - O_B)')
    # Crank (L1)
    plt.plot([O_A[0], A[0]], [O_A[1], A[1]], 'g-o', linewidth=3, label=f'Crank L1 ({mechanism.L1}mm)')
    # Coupler Triangle Body (ABP)
    plt.plot([A[0], B[0], P[0], A[0]], [A[1], B[1], P[1], A[1]], 'b-o', linewidth=2, label=f'Coupler L2 & Point P ({mechanism.LP}mm @ {mechanism.alpha_deg}°)')
    # Rocker (L3)
    plt.plot([B[0], O_B[0]], [B[1], O_B[1]], 'm-o', linewidth=3, label=f'Rocker L3 ({mechanism.L3}mm)')

    # Ground Pivot Indicators
    plt.scatter([O_A[0], O_B[0]], [O_A[1], O_B[1]], color='black', s=120, zorder=5)
    plt.annotate(f'O_A {O_A}', O_A, textcoords="offset points", xytext=(-15, -15), ha='center')
    plt.annotate(f'O_B {O_B}', O_B, textcoords="offset points", xytext=(15, -15), ha='center')

    plt.title('2D Four-Bar Mechanism Kinematic Trajectory & Coupler Curve')
    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(loc='upper right')
    plt.show()


def create_freecad_geometry(mechanism, init_state, doc_name="FourBar_2D_Planar"):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    O_A = App.Vector(mechanism.O_A[0], mechanism.O_A[1], 0)
    O_B = App.Vector(mechanism.O_B[0], mechanism.O_B[1], 0)
    A = App.Vector(init_state["A"][0], init_state["A"][1], 0)
    B = App.Vector(init_state["B"][0], init_state["B"][1], 0)
    P = App.Vector(init_state["P"][0], init_state["P"][1], 0)

    # Wire primitives representing the closed mechanism
    lines = [
        Part.makeLine(O_A, A),  # Crank
        Part.makeLine(A, B),    # Coupler centerline
        Part.makeLine(A, P),    # Coupler offset edge 1
        Part.makeLine(B, P),    # Coupler offset edge 2
        Part.makeLine(B, O_B),  # Rocker
        Part.makeLine(O_A, O_B) # Ground segment
    ]

    sketch_obj = doc.addObject("Part::Feature", "FourBar_Assembly_Wire")
    sketch_obj.Shape = Part.Wire(lines)
    doc.recompute()


if __name__ == "__main__":
    # Define complete linkage geometry:
    # Ground O_A=(0,0), Ground O_B=(140,0) [d=140mm]
    # Crank L1=50mm, Coupler L2=120mm, Rocker L3=100mm
    # Coupler Point P: 80mm from Pivot A at 30° angle relative to line AB
    mechanism = FourBarKinematicEngine(
        L1=50.0,
        L2=120.0,
        L3=100.0,
        d=140.0,
        LP=80.0,
        alpha_deg=30.0,
        O_A=(0.0, 0.0)
    )

    # 1. Export CSV dataset with complete metadata & 720 trajectory steps
    traj = export_full_dataset(mechanism, filename="fourbar_coupler_path.csv", steps=720)

    # 2. Build FreeCAD 2D wire geometry for initial posture
    init_state = mechanism.solve_position(0.0)
    create_freecad_geometry(mechanism, init_state)

    print("=" * 65)
    print("FOUR-BAR KINEMATIC SWEEP & DATA EXPORT SUCCESSFUL")
    print(f"Dataset generated: 'fourbar_coupler_path.csv' (720 steps)")
    print(f"Ground Pivots: O_A{mechanism.O_A}, O_B{mechanism.O_B}")
    print(f"Dimensions: L1={mechanism.L1}mm, L2={mechanism.L2}mm, L3={mechanism.L3}mm, d={mechanism.d}mm")
    print(f"Coupler Offset P: Distance={mechanism.LP}mm, Angle={mechanism.alpha_deg}°")
    print("=" * 65)

    # 3. Plot coupler path and mechanism structure
    plot_kinematics(mechanism, traj, j1_init=30.0)