import sys
import os

# 1. Path & DLL Isolation for Headless FreeCAD 1.1 + Matplotlib
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"
os.environ["PATH"] = freecad_bin_path + os.pathsep + os.environ.get("PATH", "")

if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception:
        pass

sys.path = [p for p in sys.path if r"FreeCAD 1.1\bin\Lib\site-packages" not in p]
if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)

import math
import csv
import numpy as np
import matplotlib.pyplot as plt
import FreeCAD as App
import Part


class FourBarAnalyticalEngine:
    """
    Closed-Loop Analytical 4-Bar Kinematic Engine with Exact Differential Kinematics
    (Position, Velocity, Acceleration, Tangential/Normal Components).
    """

    def __init__(self, L1, L2, L3, d, LP, alpha_deg, omega1=10.0, O_A=(0.0, 0.0)):
        self.L1 = float(L1)
        self.L2 = float(L2)
        self.L3 = float(L3)
        self.d = float(d)
        self.LP = float(LP)
        self.alpha_deg = float(alpha_deg)
        self.alpha = math.radians(alpha_deg)
        self.omega1 = float(omega1)  # Constant crank angular velocity (rad/s)
        self.O_A = O_A
        self.O_B = (O_A[0] + d, O_A[1])

    def solve_kinematics(self, theta1_deg):
        theta1 = math.radians(theta1_deg)
        w1 = self.omega1

        # 1. Position Kinematics
        xA = self.O_A[0] + self.L1 * math.cos(theta1)
        yA = self.O_A[1] + self.L1 * math.sin(theta1)

        Kx = self.O_B[0] - xA
        Ky = self.O_B[1] - yA
        S = math.hypot(Kx, Ky)

        if S > (self.L2 + self.L3) or S < abs(self.L2 - self.L3):
            return None  # Unresolvable geometry

        phi1 = math.atan2(Ky, Kx)
        cos_mu = (self.L2 ** 2 + S ** 2 - self.L3 ** 2) / (2.0 * self.L2 * S)
        cos_mu = max(-1.0, min(1.0, cos_mu))
        mu = math.acos(cos_mu)

        theta2 = phi1 + mu  # Coupler orientation angle

        xB = xA + self.L2 * math.cos(theta2)
        yB = yA + self.L2 * math.sin(theta2)

        theta3 = math.atan2(yB - self.O_B[1], xB - self.O_B[0])

        xP = xA + self.LP * math.cos(theta2 + self.alpha)
        yP = yA + self.LP * math.sin(theta2 + self.alpha)

        # 2. Analytical First Derivatives (Velocity)
        # Loop closure: L1*cos(t1) + L2*cos(t2) - L3*cos(t3) - d = 0
        #               L1*sin(t1) + L2*sin(t2) - L3*sin(t3) = 0
        J = np.array([
            [-self.L2 * math.sin(theta2), self.L3 * math.sin(theta3)],
            [self.L2 * math.cos(theta2), -self.L3 * math.cos(theta3)]
        ])

        b_vel = np.array([
            self.L1 * w1 * math.sin(theta1),
            -self.L1 * w1 * math.cos(theta1)
        ])

        try:
            w_passive = np.linalg.solve(J, b_vel)
            w2, w3 = w_passive[0], w_passive[1]
        except np.linalg.LinAlgError:
            return None

        # Coupler Point Velocity
        vP_x = -self.L1 * w1 * math.sin(theta1) - self.LP * w2 * math.sin(theta2 + self.alpha)
        vP_y = self.L1 * w1 * math.cos(theta1) + self.LP * w2 * math.cos(theta2 + self.alpha)
        vP_mag = math.hypot(vP_x, vP_y)

        # 3. Analytical Second Derivatives (Acceleration)
        b_acc = np.array([
            self.L1 * (w1 ** 2) * math.cos(theta1) + self.L2 * (w2 ** 2) * math.cos(theta2) - self.L3 * (
                        w3 ** 2) * math.cos(theta3),
            self.L1 * (w1 ** 2) * math.sin(theta1) + self.L2 * (w2 ** 2) * math.sin(theta2) - self.L3 * (
                        w3 ** 2) * math.sin(theta3)
        ])

        try:
            alpha_passive = np.linalg.solve(J, b_acc)
            alpha2, alpha3 = alpha_passive[0], alpha_passive[1]
        except np.linalg.LinAlgError:
            return None

        # Coupler Point Acceleration
        aP_x = (-self.L1 * (w1 ** 2) * math.cos(theta1) -
                self.LP * alpha2 * math.sin(theta2 + self.alpha) -
                self.LP * (w2 ** 2) * math.cos(theta2 + self.alpha))

        aP_y = (-self.L1 * (w1 ** 2) * math.sin(theta1) +
                self.LP * alpha2 * math.cos(theta2 + self.alpha) -
                self.LP * (w2 ** 2) * math.sin(theta2 + self.alpha))

        aP_mag = math.hypot(aP_x, aP_y)

        # Tangential & Normal Acceleration Decomposition
        if vP_mag > 1e-6:
            unit_tx = vP_x / vP_mag
            unit_ty = vP_y / vP_mag
            a_tangential = aP_x * unit_tx + aP_y * unit_ty
            a_normal = math.sqrt(max(0.0, aP_mag ** 2 - a_tangential ** 2))
        else:
            a_tangential = 0.0
            a_normal = aP_mag

        return {
            "theta1_deg": theta1_deg,
            "theta2_deg": math.degrees(theta2),
            "theta3_deg": math.degrees(theta3),
            "omega2": w2,
            "omega3": w3,
            "A": (xA, yA),
            "B": (xB, yB),
            "P": (xP, yP),
            "vP": (vP_x, vP_y),
            "vP_mag": vP_mag,
            "aP": (aP_x, aP_y),
            "aP_mag": aP_mag,
            "a_tangential": a_tangential,
            "a_normal": a_normal
        }


def extract_critical_extrema(trajectory_data):
    """
    Finds Global and Local Min/Max for Velocity & Acceleration, and zero-crossings for Tangential Acceleration.
    """
    v_mags = np.array([r["vP_mag"] for r in trajectory_data])
    a_mags = np.array([r["aP_mag"] for r in trajectory_data])
    a_tangs = np.array([r["a_tangential"] for r in trajectory_data])

    # 1. Global Velocity Extrema
    max_v_idx = np.argmax(v_mags)
    min_v_idx = np.argmin(v_mags)

    # 2. Global Acceleration Extrema
    max_a_idx = np.argmax(a_mags)
    min_a_idx = np.argmin(a_mags)

    # 3. Tangential Acceleration Zero-Crossings (a_t = 0)
    zero_at_indices = []
    for i in range(len(a_tangs)):
        curr_val = a_tangs[i]
        next_val = a_tangs[(i + 1) % len(a_tangs)]
        if curr_val * next_val <= 0 and abs(curr_val - next_val) > 1e-4:
            # Pick index closest to absolute zero
            idx = i if abs(curr_val) < abs(next_val) else (i + 1) % len(a_tangs)
            if idx not in zero_at_indices:
                zero_at_indices.append(idx)

    return {
        "max_v": trajectory_data[max_v_idx],
        "min_v": trajectory_data[min_v_idx],
        "max_a": trajectory_data[max_a_idx],
        "min_a": trajectory_data[min_a_idx],
        "zero_at_list": [trajectory_data[i] for i in zero_at_indices]
    }


def export_full_dataset(mechanism, filename="fourbar_kinematic_data.csv", steps=720):
    trajectory_data = []
    step_size = 360.0 / steps

    for i in range(steps):
        crank_deg = i * step_size
        sol = mechanism.solve_kinematics(crank_deg)
        if sol:
            trajectory_data.append(sol)

    with open(filename, mode="w", newline="") as f:
        f.write("# ========================================================\n")
        f.write("# 4-BAR PLANAR ANALYTICAL KINEMATIC DATASET EXPORT\n")
        f.write("# ========================================================\n")
        f.write(f"# Ground Pivot O_A (X, Y): ({mechanism.O_A[0]}, {mechanism.O_A[1]}) mm\n")
        f.write(f"# Ground Pivot O_B (X, Y): ({mechanism.O_B[0]}, {mechanism.O_B[1]}) mm\n")
        f.write(f"# Ground Distance (d): {mechanism.d} mm\n")
        f.write(f"# Link 1 (Crank L1): {mechanism.L1} mm\n")
        f.write(f"# Link 2 (Coupler L2): {mechanism.L2} mm\n")
        f.write(f"# Link 3 (Rocker L3): {mechanism.L3} mm\n")
        f.write(f"# Coupler Point Offset (LP): {mechanism.LP} mm\n")
        f.write(f"# Coupler Point Angle (alpha): {mechanism.alpha_deg} deg\n")
        f.write(f"# Crank Velocity (omega1): {mechanism.omega1} rad/s\n")
        f.write("# ========================================================\n")

        fieldnames = [
            "crank_angle_deg", "coupler_p_x", "coupler_p_y",
            "vel_x", "vel_y", "vel_mag",
            "acc_x", "acc_y", "acc_mag", "acc_tangential", "acc_normal"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in trajectory_data:
            writer.writerow({
                "crank_angle_deg": round(row["theta1_deg"], 2),
                "coupler_p_x": round(row["P"][0], 4),
                "coupler_p_y": round(row["P"][1], 4),
                "vel_x": round(row["vP"][0], 4),
                "vel_y": round(row["vP"][1], 4),
                "vel_mag": round(row["vP_mag"], 4),
                "acc_x": round(row["aP"][0], 4),
                "acc_y": round(row["aP"][1], 4),
                "acc_mag": round(row["aP_mag"], 4),
                "acc_tangential": round(row["a_tangential"], 4),
                "acc_normal": round(row["a_normal"], 4)
            })

    return trajectory_data


def plot_kinematics_with_extrema(mechanism, trajectory_data, extrema):
    init_state = mechanism.solve_kinematics(0.0)

    path_x = [row["P"][0] for row in trajectory_data]
    path_y = [row["P"][1] for row in trajectory_data]

    plt.figure(figsize=(11, 9))

    # 1. Coupler Trajectory
    plt.plot(path_x, path_y, color='#2b5c8f', label='Coupler Point Path (P)', linewidth=2)

    # 2. Mechanism Assembly at t=0
    O_A = mechanism.O_A
    O_B = mechanism.O_B
    A = init_state["A"]
    B = init_state["B"]
    P = init_state["P"]

    plt.plot([O_A[0], O_B[0]], [O_A[1], O_B[1]], 'k--', linewidth=2, label='Ground (O_A - O_B)')
    plt.plot([O_A[0], A[0]], [O_A[1], A[1]], color='#10b981', linewidth=3, label=f'Crank L1 ({mechanism.L1}mm)')
    plt.plot([A[0], B[0], P[0], A[0]], [A[1], B[1], P[1], A[1]], color='#6366f1', linewidth=2, label='Coupler Body ABP')
    plt.plot([B[0], O_B[0]], [B[1], O_B[1]], color='#ec4899', linewidth=3, label=f'Rocker L3 ({mechanism.L3}mm)')

    plt.scatter([O_A[0], O_B[0]], [O_A[1], O_B[1]], color='black', s=120, zorder=5)

    # 3. VELOCITY EXTREMA MARKERS
    max_v = extrema["max_v"]
    min_v = extrema["min_v"]

    plt.scatter(max_v["P"][0], max_v["P"][1], color='red', marker='^', s=150, zorder=6,
                label=f'Max Vel: {max_v["vP_mag"]:.1f} mm/s')
    plt.annotate(f'Max V ({max_v["theta1_deg"]:.1f}°)', (max_v["P"][0], max_v["P"][1]),
                 textcoords="offset points", xytext=(10, 10), fontweight='bold', color='red')

    plt.scatter(min_v["P"][0], min_v["P"][1], color='blue', marker='v', s=150, zorder=6,
                label=f'Min Vel: {min_v["vP_mag"]:.1f} mm/s')
    plt.annotate(f'Min V ({min_v["theta1_deg"]:.1f}°)', (min_v["P"][0], min_v["P"][1]),
                 textcoords="offset points", xytext=(-35, -20), fontweight='bold', color='blue')

    # 4. ACCELERATION EXTREMA MARKERS
    max_a = extrema["max_a"]
    min_a = extrema["min_a"]

    plt.scatter(max_a["P"][0], max_a["P"][1], color='darkorange', marker='s', s=130, zorder=6,
                label=f'Max Accel: {max_a["aP_mag"]:.1f} mm/s²')
    plt.annotate(f'Max Accel ({max_a["theta1_deg"]:.1f}°)', (max_a["P"][0], max_a["P"][1]),
                 textcoords="offset points", xytext=(10, -15), fontweight='bold', color='darkorange')

    plt.scatter(min_a["P"][0], min_a["P"][1], color='purple', marker='d', s=130, zorder=6,
                label=f'Min Accel: {min_a["aP_mag"]:.1f} mm/s²')
    plt.annotate(f'Min Accel ({min_a["theta1_deg"]:.1f}°)', (min_a["P"][0], min_a["P"][1]),
                 textcoords="offset points", xytext=(-40, 15), fontweight='bold', color='purple')

    # 5. ZERO TANGENTIAL ACCELERATION MARKERS (a_t = 0)
    for idx, z_at in enumerate(extrema["zero_at_list"]):
        label_text = f'a_t=0 ({z_at["theta1_deg"]:.1f}°)'
        plt.scatter(z_at["P"][0], z_at["P"][1], color='green', marker='o', s=90, zorder=7,
                    label='Zero Tangential Accel (a_t=0)' if idx == 0 else "")
        plt.annotate(label_text, (z_at["P"][0], z_at["P"][1]),
                     textcoords="offset points", xytext=(8, -8), color='darkgreen', fontsize=9)

    plt.title(f'4-Bar Kinematic Trajectory & Critical Velocity / Acceleration Extrema (ω1 = {mechanism.omega1} rad/s)')
    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(loc='upper right', framealpha=0.9)
    plt.show()


if __name__ == "__main__":
    mechanism = FourBarAnalyticalEngine(
        L1=50.0,
        L2=120.0,
        L3=100.0,
        d=140.0,
        LP=80.0,
        alpha_deg=30.0,
        omega1=10.0,  # 10 rad/s constant crank speed
        O_A=(0.0, 0.0)
    )

    # 1. Generate full dataset
    traj = export_full_dataset(mechanism, filename="fourbar_kinematic_data.csv", steps=720)

    # 2. Extract Min/Max/Zeros for Velocity & Acceleration
    extrema = extract_critical_extrema(traj)

    print("=" * 70)
    print("ANALYTICAL KINEMATIC EXTREMA SUMMARY")
    print("=" * 70)
    print(f"Max Velocity:      {extrema['max_v']['vP_mag']:.2f} mm/s at Crank θ1 = {extrema['max_v']['theta1_deg']}°")
    print(f"Min Velocity:      {extrema['min_v']['vP_mag']:.2f} mm/s at Crank θ1 = {extrema['min_v']['theta1_deg']}°")
    print(f"Max Acceleration:  {extrema['max_a']['aP_mag']:.2f} mm/s² at Crank θ1 = {extrema['max_a']['theta1_deg']}°")
    print(f"Min Acceleration:  {extrema['min_a']['aP_mag']:.2f} mm/s² at Crank θ1 = {extrema['min_a']['theta1_deg']}°")
    print("\nZero Tangential Acceleration Points (a_t = 0):")
    for item in extrema['zero_at_list']:
        print(f"  -> Crank θ1 = {item['theta1_deg']}° | Coupler P = ({item['P'][0]:.2f}, {item['P'][1]:.2f})")
    print("=" * 70)

    # 3. Plot coupler curve with all velocity & acceleration markers
    plot_kinematics_with_extrema(mechanism, traj, extrema)