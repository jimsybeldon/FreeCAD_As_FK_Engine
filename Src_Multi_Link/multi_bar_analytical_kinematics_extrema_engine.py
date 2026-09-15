import sys
import os

# Filter out FreeCAD site-packages to avoid NumPy version conflicts
sys.path = [p for p in sys.path if r"FreeCAD 1.1\bin\Lib\site-packages" not in p]

import math
import csv
import json
import numpy as np
import matplotlib.pyplot as plt


class MultiLinkageKinematicEngine:
    """
    Generalized 1-DOF Multilink Kinematic Engine.
    Dynamically analyzes JSON topology to build an automated solve pipeline
    for arbitrary 1-DOF dyad & triad planar mechanisms.
    """
    def __init__(self, json_path, omega1=10.0):
        with open(json_path, 'r') as f:
            self.data = json.load(f)

        self.omega1 = float(omega1)
        self.nodes = {n["id"]: n for n in self.data["nodes"]}
        self.edges = self.data["edges"]

        # Store nominal link lengths & build adjacency graph
        self.link_lengths = {}
        self.adj = {n_id: set() for n_id in self.nodes}
        for edge in self.edges:
            u, v = edge["nodeIds"]
            self.adj[u].add(v)
            self.adj[v].add(u)
            p1 = np.array([self.nodes[u]["x"], self.nodes[u]["y"]])
            p2 = np.array([self.nodes[v]["x"], self.nodes[v]["y"]])
            dist = np.linalg.norm(p1 - p2)
            self.link_lengths[(min(u, v), max(u, v))] = dist

        # Identify motor edge
        self.motor_edge = next(e for e in self.edges if e.get("isMotor", False))
        n0_id, n2_id = self.motor_edge["nodeIds"]

        # Ensure n0 is the ground node
        if not self.nodes[n0_id]["isGround"]:
            n0_id, n2_id = n2_id, n0_id

        self.crank_ground_id = n0_id
        self.crank_node_id = n2_id

        # Pre-calculate base crank radius and initial angle offset
        p0 = np.array([self.nodes[self.crank_ground_id]["x"], self.nodes[self.crank_ground_id]["y"]])
        p2_0 = np.array([self.nodes[self.crank_node_id]["x"], self.nodes[self.crank_node_id]["y"]])
        self.crank_r = np.linalg.norm(p2_0 - p0)
        self.initial_crank_angle = math.atan2(p2_0[1] - p0[1], p2_0[0] - p0[0])

        # Dynamic Solve Sequence Builder
        self.solve_sequence = self._build_solve_sequence()

    def get_length(self, u, v):
        return self.link_lengths[(min(u, v), max(u, v))]

    def _build_solve_sequence(self):
        resolved = set()
        for n_id, n in self.nodes.items():
            if n["isGround"]:
                resolved.add(n_id)

        resolved.add(self.crank_node_id)
        sequence = []
        unresolved = set(self.nodes.keys()) - resolved

        while unresolved:
            progress = False

            # Check for Dyad Solves (Target node connected to 2 already resolved nodes)
            for target in list(unresolved):
                neighbors = self.adj[target]
                resolved_neighbors = list(neighbors.intersection(resolved))
                if len(resolved_neighbors) >= 2:
                    sequence.append({
                        "type": "dyad",
                        "target": target,
                        "ref1": resolved_neighbors[0],
                        "ref2": resolved_neighbors[1]
                    })
                    resolved.add(target)
                    unresolved.remove(target)
                    progress = True
                    break

            if progress:
                continue

            # Check for Triad Solves (Target node forming a rigid triangle with 2 resolved nodes)
            for target in list(unresolved):
                for r1 in list(resolved):
                    for r2 in list(resolved):
                        if r1 < r2:
                            has_t_r1 = target in self.adj[r1]
                            has_t_r2 = target in self.adj[r2]
                            has_r1_r2 = r2 in self.adj[r1]
                            if has_t_r1 and has_t_r2 and has_r1_r2:
                                sequence.append({
                                    "type": "triad",
                                    "target": target,
                                    "ref1": r1,
                                    "ref2": r2
                                })
                                resolved.add(target)
                                unresolved.remove(target)
                                progress = True
                                break
                    if progress:
                        break

            if not progress:
                raise ValueError(f"Kinematic loop unresolvable at nodes: {unresolved}")

        return sequence

    @staticmethod
    def solve_dyad(P1, P2, r1, r2, prev_P3=None):
        d_vec = P2 - P1
        d = np.linalg.norm(d_vec)
        if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
            return None

        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = math.sqrt(max(0.0, r1**2 - a**2))

        P2_mid = P1 + a * (d_vec / d)

        sol1 = np.array([
            P2_mid[0] + h * (P2[1] - P1[1]) / d,
            P2_mid[1] - h * (P2[0] - P1[0]) / d
        ])
        sol2 = np.array([
            P2_mid[0] - h * (P2[1] - P1[1]) / d,
            P2_mid[1] + h * (P2[0] - P1[0]) / d
        ])

        if prev_P3 is None:
            return sol1

        d1 = np.linalg.norm(sol1 - prev_P3)
        d2 = np.linalg.norm(sol2 - prev_P3)
        return sol1 if d1 < d2 else sol2

    @staticmethod
    def solve_triad(P_ref1, P_ref2, orig1, orig2, target_orig):
        v_orig = orig2 - orig1
        v_curr = P_ref2 - P_ref1

        ang_orig = math.atan2(v_orig[1], v_orig[0])
        ang_curr = math.atan2(v_curr[1], v_curr[0])
        d_theta = ang_curr - ang_orig

        R = np.array([
            [math.cos(d_theta), -math.sin(d_theta)],
            [math.sin(d_theta),  math.cos(d_theta)]
        ])

        v_target = target_orig - orig1
        return P_ref1 + R @ v_target

    def solve_kinematics_step(self, theta_deg, prev_positions=None):
        positions = {}

        for n_id, n in self.nodes.items():
            if n["isGround"]:
                positions[n_id] = np.array([n["x"], n["y"]], dtype=float)

        theta = math.radians(theta_deg) + self.initial_crank_angle
        p0 = positions[self.crank_ground_id]
        positions[self.crank_node_id] = np.array([
            p0[0] + self.crank_r * math.cos(theta),
            p0[1] + self.crank_r * math.sin(theta)
        ])

        for step in self.solve_sequence:
            t = step["target"]
            r1, r2 = step["ref1"], step["ref2"]

            if step["type"] == "dyad":
                prev_t = prev_positions[t] if prev_positions else np.array([self.nodes[t]["x"], self.nodes[t]["y"]])
                sol = self.solve_dyad(
                    positions[r1], positions[r2],
                    self.get_length(r1, t), self.get_length(r2, t),
                    prev_P3=prev_t
                )
                if sol is None:
                    return None
                positions[t] = sol

            elif step["type"] == "triad":
                orig1 = np.array([self.nodes[r1]["x"], self.nodes[r1]["y"]])
                orig2 = np.array([self.nodes[r2]["x"], self.nodes[r2]["y"]])
                orig_t = np.array([self.nodes[t]["x"], self.nodes[t]["y"]])
                positions[t] = self.solve_triad(positions[r1], positions[r2], orig1, orig2, orig_t)

        return positions


def compute_kinematic_derivatives(engine, steps=720):
    """
    Computes numerical velocity, total acceleration, tangential acceleration,
    and normal acceleration for the Target node across a full 360-degree cycle.
    """
    target_id = next((n_id for n_id, n in engine.nodes.items() if n.get("isTarget", False)), 4)
    step_deg = 360.0 / steps
    d_theta_rad = math.radians(step_deg)
    dt = d_theta_rad / engine.omega1

    raw_positions = []
    angles_deg = []
    prev_frame = None

    for i in range(steps):
        deg = i * step_deg
        frame_sol = engine.solve_kinematics_step(deg, prev_positions=prev_frame)
        if frame_sol is None:
            raise ValueError(f"Kinematic lockup or assembly fail at theta = {deg}°")
        raw_positions.append(frame_sol)
        angles_deg.append(deg)
        prev_frame = frame_sol

    target_pts = np.array([f[target_id] for f in raw_positions])

    # Central finite difference for periodic signal velocity
    vel_x = np.gradient(target_pts[:, 0], dt)
    vel_y = np.gradient(target_pts[:, 1], dt)
    vel_mag = np.hypot(vel_x, vel_y)

    # Central finite difference for acceleration
    acc_x = np.gradient(vel_x, dt)
    acc_y = np.gradient(vel_y, dt)
    acc_mag = np.hypot(acc_x, acc_y)

    # Component projections (tangential and normal acceleration)
    acc_tangential = np.zeros(steps)
    acc_normal = np.zeros(steps)

    for i in range(steps):
        v_m = vel_mag[i]
        if v_m > 1e-6:
            u_tx = vel_x[i] / v_m
            u_ty = vel_y[i] / v_m
            a_t = acc_x[i] * u_tx + acc_y[i] * u_ty
            acc_tangential[i] = a_t
            acc_normal[i] = math.sqrt(max(0.0, acc_mag[i]**2 - a_t**2))
        else:
            acc_tangential[i] = 0.0
            acc_normal[i] = acc_mag[i]

    dataset = []
    for i in range(steps):
        dataset.append({
            "theta1_deg": angles_deg[i],
            "frame": raw_positions[i],
            "P": target_pts[i],
            "vP": (vel_x[i], vel_y[i]),
            "vP_mag": vel_mag[i],
            "aP": (acc_x[i], acc_y[i]),
            "aP_mag": acc_mag[i],
            "a_tangential": acc_tangential[i],
            "a_normal": acc_normal[i]
        })

    return dataset, target_id


def extract_critical_extrema(trajectory_data):
    v_mags = np.array([r["vP_mag"] for r in trajectory_data])
    a_mags = np.array([r["aP_mag"] for r in trajectory_data])
    a_tangs = np.array([r["a_tangential"] for r in trajectory_data])

    max_v_idx = np.argmax(v_mags)
    min_v_idx = np.argmin(v_mags)

    max_a_idx = np.argmax(a_mags)
    min_a_idx = np.argmin(a_mags)

    zero_at_indices = []
    n = len(a_tangs)
    for i in range(n):
        curr_val = a_tangs[i]
        next_val = a_tangs[(i + 1) % n]
        if curr_val * next_val <= 0 and abs(curr_val - next_val) > 1e-4:
            idx = i if abs(curr_val) < abs(next_val) else (i + 1) % n
            if idx not in zero_at_indices:
                zero_at_indices.append(idx)

    return {
        "max_v": trajectory_data[max_v_idx],
        "min_v": trajectory_data[min_v_idx],
        "max_a": trajectory_data[max_a_idx],
        "min_a": trajectory_data[min_a_idx],
        "zero_at_list": [trajectory_data[i] for i in zero_at_indices]
    }


def export_dataset_to_csv(trajectory_data, filename="multilinkage_kinematic_data.csv"):
    with open(filename, mode="w", newline="") as f:
        f.write("# ========================================================\n")
        f.write("# GENERALIZED MULTI-LINKAGE KINEMATIC DATASET EXPORT\n")
        f.write("# ========================================================\n")
        fieldnames = [
            "crank_angle_deg", "target_p_x", "target_p_y",
            "vel_x", "vel_y", "vel_mag",
            "acc_x", "acc_y", "acc_mag", "acc_tangential", "acc_normal"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in trajectory_data:
            writer.writerow({
                "crank_angle_deg": round(row["theta1_deg"], 2),
                "target_p_x": round(row["P"][0], 4),
                "target_p_y": round(row["P"][1], 4),
                "vel_x": round(row["vP"][0], 4),
                "vel_y": round(row["vP"][1], 4),
                "vel_mag": round(row["vP_mag"], 4),
                "acc_x": round(row["aP"][0], 4),
                "acc_y": round(row["aP"][1], 4),
                "acc_mag": round(row["aP_mag"], 4),
                "acc_tangential": round(row["a_tangential"], 4),
                "acc_normal": round(row["a_normal"], 4)
            })


def plot_kinematics_with_extrema(engine, trajectory_data, extrema, target_id):
    fig, ax = plt.subplots(figsize=(12, 10))

    # 1. Plot complete trace paths for floating nodes
    node_ids = sorted(list(engine.nodes.keys()))
    for n_id in node_ids:
        if not engine.nodes[n_id]["isGround"]:
            px = [frame["frame"][n_id][0] for frame in trajectory_data]
            py = [frame["frame"][n_id][1] for frame in trajectory_data]
            is_target = (n_id == target_id)
            color = '#ff8c00' if is_target else '#1e88e5'
            lw = 2.5 if is_target else 1.0
            alpha = 0.9 if is_target else 0.4
            ax.plot(px, py, color=color, linewidth=lw, alpha=alpha,
                    label=f'Target Path (Node {n_id})' if is_target else None)

    # 2. Draw initial configuration link lines
    init_frame = trajectory_data[0]["frame"]
    for edge in engine.edges:
        u, v = edge["nodeIds"]
        p1 = init_frame[u]
        p2 = init_frame[v]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='black', linewidth=1.5, zorder=3)

    # 3. Plot Ground Pivots
    gx = [engine.nodes[n]["x"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    gy = [engine.nodes[n]["y"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    ax.scatter(gx, gy, color='#d32f2f', s=140, zorder=6, label='Ground Pivots')

    # 4. Annotate Extrema Points on Target Path
    max_v = extrema["max_v"]
    min_v = extrema["min_v"]
    max_a = extrema["max_a"]
    min_a = extrema["min_a"]

    ax.scatter(max_v["P"][0], max_v["P"][1], color='red', marker='^', s=160, zorder=7,
               label=f'Max Vel: {max_v["vP_mag"]:.1f} px/s')
    ax.annotate(f'Max V ({max_v["theta1_deg"]:.1f}°)', (max_v["P"][0], max_v["P"][1]),
                textcoords="offset points", xytext=(10, 10), fontweight='bold', color='red')

    ax.scatter(min_v["P"][0], min_v["P"][1], color='blue', marker='v', s=160, zorder=7,
               label=f'Min Vel: {min_v["vP_mag"]:.1f} px/s')
    ax.annotate(f'Min V ({min_v["theta1_deg"]:.1f}°)', (min_v["P"][0], min_v["P"][1]),
                textcoords="offset points", xytext=(-35, -20), fontweight='bold', color='blue')

    ax.scatter(max_a["P"][0], max_a["P"][1], color='darkorange', marker='s', s=140, zorder=7,
               label=f'Max Accel: {max_a["aP_mag"]:.1f} px/s²')
    ax.annotate(f'Max Accel ({max_a["theta1_deg"]:.1f}°)', (max_a["P"][0], max_a["P"][1]),
                textcoords="offset points", xytext=(10, -15), fontweight='bold', color='darkorange')

    ax.scatter(min_a["P"][0], min_a["P"][1], color='purple', marker='d', s=140, zorder=7,
               label=f'Min Accel: {min_a["aP_mag"]:.1f} px/s²')
    ax.annotate(f'Min Accel ({min_a["theta1_deg"]:.1f}°)', (min_a["P"][0], min_a["P"][1]),
                textcoords="offset points", xytext=(-40, 15), fontweight='bold', color='purple')

    for idx, z_at in enumerate(extrema["zero_at_list"]):
        ax.scatter(z_at["P"][0], z_at["P"][1], color='green', marker='o', s=100, zorder=8,
                   label='Zero Tangential Accel (a_t=0)' if idx == 0 else "")
        ax.annotate(f'a_t=0 ({z_at["theta1_deg"]:.1f}°)', (z_at["P"][0], z_at["P"][1]),
                    textcoords="offset points", xytext=(8, -8), color='darkgreen', fontsize=9)

    ax.set_title(f'Multi-Link Kinematic Trajectory & Extrema (ω1 = {engine.omega1} rad/s)', fontsize=12)
    ax.set_xlabel('X Position (px)')
    ax.set_ylabel('Y Position (px)')
    ax.invert_yaxis()
    ax.axis('equal')
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='lower right', framealpha=0.9)
    plt.show()


if __name__ == "__main__":
    json_path = "mechanism2.json"

    engine = MultiLinkageKinematicEngine(json_path, omega1=10.0)

    trajectory, target_id = compute_kinematic_derivatives(engine, steps=720)

    extrema = extract_critical_extrema(trajectory)

    export_dataset_to_csv(trajectory, filename="multilinkage_kinematic_data.csv")

    print("=" * 70)
    print(f"MULTI-LINKAGE ANALYTICAL KINEMATIC EXTREMA SUMMARY (Target Node {target_id})")
    print("=" * 70)
    print(f"Loaded File:       {json_path}")
    print(f"Total Nodes:       {len(engine.nodes)}")
    print(f"Total Edges:       {len(engine.edges)}")
    print("-" * 70)
    print(f"Max Velocity:      {extrema['max_v']['vP_mag']:.2f} px/s at Crank θ = {extrema['max_v']['theta1_deg']:.1f}°")
    print(f"Min Velocity:      {extrema['min_v']['vP_mag']:.2f} px/s at Crank θ = {extrema['min_v']['theta1_deg']:.1f}°")
    print(f"Max Acceleration:  {extrema['max_a']['aP_mag']:.2f} px/s² at Crank θ = {extrema['max_a']['theta1_deg']:.1f}°")
    print(f"Min Acceleration:  {extrema['min_a']['aP_mag']:.2f} px/s² at Crank θ = {extrema['min_a']['theta1_deg']:.1f}°")
    print("=" * 70)

    plot_kinematics_with_extrema(engine, trajectory, extrema, target_id)