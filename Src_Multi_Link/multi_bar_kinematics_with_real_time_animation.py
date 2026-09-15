import sys
import os

# Filter out FreeCAD's site-packages to prevent hijacking your .venv NumPy
sys.path = [p for p in sys.path if r"FreeCAD 1.1\bin\Lib\site-packages" not in p]

# Now import standard third-party libraries
import json
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class MultiLinkageKinematicEngine:
    """
    Generalized 1-DOF Multilink Kinematic Engine.
    Dynamically analyzes JSON topology to build an automated solve pipeline
    for arbitrary 1-DOF dyad & triad planar mechanisms.
    """
    def __init__(self, json_path, omega1=10.0):
        with open(json_path, 'r') as f:
            self.data = json.load(f)

        self.omega1 = omega1
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
        """
        Dynamically builds the sequence of Dyad and Triad solves by tracking resolved nodes.
        """
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
        """
        Solves circle-circle intersection for dyad constraint given 2 known points (P1, P2)
        and distance radii (r1, r2).
        """
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
        """
        Solves rigid body triangular transformation for a third floating point on a rigid body.
        """
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
        """
        Dynamically resolves all mechanism nodes for a given crank rotation step.
        """
        positions = {}

        # 1. Ground Nodes
        for n_id, n in self.nodes.items():
            if n["isGround"]:
                positions[n_id] = np.array([n["x"], n["y"]], dtype=float)

        # 2. Motor Crank Node
        theta = math.radians(theta_deg) + self.initial_crank_angle
        p0 = positions[self.crank_ground_id]
        positions[self.crank_node_id] = np.array([
            p0[0] + self.crank_r * math.cos(theta),
            p0[1] + self.crank_r * math.sin(theta)
        ])

        # 3. Dynamic Sequence Step Solving
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


def animate_multilinkage(engine, trajectory_data):
    fig, ax = plt.subplots(figsize=(12, 9))

    node_ids = sorted(list(engine.nodes.keys()))
    paths = {n_id: [frame[n_id] for frame in trajectory_data] for n_id in node_ids}

    # Dynamic trace visualization for dynamic nodes
    for n_id in node_ids:
        if not engine.nodes[n_id]["isGround"]:
            px = [p[0] for p in paths[n_id]]
            py = [p[1] for p in paths[n_id]]
            is_target = engine.nodes[n_id].get("isTarget", False)
            color = '#ff8c00' if is_target else '#1e88e5'
            lw = 2.0 if is_target or n_id in [9, 11, 12] else 1.2
            ax.plot(px, py, color=color, linewidth=lw, alpha=0.75)

    link_lines = []
    for edge in engine.edges:
        line, = ax.plot([], [], color='black', linewidth=1.8, zorder=3)
        link_lines.append((edge["nodeIds"], line))

    ground_x = [engine.nodes[n]["x"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    ground_y = [engine.nodes[n]["y"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    ax.scatter(ground_x, ground_y, color='#d32f2f', s=140, zorder=6, label='Ground Pivots')

    node_dots = ax.scatter([], [], color='#1565c0', s=80, zorder=7)
    target_dot = ax.scatter([], [], color='#ff8c00', s=120, zorder=8, label='Target Node')

    node_texts = {n_id: ax.text(0, 0, f" {n_id}", fontsize=10, fontweight='bold', zorder=9) for n_id in node_ids}
    title_text = ax.set_title('', fontsize=12)

    ax.set_xlabel('X Coordinate (px)')
    ax.set_ylabel('Y Coordinate (px)')
    ax.invert_yaxis()
    ax.axis('equal')
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='lower right')

    target_id = next((n_id for n_id in node_ids if engine.nodes[n_id].get("isTarget", False)), 4)

    def update(frame_idx):
        frame = trajectory_data[frame_idx]

        for (u, v), line in link_lines:
            line.set_data([frame[u][0], frame[v][0]], [frame[u][1], frame[v][1]])

        dynamic_pts = np.array([frame[n_id] for n_id in node_ids if not engine.nodes[n_id]["isGround"]])
        node_dots.set_offsets(dynamic_pts)
        target_dot.set_offsets([frame[target_id]])

        for n_id in node_ids:
            node_texts[n_id].set_position((frame[n_id][0], frame[n_id][1]))

        title_text.set_text(f'Dynamic Kinematic Engine | Crank Angle = {frame_idx}°')

        artists = [line for _, line in link_lines] + [node_dots, target_dot, title_text]
        artists.extend(node_texts.values())
        return artists

    anim = animation.FuncAnimation(
        fig, update, frames=len(trajectory_data), interval=20, blit=False, repeat=True
    )

    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_filename = os.path.join(script_dir, "mechanism2.json")

    # Fallback to current working directory if not alongside script
    if not os.path.exists(json_filename):
        json_filename = "mechanism2.json"

    engine = MultiLinkageKinematicEngine(json_filename, omega1=10.0)

    trajectory = []
    prev_frame = None
    for deg in range(360):
        frame_sol = engine.solve_kinematics_step(deg, prev_positions=prev_frame)
        if frame_sol is not None:
            trajectory.append(frame_sol)
            prev_frame = frame_sol

    animate_multilinkage(engine, trajectory)