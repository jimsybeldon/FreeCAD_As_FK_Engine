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
    Solves complex multi-loop planar linkage networks via dyad decomposition.
    """
    def __init__(self, json_path, omega1=10.0):
        with open(json_path, 'r') as f:
            self.data = json.load(f)

        self.omega1 = omega1
        self.nodes = {n["id"]: n for n in self.data["nodes"]}
        self.edges = self.data["edges"]

        # Identify motor edge
        self.motor_edge = next(e for e in self.edges if e.get("isMotor", False))
        n0_id, n2_id = self.motor_edge["nodeIds"]

        # Ensure n0 is the ground node
        if not self.nodes[n0_id]["isGround"]:
            n0_id, n2_id = n2_id, n0_id

        self.crank_ground_id = n0_id
        self.crank_node_id = n2_id

        # Calculate initial link lengths (rigid constraints)
        self.link_lengths = {}
        for edge in self.edges:
            i, j = edge["nodeIds"]
            p1 = np.array([self.nodes[i]["x"], self.nodes[i]["y"]])
            p2 = np.array([self.nodes[j]["x"], self.nodes[j]["y"]])
            dist = np.linalg.norm(p1 - p2)
            self.link_lengths[(min(i, j), max(i, j))] = dist

        # Pre-calculate base crank radius and initial angle offset
        p0 = np.array([self.nodes[self.crank_ground_id]["x"], self.nodes[self.crank_ground_id]["y"]])
        p2_0 = np.array([self.nodes[self.crank_node_id]["x"], self.nodes[self.crank_node_id]["y"]])
        self.crank_r = np.linalg.norm(p2_0 - p0)
        self.initial_crank_angle = math.atan2(p2_0[1] - p0[1], p2_0[0] - p0[0])

    def get_length(self, i, j):
        return self.link_lengths[(min(i, j), max(i, j))]

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

        # Two geometric branch solutions
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

        # Select branch closest to previous continuous state
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
        Sequentially resolves all 11 nodes for a given crank rotation step.
        """
        positions = {}

        # 1. Static Ground Nodes
        for n_id, n in self.nodes.items():
            if n["isGround"]:
                positions[n_id] = np.array([n["x"], n["y"]], dtype=float)

        # 2. Node 2: Motor Crank Path
        theta = math.radians(theta_deg) + self.initial_crank_angle
        p0 = positions[0]
        positions[2] = np.array([
            p0[0] + self.crank_r * math.cos(theta),
            p0[1] + self.crank_r * math.sin(theta)
        ])

        # 3. Node 3: Grounded at Node 1, connected to Node 2
        prev_3 = prev_positions[3] if prev_positions else np.array([self.nodes[3]["x"], self.nodes[3]["y"]])
        positions[3] = self.solve_dyad(
            positions[2], positions[1],
            self.get_length(2, 3), self.get_length(3, 1),
            prev_P3=prev_3
        )
        if positions[3] is None:
            return None

        # 4. Node 4: Rigid Triangle (2-3-4)
        orig2 = np.array([self.nodes[2]["x"], self.nodes[2]["y"]])
        orig3 = np.array([self.nodes[3]["x"], self.nodes[3]["y"]])
        orig4 = np.array([self.nodes[4]["x"], self.nodes[4]["y"]])
        positions[4] = self.solve_triad(positions[2], positions[3], orig2, orig3, orig4)

        # 5. Node 6: Dyad from Ground Node 5 and Node 4
        prev_6 = prev_positions[6] if prev_positions else np.array([self.nodes[6]["x"], self.nodes[6]["y"]])
        positions[6] = self.solve_dyad(
            positions[5], positions[4],
            self.get_length(5, 6), self.get_length(4, 6),
            prev_P3=prev_6
        )
        if positions[6] is None:
            return None

        # 6. Node 7: Dyad from Node 6 and Node 4
        prev_7 = prev_positions[7] if prev_positions else np.array([self.nodes[7]["x"], self.nodes[7]["y"]])
        positions[7] = self.solve_dyad(
            positions[6], positions[4],
            self.get_length(6, 7), self.get_length(4, 7),
            prev_P3=prev_7
        )
        if positions[7] is None:
            return None

        # 7. Node 8: Dyad from Node 2 and Node 7
        prev_8 = prev_positions[8] if prev_positions else np.array([self.nodes[8]["x"], self.nodes[8]["y"]])
        positions[8] = self.solve_dyad(
            positions[2], positions[7],
            self.get_length(2, 8), self.get_length(7, 8),
            prev_P3=prev_8
        )
        if positions[8] is None:
            return None

        # 8. Node 9: Rigid Triangle (4-8-9)
        orig8 = np.array([self.nodes[8]["x"], self.nodes[8]["y"]])
        orig9 = np.array([self.nodes[9]["x"], self.nodes[9]["y"]])
        positions[9] = self.solve_triad(positions[4], positions[8], orig4, orig8, orig9)

        # 9. Node 10: Rigid Triangle (7-8-10)
        orig7 = np.array([self.nodes[7]["x"], self.nodes[7]["y"]])
        orig10 = np.array([self.nodes[10]["x"], self.nodes[10]["y"]])
        positions[10] = self.solve_triad(positions[7], positions[8], orig7, orig8, orig10)

        return positions


def animate_multilinkage(engine, trajectory_data):
    fig, ax = plt.subplots(figsize=(12, 9))

    # Pre-extract paths for all dynamic nodes
    node_ids = list(engine.nodes.keys())
    paths = {n_id: [frame[n_id] for frame in trajectory_data] for n_id in node_ids}

    # Plot coupler paths for dynamic floating nodes
    for n_id in [2, 3, 4, 6, 7, 8, 9, 10]:
        px = [p[0] for p in paths[n_id]]
        py = [p[1] for p in paths[n_id]]
        color = '#ff8c00' if n_id == 4 else '#1e88e5'
        lw = 2.0 if n_id in [4, 9] else 1.2
        ax.plot(px, py, color=color, linewidth=lw, alpha=0.85)

    # Setup graphical artists for linkage edges
    link_lines = []
    for edge in engine.edges:
        line, = ax.plot([], [], color='black', linewidth=1.8, zorder=3)
        link_lines.append((edge["nodeIds"], line))

    # Setup node markers
    ground_x = [engine.nodes[n]["x"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    ground_y = [engine.nodes[n]["y"] for n in engine.nodes if engine.nodes[n]["isGround"]]
    ax.scatter(ground_x, ground_y, color='#d32f2f', s=140, zorder=6, label='Ground Pivots (0, 1, 5)')

    node_dots = ax.scatter([], [], color='#1565c0', s=80, zorder=7)
    target_dot = ax.scatter([], [], color='#ff8c00', s=120, zorder=8, label='Target Node 4')

    # Add text labels next to nodes
    node_texts = {}
    for n_id in node_ids:
        node_texts[n_id] = ax.text(0, 0, f" {n_id}", fontsize=10, fontweight='bold', zorder=9)

    title_text = ax.set_title('', fontsize=12)

    ax.set_xlabel('X Coordinate (px)')
    ax.set_ylabel('Y Coordinate (px)')
    ax.invert_yaxis()  # Matched to standard screen/image coordinate system (Y-down)
    ax.axis('equal')
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='lower right')

    def update(frame_idx):
        frame = trajectory_data[frame_idx]

        # Update link line positions
        for (i, j), line in link_lines:
            p1, p2 = frame[i], frame[j]
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

        # Update node markers & text locations
        dynamic_pts = np.array([frame[n_id] for n_id in node_ids if not engine.nodes[n_id]["isGround"]])
        node_dots.set_offsets(dynamic_pts)
        target_dot.set_offsets([frame[4]])

        for n_id in node_ids:
            node_texts[n_id].set_position((frame[n_id][0], frame[n_id][1]))

        title_text.set_text(f'Multilink Kinematic Engine | Crank Angle = {frame_idx}°')

        artists = [line for _, line in link_lines] + [node_dots, target_dot, title_text]
        artists.extend(node_texts.values())
        return artists

    anim = animation.FuncAnimation(
        fig, update, frames=len(trajectory_data), interval=20, blit=False, repeat=True
    )

    plt.show()


if __name__ == "__main__":
    json_filename = "mechanism2.json"

    engine = MultiLinkageKinematicEngine(json_filename, omega1=10.0)

    # Solve full 360-degree crank cycle
    trajectory = []
    prev_frame = None
    for deg in range(360):
        frame_sol = engine.solve_kinematics_step(deg, prev_positions=prev_frame)
        if frame_sol is not None:
            trajectory.append(frame_sol)
            prev_frame = frame_sol

    animate_multilinkage(engine, trajectory)