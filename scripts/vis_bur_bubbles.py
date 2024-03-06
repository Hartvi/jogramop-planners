import numpy as np
from matplotlib import pyplot as plt
import math

def get_rot(theta):
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return rot

class Shape:
    def __init__(self, position: np.ndarray):
        self.position = position
    
    def plot(self, ax, figsize, color):
        ...


class Circle(Shape):
    def __init__(self, position: np.ndarray, radius: float):
        super().__init__(position)
        self.radius: float = radius

    def plot(self, ax: plt.axes, figsize, **kwargs):
        figmin = min(figsize[0], figsize[1])
        figavg = 0.5*(figsize[0] + figsize[1])
        figmax = max(figsize[0], figsize[1])
        points_whole_ax = 72*(figavg/figmax)    # 1 point = dpi / 72 pixels
        points_radius = 2 * self.radius / 1.0 * points_whole_ax
        ax.scatter(self.position[0], self.position[1], marker="o", s=points_radius**2, **kwargs)


class LineRobot:
    def __init__(self, segment_lengths: np.ndarray) -> None:
        self.num_segments: int = len(segment_lengths)
        self.segment_lengths: np.ndarray = np.concatenate([[0], segment_lengths])
        self.positions: np.ndarray = np.zeros((2, self.segment_lengths.size))
        self.causality_mask: np.ndarray = np.zeros((len(segment_lengths), len(segment_lengths)))
        for i in range(1, len(segment_lengths)):
            self.causality_mask[i-1, i] = 1.0
    
    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        return (n, 2) positions
        """
        # print("q:",len(q)," segments: ", len(self.segment_lengths))
        assert len(q) == (len(self.segment_lengths) - 1), "q: " + str(len(q)) + " segments: " + str(len(self.segment_lengths))
        final_positions: np.ndarray = np.concatenate([[np.cumsum(self.segment_lengths)], [np.zeros(len(self.segment_lengths))]], axis=0).T
        
        # n = len(positions) = len(q) + 1
        # i = n-1:1
        for i in range(len(final_positions) - 1, 0, -1):
            # shift left to center, rotate, then shift back
            for k in range(i, len(final_positions)):
                # R(i)*(p(i) - p(i-1)) + p(i-1)
                final_positions[k] = get_rot(q[i - 1]) @ (final_positions[k] - final_positions[i - 1]) + final_positions[i - 1]
        return final_positions
    
    def set_config(self, q: np.ndarray):
        self.positions = self.forward(q)

    def __str__(self) -> str:
        return str(self.segment_lengths)
    
    def plot(self, q: np.ndarray, ax: plt.axes, color=(0.0, 0.7, 0.0)):
        poss = self.forward(q)
        ax.scatter(poss[:, 0], poss[:, 1], color=color, s=5)
        ax.plot(poss[:, 0], poss[:, 1], color=color)
    
    def scatter_config(self, q, ax: plt.axes, color=(0.7, 0.0, 0)):
        ax.scatter(q[0], q[1], color=color, s=15)
    
    def dist(self, q1, q2):
        return np.max(np.linalg.norm(self.forward(q1) - self.forward(q2), axis=1))
    
    def scatter_distance_configs(self, q: np.ndarray, max_dist, ax: plt.axes, color) -> list[list[float]]:
        epsilon_q = 0.05
        rotation_resolution = 7
        self.scatter_config(q, ax, color)
        tmp_configs = list()
        for i in range(rotation_resolution):
            # while np.dot(np.array([1,0]), rand_dir) < -0.3:
                # rand_dir = np.array([math.cos(np.random.random() * 2 * np.pi), math.sin(np.random.random() * 2 * np.pi)])

            # slight change ==> PI
            q_r = np.random.random((q.size, )) * 2 - 1
            q_r /= np.linalg.norm(q_r)
            tmp_q = self.extend_in_direction(q, q_r, max_dist, epsilon_q)
            
            tmp_configs.append(tmp_q)

            self.scatter_config(tmp_q, ax, color)

        for i in range(len(tmp_configs)):
            ax.plot([q[0], tmp_configs[i][0]], [q[1], tmp_configs[i][1]], color=color)
        return tmp_configs

    def scatter_distance_configs_extended(self, q: np.ndarray, min_idx: int, max_dists: np.ndarray, ax: plt.axes, color) -> list[list[float]]:
        epsilon_q = 0.05
        rotation_resolution = 7
        self.scatter_config(q, ax, color)
        tmp_configs = list()
        extra_configs = list()
        for i in range(rotation_resolution):
            # slight change ==> PI
            q_r = np.random.random((q.size, )) * 2 - 1
            q_r /= np.linalg.norm(q_r)
            
            tmp_q = self.extend_in_direction(q, q_r, max_dists[min_idx], epsilon_q)
            self.scatter_config(tmp_q, ax, color)
            tmp_configs.append(tmp_q)

            for k in range(min_idx+1, q.size):
                # print("closest dists:", max_dists, "idx:", min_idx, "mask:", self.causality_mask[k-1], "q_r:", q_r, "tmp_q:",tmp_q)
                q_r *= self.causality_mask[k-1]
                q_r /= np.linalg.norm(q_r)
                # print("dist before:", self.dist(tmp_q, q), "max dist:", max_dists[k])
                tmp_q = self.extend_in_direction(tmp_q, q_r, max_dists[k] - max_dists[k-1], epsilon_q)
                # print("dist after:", self.dist(tmp_q, q))
                # print("closest dists:", max_dists, "idx:", min_idx, "mask:", self.causality_mask[k-1], "q_r:", q_r, "tmp_q:",tmp_q)

            extra_configs.append(tmp_q)
            self.scatter_config(tmp_q, ax, color)

        for i in range(len(tmp_configs)):
            ax.plot([q[0], tmp_configs[i][0]], [q[1], tmp_configs[i][1]], color=color)
        for i in range(len(tmp_configs)):
            ax.plot([tmp_configs[i][0], extra_configs[i][0]], [tmp_configs[i][1], extra_configs[i][1]], color=(0,0,0))
        return extra_configs
        # return tmp_configs


    def extend_in_direction(self, q, q_r, max_dist, epsilon_q=0.05):
        tmp_q = q
        # print("q:", q, " qr:",q_r)
        for k in range(1, int(2 * np.pi / epsilon_q)):
            new_q = q + k * epsilon_q * q_r
            # print("new_q:", new_q)
            
            dist = self.dist(q, new_q)
            if dist > max_dist:
                break
            tmp_q = new_q
        return tmp_q

    def closest_distance(self, q: np.ndarray, shape: Shape):
        self.set_config(q)
        if isinstance(shape, Circle):
            min_dist = float("inf")
            for i in range(1, len(self.positions)):
                b = self.positions[i] - self.positions[i-1]
                
                c = shape.position - self.positions[i-1]
                c_on_b = np.dot(c, b) / np.dot(b, b) * b
                # now to check if c is actually in the segment
                tmp_dist = 0
                if np.linalg.norm(c_on_b) < np.linalg.norm(b) and np.dot(b, c) > 0:
                    c_perp_b = c - c_on_b
                    tmp_dist = np.linalg.norm(c_perp_b) - shape.radius
                else:
                    tmp_dist = min(np.linalg.norm(c), np.linalg.norm(c - b)) - shape.radius
                if tmp_dist < min_dist:
                    min_dist = tmp_dist
            return min_dist
                
        else:
            raise NotImplementedError(f"Shape {type(shape)} not implemented")

    def closest_distances(self, q: np.ndarray, shape: Shape) -> tuple[int, np.ndarray]:
        self.set_config(q)
        dists: np.ndarray = np.zeros((len(self.segment_lengths)-1, ))
        min_idx: int = -1
        if isinstance(shape, Circle):
            min_dist = float("inf")
            for i in range(1, len(self.positions)):
                b = self.positions[i] - self.positions[i-1]
                
                c = shape.position - self.positions[i-1]
                c_on_b = np.dot(c, b) / np.dot(b, b) * b
                # now to check if c is actually in the segment
                tmp_dist = 0
                if np.linalg.norm(c_on_b) < np.linalg.norm(b) and np.dot(b, c) > 0:
                    c_perp_b = c - c_on_b
                    tmp_dist = np.linalg.norm(c_perp_b) - shape.radius
                else:
                    tmp_dist = min(np.linalg.norm(c), np.linalg.norm(c - b)) - shape.radius
                if tmp_dist < min_dist:
                    min_dist = tmp_dist
                    min_idx = i-1
                dists[i-1] = tmp_dist
            return min_idx, dists
        else:
            raise NotImplementedError(f"Shape {type(shape)} not implemented")


class Env:
    def __init__(self, robot: LineRobot, shapes: list[Shape]) -> None:
        self.robot: LineRobot = robot
        self.shapes: list[Shape] = shapes

        self.figwidth = 10
        cols = 2
        rows = 1
        self.figsize = (self.figwidth/rows, self.figwidth/cols)
        fig, axs = plt.subplots(figsize=self.figsize, nrows=rows, ncols=cols)
        fig.set_figheight(8)
        fig.set_figwidth(18)
        self.fig = fig
        self.ax1 = axs[0]
        self.ax2 = axs[1]

        ax1 = self.ax1
        ax2 = self.ax2

        ax1.set_title("Workspace")
        ax2.set_title("Configuration space")
        ax1.set_xlabel("y [m]")
        ax1.set_ylabel("x [m]")
        ax2.set_xlabel("theta1 [rad]")
        ax2.set_ylabel("theta2 [rad]")

        ax1.set_xlim([-4, 4])
        ax1.set_ylim([-4, 4])

        ax2.set_xlim([-3.2, 3.14])
        ax2.set_ylim([-3.2, 3.14])
    
    def closest_distance(self, q: np.ndarray) -> float:
        ...
        min_dist = float('inf')
        for shape in self.shapes:
            tmp_dist = self.robot.closest_distance(q, shape)
            if tmp_dist < min_dist:
                min_dist = tmp_dist
        return min_dist

    def closest_distances(self, q: np.ndarray) -> tuple[int, np.ndarray]:
        min_dist = float('inf')
        distances = np.ones((q.size, )) * float("inf")
        min_idx = -1
        for k, shape in enumerate(self.shapes):
            # closest distance of robot segments i to obstacle k 
            tmp_min_idx, dists = self.robot.closest_distances(q, shape)
            tmp_dist = dists[tmp_min_idx]
            if tmp_dist < min_dist:
                min_dist = tmp_dist
                min_idx = tmp_min_idx
            
            # for each segment i, get the smallest distance
            for i in range(len(dists)):
                if dists[i] < distances[i]:
                    distances[i] = dists[i]
        return min_idx, distances
    
    def add_shape(self, shape: Shape):
        assert isinstance(shape, Shape)
        self.shapes.append(shape)
    
    def set_robot(self, robot: LineRobot):
        assert isinstance(robot, LineRobot)
        self.robot = robot
    
    def __str__(self)->str:
        return "robot: " + str(self.robot) + " shapes: " + str(self.shapes)
    
    def plot_config(self, q: np.ndarray, color=(0,0,0)):
        self.robot.plot(q, self.ax1, color)
    
    def plot_bur(self, q: np.ndarray, color):
        max_dist = env.closest_distance(q)
        endpoints = self.robot.scatter_distance_configs(q, max_dist, self.ax2, color)

        self.plot_config(q)
        for i in range(len(endpoints)):
            self.plot_config(endpoints[i], color)
    
    def plot_extended_bur(self, q: np.ndarray, color):
        min_idx, max_dists = env.closest_distances(q)
        endpoints = self.robot.scatter_distance_configs_extended(q, min_idx, max_dists, self.ax2, color)

        self.plot_config(q)
        for i in range(len(endpoints)):
            self.plot_config(endpoints[i], color)
    
    def plot_obstacles(self, color=(1,0,0)):
        for i in range(len(self.shapes)):
            self.shapes[i].plot(self.ax1, self.figsize, color=color)
        line_size = 100
        for i in range(line_size):
            for k in range(line_size):
                q = np.array([2.0*i/line_size - 1, 2.0*k/line_size - 1])*np.pi
                d = self.closest_distance(q)
                if d <= 0:
                    self.robot.scatter_config(q, self.ax2)
    

if __name__ == "__main__":
    plot_thing = 2
    robot = LineRobot([0.5, 1])
    # test_config = [1.57/2, 1.57/2]
    test_config = [0, -1.57/2]

    circle: Circle = Circle(np.array([1, 0.5]), 0.3)
    circle2: Circle = Circle(np.array([-1, 1.5]), 0.5)
    circle3: Circle = Circle(np.array([-0.5, -0.5]), 0.3)
    circle4: Circle = Circle(np.array([2, 2]), 0.2)
    env: Env = Env(robot, [circle2, circle3, circle4])

    env.plot_obstacles()
    
    np.random.seed(54)
    if plot_thing == 0:
        env.plot_bur(np.array([-1, 1.5]), (0.8, 0, 0.8))
        env.plot_bur(np.array([-1.65, 1.5]), (1, 0, 0))
        env.plot_bur(np.array([-2.30, 1.5]), (1, 0.5, 0))
        env.plot_bur(np.array([-2.55, 1.5]), (0.8, 0.8, 0))
        env.plot_bur(np.array([-0.35, 1.5]), (0, 0, 1))
    elif plot_thing == 1:
        num_burs = 15
        for i in range(0, num_burs):
            rand_config = (2 * np.pi * np.random.random((len(test_config),)) - np.pi)*0.9
            # rand_config = (2 * np.pi * np.array([i/num_burs, 0.75]) - np.pi)*0.9
            th = 2 * np.pi * i / num_burs
            # rand_config = np.array([th, 7*th]) % (2*np.pi) - np.pi
            rand_color = (np.random.random(), np.random.random(), np.random.random())
            env.plot_bur(rand_config, (0.5, rand_color[0], rand_color[1]))
    elif plot_thing == 2:
        num_burs = 2
        rand_config = np.array([-0.67, -1.792])
        rand_color = (np.random.random(), np.random.random(), np.random.random())

        env.plot_extended_bur(rand_config, (0.5, rand_color[0], rand_color[1]))

        for i in range(0, num_burs):
            rand_config = (2 * np.pi * np.random.random((len(test_config),)) - np.pi)*0.9
            # rand_config = (2 * np.pi * np.array([i/num_burs, 0.75]) - np.pi)*0.9
            th = 2 * np.pi * i / num_burs
            # rand_config = np.array([th, 7*th]) % (2*np.pi) - np.pi
            rand_color = (np.random.random(), np.random.random(), np.random.random())

            # env.plot_bur(rand_config, (0.375, 0.75*rand_color[0], 0.75*rand_color[1]))
            env.plot_extended_bur(rand_config, (0.5, rand_color[0], rand_color[1]))


    plt.show()

