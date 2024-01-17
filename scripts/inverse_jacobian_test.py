from matplotlib import pyplot as plt
import numpy as np
import math


l1 = 1
l2 = 0.5


def forward(q):
    theta1 = q[0]
    theta2 = q[1]

    R1 = np.array([
        [math.cos(theta1), -math.sin(theta1)],
        [math.sin(theta1), math.sin(theta1)]
    ])
    R2 = np.array([
        [math.cos(theta2), -math.sin(theta2)],
        [math.sin(theta2), math.sin(theta2)]
    ])

    p1_local = l1 * np.array([1, 0], dtype=float).T
    p1_world = R1 @ p1_local

    p2_local = l2 * np.array([1, 0], dtype=float).T

    p2_world = R2 @ p2_local + p1_world

    return np.zeros((2, )), p1_world, p2_world

def jacobians(q):
    theta1 = q[0]
    theta2 = q[1]
    JR1 = np.array([
        [-math.sin(theta1), -math.cos(theta1)],
        [math.cos(theta1), -math.sin(theta1)]
    ])
    JR2 = np.array([
        [-math.sin(theta2), -math.cos(theta2)],
        [math.cos(theta2), -math.sin(theta2)]
    ])
    p1_local = l1 * np.array([1, 0], dtype=float).T

    p2_local = l2 * np.array([1, 0], dtype=float).T

    Jp1_world = p1_local.T @ JR1
    Jp2_world = p2_local.T @ JR2

    return np.zeros((2, )), Jp1_world, Jp2_world

def plot_robot(q, color=(1, 0, 0)):
    joint_positions = forward(q)

    # print(joint_positions)
    xs = [x[0] for x in joint_positions]
    ys = [y[1] for y in joint_positions]

    plt.plot(xs, ys, color=color)
    for x,y in zip(xs,ys):
        plt.scatter(x,y, color=color)

    plt.ylim([-np.pi/2, np.pi/2])
    plt.xlim([-np.pi/2, np.pi/2])
    # plt.show()
    
def get_color(num):
    # 000, 001, 010, 011, 100, 101, 110

    col = np.array([num, ((num * 100) % 10)/10.0, (num * 10) % 1 / 1.0])
    col /= np.max(col)
    return col

def main():
    # plot some config
    np.random.seed(42)
    start_q = np.random.rand(2) * np.pi
    
    plot_robot(start_q)
    

    jacs = jacobians(start_q)
    jacs = jacs[1:3]
    jac_t = np.array([jacs[0], jacs[1]])
    print("jac_t:", jac_t)
    pinv_jac_t = np.linalg.pinv(jac_t)

    tmp_q = start_q
    current_pos = forward(tmp_q)[-1]
    target_pos = current_pos * 0.9

    print("current pos:", current_pos, " target pos:", target_pos)
    # color = np.array([1, 0, 0])
    num_loops = 10
    for i in range(num_loops):
        current_pos = forward(tmp_q)[-1]

        jacs = jacobians(tmp_q)
        jacs = jacs[1:3]
        jac_t = np.array([jacs[0], jacs[1]])
        pinv_jac_t = np.linalg.pinv(jac_t)

        # make position update
        delta_pos = target_pos - current_pos
        delta_q = pinv_jac_t @ delta_pos

        print("delta_q:", delta_q[0], delta_q[1])
        new_joint_pos = tmp_q + delta_q
        color = get_color((i+1)/num_loops)
        print("color:", color)
        plot_robot(new_joint_pos, color=color)
        
        tmp_q = new_joint_pos
        # if np.linalg.norm(delta_q) < 0.1:
        #     break

    plt.scatter(target_pos[0], target_pos[1], s=100)

    plt.show()  


if __name__ == "__main__":
    main()
    
