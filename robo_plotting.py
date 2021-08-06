import matplotlib as mpl
import pandas as pd
import matplotlib.pyplot as plt
from math import sin, cos, pi
import numpy as np
import matplotlib.animation as animation
from portmark import generate_coords, carton
mpl.rcParams['legend.fontsize'] = 10


def translation_matrix(offset, axis, rads, counter=False):
    """Create a translation matrix given an adxis, and rotation"""
    x, y, z, _ = offset
    if counter:
        rads *= -1

    if  axis == 'z':
        return np.array([
            [cos(rads), -sin(rads), 0, x],
            [sin(rads), cos(rads), 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    elif axis == 'x':
        return np.array([
            [1, 0, 0, x],
            [0, cos(rads), -sin(rads), y],
            [0, sin(rads), cos(rads), z],
            [0, 0, 0, 1]
        ])
    elif axis == 'y':
        return np.array([
            [cos(rads), 0, sin(rads), x],
            [0, 1, 0, y],
            [-sin(rads), 0, cos(rads), z],
            [0, 0, 0, 1]
        ])


def get_joint_data(df, dfi):
    """given a dataframe, and index decipher joint position information. Note this isn't the nicest way to find fwd kinematics"""


    # setup
    base_axis, base_counter = 'z', False
    shoulder_axis, shoulder_counter = 'y', True
    elbow_axis, elbow_counter = 'y', True
    w1_axis, w1_counter = 'y', True
    w2_axis, w2_counter = 'z', True
    w3_axis, w3_counter = 'y', True

    # Represent the robot sections
    origin = [0, 0, 0, 1] # x, y, z, _
    base = [0, 0, 0.038, 1]
    shoulder_vec_a = [0, 0, 0.0893, 1]
    shoulder_vec_b = [0, -0.086, 0, 1]
    elbow_vec_a = [0, -0.0303, 0, 1]
    elbow_vec_b = [-0.612, 0, 0, 1]
    w1_vec_a = [0, 0.006859, 0, 1]
    w1_vec_b = [-0.5723, 0, 0, 1]
    w2_vec_a = [0, -0.0545, 0, 1]
    w2_vec_b = [0, 0, -0.0617, 1]
    w3_vec_a = [0, 0, -0.054, 1]
    w3_vec_b = [0, -0.06141, 0, 1]

    # Represents the print head extension
    flange_vec_a = [-0.12, 0, -0.038, 1]  # altered to tcp
    flange_vec_b = [0, -0.18, 0, 1]  # altered to tcp

    # Represents the printer head surface
    print_vec_a = [0.05, 0, -0.025, 1]
    print_vec_b = [-0.05, 0, 0.025, 1]

    # Joints
    base_rotation, shoulder_rotation, elbow_rotation, w1_rotation, w2_rotation, w3_rotation = df.iloc[dfi]

    # Shoulder
    tx = translation_matrix(base, base_axis, base_rotation, counter=base_counter)
    shoulder_a = tx @ shoulder_vec_a
    tx = tx @ translation_matrix(shoulder_vec_a, base_axis, 0, counter=base_counter)
    shoulder_b = tx @ shoulder_vec_b

    # Elbow
    tx = tx @ translation_matrix(shoulder_vec_b, shoulder_axis, shoulder_rotation, counter=shoulder_counter)
    elbow_a = tx @ elbow_vec_a
    tx = tx @ translation_matrix(elbow_vec_a, shoulder_axis, 0, counter=shoulder_counter)
    elbow_b = tx @ elbow_vec_b

    # Wrist 1
    tx = tx @ translation_matrix(elbow_vec_b, elbow_axis, elbow_rotation, counter=elbow_counter)
    w1_a = tx @ w1_vec_a
    tx = tx @ translation_matrix(w1_vec_a, elbow_axis, 0, counter=elbow_counter)
    w1_b = tx @ w1_vec_b

    # Wrist 2
    tx = tx @ translation_matrix(w1_vec_b, w1_axis, w1_rotation, counter=w1_counter)
    w2_a = tx @ w2_vec_a
    tx = tx @ translation_matrix(w2_vec_a, w1_axis, 0, counter=w1_counter)
    w2_b = tx @ w2_vec_b

    # Wrist 3
    tx = tx @ translation_matrix(w2_vec_b, w2_axis, w2_rotation, counter=w2_counter)
    w3_a = tx @ w3_vec_a
    tx = tx @ translation_matrix(w3_vec_a, w2_axis, 0, counter=w2_counter)
    w3_b = tx @ w3_vec_b

    # Printer extension
    tx = tx @ translation_matrix(w3_vec_b, w3_axis, w3_rotation, counter=w3_counter)
    flange_a = tx @ flange_vec_a
    tx = tx @ translation_matrix(flange_vec_a, w3_axis, 0, counter=w3_counter)
    flange_b = tx @ flange_vec_b

    # Printer surface
    tx = tx @ translation_matrix(flange_vec_b, 'z', 0)
    print_a = tx @ print_vec_a
    print_b = tx @ print_vec_b

    # join x y z
    plots = [
        origin, base,
        shoulder_a, shoulder_b,
        elbow_a, elbow_b,
        w1_a, w1_b,
        w2_a, w2_b,
        w3_a, w3_b,
        flange_a, flange_b,
        print_a, print_b
    ]

    # Where to represent the joints with a special scatter plot
    pivots = [
        True, False,  # origin, base
        True, False,  # shoulder
        True, False,  # elbow
        True, False,  # w1
        True, False,  # w2
        True, False,  # w3
        False, False,  # print extension
        False, False  # print surface
    ]

    joint_x = [p[0] for p in plots]
    joint_y = [p[1] for p in plots]
    joint_z = [p[2] for p in plots]

    return np.array(joint_x), np.array(joint_y), np.array(joint_z), pivots

def get_pos_data(df, dfi):
    # setup
    x, y, z, _,_,_,_ = df.iloc[dfi]
    return x, y, z

def get_speed_data(df, dfi):
    # setup
    vx, vy, vz, wx, wy, wz = df.iloc[dfi]
    return vx, vy, vz, wx, wy, wz

def get_path_print_data(df):
    path_x, path_y, path_z = df.x, df.y, df.z
    print_x, print_y, print_z = path_x[df.print], path_y[df.print],path_z[df.print]
    return np.array((path_x, path_y, path_z)), np.array((print_x, print_y, print_z))


if __name__ == "__main__":
    pause = True
    data = pd.read_csv("data.csv")

    # joint
    joint_df = data[["q1", "q2", "q3", "q4", "q5", "q6"]]
    joint_x, joint_y, joint_z = np.array([]), np.array([]), np.array([])

    # Where printing at
    path_df = data[["x", "y", "z", "rx", "ry", "rz", "print"]]
    path_xyz, print_xyz = get_path_print_data(path_df)
    path_x, path_y, path_z = path_xyz
    print_x, print_y, print_z = print_xyz

    speed_df = data[["vx", "vy", "vz", "wx", "wy", "wz"]]
    vx, vy, vz, wx, wy, wz = 0, 0, 0, 0, 0, 0

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.view_init(elev=-155, azim=-116)
    ax.plot(path_x, path_y, path_z, 'y--', label="path")
    ax.plot(print_x, print_y, print_z, 'r.', label="prints")

    # Coords stack coords
    def getpcs(cartonpos):
        rx = translation_matrix([0, 0, 0, 1], 'x', -pi/2)
        rz = translation_matrix([0, 0, 0, 1], 'z', pi*7/4)
        tx = translation_matrix([-0.521, -0.541, 1.215, 1], 'x', 0)
        xx, yy, zz, one = tx @ rz @ rx @ np.array(cartonpos + [1])
        return xx, yy, zz

    # for side in ["A", "B"]:
    #     xp1, xp2, xp3, yp, zp, _ = zip(*generate_coords(carton, side=side, perfect=True))
    #     kwargs = {"marker": "*", "color": "m" if side == "A" else "g"}
    #     #pc_x1 = [ax.scatter(xx, yy, zz, **kwargs) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp1, yp, zp)]]
    #     #pc_x2 = [ax.scatter(xx, yy, zz, **kwargs) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp2, yp, zp)]]
    #     #pc_x3 = [ax.scatter(xx, yy, zz, **kwargs) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp3, yp, zp)]]
    #     pc_x1 = [(xx, yy, zz) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp1, yp, zp)]]
    #     pc_x2 = [(xx, yy, zz) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp2, yp, zp)]]
    #     pc_x3 = [(xx, yy, zz) for xx, yy, zz in [getpcs([x, y, z]) for x, y, z in zip(xp3, yp, zp)]]
    #
    #     x1, y1, z1 = zip(*pc_x1)
    #     x2, y2, z2 = zip(*pc_x2)
    #     x3, y3, z3 = zip(*pc_x3)
    #
    #     ax.scatter(x1+x2+x3, y1+y2+y3, z1+z2+z3, **kwargs, label=f"Side {side}\n areas")

    line, = ax.plot([0], [0], [0], 'b')
    scat, = ax.plot([0], [0], [0], 'xb')

    def onClick(event):
        if event.dblclick:
            global pause
            pause ^= True
            print("PAUSE" if pause else "PLAY")

    def data_gen():
        cnt = 0
        while cnt < len(joint_df):
            yield get_joint_data(joint_df, cnt), get_speed_data(speed_df, cnt), get_pos_data(path_df, cnt)
            if not pause:
                cnt += 1

    def init():
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        line.set_data(joint_x, joint_y)
        line.set_3d_properties(joint_z)
        return line,

    def run(data):
        # update the data
        (joint_x, joint_y, joint_z, isjoint), (vx, vy, vz, wx, wy, wz), (x, y, z) = data
        line.set_data(joint_x, joint_y)
        line.set_3d_properties(joint_z)

        ax.set(title=f"vx: {round(vx,2)}, vy: {round(vy, 2)}, vz: {round(vz, 2)}")
        # ax.set(title=f"x: {round(x,3)}, y: {round(y, 3)}, z: {round(z, 3)} \n" +
        #              f"vx: {round(vx,2)}, vy: {round(vy, 2)}, vz: {round(vz, 2)}")

        # get the points which are joints
        joint_x, joint_y, joint_z = np.array(list((zip(*[(x, y, z) for x, y, z, t in zip(joint_x, joint_y, joint_z, isjoint) if t]))))
        scat.set_data(joint_x, joint_y)
        scat.set_3d_properties(joint_z)
        return line,

    ax.legend(bbox_to_anchor=(1.04,1), loc="upper left")
    fig.canvas.mpl_connect('button_press_event', onClick)
    ani = animation.FuncAnimation(fig, run, data_gen, interval=1, blit=False, init_func=init)
    fig.show()