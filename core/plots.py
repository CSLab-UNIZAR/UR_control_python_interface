import matplotlib.pyplot as plt
import numpy as np

def plot_frame(ax, pos, R, scale=0.05):

    x_axis = R @ np.array([1,0,0])
    y_axis = R @ np.array([0,1,0])
    z_axis = R @ np.array([0,0,1])

    ax.plot([pos[0], pos[0] + scale*x_axis[0]],
            [pos[1], pos[1] + scale*x_axis[1]],
            [pos[2], pos[2] + scale*x_axis[2]],
            color='r', linewidth=2)

    ax.plot([pos[0], pos[0] + scale*y_axis[0]],
            [pos[1], pos[1] + scale*y_axis[1]],
            [pos[2], pos[2] + scale*y_axis[2]],
            color='g', linewidth=2)

    ax.plot([pos[0], pos[0] + scale*z_axis[0]],
            [pos[1], pos[1] + scale*z_axis[1]],
            [pos[2], pos[2] + scale*z_axis[2]],
            color='b', linewidth=2)
    
def plot_curve_with_arrow(ax, center, axis, color, radius=0.1):
    steps = 50
    theta = np.linspace(0, np.pi, steps)  # 180°
    
    if axis=='x':
        x = np.full_like(theta, center[0])
        y = center[1] + radius*np.cos(theta)
        z = center[2] + radius*np.sin(theta)
        tangent = np.array([0, y[-1]-y[-2], z[-1]-z[-2]])
    elif axis=='y':
        x = center[0] + radius*np.cos(theta)
        y = np.full_like(theta, center[1])
        z = center[2] + radius*np.sin(theta)
        tangent = np.array([x[-1]-x[-2], 0, z[-1]-z[-2]])
    else: # 'z'
        x = center[0] + radius*np.cos(theta)
        y = center[1] + radius*np.sin(theta)
        z = np.full_like(theta, center[2])
        tangent = np.array([x[-1]-x[-2], y[-1]-y[-2], 0])
    
    ax.plot(x, y, z, color=color, lw=2)

    length = radius*0.5
    tangent_unit = tangent / np.linalg.norm(tangent)
    ax.quiver(x[-1], y[-1], z[-1],
              tangent_unit[0]*length,
              tangent_unit[1]*length,
              tangent_unit[2]*length,
              color=color, arrow_length_ratio=0.5, linewidth=2)
    


def plot_robot_tcp(ax, joint_poses, delta_trans, delta_rot):
    
    eef_pos = joint_poses[-1][:3, 3]
    eef_rot = joint_poses[-1][:3, :3]

    base_pos = np.zeros(3)
    base_rot = np.eye(3)

    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.view_init(elev=10, azim=-75)
    
    # Plot robot
    ax.plot(joint_poses[:, 0, 3], joint_poses[:, 1, 3], joint_poses[:, 2 , 3], '-o', color='gray', lw=2)
    
    plot_frame(ax, base_pos, base_rot)
    plot_frame(ax, eef_pos, eef_rot)

    # Translation arrows
    colors = ['g','r','b']
    for i in range(3):
        if abs(delta_trans[i])>1e-4:
            d = np.zeros(3)
            d[i] = delta_trans[i]*2
            ax.quiver(eef_pos[0], eef_pos[1], eef_pos[2],
                      d[0], d[1], d[2],
                      color=colors[i], arrow_length_ratio=0.3, linewidth=2)
    # Rotation arrows
    if abs(delta_rot[1])>1e-4: plot_curve_with_arrow(ax,[eef_pos[0] - 0.01, eef_pos[1], eef_pos[2]],'x','g',radius=0.1)
    if abs(delta_rot[0])>1e-4: plot_curve_with_arrow(ax,[eef_pos[0], eef_pos[1] + 0.01, eef_pos[2]],'y','r',radius=0.1)
    if abs(delta_rot[2])>1e-4: plot_curve_with_arrow(ax,[eef_pos[0], eef_pos[1], eef_pos[2] - 0.1],'z','b',radius=0.1)


def plot_robot_joint(ax, joint_poses, moving_joint, delta_rot):
    
    T_base = np.eye(4)
    joint_poses = np.insert(joint_poses, 0, T_base, axis=0)

    joint_pos = joint_poses[moving_joint][:3,3]
    joint_rot = joint_poses[moving_joint][:3,:3]

    base_pos = joint_poses[0][:3,3]
    base_rot = joint_poses[0][:3,:3]

    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.view_init(elev=20, azim=30)

    # Robot skeleton
    ax.plot(
        joint_poses[:,0,3],
        joint_poses[:,1,3],
        joint_poses[:,2,3],
        '-o', color='gray', lw=2
    )

    # Frames
    plot_frame(ax, base_pos, base_rot)
    plot_frame(ax, joint_pos, joint_rot)
    ax.scatter(joint_pos[0], joint_pos[1], joint_pos[2], color='magenta', s=80)

    axis = joint_rot @ np.array([0,0,1])  
    axis = axis / np.linalg.norm(axis)

    if abs(axis[0]) < 0.9:
        v = np.array([1,0,0])
    else:
        v = np.array([0,1,0])

    v1 = np.cross(axis, v)
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(axis, v1)

    radius = 0.1
    theta = np.linspace(0, np.pi, 40)

    joint_pos = joint_pos + 0.025 * axis
    circle = np.array([joint_pos + radius*(np.cos(t)*v1 + np.sin(t)*v2) for t in theta])
    ax.plot(circle[:,0], circle[:,1], circle[:,2], color='orange', lw=2)

   