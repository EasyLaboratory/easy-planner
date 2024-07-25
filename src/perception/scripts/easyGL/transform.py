import numpy as np
import math


def construct_extrinsic(R, t):
    """
       构造变换矩阵 [R | t]。

       参数:
       R -- 3x3 旋转矩阵
       t -- 3x1 平移向量

       返回:
       T -- 4x4 变换矩阵
       """
    # 确保 R 是 3x3 矩阵，t 是长度为 3 的向量
    if R.shape != (3, 3) or t.shape != (3,):
        raise ValueError("旋转矩阵必须是 3x3，平移向量必须是长度为 3 的一维数组。")
    # 创建一个 4x4 的单位矩阵
    T = np.eye(4)

    # 将旋转矩阵 R 和平移向量 t 填充到变换矩阵 T 中
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def construct_inverse_extrinsic(R, t):
    # 构造 [R|t] 矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # 计算 [R|t] 的逆矩阵
    R_inv = R.T
    t_inv = -np.dot(R_inv, t)
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def construct_intrinsic(fov: float, width: int, height: int, is_degree=True):
    """
    @param fov: field of view
    @return:intrinsic matrix
    [[fx,0,cx]
    [0,fy,cy]
    [0,0,1]]
    """
    if is_degree:
        fov = fov * (math.pi / 180)
    fx = width / (2 * math.tan(fov / 2) + 0.1)
    fy = height / (2 * math.tan(fov / 2) + 0.1)
    cx = width / 2
    cy = height / 2
    intrinsic = np.array([[fx, 0, cx][0, fy, cy][0, 0, 1]])
    return intrinsic


def construct_inverse_intrinsic(fov: float, width: int, height: int, is_degree=True):
    if is_degree:
        fov = fov * (math.pi / 180)
    fx = width / (2 * math.tan(fov / 2) + 0.1)
    fy = height / (2 * math.tan(fov / 2) + 0.1)
    cx = width / 2
    cy = height / 2
    k_inv = np.array([[1 / fx, 0, -cx / fx]
                      [0, 1 / fy, -cy / fy]
                      [0, 0, 1]])
    return k_inv


def rotation_x(theta):
    """ 绕x轴旋转的旋转矩阵 """
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, cos_theta, -sin_theta],
        [0, sin_theta, cos_theta]
    ])


def rotation_y(theta):
    """ 绕y轴旋转的旋转矩阵 """
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, 0, sin_theta],
        [0, 1, 0],
        [-sin_theta, 0, cos_theta]
    ])


def rotation_z(theta):
    """ 绕z轴旋转的旋转矩阵 """
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])


def unproject(u, v, inverse_K, inverse_E):
    homo_uv = np.array([u, v, 1])
    camera_xyz = inverse_K @ homo_uv
    homo_camera_xyz = np.array([camera_xyz, 1])
    homo_word_xyz = inverse_E @ homo_camera_xyz
    return homo_word_xyz[0],homo_camera_xyz[1],homo_word_xyz[2]




