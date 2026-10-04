from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Path

from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_multiply,
    quaternion_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    translation_from_matrix,
)
import numpy as np


def pose_to_matrix(pose: Pose) -> np.ndarray:
    # do not use outside of this file
    quat_np = np.array(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    pose_np = np.array([pose.position.x, pose.position.y, pose.position.z]).reshape(
        3, 1
    )
    matrix = quaternion_matrix(quat_np)
    matrix[:3, 3] = pose_np.flatten()
    return matrix


def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    # do not use outside of this file
    quat_np = np.array(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
    )
    pose_np = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
    ).reshape(3, 1)
    matrix = quaternion_matrix(quat_np)
    matrix[:3, 3] = pose_np.flatten()
    return matrix


def transform_pose(pose: Pose, transform: TransformStamped) -> Pose:
    # transforms the pose by the transform
    transformed_pose = Pose()
    pose_ = pose_to_matrix(pose)
    transform_ = transform_to_matrix(transform)
    transformed_matrix = np.matmul(transform_, pose_)
    qx, qy, qz, qw = quaternion_from_matrix(transformed_matrix)
    x, y, z = translation_from_matrix(transformed_matrix)
    transformed_pose.position.x = x
    transformed_pose.position.y = y
    transformed_pose.position.z = z
    transformed_pose.orientation.x = qx
    transformed_pose.orientation.y = qy
    transformed_pose.orientation.z = qz
    transformed_pose.orientation.w = qw
    return transformed_pose


def transform_pose_from_transform_matrix(pose: Pose, transform_) -> Pose:
    # transforms the pose by the transform matrix
    transformed_pose = Pose()
    pose_ = pose_to_matrix(pose)
    transformed_matrix = np.matmul(transform_, pose_)
    qx, qy, qz, qw = quaternion_from_matrix(transformed_matrix)
    x, y, z = translation_from_matrix(transformed_matrix)
    transformed_pose.position.x = x
    transformed_pose.position.y = y
    transformed_pose.position.z = z
    transformed_pose.orientation.x = qx
    transformed_pose.orientation.y = qy
    transformed_pose.orientation.z = qz
    transformed_pose.orientation.w = qw
    return transformed_pose


def transform_path(path: Path, transform: TransformStamped) -> Path:
    # transforms the path by the transform
    path_ = Path()
    path_.header = path.header
    transform_ = transform_to_matrix(transform)
    for i in range(0, len(path.poses)):
        ps = PoseStamped()
        ps.header = path_.header
        ps.pose = transform_pose_from_transform_matrix(
            path.poses[i].pose, transform_)

        path_.poses.append(ps)

    return path_


def move_point_along_line(point, angle, distance):
    new_x = point[0] + distance * np.cos(angle)
    new_y = point[1] + distance * np.sin(angle)
    return new_x, new_y


if __name__ == "__main__":
    # for test purposes
    P1 = Pose()
    P1.position.x = 0.0
    P1.position.y = 0.0
    P1.position.z = 0.0

    P = Pose()
    P.position.x = 1.0
    P.position.y = 2.0
    P.position.z = 3.0

    P2 = Pose()
    P2.position.x = -1.0
    P2.position.y = -2.0
    P2.position.z = -3.0

    P_ = pose_to_matrix(P)
    P1_ = pose_to_matrix(P1)
    P2_ = pose_to_matrix(P2)

    ppp = np.matmul(P_, P1_)
    ppp = np.matmul(ppp, P2_)
    print(ppp)
