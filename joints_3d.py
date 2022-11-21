import numpy as np
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from utils import (
    get_z_axis,
    get_x_axis,
    get_y_axis,
    mat3d_to_homogeneous,
    get_translation_matrix,
    get_rot_matrix,
    get_translation_from_matrix,
    Vec3,
    project_vector_onto_rotational_plane,
)


class Joint:
    def __init__(
        self,
        r=0,
        alpha=0,
        d=0,
        theta=0,
        color="red",
        joint_color="blue",
    ):
        self.rot_angle = 0

        self.r = r
        self.alpha = alpha
        self.d = d
        self.theta = theta

        self.parent = None

        self.frame = self.get_frame_from_dh_params(
            self.r, self.alpha, self.d, self.theta
        )
        self.pose = self.frame

        self.color = color
        self.joint_color = joint_color

    @property
    def z_axis(self):
        pass

    @property
    def y_axis(self):
        pass

    @property
    def z_axis(self):
        pass

    @property
    def length(self):
        return Vec3(self.frame[0][3], self.frame[1][3], self.frame[2][3]).magnitude()

    @property
    def position(self):
        v = Vec3(self.pose[0][3], self.pose[1][3], self.pose[2][3])
        return v

    def rotate(self, angle):
        self.rot_angle = angle

    def set_parent(self, j):
        self.parent = j

    @property
    def x_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([1, 0, 0]))).norm()

    @property
    def y_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([0, 1, 0]))).norm()

    @property
    def z_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([0, 0, 1]))).norm()

    @property
    def orientation(self):
        if self.parent:
            return (self.position - self.parent.position).norm()
        else:
            return Vec3(0, 0, 1)

    def project_vector_onto_self_rotational_plane(self, vec):
        """https://www.youtube.com/watch?v=NA0lC3wucG0"""
        u1 = self.x_axis
        u2 = self.y_axis
        a = (vec * u1) / (u1.magnitude() ** 2)
        b = (vec * u2) / (u2.magnitude() ** 2)
        return a * u1 + b * u2, math.atan2(b, a)

    def get_rot_angle_to(self, vec):
        """https://www.youtube.com/watch?v=NA0lC3wucG0"""
        u1 = self.x_axis
        u2 = self.y_axis
        a = (vec * u1) / (u1.magnitude() ** 2)
        b = (vec * u2) / (u2.magnitude() ** 2)
        return math.atan2(b, a)

    def project_vector_onto_self_transformation(self, vec):
        """https://reader.elsevier.com/reader/sd/pii/S1000936121002910?token=3CC7C9EAB41E474AFB63C60EC4AEEDE6C27EC025F52A5D6CAEC0E21C58CE019BA8127930DF9C9E4A420EC5B3566A909B&originRegion=eu-west-1&originCreation=20221120160212"""
        P = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1],
            ]
        )
        return Vec3.from_numpy(
            np.linalg.inv(self.pose)
            @ P
            @ self.pose
            @ np.array([vec.x, vec.y, vec.z, 1])
        )

    def get_frame_from_dh_params(self, r, alpha, d, theta):
        c = math.cos
        s = math.sin
        """
        T_d = [
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,parent_d],
            [0,0,0,1],
        ]
        R_z = [
            [c(theta_p),-s(theta_p),0,0],
            [s(theta_p),c(theta_p),0,0],
            [0,0,1,0],
            [0,0,0,1],
        ]        
        T_x = [
            [1,0,0,r],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1],
        ]
        R_x = [
            [1,0,0,0],
            [0,c(alpha),-s(alpha),0],
            [0,s(alpha),c(alpha),0],
            [0,0,0,1],
        ]
        Z_i = np.array(T_d) @ np.array(R_z)
        X_i = np.array(T_x) @ np.array(R_x)
        return Z_i @ X_i
        """
        T = np.array(
            [
                [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), r * c(theta)],
                [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), r * s(theta)],
                [0, s(alpha), c(alpha), d],
                [0, 0, 0, 1],
            ]
        )
        return T

    def forward_kinematics_from_parent(self):
        self.frame = self.get_frame_from_dh_params(
            self.r, self.alpha, self.d, self.theta + self.rot_angle
        )
        if self.parent:
            self.pose = self.parent.pose @ self.frame
        else:
            self.pose = self.frame


class JointChain:
    def __init__(self, joints=[]):
        self._joints = joints
        self.register_parents()

    def register_parents(self):
        base = None
        for idx, j in enumerate(self._joints):
            j.set_parent(base)
            base = j

    def propagate_joints(self):
        for idx, j in enumerate(self._joints):
            j.forward_kinematics_from_parent()

    def get_joints(self):
        return self._joints

    def perform_fabrik(self, target):
        # First propagate joints to get current positions
        self.propagate_joints()

        # Inverse propagate
        p_target = target
        back_vecs_p_prev = []
        back_poses_p_prev = []
        back_vecs = []
        back_poses = []
        rots = []
        p_prev_position = self._joints[-1].position
        for i in reversed(range(len(self._joints))):
            theta = 0 #Ancestor rotation
            joint_parent_pose = self._joints[0].pose
            joint_parent_position = self._joints[0].position
            for j in range(i-1): #Every previous joint from root except for joint concerned
                joint = self._joints[j+1]

                # Get backwards rot vec for parent
                back_rot_vec = p_target - joint_parent_position
                back_rot_vec, theta = project_vector_onto_rotational_plane(
                    back_rot_vec, joint_parent_pose
                )

                joint_frame = joint.get_frame_from_dh_params(
                    joint.r, joint.alpha, joint.d, joint.theta+theta
                )
                joint_parent_pose = joint_parent_pose @ joint_frame
                joint_parent_position = get_translation_from_matrix(joint_parent_pose)
            #Now we have a joint parent_position aka p_prev
            joint = self._joints[i]
            # Get backwards rot vec for parent
            back_rot_vec = p_target - joint_parent_position
            back_rot_vec, theta = project_vector_onto_rotational_plane(
                back_rot_vec, joint_parent_pose
            )
            # Offset based on joint.d
            offset_vec = get_z_axis(joint_parent_pose) * joint.d
            # Get forward pos from rot and offset
            back_pos = p_target - back_rot_vec.norm() * joint.r
            back_pos -= offset_vec

            back_vec = p_target - back_pos
            back_vecs.append(back_vec)
            back_poses.append(back_pos)

            p_target = back_pos
        
        # Forwards propagate
        theta = 0 #Ancestor rotation
        joint_parent_pose = self._joints[0].pose
        joint_parent_position = self._joints[0].position            
        forward_vecs = []        
        forward_poses = [joint_parent_position]
        for i, p_target_forward in enumerate(reversed([target] + back_poses[:-1])):
            break
            joint = self._joints[i + 1]
            # Get forward rot vec
            forward_rot_vec = p_target_forward - joint_parent_position
            forward_rot_vec, theta = project_vector_onto_rotational_plane(
                forward_rot_vec, joint_parent_pose
            )

            joint_frame = joint.get_frame_from_dh_params(
                joint.r, joint.alpha, joint.d, joint.theta+theta
            )
            joint_parent_pose = joint_parent_pose @ joint_frame
            joint_parent_position = get_translation_from_matrix(joint_parent_pose)

            #forward_vec = (p_target_forward - joint_parent_position).norm()
            forward_vecs.append(forward_rot_vec)
            forward_poses.append(joint_parent_position)
            rots.append(theta)
            
        for i, vec in enumerate(rots):
            joint = self._joints[i + 1]
            joint.rotate(rots[i])

        return (
            back_poses_p_prev,
            back_vecs_p_prev,
            back_vecs,
            back_poses,
            forward_vecs,
            forward_poses,
        )
