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
        lower=-math.pi*2,
        upper=math.pi*2,
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

        self.lower = lower
        self.upper = upper

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
    def get_jacobian(self):
        jacobian = np.zeros((3,len(self._joints)-1))
        s_i = self._joints[-1].position
        for j in range(1,len(self._joints)):
            if(j<len(self._joints)):
                v_j = self._joints[j-1].z_axis.norm()
                ds_dtheta = v_j.cross(s_i-self._joints[j-1].position)
                jacobian[0][j-1] = ds_dtheta.x
                jacobian[1][j-1] = ds_dtheta.y
                jacobian[2][j-1] = ds_dtheta.z
            else:
                ds_dtheta = 0
                for k in range(3):
                    jacobian[k][j-1] = ds_dtheta
        return jacobian
    def perform_ik(self, target):
        self.propagate_joints()        
        jacobian = self.get_jacobian()
        e = target - self._joints[-1].position
        if(e.magnitude()>=1.5):
            alpha = 0.01
        elif(e.magnitude()<1.5):
            alpha = 0.001
        elif(e.magnitude()<0.5):
            return
            
        dtheta = alpha * np.linalg.pinv(jacobian) @ e.numpy()
        for i,j in enumerate(self._joints[1:]):
            j.rot_angle = max(j.lower,min(j.upper,j.rot_angle+dtheta[i]))
        
