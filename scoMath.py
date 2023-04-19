import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import SimpleITK as sitk

from pyquaternion import Quaternion



class CScoVec3 :
    def __init__(self, x = 0.0, y = 0.0, z = 0.0) :
        npVec = np.array([x, y, z])
        self.m_npVec = npVec.reshape(1, 3)

    def clone(self) : 
        vecClone = CScoVec3()
        vecClone.m_npVec = np.array(self.m_npVec)
        return vecClone
    def clone_from(self, vec3) :
        self.m_npVec = np.array(vec3.m_npVec)
    def length(self) :
        return np.linalg.norm(self.m_npVec)
    def length_square(self) :
        return self.X * self.X + self.Y * self.Y + self.Z * self.Z
    def dot(self, v) :
        return np.dot(self.m_npVec[0], v.m_npVec[0])
    def cross(self, v) :
        npCross = np.cross(self.m_npVec[0], v.m_npVec[0])
        return CScoVec3(npCross[0], npCross[1], npCross[2])
    def normalize(self) :
        fLen = self.length()
        fFactor = 1.0 / fLen
        
        fX = self.X * fFactor
        fY = self.Y * fFactor
        fZ = self.Z * fFactor

        return CScoVec3(fX, fY, fZ)
    def add(self, v) :
        npAdd = np.add(self.m_npVec, v.m_npVec)
        return CScoVec3(npAdd[0, 0], npAdd[0, 1], npAdd[0, 2])
    def subtract(self, v) :
        npSub = np.subtract(self.m_npVec, v.m_npVec)
        return CScoVec3(npSub[0, 0], npSub[0, 1], npSub[0, 2])
    
    def print(self) :
        print(f"X:{self.X}, Y:{self.Y}, Z:{self.Z}")
    

    @property
    def X(self) :
        return self.m_npVec[0, 0]
    @X.setter
    def X(self, x : float) :
        self.m_npVec[0, 0] = x
    @property
    def Y(self) :
        return self.m_npVec[0, 1]
    @Y.setter
    def Y(self, y : float) :
        self.m_npVec[0, 1] = y
    @property
    def Z(self) :
        return self.m_npVec[0, 2]
    @Z.setter
    def Z(self, z : float) :
        self.m_npVec[0, 2] = z


class CScoVec4 :
    def __init__(self, x = 0.0, y = 0.0, z = 0.0, w = 0.0) :
        npVec = np.array([x, y, z, w])
        self.m_npVec = npVec.reshape(1, 4)
    
    def clone(self) : 
        vecClone = CScoVec4()
        vecClone.m_npVec = np.array(self.m_npVec)
        return vecClone
    def clone_from(self, vec4) :
        self.m_npVec = np.array(vec4.m_npVec)
    def length(self) : 
        return np.linalg.norm(self.m_npVec)
    def length_square(self) :
        return self.X * self.X + self.Y * self.Y + self.Z * self.Z + self.W * self.W
    def dot(self, v) : 
        return np.dot(self.m_npVec[0], v.m_npVec[0])
    def normalize(self) : 
        fLen = self.length()
        fFactor = 1.0 / fLen
        
        fX = self.X * fFactor
        fY = self.Y * fFactor
        fZ = self.Z * fFactor
        fW = self.W * fFactor

        return CScoVec4(fX, fY, fZ, fW)
    def add(self, v) :
        npAdd = np.add(self.m_npVec, v.m_npVec)
        return CScoVec4(npAdd[0, 0], npAdd[0, 1], npAdd[0, 2], npAdd[0, 3])
    def subtract(self, v) :
        npSub = np.subtract(self.m_npVec, v.m_npVec)
        return CScoVec4(npSub[0, 0], npSub[0, 1], npSub[0, 2], npSub[0, 3])
    
    def print(self) :
        print(f"X:{self.X}, Y:{self.Y}, Z:{self.Z}, W:{self.W}")
    

    @property
    def X(self) :
        return self.m_npVec[0, 0]
    @X.setter
    def X(self, x : float) :
        self.m_npVec[0, 0] = x
    @property
    def Y(self) :
        return self.m_npVec[0, 1]
    @Y.setter
    def Y(self, y : float) :
        self.m_npVec[0, 1] = y
    @property
    def Z(self) :
        return self.m_npVec[0, 2]
    @Z.setter
    def Z(self, z : float) :
        self.m_npVec[0, 2] = z
    @property
    def W(self) :
        return self.m_npVec[0, 3]
    @W.setter
    def W(self, w : float) :
        self.m_npVec[0, 3] = w


class CScoMat4 :
    '''
    4x4 열우선 행렬을 정의한다. 
    '''

    def __init__(self) :
        self.identity()


    def clone(self) :
        matClone = CScoMat4()
        matClone.m_npMat = np.array(self.m_npMat)
        return matClone
    def clone_from(self, mat4) :
        self.m_npMat = np.array(mat4.m_npMat)
    def identity(self) :
        self.m_npMat = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    def inverse(self) :
        npMat = self.m_npMat
        matInv = CScoMat4()
        matInv.m_npMat = np.linalg.inv(npMat)
        return matInv
    def translate_3d(self, x, y, z) :
        self.m_npMat = np.array([
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ])
    def scale(self, x, y, z) :
        self.m_npMat = np.array([
            [x, 0.0, 0.0, 0.0],
            [0.0, y, 0.0, 0.0],
            [0.0, 0.0, z, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    def rot_from_row(self, listRot : tuple) :
        self.m_npMat = np.array([
            [listRot[0], listRot[1], listRot[2], 0.0],
            [listRot[3], listRot[4], listRot[5], 0.0],
            [listRot[6], listRot[7], listRot[8], 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    def rot_from_column(self, listRot : tuple) :
        self.m_npMat = np.array([
            [listRot[0], listRot[3], listRot[6], 0.0],
            [listRot[1], listRot[4], listRot[7], 0.0],
            [listRot[2], listRot[5], listRot[8], 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    def rot_from_axis(self, xAxis : CScoVec3, yAxis : CScoVec3, zAxis : CScoVec3) :
        self.m_npMat = np.array([
            [xAxis.X, yAxis.X, zAxis.X, 0.0],
            [xAxis.Y, yAxis.Y, zAxis.Y, 0.0],
            [xAxis.Z, yAxis.Z, zAxis.Z, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    def rot_from_axis_radian(self, axis : CScoVec3, radian : float) :
        theta = radian / 2.0
        qAxis = axis.m_npVec * np.sin(theta)
        r = R.from_quat([qAxis[0][0], qAxis[0][1], qAxis[0][2], np.cos(theta)])
        self.m_npMat = r.as_matrix()
        self.m_npMat = np.hstack([self.m_npMat, np.array([0.0, 0.0, 0.0]).reshape(3, 1)])
        self.m_npMat = np.vstack([self.m_npMat, np.array([0.0, 0.0, 0.0, 1.0])])
    def set_translate(self, x : float, y : float, z : float) :
        self.m_npMat[0][3] = x
        self.m_npMat[1][3] = y
        self.m_npMat[2][3] = z
    def get_translate(self) :
        return CScoVec3(self.m_npMat[0][3], self.m_npMat[1][3], self.m_npMat[2][3])
    def set_scale(self, x : float, y : float, z : float) :
        self.m_npMat[0][0] = x
        self.m_npMat[1][1] = y
        self.m_npMat[2][2] = z
    def get_scale(self) :
        return CScoVec3(self.m_npMat[0][0], self.m_npMat[1][1], self.m_npMat[2][2])
    def get_axis_radian(self) :
        """
        desc : 행렬의 회전 속성에 대한 회전축과 radian을 리턴한다. 
        return : (axis : CScoVec3, radian : float)
        """
        q = CScoMath.mat4_to_quat(self)
        axis = CScoVec3(q.axis[0], q.axis[1], q.axis[2])
        radian = q.radians
        return (axis, radian)
    def print(self) :
        print(self.m_npMat)


class CScoPlane :
    def __init__(self) :
        self.m_normal = CScoVec3(0, 0, 1)
        self.m_d = 0
        self.m_point = CScoVec3(0, 0, 0)


    def make_with_point(self, v0 : CScoVec3, v1 : CScoVec3, v2 : CScoVec3) :
        u = v1.subtract(v0)
        v = v2.subtract(v0)
        self.m_normal = (u.cross(v)).normalize()
        self.m_d = -v0.dot(self.m_normal)
        self.m_point = v0.clone()
    def dot(self, v : CScoVec3) :
        ret = self.Normal.dot(v) + self.D
        return ret
    # infinity error 조심 
    def get_x(self, y : float, z : float) :
        x = (-self.Normal.Y * y - self.Normal.Z * z - self.D) * 1. / self.Normal.X
        return x
    def get_y(self, x : float, z : float) :
        y = (-self.Normal.X * x - self.Normal.Z * z - self.D) * 1. / self.Normal.Y
        return y
    def get_z(self, x : float, y : float) :
        z = (-self.Normal.X * x - self.Normal.Y * y - self.D) * 1. / self.Normal.Z
        return z

    def print(self) :
        print(f"NX:{self.Normal.X}, NY:{self.Normal.Y}, NZ:{self.Normal.Z}, D:{self.D}")


    @property
    def Normal(self) :
        return self.m_normal
    @Normal.setter
    def Normal(self, normal : CScoVec3) :
        self.m_normal = normal.clone()
    @property
    def D(self) :
        return self.m_d
    @D.setter
    def D(self, d : float) :
        self.m_d = d
    @property
    def Point(self) :
        return self.m_point
    @Point.setter
    def Point(self, pt : CScoVec3) :
        self.m_point = pt.clone()


class CScoRay : 
    def __init__(self) :
        self.m_origin = CScoVec3()
        self.m_dir = CScoVec3()


    def make_with_point(self, startPt : CScoVec3, endPt : CScoVec3, bNormalized = False) :
        self.m_origin = startPt.clone()
        self.m_dir = endPt.subtract(startPt)

        if bNormalized == True :
            self.m_dir = self.m_dir.normalize()
    def get_pos(self, ratio : float) :
        pos = self.m_origin.add(CScoMath.mul_vec3_scalar(self.m_dir, ratio))
        return pos
    
    def print(self) :
        self.m_origin.print()
        self.m_dir.print()

    
    @property
    def Origin(self) :
        return self.m_origin
    @Origin.setter
    def Origin(self, origin : CScoVec3) :
        self.m_origin = origin
    @property
    def Dir(self) :
        return self.m_dir
    @Dir.setter
    def Dir(self, dir : CScoVec3) :
        self.m_dir = dir


class CScoOBB :
    """
    note
        여기에서 정의된 OBB는 z축을 기준으로 원점이 center가 아닌 bottom이다. 
        x,y는 기존처럼 원점이 center이다. 
    """
    def __init__(self) :
        self.m_pos = CScoVec3(0, 0, 0)
        self.m_halfSize = CScoVec3(1, 1, 1)
        self.m_view = CScoVec3(0, 0, 1)
        self.m_up = CScoVec3(0, 1, 0)
        self.m_tangent = CScoVec3(1, 0, 0)


    def make_with_pos_view_up(self, pos : CScoVec3, view : CScoVec3, up : CScoVec3, halfSize : CScoVec3) :
        viewDir = view.subtract(pos)
        self.m_tangent = up.cross(viewDir).normalize()
        self.m_view = viewDir.normalize()
        self.m_up = self.m_view.cross(self.m_tangent).normalize()
        self.m_pos = pos.clone()
        self.m_halfSize = halfSize.clone()
    def make_with_2_point(self, p0 : CScoVec3, p1 : CScoVec3, halfSize : CScoVec3) :
        pos = p1
        dir = p1.subtract(p0)
        view = p1.add(dir)
        up = CScoVec3(0, 1, 0)
        self.make_with_pos_view_up(pos, view, up, halfSize)

    def print(self) :
        print("-"*30)
        self.m_pos.print()
        self.m_halfSize.print()
        self.m_view.print()
        self.m_tangent.print()
        self.m_up.print()
        print("-"*30)

    
    @property
    def Pos(self) :
        return self.m_pos
    @Pos.setter
    def Pos(self, pos : CScoVec3) :
        self.m_pos = pos.clone()
    @property
    def HalfSize(self) :
        return self.m_halfSize
    @HalfSize.setter
    def HalfSize(self, halfSize : CScoVec3) :
        self.m_halfSize = halfSize.clone()
    @property
    def View(self) :
        return self.m_view
    @View.setter
    def View(self, view : CScoVec3) :
        self.m_view = view.clone()
    @property
    def Tangent(self) :
        return self.m_tangent
    @Tangent.setter
    def Tangent(self, tangent : CScoVec3) :
        self.m_tangent = tangent.clone()
    @property
    def Up(self) :
        return self.m_up
    @Up.setter
    def Up(self, up : CScoVec3) :
        self.m_up = up.clone()
    @property
    def WorldMatrix(self) :
        mat4 = CScoMat4()
        mat4.rot_from_axis(self.Tangent, self.Up, self.View)
        mat4.set_translate(self.Pos.X, self.Pos.Y, self.Pos.Z)
        return mat4


class CScoMath :
    def __init__(self) :
        pass


    @staticmethod
    def deg_to_rad(deg : float) :
        return math.radians(deg)
    @staticmethod
    def rad_to_deg(rad : float) :
        return math.degrees(rad)

    # vector & matrix 
    @staticmethod
    def mul_vec3_scalar(v : CScoVec3, s : float) :
        return CScoVec3(v.X * s, v.Y * s, v.Z * s)
    @staticmethod
    def mul_vec4_scalar(v : CScoVec3, s : float) :
        return CScoVec4(v.X * s, v.Y * s, v.Z * s, v.W * s)
    @staticmethod
    def mul_mat4(m0 : CScoMat4, m1 : CScoMat4) :
        retMat = CScoMat4()
        retMat.m_npMat = np.dot(m0.m_npMat, m1.m_npMat)
    @staticmethod
    def mul_mat4_vec3(m : CScoMat4, v : CScoVec3) :
        npVec3 = v.m_npVec.reshape(3, 1)
        npVec4 = np.vstack([npVec3, np.array([1.0])])

        retV = CScoVec4()
        retV.m_npVec = np.dot(m.m_npMat, npVec4).reshape(1, 4)
        return retV
    @staticmethod
    def mul_mat4_vec4(m : CScoMat4, v : CScoVec4) :
        npVec4 = v.m_npVec.reshape(4, 1)

        retV = CScoVec4()
        retV.m_npVec = np.dot(m.m_npMat, npVec4).reshape(1, 4)
        return retV
    
    # quaternion 
    @staticmethod
    def make_quaternion(x, y, z, w) :
        """
        x, y, z : vector 성분
        w       : scalar 성분
        """
        return Quaternion(w=w, x=x, y=y, z=z)
    @staticmethod
    def make_quaternion_with_axis_angle(axis : CScoVec3, radian=0.0) :
        return Quaternion(axis=axis.m_npVec[0], radians=radian)
    @staticmethod
    def quat_add(q0 : Quaternion, q1 : Quaternion) :
        return q0 + q1
    @staticmethod
    def quat_subtract(q0 : Quaternion, q1 : Quaternion) :
        return q0 - q1
    @staticmethod
    def quat_slerp(q0 : Quaternion, q1 : Quaternion, ratio=0.0) :
        return Quaternion.slerp(q0, q1, amount=ratio)
    @staticmethod
    def mul_quat(q0 : Quaternion, q1 : Quaternion) :
        return q0 * q1
    @staticmethod
    def get_quat_axis_radian(q : Quaternion) :
        """
        desc : quaternion의 회전축과 radian을 리턴한다. 
        return : (axis : CScoVec3, radian : float)
        """
        axis = CScoVec3(q.axis[0], q.axis[1], q.axis[2])
        radian = q.radians
        return (axis, radian)
    @staticmethod
    def quat_to_mat4(quat : Quaternion) :
        mat4 = CScoMat4()
        mat4.m_npMat = np.copy(quat.transformation_matrix)
        return mat4
    @staticmethod
    def mat4_to_quat(m : CScoMat4) :
        npMat = m.m_npMat.copy()
        npMat = np.delete(npMat, (3), axis=0)
        npMat = np.delete(npMat, (3), axis=1)
        q = R.from_matrix(npMat).as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    @staticmethod
    def quat_rotation(q : Quaternion, v : CScoVec3) :
        npArr = np.array(q.rotate(v.m_npVec[0]))
        return CScoVec3(npArr[0], npArr[1], npArr[2])
    

    # with SimpleITK
    @staticmethod
    def convert_mat4_from_sitk_versor_rigid3d_transform(transform : sitk.Transform) :
        tr = sitk.VersorRigid3DTransform(transform)

        center = tr.GetCenter()
        translation = tr.GetTranslation()
        rot = tr.GetMatrix()

        matCenter = CScoMat4()
        matTrans = CScoMat4()
        matRot = CScoMat4()

        matCenter.translate_3d(center[0], center[1], center[2])
        matInvCenter = matCenter.inverse()
        matTrans.translate_3d(translation[0], translation[1], translation[2])
        matRot.rot_from_row(rot)

        retMat = CScoMath.mul_mat4(matTrans, matCenter)
        retMat = CScoMath.mul_mat4(retMat, matRot)
        retMat = CScoMath.mul_mat4(retMat, matInvCenter)

        return retMat
    @staticmethod
    def convert_mat4_from_sitk_translate_transform(transform : sitk.Transform) :
        tr = sitk.Similarity3DTransform(transform)

        center = tr.GetCenter()
        scale = tr.GetScale()
        translation = tr.GetTranslation()
        rot = tr.GetMatrix()

        matCenter = CScoMat4()
        matScale = CScoMat4()
        matTrans = CScoMat4()
        matRot = CScoMat4()

        matCenter.translate_3d(center[0], center[1], center[2])
        matInvCenter = matCenter.inverse()
        matTrans.translate_3d(translation[0], translation[1], translation[2])
        matRot.rot_from_row(rot)
        matScale.scale(scale, scale, scale)

        retMat = CScoMath.mul_mat4(matTrans, matCenter)
        retMat = CScoMath.mul_mat4(retMat, matRot)
        retMat = CScoMath.mul_mat4(retMat, matScale)
        retMat = CScoMath.mul_mat4(retMat, matInvCenter)

        return retMat
    @staticmethod
    def convert_mat4_from_sitk_affine_transform(transform : sitk.Transform) :
        tr = sitk.AffineTransform(transform)

        center = tr.GetCenter()
        translation = tr.GetTranslation()
        rot = tr.GetMatrix()

        matCenter = CScoMat4()
        matTrans = CScoMat4()
        matRot = CScoMat4()

        matCenter.translate_3d(center[0], center[1], center[2])
        matInvCenter = matCenter.inverse()
        matTrans.translate_3d(translation[0], translation[1], translation[2])
        matRot.rot_from_row(rot)

        retMat = CScoMath.mul_mat4(matTrans, matCenter)
        retMat = CScoMath.mul_mat4(retMat, matRot)
        retMat = CScoMath.mul_mat4(retMat, matInvCenter)

        return retMat


    # intersection and collision
    @staticmethod
    def intersect_plane_ray(plane : CScoPlane, ray : CScoRay) :
        """
        desc : plane과 ray의 교차 테스트를 수행
        ret  : (bIntersect : Bool, ratio : float)
                bIntersect : True    교차
                            False   비교차
                ratio      : 교차 시점 
        """
        nd = plane.Normal.dot(ray.Dir)
        if abs(nd) < 0.0001 :
            return (False, 0.0)
        
        w = ray.Origin.subtract(plane.Point)
        ratio = -plane.Normal.dot(w) / nd

        return (True, ratio)
    @staticmethod
    def intersect_obb_vec3(obb : CScoOBB, v : CScoVec3) :
        """
        desc : obb 내부에 v가 존재하는가를 리턴
        ret  : True    내부에 있음
               False   외부에 있다.
        """
        mat4 = CScoMat4()
        mat4.rot_from_axis(obb.Tangent, obb.Up, obb.View)
        mat4.set_translate(obb.Pos.X, obb.Pos.Y, obb.Pos.Z)
        invMat4 = mat4.inverse()

        localV = CScoMath.mul_mat4_vec3(invMat4, v)

        # x, y는 기존과 동일하게 
        if np.abs(localV.X) > obb.HalfSize.X:
            return False
        if np.abs(localV.Y) > obb.HalfSize.Y:
            return False
        # z는 원점이 bottom으로  
        if localV.Z < 0 or localV.Z > obb.HalfSize.Z : 
            return False
            
        return True







