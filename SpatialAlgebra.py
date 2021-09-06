import sympy as sp
import copy

class Translation:
    def __init__(self, x, y = None, z = None):
        if y == None: # passed in as tuple
            self.y = x[1]
            self.z = x[2]
            self.x = x[0]
        else:
            self.x = x
            self.y = y
            self.z = z
        self.rx = self.skew(self.x,self.y,self.z)
        self.Xmat_sp_fixed = self.xlt(self.rx)
        self.tx_hom = sp.Matrix([[1,0,0,self.x],[0,1,0,self.y],[0,0,1,self.z],[0,0,0,1]])
        self.tx_hom_inv = sp.Matrix([[1,0,0,-self.x],[0,1,0,-self.y],[0,0,1,-self.z],[0,0,0,1]])

    def skew(self, x, y, z):
        return sp.Matrix([[0,-z,y],[z,0,-x],[-y,x,0]])

    def xlt(self, rx):
        col1 = sp.Matrix.vstack(sp.eye(3), -rx)
        col2 = sp.Matrix.vstack(sp.zeros(3, 3), sp.eye(3))
        return sp.Matrix.hstack(col1, col2)

class Rotation:
    def __init__(self, r, p = None, y = None):
        if p == None: # passed in as tuple
            self.p = r[1]
            self.y = r[2]
            self.r = r[0]
        else:
            self.r = r
            self.p = p
            self.y = y

        roll_mat = self.rx(self.r)
        pitch_mat = self.ry(self.p)
        yaw_mat = self.rz(self.y)
        self.E = roll_mat * pitch_mat * yaw_mat
        self.Xmat_sp_fixed = self.rot(self.E)
        self.E_hom = sp.Matrix.vstack(copy.deepcopy(self.E),sp.Matrix([[0,0,0]]))
        self.E_hom = sp.Matrix.hstack(self.E_hom,sp.Matrix([[0],[0],[0],[1]]))
        self.E_hom_inv = self.E_hom.transpose()

    def rx(self, theta):
        c = sp.cos(theta)
        s = sp.sin(theta)
        E = sp.Matrix([[1, 0, 0], [0, c, s], [0, -s, c]])
        return E

    def ry(self, theta):
        c = sp.cos(theta)
        s = sp.sin(theta)
        E = sp.Matrix([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        return E

    def rz(self, theta):
        c = sp.cos(theta)
        s = sp.sin(theta)
        E = sp.Matrix([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        return E

    def rot(self, E):
        z = sp.zeros(3, 3)
        col1 = sp.Matrix.vstack(E, z)
        col2 = sp.Matrix.vstack(z, E)
        return sp.Matrix.hstack(col1, col2)

class Origin:
    def __init__(self):
        self.translation = None
        self.rotation = None
        self.Xmat_sp_fixed = None
        self.Xmat_sp_fixed_hom = None

    def set_translation(self, x, y = None, z = None):
        self.translation = Translation(x,y,z)

    def set_rotation(self, r, p = None, y = None):
        self.rotation = Rotation(r,p,y)

    def build_fixed_transform(self):
        if self.translation is None or self.rotation is None:
            print("[!Error] First set the origin translation and rotation!")
        else:
            self.Xmat_sp_fixed =  self.rotation.Xmat_sp_fixed * self.translation.Xmat_sp_fixed
            self.Xmat_sp_fixed_hom = self.rotation.E_hom * self.translation.tx_hom
            self.Xmat_sp_fixed_hom_inv = self.rotation.E_hom_inv * self.translation.tx_hom_inv

