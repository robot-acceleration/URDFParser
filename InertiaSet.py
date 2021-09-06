import numpy as np

class InertiaSet:
    def __init__(self, ixx_in, ixy_in, ixz_in, iyy_in, iyz_in, izz_in):
        self.ixx = ixx_in
        self.ixy = ixy_in
        self.ixz = ixz_in
        self.iyy = iyy_in
        self.iyz = iyz_in
        self.izz = izz_in

    def to_vector(self):
        return np.array([self.ixx,self.ixy,self.ixz,self.iyy,self.iyz,self.izz])

    def to_matrix(self):
        return np.array([[self.ixx,self.ixy,self.ixz],[self.ixy,self.iyy,self.iyz],[self.ixz,self.iyz,self.izz]])

    def is_zero(self):
        if self.ixx == 0 and self.ixy == 0 and self.ixz == 0 and \
                self.iyy == 0 and self.iyz == 0 and self.izz == 0:
            return True
        else:
            return False
