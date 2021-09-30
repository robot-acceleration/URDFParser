import numpy as np
import sympy as sp
from .InertiaSet import InertiaSet
from .SpatialAlgebra import Origin, Translation, Rotation

class Link:
    def __init__(self, name, lid):
        self.name = name        # name
        self.lid = lid          # temporary ID (replaced by standard DFS parse ordering)
        self.urdf_lid = lid     # URDF ordered ID
        self.bfs_lid = lid      # temporary ID (replaced by BFS parse ordering)
        self.bfs_level = 0      # temporary level (replaced by BFS parse ordering)
        self.parent_id = None   # temporary ID (replaced later)
        self.origin = Origin()  # Fixed origin location
        self.mass = None
        self.inertia = None
        self.spatial_ineratia = None

    def set_id(self, id_in):
        self.lid = id_in

    def set_parent_id(self, id_in):
        self.parent_id = id_in    

    def set_bfs_id(self, id_in):
        self.bfs_id = id_in

    def set_bfs_level(self, level_in):
        self.bfs_level = level_in

    def set_subtree(self, subtree_in):
        self.subtree = subtree_in

    def set_origin_xyz(self, x, y = None, z = None):
        self.origin.set_translation(x,y,z)

    def set_origin_rpy(self, r, p = None, y = None):
        self.origin.set_rotation(r,p,y)

    def set_inertia(self, mass, ixx, ixy, ixz, iyy, iyz, izz):
        self.mass = mass
        self.inertia = InertiaSet(ixx,ixy,ixz,iyy,iyz,izz)
        self.build_spatial_inertia()

    def set_spatial_inertia(self, inertia_in):
        self.spatial_ineratia = inertia_in

    def build_spatial_inertia(self):
        if self.inertia is None or self.origin.translation is None:
            print("[!Error] Set origin and inertia first!")
        # I6x6 = I3x3 + mccT   mc    I3x3 = Ixx   Ixy   Ixz    c =   0 -cz  cy
        #         mcT          mI           Ixy   Iyy   Iyz         cz   0 -cx
        #                                   Ixz   Iyz   Izz        -cy  cx  0
        rx = sp.nsimplify(self.origin.translation.rx, tolerance=1e-12, rational=True)
        com_trans = np.reshape(np.array([expr.evalf() for expr in rx]),(3,3))
        
        mc = self.mass*com_trans
        mccT = np.matmul(mc,com_trans.transpose())
        
        topLeft = self.inertia.to_matrix() + mccT
        top = np.hstack((topLeft,mc))
        bottom = np.hstack((mc.transpose(),self.mass*np.eye(3)))
        self.spatial_ineratia = np.vstack((top,bottom)).astype(float)
        # remove numerical noise (e.g., URDF's often specify angles as 3.14 or 3.14159 but that isn't exactly PI)
        self.spatial_ineratia[np.isclose(self.spatial_ineratia, np.zeros((6,6)), 1e-10, 1e-10)] = 0

    def get_spatial_inertia(self):
        return self.spatial_ineratia

    def get_name(self):
        return self.name

    def get_id(self):
        return self.lid

    def get_parent_id(self):
        return self.parent_id

    def get_bfs_id(self):
        return self.bfs_id

    def get_bfs_level(self):
        return self.bfs_level

    def get_subtree(self):
        return self.subtree

    def is_world_base_frame(self):
        if self.mass == 0 and self.inertia.is_zero():
            return True
        else:
            return False
