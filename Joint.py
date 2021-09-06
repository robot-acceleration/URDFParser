# import numpy as np
import numpy as np
import sympy as sp
from .SpatialAlgebra import Origin, Translation, Rotation

class Joint:
    def __init__(self, name, jid, parent, child):
        self.name = name         # name
        self.jid = jid           # temporary ID (replaced by standard DFS parse ordering)
        self.urdf_jid = jid      # URDF ordered ID
        self.bfs_jid = jid       # temporary ID (replaced by BFS parse ordering)
        self.bfs_level = 0       # temporary level (replaced by BFS parse ordering)
        self.origin = Origin()   # Fixed origin location
        self.jtype = None        # type of joint
        self.parent = parent     # parent link name
        self.child = child       # child link name
        self.theta = sp.symbols("theta") # Free joint variable
        self.Xmat_sp = None      # Sympy X matrix placeholder
        self.Xmat_sp_free = None # Sympy X_free matrix placeholder
        self.Smat_sp = None      # Sympy S matrix placeholder (usually a vector)
        self.damping = 0         # damping placeholder

    def set_id(self, id_in):
        self.jid = id_in

    def set_parent(self, parent_name):
        self.parent = parent_name

    def set_child(self, child_name):
        self.child = child_name

    def set_bfs_id(self, id_in):
        self.bfs_id = id_in

    def set_bfs_level(self, level_in):
        self.bfs_level = level_in

    def set_origin_xyz(self, x, y = None, z = None):
        self.origin.set_translation(x,y,z)

    def set_origin_rpy(self, r, p = None, y = None):
        self.origin.set_rotation(r,p,y)

    def set_damping(self, damping):
        self.damping = damping

    def set_transformation_matrix(self, matrix_in):
        self.Xmat_sp = matrix_in

    def set_type(self, jtype, axis = None):
        self.jtype = jtype
        self.origin.build_fixed_transform()
        if self.jtype == 'revolute':
            if axis[2] == 1:
                self.Xmat_sp_free = self.origin.rotation.rot(self.origin.rotation.rz(self.theta))
                self.S = np.array([0,0,1,0,0,0])
            elif axis[1] == 1:
                self.Xmat_sp_free = self.origin.rotation.rot(self.origin.rotation.ry(self.theta))
                self.S = np.array([0,1,0,0,0,0])
            elif axis[0] == 1:
                self.Xmat_sp_free = self.origin.rotation.rot(self.origin.rotation.rx(self.theta))
                self.S = np.array([1,0,0,0,0,0])
        elif self.jtype == 'prismatic':
            if axis[2] == 1:
                self.Xmat_sp_free = self.origin.translation.xlt(self.origin.translation.skew(0,0,self.theta))
                self.S = np.array([0,0,0,0,0,1])
            elif axis[1] == 1:
                self.Xmat_sp_free = self.origin.translation.xlt(self.origin.translation.skew(0,self.theta,0))
                self.S = np.array([0,0,0,0,1,0])
            elif axis[0] == 1:
                self.Xmat_sp_free = self.origin.translation.xlt(self.origin.translation.skew(self.theta,0,0))
                self.S = np.array([0,0,0,1,0,0])
        elif self.jtype == 'fixed':
            self.Xmat_sp_free = sp.eye(6)
            self.S = np.array([0,0,0,0,0,0])
        else:
            print('Only revolute and fixed joints currently supported!')
            exit()
        self.Xmat_sp = self.Xmat_sp_free * self.origin.Xmat_sp_fixed
        # remove numerical noise (e.g., URDF's often specify angles as 3.14 or 3.14159 but that isn't exactly PI)
        self.Xmat_sp = sp.nsimplify(self.Xmat_sp, tolerance=1e-6, rational=True).evalf()

    def get_transformation_matrix_function(self):
        return sp.utilities.lambdify(self.theta, self.Xmat_sp, 'numpy')

    def get_transformation_matrix(self):
        return self.Xmat_sp

    def get_joint_subspace(self):
        return self.S

    def get_damping(self):
        return self.damping

    def get_name(self):
        return self.name

    def get_id(self):
        return self.jid

    def get_bfs_id(self):
        return self.bfs_id

    def get_bfs_level(self):
        return self.bfs_level

    def get_parent(self):
        return self.parent

    def get_child(self):
        return self.child
