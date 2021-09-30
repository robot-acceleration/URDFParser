from bs4 import BeautifulSoup
import numpy as np
import sympy as sp
import copy
from .Robot import Robot
from .Link import Link
from .Joint import Joint

class URDFParser:
    def __init__(self):
        pass
    
    def parse(self, filename, alpha_tie_breaker = False):
        try:
            # parse the file
            urdf_file = open(filename, "r")
            self.soup = BeautifulSoup(urdf_file.read(),"xml").find("robot")
            # set up the robot object
            self.robot = Robot(self.soup["name"])
            # collect links
            self.parse_links()
            # collect joints
            self.parse_joints()
            # remove all fixed joints, renumber links and joints, and build parent and subtree lists
            self.renumber_linksJoints(alpha_tie_breaker)
            # report joint ordering to user
            self.print_joint_order()
            # return the robot object
            return copy.deepcopy(self.robot)
        except:
            return None

    def to_float(self, string_arr):
        try:
            return [float(value) for value in string_arr]
        except:
            return string_arr

    def parse_links(self):
        lid = 0
        for raw_link in self.soup.find_all('link', recursive=False):
            # construct link object
            curr_link = Link(raw_link["name"],lid)
            lid = lid + 1
            # parse origin
            raw_origin = raw_link.find("origin")
            if raw_origin == None:
                print("Link [" + curr_link.name + "] does not have an origin. Assuming this is the fixed world base frame. Else there is an error with your URDF file.")
                curr_link.set_origin_xyz([0, 0, 0])
                curr_link.set_origin_rpy([0, 0, 0])
            else:
                curr_link.set_origin_xyz(self.to_float(raw_origin["xyz"].split(" ")))
                curr_link.set_origin_rpy(self.to_float(raw_origin["rpy"].split(" ")))
            # parse inertial properties
            raw_inertial = raw_link.find("inertial")
            if raw_inertial == None:
                print("Link [" + curr_link.name + "] does not have inertial properties. Assuming this is the fixed world base frame. Else there is an error with your URDF file.")
                curr_link.set_inertia(0, 0, 0, 0, 0, 0, 0)
            else:
                # get mass and inertia values
                raw_inertia = raw_inertial.find("inertia")
                curr_link.set_inertia(float(raw_inertial.find("mass")["value"]), \
                                      float(raw_inertia["ixx"]), \
                                      float(raw_inertia["ixy"]), \
                                      float(raw_inertia["ixz"]), \
                                      float(raw_inertia["iyy"]), \
                                      float(raw_inertia["iyz"]), \
                                      float(raw_inertia["izz"]))
            # store
            self.robot.add_link(copy.deepcopy(curr_link))

    def parse_joints(self):
        jid = 0
        for raw_joint in self.soup.find_all('joint', recursive=False):
            # construct joint object
            curr_joint = Joint(raw_joint["name"], jid, \
                               raw_joint.find("parent")["link"], \
                               raw_joint.find("child")["link"])
            jid += 1
            # get origin position and rotation
            curr_joint.set_origin_xyz(self.to_float(raw_joint.find("origin")["xyz"].split(" ")))
            curr_joint.set_origin_rpy(self.to_float(raw_joint.find("origin")["rpy"].split(" ")))
            # set joint type and axis of motion for joints if applicable
            raw_axis = raw_joint.find("axis")
            if raw_axis is None:
                curr_joint.set_type(raw_joint["type"])
            else:
                curr_joint.set_type(raw_joint["type"],self.to_float(raw_axis["xyz"].split(" ")))
            raw_dynamics = raw_joint.find("dynamics")
            if raw_dynamics is None:
                curr_joint.set_damping(0)
            else:
                curr_joint.set_damping(float(raw_dynamics["damping"]))
            # store
            self.robot.add_joint(copy.deepcopy(curr_joint))

    def remove_fixed_joints(self):
        for curr_joint in self.robot.get_joints_ordered_by_id():
            if curr_joint.jtype == "fixed":
                # updated fixed transforms and parents of grandchild_joints
                # to account for the additional fixed transform
                # X_grandchild = X_granchild * X_child
                for gcjoint in self.robot.get_joints_by_parent_name(curr_joint.child):
                    gcjoint.set_parent(curr_joint.get_parent())
                    gcjoint.set_transformation_matrix(gcjoint.get_transformation_matrix() * curr_joint.get_transformation_matrix())
                # combine inertia tensors of child and parent at parent
                # note:  if X is the transform from A to B the I_B = X^T I_A X
                # note2: inertias in the same from add so I_parent_final = I_parent + X^T I_child X
                child_link = self.robot.get_link_by_name(curr_joint.child)
                parent_link = self.robot.get_link_by_name(curr_joint.parent)
                child_I = child_link.get_spatial_inertia()
                curr_Xmat = np.reshape(np.array(curr_joint.get_transformation_matrix()).astype(float),(6,6))
                transformed_Imat = np.matmul(np.matmul(np.transpose(curr_Xmat),child_I),curr_Xmat)
                parent_link.set_spatial_inertia(parent_link.get_spatial_inertia() + transformed_Imat)
                
                # delete the bypassed fixed joint and link
                self.robot.remove_joint(curr_joint)
                self.robot.remove_link(child_link)

    def build_subtree_lists(self):
        subtree_lid_lists = {}
        # initialize all subtrees to include itself
        for lid in self.robot.get_links_dict_by_id().keys():
            subtree_lid_lists[lid] = [lid]
        # start at the leaves and build up!
        for curr_joint in self.robot.get_joints_ordered_by_id(reverse=True):
            parent_lid = self.robot.get_link_by_name(curr_joint.parent).get_id()
            child_lid = self.robot.get_link_by_name(curr_joint.child).get_id()
            # add the child's subtree list to the parent (includes the child)
            if child_lid in subtree_lid_lists.keys():
                subtree_lid_lists[parent_lid] = list(set(subtree_lid_lists[parent_lid]).union(set(subtree_lid_lists[child_lid])))
        # save to the links
        for link in self.robot.links:
            curr_subtree = subtree_lid_lists[link.get_id()]
            link.set_subtree(copy.deepcopy(curr_subtree))

    def dfs_order_update(self, parent_name, alpha_tie_breaker = False, next_lid = 0, next_jid = 0):
        while True:
            child_joints = self.robot.get_joints_by_parent_name(parent_name)
            parent_id = self.robot.get_link_by_name(parent_name).lid
            if alpha_tie_breaker:
                child_joints.sort(key=lambda joint: joint.name)
            for curr_joint in child_joints:
                # save the new id
                curr_joint.set_id(next_jid)
                # save the next_lid to the child
                child = self.robot.get_link_by_name(curr_joint.child)
                child.set_id(next_lid)
                child.set_parent_id(parent_id)
                # recurse
                next_lid, next_jid = self.dfs_order_update(child.name, alpha_tie_breaker, next_lid + 1, next_jid + 1)
            # return to parent
            return next_lid, next_jid

    def bfs_order(self, root_name):
        # initialize
        next_lid = 0
        next_jid = 0
        next_parent_names = [(root_name,-1)]
        self.robot.get_link_by_name(root_name).set_bfs_id(-1)
        self.robot.get_link_by_name(root_name).set_bfs_level(-1)
        # until there are no parent to parse
        while len(next_parent_names) != 0:
            # get the next parent and save its level
            (parent_name, parent_level) = next_parent_names.pop(0)
            next_level = parent_level + 1
            # then until there are no children to parse (of that parent)
            child_joints = self.robot.get_joints_by_parent_name(parent_name)
            while len(child_joints) != 0:
                # update the current link
                curr_joint = child_joints.pop(0)
                curr_joint.set_bfs_id(next_jid)
                curr_joint.set_bfs_level(next_level)
                # append the child to the list of future possible parents
                curr_child_name = curr_joint.get_child()
                next_parent_names.append((curr_child_name,next_level))
                # update the child
                curr_link = self.robot.get_link_by_name(curr_child_name)
                curr_link.set_bfs_id(next_lid)
                curr_link.set_bfs_level(next_level)
                # update the global lid, jid
                next_lid += 1
                next_jid += 1


    def renumber_linksJoints(self, alpha_tie_breaker = False):
        # remove all fixed joints where applicable (merge links)
        self.remove_fixed_joints()
        # find the root link
        link_names = set([link.name for link in self.robot.get_links_ordered_by_id()])
        links_that_are_children = set([joint.get_child() for joint in self.robot.get_joints_ordered_by_id()])
        root_link_name = list(link_names.difference(links_that_are_children))[0]
        # start renumbering at -1 as the base link is fixed by default
        self.robot.get_link_by_name(root_link_name).set_id(-1)
        # generate the standard dfs ordering of joints/links
        self.dfs_order_update(root_link_name, alpha_tie_breaker)
        # also save a bfs parse ordering and levels of joints/links
        self.bfs_order(root_link_name)
        # build subtree lists
        self.build_subtree_lists()

    def print_joint_order(self):
        print("------------------------------------------")
        print("Assumed Input Joint Configuration Ordering")
        print("------------------------------------------")
        for curr_joint in self.robot.get_joints_ordered_by_id():
            print(curr_joint.get_name())
        print("----------------------------")
        print("Total of n = " + str(self.robot.get_num_pos()) + " joints")
        print("----------------------------")