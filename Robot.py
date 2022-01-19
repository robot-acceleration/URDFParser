from .Link import Link
from .Joint import Joint

class Robot:
    # initialization
    def __init__(self, name):
        self.name = name
        self.links = []
        self.joints = []

    def next_none(self, iterable):
        try:
            return next(iterable)
        except:
            return None

    #################
    #    Setters    #
    #################

    def add_joint(self, joint):
        self.joints.append(joint)

    def add_link(self, link):
        self.links.append(link)

    def remove_joint(self, joint):
        self.joints.remove(joint)

    def remove_link(self, link):
        self.links.remove(link)

    #########################
    #    Generic Getters    #
    #########################

    def get_num_pos(self):
        return self.get_num_joints()

    def get_num_vel(self):
        return self.get_num_joints()

    def get_num_cntrl(self):
        return self.get_num_joints()

    def get_name(self):
        return self.name

    def is_serial_chain(self):
        return all([jid - self.get_parent_id(jid) == 1 for jid in range(self.get_num_joints())])

    def get_parent_id(self, lid):
        return self.get_link_by_id(lid).get_parent_id()

    def get_parent_ids(self, lids):
        return [self.get_parent_id(lid) for lid in lids]

    def get_unique_parent_ids(self, lids):
        return list(set(self.get_parent_ids(lids)))

    def get_parent_id_array(self):
        return [tpl[1] for tpl in sorted([(link.get_id(),link.get_parent_id()) for link in self.links], key=lambda tpl: tpl[0])[1:]]

    def has_repeated_parents(self, jids):
        return len(self.get_parent_ids(jids)) != len(self.get_unique_parent_ids(jids))

    def get_subtree_by_id(self, lid):
        return sorted(self.get_link_by_id(lid).get_subtree())

    def get_total_subtree_count(self):
        return sum([len(self.get_subtree_by_id(lid)) for lid in range(self.get_num_joints())])

    def get_ancestors_by_id(self, jid):
        ancestors = []
        curr_id = jid
        while True:
            curr_id = self.get_parent_id(curr_id)
            if curr_id == -1:
                break
            else:
                ancestors.append(curr_id)
        return ancestors

    def get_total_ancestor_count(self):
        return sum([len(self.get_ancestors_by_id(jid)) for jid in range(self.get_num_joints())])

    def get_is_ancestor_of(self, jid, jid_of):
        return jid in self.get_ancestors_by_id(jid_of)

    def get_is_in_subtree_of(self, jid, jid_of):
        return jid in self.get_subtree_by_id(jid_of)

    def get_max_bfs_level(self):
        return sorted(self.joints, key=lambda joint: joint.bfs_level, reverse = True)[0].bfs_level

    def get_ids_by_bfs_level(self, level):
        return [joint.jid for joint in self.get_joints_by_bfs_level(level)]

    def get_bfs_level_by_id(self, jid):
        return(self.get_joint_by_id(jid).get_bfs_level())

    def get_max_bfs_width(self):
        return max([len(self.get_ids_by_bfs_level(level)) for level in range(self.get_max_bfs_level() + 1)])


    ###############
    #    Joint    #
    ###############

    def get_num_joints(self):
        return len(self.joints)

    def get_joint_by_id(self, jid):
        return self.next_none(filter(lambda fjoint: fjoint.jid == jid, self.joints))

    def get_joint_by_name(self, name):
        return self.next_none(filter(lambda fjoint: fjoint.name == name, self.joints))

    def get_joints_by_bfs_level(self, level):
        return list(filter(lambda fjoint: fjoint.bfs_level == level, self.joints))

    def get_joints_ordered_by_id(self, reverse = False):
        return sorted(self.joints, key=lambda item: item.jid, reverse = reverse)

    def get_joints_ordered_by_name(self, reverse = False):
        return sorted(self.joints, key=lambda item: item.name, reverse = reverse)

    def get_joints_dict_by_id(self):
        return {joint.jid:joint for joint in self.joints}

    def get_joints_dict_by_name(self):
        return {joint.name:joint for joint in self.joints}

    def get_joints_by_parent_name(self, parent_name):
        return list(filter(lambda fjoint: fjoint.parent == parent_name, self.joints))

    def get_joints_by_child_name(self, child_name):
        return list(filter(lambda fjoint: fjoint.child == child_name, self.joints))

    def get_joint_by_parent_child_name(self, parent_name, child_name):
        return self.next_none(filter(lambda fjoint: fjoint.parent == parent_name and fjoint.child == child_name, self.joints))

    def get_damping_by_id(self, jid):
        return self.get_joint_by_id(jid).get_damping()

    ##############
    #    Link    #
    ##############

    def get_num_links(self):
        return len(self.links)

    def get_num_links_effective(self):
        # subtracting base link from total # of links
        return get_num_links() - 1

    def get_link_by_id(self, lid):
        return self.next_none(filter(lambda flink: flink.lid == lid, self.links))

    def get_link_by_name(self, name):
        return self.next_none(filter(lambda flink: flink.name == name, self.links))

    def get_links_by_bfs_level(self, level):
        return list(filter(lambda flink: flink.bfs_level == level, self.links))

    def get_links_ordered_by_id(self, reverse = False):
        return sorted(self.links, key=lambda item: item.lid, reverse = reverse)

    def get_links_ordered_by_name(self, reverse = False):
        return sorted(self.links, key=lambda item: item.name, reverse = reverse)

    def get_links_dict_by_id(self):
        return {link.lid:link for link in self.links}

    def get_links_dict_by_name(self):
        return {link.name:link for link in self.links}

    ##############
    #    XMAT    #
    ##############

    def get_Xmat_by_id(self, jid):
        return self.get_joint_by_id(jid).get_transformation_matrix()

    def get_Xmat_by_name(self, name):
        return self.get_joint_by_name(name).get_transformation_matrix()

    def get_Xmats_by_bfs_level(self, level):
        return [joint.get_transformation_matrix() for joint in self.get_joints_by_bfs_level(level)]

    def get_Xmats_ordered_by_id(self, reverse = False):
        return [joint.get_transformation_matrix() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Xmats_ordered_by_name(self, reverse = False):
        return [joint.get_transformation_matrix() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Xmats_dict_by_id(self):
        return {joint.jid:joint.get_transformation_matrix() for joint in self.joints}

    def get_Xmats_dict_by_name(self):
        return {joint.name:joint.get_transformation_matrix() for joint in self.joints}

    ###################
    #    XMAT_Func    #
    ###################

    def get_Xmat_Func_by_id(self, jid):
        return self.get_joint_by_id(jid).get_transformation_matrix_function()

    def get_Xmat_Func_by_name(self, name):
        return self.get_joint_by_name(name).get_transformation_matrix_function()

    def get_Xmat_Funcs_by_bfs_level(self, level):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_by_bfs_level(level)]

    def get_Xmat_Funcs_ordered_by_id(self, reverse = False):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Xmat_Funcs_ordered_by_name(self, reverse = False):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Xmat_Funcs_dict_by_id(self):
        return {joint.jid:joint.get_transformation_matrix_function() for joint in self.joints}

    def get_Xmat_Funcs_dict_by_name(self):
        return {joint.name:joint.get_transformation_matrix_function() for joint in self.joints}

    ##############
    #    IMAT    #
    ##############

    def get_Imat_by_id(self, lid):
        return self.get_link_by_id(lid).get_spatial_inertia()

    def get_Imat_by_name(self, name):
        return self.get_joint_by_name(name).get_spatial_inertia()

    def get_Imats_by_bfs_level(self, level):
        return [link.get_spatial_inertia() for link in self.get_links_by_bfs_level()]

    def get_Imats_ordered_by_id(self, reverse = False):
        return [link.get_spatial_inertia() for link in self.get_links_ordered_by_id(reverse)]

    def get_Imats_ordered_by_name(self, reverse = False):
        return [link.get_spatial_inertia() for link in self.get_links_ordered_by_name(reverse)]

    def get_Imats_dict_by_id(self):
        return {link.lid:link.get_spatial_inertia() for link in self.links}

    def get_Imats_dict_by_name(self):
        return {link.name:link.get_spatial_inertia() for link in self.links}

    ###############
    #      S      #
    ###############

    def get_S_by_id(self, jid):
        return self.get_joint_by_id(jid).get_joint_subspace()

    def get_S_by_name(self, name):
        return self.get_joint_by_name(name).get_joint_subspace()

    def get_S_by_bfs_level(self, level):
        return [joint.get_joint_subspace() for joint in self.get_joints_by_bfs_level(level)]

    def get_Ss_ordered_by_id(self, reverse = False):
        return [joint.get_joint_subspace() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Ss_ordered_by_name(self, reverse = False):
        return [joint.get_joint_subspace() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Ss_dict_by_id(self):
        return {joint.jid:joint.get_joint_subspace() for joint in self.joints}

    def get_Ss_dict_by_name(self):
        return {joint.name:joint.get_joint_subspace() for joint in self.joints}

    def are_Ss_identical(self,jids):
        return all(all(self.get_S_by_id(jid) == self.get_S_by_id(jids[0])) for jid in jids)