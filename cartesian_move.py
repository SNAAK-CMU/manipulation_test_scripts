import numpy as np
from autolab_core import RigidTransform
import time

from frankapy import FrankaArm
from frankapy import FrankaConstants as FC


if __name__ == "__main__":
    fa = FrankaArm()
    #fa.reset_joints()
    #print(fa.get_collision_boxes_poses())
    new_pose = RigidTransform()
    #new_pose.translation = np.array([0, 0, 0])
    orig_pose = fa.get_pose()
    # new_pose = orig_pose.copy()
    # new_pose.translation = [ 0.38507747, -0.24862385,  0.46716527]
    # fa.goto_pose(new_pose)
    print(orig_pose.translation)
    print(orig_pose.rotation)
    #new_pose = orig_pose.copy()
    #new_pose.translation -= [0, 0, 0.275]
    #fa.goto_pose(new_pose, joint_impedances=[100, 100, 100, 100, 100, 100, 100])#, use_impedance=False)
    #time.sleep(2)
    #fa.goto_pose(orig_pose, joint_impedances=[100, 100, 100, 100, 100, 100, 100]) # XYZ, rpy
    #print(orig_pose.rotation)
    # using default IK fails when rotated by 90 degrees
    
    # joint 7 can be rotated by 270 degreees in positive direection, but is very limited in negative direction
    # need to ensure that if we rotate, that we rotate in the direction that is within joint limits
    # can use is_joints_reachable(joints), to determone
    #z_rot = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
    
    #new_joints[6] = 0
    #fa.goto_joints(new_joints)

    #new_pose = orig_pose.copy()
    #new_pose.translation -= [0, 0, 0.1]
    #fa.goto_pose(new_pose)
    #time.sleep(3)
    #fa.goto_pose(orig_pose)
    #print(fa.get_pose())
    #fa.reset_joints()
