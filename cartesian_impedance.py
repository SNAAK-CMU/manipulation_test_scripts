import numpy as np
from autolab_core import RigidTransform
import time
from frankapy import FrankaConstants as FC


from frankapy import FrankaArm
if __name__ == "__main__":
    fa = FrankaArm()
    orig_pose = fa.get_pose()
    fa.reset_joints()
    new_pose = orig_pose.copy()
    time.sleep(1)
    new_pose.translation += [0, 0, 0.1]
    fa.goto_pose(new_pose, cartesian_impedances=[3000, 3000, 300, 300, 300, 300], use_impedance=False, block=False)

    while(not fa.is_skill_done()): # looping, and at each iteration detect if arm is in collision with boxes (this uses the frankapy boxes)
        if (fa.is_joints_in_collision_with_boxes()):
            print("In Collision with boxes, cancelling motion")
            fa.stop_skill() # this seems to make the motion break, but it does prevent collision
            break

    time.sleep(1)
    fa.goto_pose(orig_pose, cartesian_impedances=[3000, 3000, 300, 300, 300, 300], use_impedance=False, block=False)
    while(not fa.is_skill_done()):
        if (fa.is_joints_in_collision_with_boxes()):
            print("In Collision with boxes, cancelling motion")
            fa.stop_skill()
            break

    fa.goto_pose(FC.HOME_POSE, cartesian_impedances=FC.DEFAULT_CARTESIAN_IMPEDANCES, use_impedance=False) # reset impedances