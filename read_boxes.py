import numpy as np
from autolab_core import RigidTransform
import time

from frankapy import FrankaArm
from frankapy import FrankaConstants as FC

if __name__ == '__main__':
    fa = FrankaArm()
    fa.reset_joints()
    print(np.array(fa.get_collision_boxes_poses()))