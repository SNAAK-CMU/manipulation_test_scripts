import numpy as np
import argparse

from frankapy import FrankaArm
from frankapy import utils
from frankapy import FrankaConstants as FC
import keyboard

if __name__ == "__main__":
    fa = FrankaArm()

    '''
    Side view
        
    ---   -   -   ---
    |||   |   |   |||
    -----------------
    |		|
    |  Box 1	|
    -----------------
     
    Top view 

                           Arm
        
    ------------------
    |    Box 2       |
    |----------------|
    |  |          |  |
    |  |          |  |
    |B |----------| B|
    |o     Box 3    o|
    |x |----------| x|
    |  |          |  |
    |7 |          | 6|
    |  |----------|  |
    |      Box 4     |
    |  |----------|  |
    |  |          |  |
    |  |          |  |
    |----------------|
    |      Box 5     |
    ------------------
        
        '''
    fa.run_guide_mode(duration=600, block=False)
    WORKSPACE_WALLS = np.array([
        # sides
        [0.25, 0.55, 0.5, 0, 0, 0, 1.1, 0.01, 1.1],
        [0.25, -0.55, 0.5, 0, 0, 0, 1.1, 0.01, 1.1],
        # back
        [-0.41, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1], # keep the same for now
        # front
        [0.77, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
        # top
        [0.25, 0, 1, 0, 0, 0, 1.2, 1, 0.01],
        # bottom
        [0.25, 0, -0.05, 0, 0, 0, 1.2, 1, 0.01],
        
        # sandwich assembly area
        [0.5, 0.25, 0.125, 0, 0, 0, 0.68, 0.695, 0.26],
        
        # right bin area

        # Box 1 
        [0.43, -0.3615, 0.0, 0, 0, 0, 0.68, 0.375, 0.001], # Assuming roughly 6 inch deep bins
        
        # Box 2
        [0.14, -0.3615, 0.125, 0, 0, 0, 0.08, 0.375, 0.26],

        # Box 3 
        [0.344, -0.3615, 0.125, 0, 0, 0, 0.05, 0.375, 0.26],

        # Box 4
        [0.542, -0.3615, 0.125, 0, 0, 0, 0.05, 0.375, 0.26],

        # Box 5
        [0.75, -0.3615, 0.125, 0, 0, 0, 0.08, 0.375, 0.26],
        
        # Box 6
        [0.43, -0.215, 0.125, 0, 0, 0, 0.68, 0.07, 0.26],
        
        # Box 7
        [0.43, -0.52, 0.125, 0, 0, 0, 0.68, 0.07, 0.26]
    ])
    while True:
        print(fa.is_joints_in_collision_with_boxes(boxes=WORKSPACE_WALLS))
