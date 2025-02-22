import numpy as np
import argparse

from frankapy import FrankaArm
from frankapy import utils
from record_trajectory import create_formated_skill_dict
import pickle as pkl
from frankapy import FrankaConstants as FC



# See run_dynamic_joints, may be able to generate trajectory with the different planners, and then record said trajectory
# TODO tomorrow with Abhi, we can discuss, do IK?

# for picking up ingredients, something like the run_dynamic_pose could be useful, we could generate the trajectory with some
# planner and then execute, may be able to constrain x and y with it

# DO NOT RUN BEFORE DEBUGGING, causing suddent jolts in the arm

def generate_trajectory(start_pose, end_pose, args, fa, dt=0.01):
    fa.log_info('Visiting Pose 1')
    fa.goto_pose(start_pose, use_impedance=False) # supposed to be a bit more accurate?

    q1 = fa.get_joints()

    fa.log_info('Visiting Pose 2')
    fa.goto_pose(end_pose, use_impedance=False)
    q2 = fa.get_joints()
    ts = np.arange(0, args.time, dt)

    joints_traj = [utils.min_jerk(q1, q2, t, args.time) for t in ts] #could maybe use another planner
    #print(len(joints_traj))
    #print("ts", ts)
    #print("joint_traj", joints_traj)
    skill_dict = create_formated_skill_dict(joints_traj, ts)
    with open(args.file, 'wb') as pkl_f:
        pkl.dump(skill_dict, pkl_f)
        print("Did save skill dict: {}".format(args.file))

if __name__ == "__main__":
    fa = FrankaArm()
    parser = argparse.ArgumentParser()
    parser.add_argument('--time', '-t', type=float, default=5)
    parser.add_argument('--file', '-f', default='bin32assembly.pkl') #filename to save generated trajectory
    args = parser.parse_args()

    # before anything, reset joints to home position
    #fa.reset_joints()
    


    # howto get current pose from arm
    #fa.reset_joints()
    pose = FC.HOME_POSE
    start_pose = pose.copy()
    end_pose = start_pose.copy()

    start_pose.translation = [0.25766588, -0.2418791, 0.47701201]
    end_pose.translation = [0.45895706, 0.08462167, 0.51798657]

    generate_trajectory(start_pose, end_pose, args, fa)


