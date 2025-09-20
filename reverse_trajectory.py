import numpy as np
import argparse

from frankapy import FrankaArm
from frankapy import utils
from record_trajectory import create_formated_skill_dict
import pickle as pkl
from frankapy import FrankaConstants as FC
import pickle

def generate_joints_trajectory(q1, q2, args, dt=0.01):
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory_pickle', '-p', type=str, #required=True,
                        help='Path to trajectory (in pickle format) to replay.', default='assembly2bin4.pkl') # filename to load trajectory
    parser.add_argument('--time', '-t', type=float, default=3)
    parser.add_argument('--file', '-f', default='bin42assembly.pkl') #filename to save generated trajectory
    args = parser.parse_args()
    
    with open(args.trajectory_pickle, 'rb') as pkl_f:
        skill_data = pickle.load(pkl_f)
    skill_state_dict = skill_data[0]['skill_state_dict']

    #pose_traj = skill_state_dict['O_T_EE']
    joints_traj = skill_state_dict['q']
    
    generate_joints_trajectory(joints_traj[-1], joints_traj[0], args)