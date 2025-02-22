import argparse
import pickle
import numpy as np
import time

from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup


def execute_trajectory(fa, args):
    fa.log_info('Loading Trajectory')

    with open(args.trajectory_pickle, 'rb') as pkl_f:
        skill_data = pickle.load(pkl_f)
    
    assert skill_data[0]['skill_description'] == 'GuideMode', \
        "Trajectory not collected in guide mode"
    skill_state_dict = skill_data[0]['skill_state_dict']

    T = float(skill_state_dict['time_since_skill_started'][-1])
    #dt = 0.01
    dt = 0.01 # lower dt corresponds to faster movement
    ts = np.arange(0, T, dt)

    #pose_traj = skill_state_dict['O_T_EE']
    joints_traj = skill_state_dict['q']


    # Goto the first position in the trajectory.
    fa.log_info('Initializing Sensor Publisher')
    fa.goto_joints(joints_traj[1], duration=T, dynamic=False, buffer_time=5) # go to initial pose
    fa.log_info('Publishing joints trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_joints(joints_traj[1], duration=T, dynamic=True, buffer_time=10)
    init_time = fa.get_time()
    for i in range(2, len(joints_traj)):
        traj_gen_proto_msg = JointPositionSensorMessage(
            id=i, timestamp=fa.get_time() - init_time, 
            joints=joints_traj[i]
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
        )
        
        fa.log_info('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        fa.publish_sensor_data(ros_msg)
        time.sleep(dt)
    

    try:
        # Stop the skill
        print("Trajectory executed! Stopping Skill...")
        fa._in_skill = False
        
        # Alternatively can call fa.stop_skill()
        # term_proto_msg = ShouldTerminateSensorMessage(timestamp=fa.get_time() - init_time, should_terminate=True)
        # ros_msg = make_sensor_group_msg(
        #     termination_handler_sensor_msg=sensor_proto2ros_msg(
        #         term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        #     )
        # fa.publish_sensor_data(ros_msg)

        fa.stop_skill()
        if fa.is_skill_done():
            fa.log_info('Done! Arm is free')
    except Exception as e:
        raise e
    finally:
        return

    
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory_pickle', '-t', type=str, #required=True,
                        help='Path to trajectory (in pickle format) to replay.', default='home2bin3_cam_verified.pkl')
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()

    # goto home
    fa.reset_joints()

    # execute trajectory from file in args or default
    execute_trajectory(fa, args)
    # fa.reset_joints()

    # execute trajectory in reverse
    # joints_trav_rev = joints_traj[::-1]
    # execute_trajectory(fa, joints_trav_rev)
    # execute_trajectory(fa, joints_traj)

    # test stuff
    # fa.reset_joints()
    # fa.stop_skill()
    print(fa.is_skill_done()) # timing is off, robot goal completed is called before the robot even starts moving.
    # fa.reset_joints()
    # fa.k
    # fa.stop_skill()
    # fa.reset_joints()

    # adding cartesian move
    fa.log_info("Starting cartesian move for pickup - go down")
    orig_pose = fa.get_pose()
    new_pose = orig_pose.copy()
    new_pose.translation -= [0, 0, 0.215]
    fa.goto_pose(new_pose, joint_impedances=[100, 100, 100, 100, 100, 100, 100], use_impedance=True)
    time.sleep(2)

    fa.log_info("Starting cartesian move for pickup - go up")
    orig_pose = fa.get_pose()
    new_pose = orig_pose.copy()
    new_pose.translation += [0, 0, 0.2]
    fa.goto_pose(new_pose, joint_impedances=[100, 100, 100, 100, 100, 100, 100], use_impedance=True)
    time.sleep(2)

    # execute_trajectory(fa, joints_traj)
    fa.reset_joints()
