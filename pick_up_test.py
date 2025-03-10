import argparse
import pickle
import numpy as np
import time

from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from autolab_core import RigidTransform
from frankapy.utils import convert_array_to_rigid_transform

import numpy as np
from scipy.integrate import cumtrapz


def pickup_traj(fa, x, y, start_z, end_z, step_size=0.001, acceleration = 0.1):
    '''
    Generates a trajectory from the current x, y, z, to x, y, end_z 
    using a trapazoidal velocity profile.

    Inputs:
        end_z: desired end z position in franka base link frame
        step_size: maximum z displacement that occur in one time step (0.01 s)
        acceleration: maximum allowable acceleration
    '''

    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    total_distance = abs(end_z - start_z)
    direction = 1 if end_z > start_z else -1

    dt = 0.01
    max_velocity = step_size / dt

    t_accel = max_velocity / acceleration # time for robot to get up to speed
    d_accel = 0.5 * acceleration * t_accel**2 # distance to get up to max speed or return from max speed to 0
    t_const = 0
    if 2 * d_accel < total_distance:
        # Full trapezoidal profile
        d_const = total_distance - 2 * d_accel # distance of constant speed
        t_const = d_const / max_velocity # time in constant speed
        t_total = 2 * t_accel + t_const
    else:
        # Triangular profile (not enough distance for max velocity)
        # under constanst accel: distance = 1/2*a*t^2 (each acceleration phase is 1/2 of distance)
        print("here")
        t_accel = np.sqrt(total_distance / acceleration) 
        t_total = 2 * t_accel
        max_velocity = acceleration * t_accel  # Adjusted max velocity

    t = np.arange(0, t_total + dt, dt)

    v = np.piecewise(t,
                     [t < t_accel,
                      (t >= t_accel) & (t < t_accel + t_const),
                      t >= t_accel + t_const],
                     [lambda t: acceleration * t,
                      lambda _: max_velocity,
                      lambda t: max_velocity - acceleration * (t - (t_accel + t_const))])

    z_values = direction * cumtrapz(v, t, initial=0) + start_z
    print(z_values[-10:-1])
    # Ensure the last value is exactly end_z
    if z_values[-1] != end_z:
        z_values = np.append(z_values, end_z)

    pose_traj = [RigidTransform(rotation=default_rotation,
                                translation=[x, y, z],
                                from_frame='franka_tool',
                                to_frame='world') for z in z_values]

    T = len(pose_traj) * dt
    follow_pose_trajectory(pose_traj, dt, T)





def follow_pose_trajectory(pose_traj, dt, T, at_start=True):
    '''
    Follow a pose trajectory based on a list of rigid transforms

    CAUTION: YOU MUST BE AT START X, Y, Z TO SAFELY CALL THIS FUNCTION\n
    If not, set at_start flag to false

    Inputs:
        pose_traj: list of rigid transforms
        dt: time between publishing
        T: time duration of pose trajectory
    
    '''
    if not at_start:
        fa.goto_pose(pose_traj[0], 
                    duration=4.0, 
                    use_impedance=False,
                    cartesian_impedances=[2000.0, 2000.0, 600.0, 50.0, 50.0, 50.0])
    
    fa.log_info('Initializing Sensor Publisher')

    fa.log_info('Publishing pose trajectory...')
    fa.goto_pose(pose_traj[1], 
                 duration=T, 
                 dynamic=True, 
                 buffer_time=1, 
                 use_impedance=False,
                 cartesian_impedances=[2000.0, 2000.0, 600.0, 50.0, 50.0, 50.0]
    )

    init_time = fa.get_time()
    for i in range(2, len(pose_traj)):
        timestamp = fa.get_time() - init_time
        pose_tf = pose_traj[i]
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, 
            timestamp=timestamp,
            position=pose_tf.translation, 
            quaternion=pose_tf.quaternion
		)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, 
                SensorDataMessageType.POSE_POSITION),
            )
        # Sleep the same amount as the trajectory was recorded in
        dt = 0.01
        fa.publish_sensor_data(ros_msg)
        time.sleep(dt)

    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=fa.get_time() - init_time, 
                                                  should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    
    fa.publish_sensor_data(ros_msg)
    fa.wait_for_skill()
    fa.log_info('Done')


if __name__ == "__main__":
    print('Starting robot')
    fa = FrankaArm()
    fa.reset_joints()

    curr_pose = fa.get_pose()
    init_z = 0.3
    final_z = 0.137
    x = 0.44
    y = -0.302

    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    new_pose = curr_pose.copy()
    new_pose.translation = np.array([x, y, init_z])
    new_pose.rotation = default_rotation

    fa.goto_pose(new_pose, use_impedance=False)
    curr_pose = fa.get_pose()
    start_z = curr_pose.translation[2]
    pickup_traj(fa, x, y, start_z, final_z)
    time.sleep(2)
    curr_pose = fa.get_pose()
    start_z = curr_pose.translation[2]
    pickup_traj(fa, x, y, start_z, init_z)
