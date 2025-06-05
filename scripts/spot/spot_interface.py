#!/usr/bin/env python
"""
.. module:: spot_interface
    :platform: Windows
    :synopsis: The spot_interface python script in ``zed-oculus-spot`` package

.. moduleauthor:: Ali Yousefi <ali.yousefi@edu.unige.it>
	Initializes the required service clients. Provides the required method for sending the control 
    signals to the robot, and receving robot angular velocities.
"""
import logging
import time
import math
import numpy as np
import os
import sys
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client import create_standard_sdk
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b
from bosdyn.api.basic_command_pb2 import StandCommand, SitCommand


ROBOT_REMOTE_IP = '10.0.0.3'
ROBOT_LOCAL_IP = '192.168.80.3' 
ROBOT_IP = ROBOT_REMOTE_IP
ROBOT_USERNAME = 'user'
ROBOT_PASSWORD = 'wruzvkg4rce4'

LOGGER = logging.getLogger()
VELOCITY_CMD_DURATION = 0.6  # seconds
MAX_YAW = 0.5
MAX_PITCH = 0.5
MAX_ROLL = 0.5 


class SpotInterface:
    """
        Defines the Lease, eStop, Power, RobotState, and RobotCommand clients. Provides the required method for sending the control 
        signals to the robot ``set_controls(controls, dt)``, and receving robot angular velocities ``get_body_vel()``.
    """
    def __init__(self):
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_rot = 0.0
        self._powered_on = False
        self._estop_keepalive = None

        self._calibrated_with_katvr = False
        self._alignment_in_progress = False
        self._katvr_yaw_offset = 0.0
        
        sdk = create_standard_sdk("spot_interface")
        self._robot = sdk.create_robot(ROBOT_IP)
        self._robot.authenticate(ROBOT_USERNAME, ROBOT_PASSWORD, timeout=60)
        self._robot.sync_with_directory()
        self._robot.time_sync.wait_for_sync()
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._start()

    def _start(self):
        """
            Takes the lease of the robot and powers on the motors
        """
        self._lease = self._lease_client.take()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client)
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.
        self._toggle_estop()
        self._robot.power_on()
        self._powered_on=True
        time.sleep(5)
        # blocking_stand(self._robot_command_client)
        # time.sleep(5)
        LOGGER.info("ready to take commands")

    def _toggle_estop(self):
        """
            Toggle estop on/off. Initial state is ON.
        """
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                LOGGER.info('stopping estop')
                self._estop_keepalive.stop()
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    def _set_mobility_params(self):
        """
            Sets the required mobility parameters, including the obstacle avoidance, velocity limits, orientation offset between the robot 
            and footprint frames.

            Returns:
                mobility_params(spot_command_pb2.MobilityParams)
        """
        obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=False,
                                                    disable_vision_foot_obstacle_avoidance=False,
                                                    disable_vision_foot_constraint_avoidance=False,
                                                    disable_vision_foot_obstacle_body_assist= False,
                                                    disable_vision_negative_obstacles=False,
                                                    obstacle_avoidance_padding=0.1)
        
        # Set the yaw to 0 if the robot is moving in any direction
        if self._v_x != 0 or self._v_y != 0 or self._v_rot != 0:
            self._yaw = 0
        
        # Set the orientation of the robot body frame with respect to the footprint frame
        footprint_R_body = geometry.EulerZXY(roll=0.0, pitch=self._pitch, yaw=self._yaw)

        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control=spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=1.0, y=1.0), angular=1.0))
        mobility_params = spot_command_pb2.MobilityParams(obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control, locomotion_hint=spot_command_pb2.HINT_AUTO)
        return mobility_params
        
    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0.0, height=0.0):
        """
            A helper function to set the robot attitude and height values.
            
            Args:
                yaw(float)
                roll(float)
                pitch(float)
                height(float)
        """
        orientation = geometry.EulerZXY(yaw, roll, pitch)
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height, footprint_R_body=orientation)
        self._robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _velocity_cmd_helper(self, v_x=0.0, v_y=0.0, v_rot=0.0):
        """
            A helper function to set the velocity values generated from the locomotion.
            
            Args:
                v_x(float)
                v_y(float)
                v_rot(float)
        """
        mobility_params = self._set_mobility_params() 
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=mobility_params)
        self._robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
            
    def get_body_vel(self):
        """
            Provides the angular velocity values of the robot base frame.

            Returns:
                measures([float])
        """
        robot_state = self._robot_state_client.get_robot_state()
        vis_vel_ang = robot_state.kinematic_state.velocity_of_body_in_vision.angular
        measures = [vis_vel_ang.z, vis_vel_ang.y, vis_vel_ang.x]
        return measures
    
    def get_body_orientation(self):
        """
            Provides the orientation values of the robot base frame.

            Returns:
                measures([float])
        """
        measures = [round(self._yaw,3), round(self._pitch,3), round(self._roll,3)]
        return measures

    
    def stand(self):
        blocking_stand(self._robot_command_client)

    def sit(self):
        blocking_sit(self._robot_command_client)

    def is_robot_standing(state_client):
        """Returns True if the robot is currently standing, otherwise False."""
        state = state_client.get_robot_state()
        mobility_state = state.mobility_state

        return (
            hasattr(mobility_state, "has_stand_state") and mobility_state.has_stand_state and
            mobility_state.stand_state == StandCommand.Feedback.STATUS_IS_STANDING
        )

    def is_robot_sitting(state_client):
        """Returns True if the robot is currently sitting, otherwise False."""
        state = state_client.get_robot_state()
        mobility_state = state.mobility_state

        return (
            hasattr(mobility_state, "has_sit_state") and mobility_state.has_sit_state and
            mobility_state.sit_state == SitCommand.Feedback.STATUS_IS_SITTING
        )


    def shutdown(self):
        """
            Makes the robot to be configured with no orientation offsets, sit down, and powers off the motors.
        """
        self._orientation_cmd_helper(roll=0.0, pitch=0.0, yaw=0.0)
        time.sleep(1)
        blocking_sit(self._robot_command_client)
        time.sleep(5)
        safe_power_off_cmd=RobotCommandBuilder.safe_power_off_command()
        self._robot_command_client.robot_command(command= safe_power_off_cmd)
        time.sleep(2.5)
        self._powered_on=False
        LOGGER.info('Shutting down SpotInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    
    def get_absolute_yaw(self):
        """Returns the yaw (in radians) of the robot in the odometry frame."""
        robot_state = self._robot_state_client.get_robot_state()
        frame_tree_snapshot = robot_state.kinematic_state.transforms_snapshot
        odom_T_body = get_a_tform_b(frame_tree_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        # Convert to SDK Quaternion object
        quat = geometry.Quaternion(
            w=odom_T_body.rot.w,
            x=odom_T_body.rot.x,
            y=odom_T_body.rot.y,
            z=odom_T_body.rot.z
        )

        euler = geometry.to_euler_zxy(quat)
        return euler.yaw
    
    def get_absolute_position(self):
        """Returns the (x, y) position of the robot in the odometry frame."""
        robot_state = self._robot_state_client.get_robot_state()
        frame_tree_snapshot = robot_state.kinematic_state.transforms_snapshot
        odom_T_body = get_a_tform_b(frame_tree_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        pos = odom_T_body.position
        return pos.x, pos.y
    

    def set_absolute_yaw(self, target_yaw_deg, min_error_threshold=0.05):
        """
        Rotate Spot to match a world-relative yaw, calibrated on first call.

        Args:
            target_yaw_deg (float): Target yaw in degrees from VR platform.
            min_error_threshold (float): Minimum angle (in radians) under which we skip rotation.
        """
        target_yaw_rad = math.radians(target_yaw_deg)

        if not self._calibrated_with_katvr:
            current_yaw_rad = self.get_absolute_yaw()
            self._katvr_yaw_offset = target_yaw_rad - current_yaw_rad
            self._calibrated_with_katvr = True
            print(f"[CALIBRATION] Platform yaw {target_yaw_deg:.2f}° -> Calibrated offset set.")
            return

        current_yaw = self.get_absolute_yaw()
        target_robot_yaw = target_yaw_rad - self._katvr_yaw_offset

        # Normalize to [-π, π]
        yaw_error = (target_robot_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi

        if abs(yaw_error) < min_error_threshold:
            print("[YAW CMD] Within threshold, no rotation needed.")
            return

        print(f"[YAW CMD] Robot: {math.degrees(current_yaw):.2f}°, Target: {math.degrees(target_robot_yaw):.2f}°, Δ: {math.degrees(yaw_error):.2f}°")

        mobility_params = self._set_mobility_params()

        current_x, current_y = self.get_absolute_position()
        
        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=current_x,
            goal_y=current_y,
            goal_heading=yaw_error,
            frame_name="body",
            params=mobility_params
        )

        self._robot_command_client.robot_command(cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    
    def set_hmd_controls(self, hmd_controls):
        """
        Updates the robot's orientation values (body frame) based on the HMD controls.
        They are later sent to SPOT with the ``_set_mobility_params`` function through ``_velocity_cmd_helper()`` 
        or ``set_absolute_yaw()``.
        """
        # Obtain the HMD control values
        dyaw = hmd_controls[0]
        dpitch = hmd_controls[1]
        droll = hmd_controls[2]

        # Increment the robot's RPY values
        self._yaw += dyaw
        self._pitch += dpitch
        # self._roll = self._roll + droll

        # Clamp the values to the specified limits
        self._yaw = max(-MAX_YAW, min(MAX_YAW, self._yaw))
        self._pitch = max(-MAX_PITCH, min(MAX_PITCH, self._pitch))
        self._roll = max(-MAX_ROLL, min(MAX_ROLL, self._roll))

    
    def set_touch_controls(self, controls):
        """
            Sets the computed control signals on the robot base frame, using ``_velocity_cmd_helper()`` function.
        """
        # Update the robot's velocity values
        self._v_x = controls[0]
        self._v_y = controls[1]
        self._v_rot = controls[2]

        # Call the velocity command helper function
        self._velocity_cmd_helper(v_x=self._v_x, v_y=self._v_y, v_rot=self._v_rot)

    
    def set_katvr_inputs(self, inputs):
        """
        Sets the KATVR inputs to the robot.
        """
        self._v_x = inputs[1]
        self._alignment_in_progress = bool(inputs[2])

        # -- When the user is walking --
        if abs(self._v_x) > 0:
            self._velocity_cmd_helper(v_x=self._v_x, v_y=0.0, v_rot=0.0)
            self._calibrated_with_katvr = False # Reset calibration flag
            return
    
        # -- When the user is not walking (turning or looking around) --
        # Robot does not move during alignment
        if self._alignment_in_progress:
            self._calibrated_with_katvr = False # Reset calibration flag
            return

        # Set the robot's yaw to match the KATVR yaw
        self.set_absolute_yaw(inputs[0])
