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
import sys
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.util
from bosdyn.api import estop_pb2
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn import geometry
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b


LOGGER = logging.getLogger()

ROBOT_REMOTE_IP = '10.0.0.3'
ROBOT_LOCAL_IP = '192.168.80.3' 
ROBOT_IP = ROBOT_REMOTE_IP
ROBOT_USERNAME = 'user'
ROBOT_PASSWORD = 'wruzvkg4rce4'

YAW_MAX = 0.7805
PITCH_MAX = 0.7805
ROLL_MAX = 0.4
LINEAR_VEL_MAX = 0.5  # m/s
ANGULAR_VEL_MAX = 1.5  # rad/s
VELOCITY_CMD_DURATION = 0.6  # seconds


class SpotInterface:
    def __init__(self):
        # Robot state
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_rot = 0.0
        self._yaw_odom = None
        self._estop_keepalive = None
        self.is_standing = False

        # For KATVR integration
        self._katvr_yaw_offset = 0.0
        self._calibrated_with_katvr = False

        # For PID controller
        self._yaw_error = 0.0
        self._prev_yaw_error = 0.0
        self._prev_time = time.time()
        self._pid_kp = 1.5
        self._pid_kd = 0.1
        self._pid_dead_zone_degrees = 1.0

        self._initialize_robot()
        

    def _initialize_robot(self):
        """ 
        Initializes SDK, authenticates, and ensures the required clients are available. 
        """
        print("Initializing SpotInterface. Wait to a receive confirmation message...")
        self._client_name = "spot_interface"
        sdk = bosdyn.client.create_standard_sdk(self._client_name)
        self._robot = sdk.create_robot(ROBOT_IP)
        self._robot.authenticate(ROBOT_USERNAME, ROBOT_PASSWORD, timeout=60)
        self._robot.sync_with_directory()
        self._robot.time_sync.wait_for_sync()
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._start()
        

    def _start(self):
        """
        Takes the lease of the robot and powers on the motors
        """
        # Gain Control of the robot
        self._lease_client.acquire()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, return_at_exit=True)
        self._toggle_estop()

        # Power on the robot
        self._robot.power_on()
        self._robot.is_powered_on()


    def _toggle_estop(self):
        """
        Toggles on/off E-Stop. Initial state is ON.
        """
        if not self._estop_keepalive:
            if self._estop_client.get_status().stop_level == estop_pb2.ESTOP_LEVEL_NONE:
                print('Taking E-Stop from another controller')

            #register endpoint with 9 second timeout
            estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self._estop_client,
                                                               name=self._client_name,
                                                               estop_timeout=9.0)
            estop_endpoint.force_simple_setup()

            self._estop_keepalive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
        else:
            self._estop_keepalive.stop()
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None
    

    def _set_mobility_params(self):
        """
        Sets the required mobility parameters, including obstacle avoidance, velocity limits, and orientation offset between the robot 
        and footprint frames.

        Returns:
            mobility_params(spot_command_pb2.MobilityParams)
        """
        obstacles = spot_command_pb2.ObstacleParams(
            disable_vision_body_obstacle_avoidance=False,
            disable_vision_foot_obstacle_avoidance=False,
            disable_vision_foot_constraint_avoidance=False,
            disable_vision_foot_obstacle_body_assist=False,
            disable_vision_negative_obstacles=False,
            obstacle_avoidance_padding=0.1
        )
        
        # Set Spot's body frame yaw to 0 if the robot is moving in any direction
        if self._v_x != 0 or self._v_y != 0 or self._v_rot != 0:
            self._yaw = 0
        
        # Set the orientation of the robot body frame with respect to the footprint frame
        footprint_R_body = geometry.EulerZXY(roll=0.0, pitch=self._pitch, yaw=self._yaw)

        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=LINEAR_VEL_MAX, y=LINEAR_VEL_MAX), angular=ANGULAR_VEL_MAX))
        mobility_params = spot_command_pb2.MobilityParams(obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control, locomotion_hint=spot_command_pb2.HINT_AUTO)
        return mobility_params
        

    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0.0, height=0.0):
        """
        Helper function that commands the robot with an orientation command
            
        Args:
            yaw: Yaw of the robot body. Defaults to 0.0.
            roll: Roll of the robot body. Defaults to 0.0.
            pitch: Pitch of the robot body. Defaults to 0.0.
            height: Height of the robot body from normal stand height. Defaults to 0.0.
        """
        orientation = geometry.EulerZXY(yaw, roll, pitch)
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height, footprint_R_body=orientation)
        self._robot_command_client.robot_command_async(command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)


    def _velocity_cmd_helper(self, v_x=0.0, v_y=0.0, v_rot=0.0):
        """
        Helper function that commands the robot with a velocity command

        Args:
            v_x: Forward/backward velocity of the robot body.
            v_y: Left/right velocity of the robot body.
            v_rot: Rotational velocity of the robot body.
        """
        mobility_params = self._set_mobility_params() 
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=mobility_params)
        self._robot_command_client.robot_command_async(command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
    

    def get_body_orientation(self):
        """
        Provides the orientation values of the robot base frame.

        Returns:
            List[float]: A list containing the yaw, pitch, and roll angles of the robot's base frame,
            in the following order: [yaw, pitch, roll].
        """
        
        return [round(self._yaw, 3), round(self._pitch, 3), round(self._roll, 3)]

    
    def stand(self):
        if not self.is_standing:
            blocking_stand(self._robot_command_client)
            self.is_standing = True
            self._calibrated_with_katvr = False  # Reset calibration state


    def sit(self):
        if self.is_standing:
            blocking_sit(self._robot_command_client)
            self.is_standing = False


    def shutdown(self):
        """
            Makes the robot to be configured with no orientation offsets, sit down, and powers off the motors.
        """
        print('Shutting down SpotInterface...')
        if self.is_standing:
            self.set_idle_mode(default_height=-0.2)
            time.sleep(1.5)
        self.sit()
        time.sleep(3)
        safe_power_off_cmd=RobotCommandBuilder.safe_power_off_command()
        self._robot_command_client.robot_command(command= safe_power_off_cmd)
        time.sleep(2.5)
        self._toggle_estop()  # Toggle E-Stop back ON
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()


    def set_hmd_controls(self, hmd_controls: list):
        """
        Updates the robot's orientation values (body frame) based on the HMD orientation and LQR computation.

        Args:
            hmd_controls (list): A list containing the processed HMD control values in the order of [dyaw, dpitch, droll].
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
        self._yaw = max(-YAW_MAX, min(YAW_MAX, self._yaw))
        self._pitch = max(-PITCH_MAX, min(PITCH_MAX, self._pitch))
        self._roll = max(-ROLL_MAX, min(ROLL_MAX, self._roll))

    
    def set_touch_controls(self, touch_controls: list):
        """
        Updates the robot's velocity values (body frame) based on the HMD Touch Controller Joysticks.

        Args:
            touch_controls (list): A list containing the computed velocity control signals in the order of [v_x, v_y, v_rot].
        """
        self._v_x = touch_controls[0]
        self._v_y = touch_controls[1]
        self._v_rot = touch_controls[2]

        if self._v_rot != 0:
            self._calibrated_with_katvr = False  # Reset calibration state


    def send_velocity_command(self, vx=None, vy=None, vrot=None):
        """
        Sends a velocity command to the robot using either the current stored values of the SpotInterface object, 
        or the provided arguments.

        Args:
            vx (float, optional): Forward/backward speed (m/s). If None, uses the current value.
            vy (float, optional): Left/right speed (m/s). If None, uses the current value.
            vrot (float, optional): Rotational speed (rad/s). If None, uses the current value.
        """
        # Use provided values if given, otherwise use the stored values
        vx = self._v_x if vx is None else vx
        vy = self._v_y if vy is None else vy
        vrot = self._v_rot if vrot is None else vrot

        # Send the velocity command to the robot
        self._velocity_cmd_helper(v_x=vx, v_y=vy, v_rot=vrot)

    
    # ====================================================================================
    #   METHODS FOR KATVR INTEGRATION
    # ====================================================================================

    def auto_sit_in_katvr(self, inputs, height_threshold=0.2, hysteresis_margin=0.1):
        """
        Makes the robot sit if the HDM height drops below (base_height - height_threshold) for over a time window.
        Applies hysteresis to avoid sudden transitions.

        Returns:
            bool: True if the robot idles or sits, False otherwise.
        """
        hmd_height = inputs['hmd_height']  # Get the current HMD height from inputs

        if inputs['stand'] == 1.0:
            self.base_hmd_height = hmd_height
            self.low_height_start_time = None
            return False

        if hasattr(self, 'base_hmd_height'):
            lower_limit = self.base_hmd_height - height_threshold
            recovery_limit = self.base_hmd_height - height_threshold + hysteresis_margin

            if hmd_height < lower_limit:
                # Height is low. Start the timer if not already started.
                if not hasattr(self, 'low_height_start_time') or self.low_height_start_time is None:
                    self.low_height_start_time = time.time()

                elapsed = time.time() - self.low_height_start_time

                if elapsed > 0.1:
                    if self.is_standing:
                        self.set_idle_mode(height=-0.25)

                    if elapsed > 2:
                        self.sit()
                        self.low_height_start_time = None

                    return True
                
            elif hmd_height > recovery_limit:
                self.low_height_start_time = None  # Height recovered, so reset

        return False


    def update_odometry_angles(self):
        """
        Updates the class variable _odom_yaw with the robot's yaw in the odometry frame.
        """
        robot_state = self._robot_state_client.get_robot_state()
        frame_tree_snapshot = robot_state.kinematic_state.transforms_snapshot
        odom_T_body = get_a_tform_b(frame_tree_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        quat = geometry.Quaternion(
            w=odom_T_body.rot.w,
            x=odom_T_body.rot.x,
            y=odom_T_body.rot.y,
            z=odom_T_body.rot.z
        )
        euler = geometry.to_euler_zxy(quat)
        self._yaw_odom = euler.yaw 


    def update_pid_controller(self, kp, kd, dead_zone_degrees):
        """
        Update the PID controller parameters for KATVR yaw control.
        """
        if kp is not None:
            self._pid_kp = kp
        if kd is not None:
            self._pid_kd = kd
        if dead_zone_degrees is not None:
            self._pid_dead_zone_degrees = dead_zone_degrees


    def _reset_body_frame_orientation_values(self):
        """
        Resets the body frame orientation to zero. It does NOT send a command to the robot.
        """
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0


    def _compute_angular_velocity(self, katvr_yaw_deg):
        """
        Compute rotational velocity using PID control to align yaw with KATVR.
        """
        self.update_odometry_angles()
        katvr_yaw = math.radians(katvr_yaw_deg)

        if not self._calibrated_with_katvr:
            self._katvr_yaw_offset = katvr_yaw - self._yaw_odom
            self._calibrated_with_katvr = True
            # Reset Yaw Error and Time
            self._yaw_error = 0.0
            self._prev_time = time.time()
            return 0.0

        # Compute calibrated yaw target for the robot
        target_robot_yaw = katvr_yaw - self._katvr_yaw_offset

        # Normalize yaw error to [-π, π]
        self._yaw_error = (target_robot_yaw - self._yaw_odom + np.pi) % (2 * np.pi) - np.pi

        # Set variables for PID control
        Kp = self._pid_kp
        Kd = self._pid_kd
        DEAD_ZONE_RAD = math.radians(self._pid_dead_zone_degrees)

        # Time step for derivative
        now = time.time()
        dt = now - self._prev_time
        self._prev_time = now

        # Derivative term
        prev_error = self._prev_yaw_error
        d_error = (self._yaw_error - prev_error) / dt if dt > 0 else 0.0
        self._prev_yaw_error = self._yaw_error

        # Dead zone logic
        if abs(self._yaw_error) < DEAD_ZONE_RAD:
            output = 0.0

        # Output calculation
        output = Kp * self._yaw_error + Kd * d_error
        output = max(-ANGULAR_VEL_MAX, min(ANGULAR_VEL_MAX, output))
        return output
    
    
    def set_katvr_command(self, inputs, hmd_controls):
        """
        Sets commands for velocity and rotation to the robot based on KATVR inputs and HMD controls.
        """
        # Parse inputs
        katvr_yaw_deg = inputs['katvr_yaw']
        katvr_vx = inputs['katvr_forward_velocity']
        katvr_vy = inputs['katvr_lateral_velocity']
        speed_lock = bool(inputs['speed_lock'])
        rotation_lock = bool(inputs['rotation_lock'])

        if speed_lock:
            self._v_x = 0.0
            self._v_y = 0.0
            self._v_rot = 0.0
            self._calibrated_with_katvr = False
            self._reset_body_frame_orientation_values() 
            self._velocity_cmd_helper() # Send zero velocity command
            return  # Exit early if speed is locked

        ''' Use KATVR velocity inputs if they are greater than zero, otherwise keep the current values
        updated from the Touch controls.'''
        if abs(katvr_vx) > 0:
            self._v_x = katvr_vx
        if abs(katvr_vy) > 0:
            self._v_y = katvr_vy

        # Rotation lock logic
        if rotation_lock:
            self._v_rot = 0.0
            self._calibrated_with_katvr = False
            self.set_hmd_controls(hmd_controls)
        else:
            self._reset_body_frame_orientation_values()  
            # Compute KATVR rotation. If it's greater than the joystick rotation, update self._v_rot
            katvr_v_rot = self._compute_angular_velocity(katvr_yaw_deg)
            if abs(katvr_v_rot) > abs(self._v_rot):
                self._v_rot = katvr_v_rot

        # Send the velocity command to the robot
        self._velocity_cmd_helper(v_x=self._v_x, v_y=self._v_y, v_rot=self._v_rot)

    
    def set_idle_mode(self, height=0.0):
        """
        Sets the robot to idle mode by setting all velocities and orientations to zero.
        """
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_rot = 0.0
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._orientation_cmd_helper(yaw=self._yaw, pitch=self._pitch, roll=self._roll, height=height)
        self._calibrated_with_katvr = False


    def print_spot_motion_status(self):
        """
        Prints robot velocities and orientation on the same console line.
        """
        sys.stdout.write(
            f"\r[SPOT] Vx: {self._v_x:.2f} m/s | Vy: {self._v_y:.2f} | Rot: {self._v_rot:.2f} rad/s | "
            f"Yaw Error: {math.degrees(self._yaw_error):.1f}°"
        )
        sys.stdout.flush()
