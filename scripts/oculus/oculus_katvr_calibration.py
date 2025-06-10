import math

class KATVRCalibration:
    def __init__(self):
        # KATVR device properties
        self.yaw = None
        self.forward_velocity = None
        self.angular_velocity = None
        # KATVR status properties
        self.is_active = False
        # Calibration properties
        self.requires_hdm_calibration = False
        self.hdm_yaw = None
        self.offset = 0

    # --- HELPER FUNCTIONS ---
    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    # --- INTERNAL METHODS ---
    def calibrate_with_hdm(self):
        """
        Calibrate using current platform and head-mounted display angles.
        Assumes the user is facing forward.
        """
        self.offset = self.normalize_angle(self.hdm_yaw - self.yaw)
        print(f"Calibrated HDM/KATVR with offset: {self.offset:.2f}°")

    def get_hdm_relative_angle(self):
        """
        Get the head-relative angle after calibration.
        Converts the HDM input back to radians.
        """
        relative_angle = self.normalize_angle(self.hdm_yaw - self.yaw - self.offset)
        hdm_relative_angle_rad = math.radians(relative_angle)  # Convert to radians
        return hdm_relative_angle_rad

    def create_alternative_inputs(self, inputs):
        self.hdm_yaw = inputs[0]  # In radians
        self.hdm_yaw = math.degrees(self.hdm_yaw)  # Convert to degrees
        self.hdm_yaw = self.normalize_angle(self.hdm_yaw)  # Normalize to [-180, 180]

        if self.requires_hdm_calibration:
            self.calibrate_with_hdm()
            self.requires_hdm_calibration = False

        inputs_alternative = [
            self.get_hdm_relative_angle(),  # Relative HDM Yaw (radians)
            inputs[1],                      # HDM Pitch (radians)
            inputs[2],                      # HDM Roll (radians)
            self.yaw,                       # KATVR Yaw (degrees)
            self.forward_velocity,          # KATVR Forward Velocity (m/s)
            inputs[7],                      # Command: Stand/Sit
            inputs[9],                      # Command: Alignment
            inputs[5],                      # Right Controller X (0-1)
            inputs[6],                      # Right Controller Y (0-1)
        ]

        return inputs_alternative
