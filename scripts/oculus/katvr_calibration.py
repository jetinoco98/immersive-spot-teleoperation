import math

class KATVRCalibration:
    def __init__(self):
        # KATVR device properties
        self.hdm_yaw = None
        self.yaw = None
        # Calibration properties
        self.requires_hdm_calibration = True
        self.offset = 0

    # --- PRIVATE METHODS ---
    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    def calibrate_with_hdm(self):
        """
        Calibrate using current platform and head-mounted display angles.
        Assumes the user is facing forward.
        """
        if self.requires_hdm_calibration:
            self.offset = self.normalize_angle(self.hdm_yaw - self.yaw)
            self.requires_hdm_calibration = False
        
    # --- PUBLIC METHODS ---
    def update_values(self, inputs):
        self.yaw = inputs['katvr_yaw']  # In degrees
        self.hdm_yaw = self.normalize_angle(math.degrees(inputs['hmd_yaw'])) 

    def get_hdm_relative_angle(self):
        """
        Get the head-relative angle between the head-mounted display (HDM) and the KATVR device.
        The result is in radians.
        """
        self.calibrate_with_hdm()
        relative_angle = self.normalize_angle(self.hdm_yaw - self.yaw - self.offset)
        return math.radians(relative_angle)  # Convert to radians
