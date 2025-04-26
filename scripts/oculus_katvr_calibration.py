class KATVRInputs:
    def __init__(self):
        self.turn = 0
        self.move = 0
        self.yaw = None
        self.is_active = False
        self.requires_calibration = False

class HDMCalibrator:
    def __init__(self):
        self.offset = 0

    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    def calibrate(self, kat_angle, hdm_angle):
        """
        Calibrate using current platform and head-mounted display angles.
        Assumes the user is facing forward in both systems.
        """
        # Normalize the individual angles
        kat_angle = self.normalize_angle(kat_angle)
        hdm_angle = self.normalize_angle(hdm_angle)
        # Obtain offset
        self.offset = self.normalize_angle(hdm_angle - kat_angle)
        print(f"Calibrated HDM/KATVR with offset: {self.offset:.2f}°")

    def get_hdm_relative_angle(self, kat_angle, hdm_angle):
        """
        Get the head-relative angle after calibration.
        Returns how much the head has turned relative to the platform.
        """
        # Normalize the individual angles
        kat_angle = self.normalize_angle(kat_angle)
        hdm_angle = self.normalize_angle(hdm_angle)
        # Compensate for initial calibration offset
        relative_angle = self.normalize_angle(hdm_angle - kat_angle - self.offset)
        # Shift the angle back to the range of 0 to 360
        shifted_angle = (relative_angle + 360) % 360
        return shifted_angle
    

def update_inputs(inputs, katvr: KATVRInputs, hdm_calibrator: HDMCalibrator):
    hdm_yaw = inputs[0]

    if katvr.requires_calibration:
        hdm_calibrator.calibrate(katvr.yaw, hdm_yaw)
        katvr.requires_calibration = False

    new_inputs = [0,0,0,0,0,0]

    new_inputs[0] = hdm_calibrator.get_hdm_relative_angle(katvr.yaw, hdm_yaw)
    new_inputs[1] = inputs[1]
    new_inputs[2] = inputs[2]
    new_inputs[3] = katvr.move
    new_inputs[4] = 0   # Sideway movement not implemented
    new_inputs[5] = katvr.turn

    return new_inputs