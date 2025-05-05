import math

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
        HDM angle is in radians and needs to be converted.
        """
        hdm_angle_deg = -math.degrees(hdm_angle)  # Convert to degrees and invert sign

        kat_angle = self.normalize_angle(kat_angle)
        hdm_angle_deg = self.normalize_angle(hdm_angle_deg)

        self.offset = self.normalize_angle(hdm_angle_deg - kat_angle)
        print(f"Calibrated HDM/KATVR with offset: {self.offset:.2f}°")

    def get_hdm_relative_angle(self, kat_angle, hdm_angle):
        """
        Get the head-relative angle after calibration.
        Converts the HDM input to degrees and inverts it for internal use,
        then converts it back to the HDM format before returning.
        """
        hdm_angle_deg = -math.degrees(hdm_angle)  # Convert to degrees and invert

        kat_angle = self.normalize_angle(kat_angle)
        hdm_angle_deg = self.normalize_angle(hdm_angle_deg)

        relative_angle = self.normalize_angle(hdm_angle_deg - kat_angle - self.offset)

        hdm_output_angle = -math.radians(relative_angle)  # Invert sign and convert to radians
        return hdm_output_angle

def update_inputs(inputs, katvr: KATVRInputs, hdm_calibrator: HDMCalibrator):
    hdm_yaw = inputs[0]  # In radians, from -π to π, inverted

    if katvr.requires_calibration:
        hdm_calibrator.calibrate(katvr.yaw, hdm_yaw)
        katvr.requires_calibration = False

    # Create a list of zeroes with length 7
    new_inputs = [0] * 7

    new_inputs[0] = hdm_calibrator.get_hdm_relative_angle(katvr.yaw, hdm_yaw)
    new_inputs[1] = inputs[1]
    new_inputs[2] = inputs[2]
    new_inputs[3] = katvr.move
    new_inputs[4] = 0  # Sideway movement not implemented
    new_inputs[5] = katvr.turn
    new_inputs[6] = inputs[6]

    return new_inputs
