import sys
import pyzed.sl as sl
import cv2


class ZEDInterface :

    def __init__(self):
        self.zed = sl.Camera()
        input_type = sl.InputType()
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 60
        init.depth_mode = sl.DEPTH_MODE.NONE

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            self.zed.close()
            exit(1)

        self.runtime = sl.RuntimeParameters()
        self.image_size_out = self.zed.get_camera_information().camera_configuration.resolution
        self.image_size_out.height = self.image_size_out.height/2
        #self.image_size_out.width = self.image_size_out.width/2
        self.image_zed_out = sl.Mat(self.image_size_out.width, self.image_size_out.height, sl.MAT_TYPE.U8_C4)

    def get_image(self):
        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed_out, sl.VIEW.SIDE_BY_SIDE, sl.MEM.CPU, self.image_size_out)
            image_ocv_out = self.image_zed_out.get_data()
            image = cv2.cvtColor(image_ocv_out, cv2.COLOR_RGBA2RGB)
            return image
        else:
            print("error grabbing image from ZED")
            return None, None


    def shutdown(self):
        self.zed.close()
        print("\nShutting down ZEDInterface.")

