import threading
import pyzed.sl as sl
import cv2

class ZEDInterface:
    def __init__(self):
        self.zed = sl.Camera()
        input_type = sl.InputType()
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 60
        init.depth_mode = sl.DEPTH_MODE.NONE
        self.stop_event = threading.Event() # Event to signal termination

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        self.runtime = sl.RuntimeParameters()
        self.image_size_out = self.zed.get_camera_information().camera_configuration.resolution
        self.image_size_out.height = self.image_size_out.height // 2
        self.image_zed_out = sl.Mat(self.image_size_out.width, self.image_size_out.height, sl.MAT_TYPE.U8_C4)

    def get_image(self):
        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed_out, sl.VIEW.SIDE_BY_SIDE, sl.MEM.CPU, self.image_size_out)
            image_ocv_out = self.image_zed_out.get_data()
            image = cv2.cvtColor(image_ocv_out, cv2.COLOR_RGBA2RGB)
            return image
        else:
            print("Error grabbing image from ZED")
            return None

    def start_streaming(self, broker_address):
        try:
            pipeline = (
                'appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=1280,height=360,framerate=60 ! '
                'nvvidconv ! nvv4l2h264enc bitrate=3000000 ! video/x-h264, '
                'stream-format=byte-stream ! rtspclientsink protocols=udp location=rtsp://%s:8554/spot-stream'
                % broker_address
            )

            image_size = (1280, 360)
            out_send = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 60, image_size, True)
            if not out_send.isOpened():
                print('VideoWriter not opened')
                exit(0)

            print("\n *** Launched RTSP Streaming at rtsp://%s:8554/spot-stream ***\n\n" % broker_address)

            while not self.stop_event.is_set():
                image = self.get_image()
                if image is not None:
                    out_send.write(image)

        except KeyboardInterrupt:
            print("Streaming interrupted by user.")
        finally:
            self.shutdown()

    def shutdown(self):
        self.zed.close()


if __name__ == "__main__":
    try:
        zed_interface = ZEDInterface()
        zed_interface.start_streaming("100.119.186.122")
    except KeyboardInterrupt:
        print("ZEDInterface interrupted by user.")
    finally:
        zed_interface.shutdown()
        print("ZEDInterface shutdown complete.")