import pyzed.sl as sl
import cv2
import argparse

class ZEDInterface:
    def __init__(self):
        self.zed = sl.Camera()
        input_type = sl.InputType()
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 60
        init.depth_mode = sl.DEPTH_MODE.NONE

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        self.runtime = sl.RuntimeParameters()
        self.image_size_out = self.zed.get_camera_information().camera_configuration.resolution
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

    def start_streaming(self, broker_address, resolution_horizontal=1280, resolution_vertical=360):
        try:
            pipeline = (
                'appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=%d,height=%d,framerate=60/1 ! '
                'nvvidconv ! nvv4l2h264enc bitrate=3000000 ! video/x-h264, '
                'stream-format=byte-stream ! rtspclientsink protocols=udp location=rtsp://%s:8554/spot-stream'
                % (resolution_horizontal, resolution_vertical, broker_address)
            )

            image_size = (resolution_horizontal, resolution_vertical)
            out_send = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 60, image_size, True)
            if not out_send.isOpened():
                print('VideoWriter not opened')
                exit(0)

            print("\n *** Launched RTSP Streaming at rtsp://%s:8554/spot-stream ***\n\n" % broker_address)

            while True:
                image = self.get_image()
                image = cv2.resize(image, (resolution_horizontal, resolution_vertical))
                if image is not None:
                    out_send.write(image)

        except KeyboardInterrupt:
            print("Streaming interrupted by user.")
        finally:
            self.shutdown()

    def shutdown(self):
        self.zed.close()
        print("\nShutting down ZEDInterface.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ZED RTSP Streamer")
    parser.add_argument("--broker_address", type=str, default='100.119.186.122', help="RTSP broker address")
    parser.add_argument("--quality", type=int, default=1, choices=[1, 2, 3, 4],
                        help="Streaming quality preset (1=1280x360, 2=1708x480, 3=1920x540, 4=2560x720)")

    args = parser.parse_args()

    quality_presets = {
        1: (1280, 360),
        2: (1708, 480),
        3: (1920, 540),
        4: (2560, 720)
    }

    resolution_horizontal, resolution_vertical = quality_presets.get(args.quality, (1280, 360))
    print(f"[INFO] Selected quality {args.quality}: {resolution_horizontal}x{resolution_vertical}")

    zed = ZEDInterface()
    zed.start_streaming(args.broker_address, resolution_horizontal, resolution_vertical)