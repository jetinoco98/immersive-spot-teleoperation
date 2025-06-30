import pyzed.sl as sl
import cv2
import time
import sys

QUALITY_PRESETS = {
    1: {"resolution": (1280, 360), "fps": 30},
    2: {"resolution": (1280, 360), "fps": 60},
    3: {"resolution": (1280, 480), "fps": 30},
    4: {"resolution": (1280, 480), "fps": 60},
    5: {"resolution": (1280, 720), "fps": 30},
    6: {"resolution": (1280, 720), "fps": 60},
}

class ZEDInterface:
    def __init__(self, quality_level):
        self.quality = QUALITY_PRESETS.get(quality_level, QUALITY_PRESETS[3])
        self.width, self.height = self.quality["resolution"]
        self.fps = self.quality["fps"]

        self.zed = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = min(self.fps, 60)  # ZED only supports up to 60
        init.depth_mode = sl.DEPTH_MODE.NONE

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        self.runtime = sl.RuntimeParameters()
        self.image_zed_out = sl.Mat()

    def get_image(self):
        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed_out, sl.VIEW.SIDE_BY_SIDE, sl.MEM.CPU)
            image_ocv_out = self.image_zed_out.get_data()
            image = cv2.cvtColor(image_ocv_out, cv2.COLOR_RGBA2RGB)
            image = cv2.resize(image, (self.width, self.height))
            return image
        else:
            print("Error grabbing image from ZED")
            return None

    def start_streaming(self, broker_address):
        try:
            pipeline = (
                f'appsrc ! videoconvert ! videoscale ! video/x-raw,format=RGB,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                'nvvidconv ! nvv4l2h264enc bitrate=3000000 ! video/x-h264, '
                'stream-format=byte-stream ! rtspclientsink protocols=udp location=rtsp://%s:8554/spot-stream'
                % broker_address
            )

            out_send = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, self.fps, (self.width, self.height), True)
            if not out_send.isOpened():
                print('VideoWriter not opened')
                exit(0)

            print(f"\n *** Launched RTSP Streaming ({self.width}x{self.height} @ {self.fps}fps) at rtsp://{broker_address}:8554/spot-stream ***\n")

            while True:
                image = self.get_image()
                if image is not None:
                    out_send.write(image)

        except KeyboardInterrupt:
            print("Streaming interrupted by user.")
        finally:
            self.shutdown()

    def record_locally(self, duration_sec=60):
        try:
            out_file = 'zed_recording.mp4'
            out_send = cv2.VideoWriter(out_file, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (self.width, self.height))
            if not out_send.isOpened():
                print("Failed to open output file for recording.")
                return

            print(f"\n *** Recording locally to '{out_file}' at {self.width}x{self.height} @ {self.fps}fps for {duration_sec} seconds ***\n")
            end_time = time.time() + duration_sec

            while time.time() < end_time:
                image = self.get_image()
                if image is not None:
                    out_send.write(image)

            print("\nRecording complete.\n")

        except KeyboardInterrupt:
            print("Recording interrupted by user.")
        finally:
            self.shutdown()

    def shutdown(self):
        self.zed.close()
        print("\nShutting down ZEDInterface.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['stream', 'record'], default='stream', help="Choose to 'stream' or 'record'")
    parser.add_argument('--ip', default='100.119.186.122', help="RTSP broker address for streaming")
    parser.add_argument('--duration', type=int, default=60, help="Recording duration in seconds (for 'record' mode)")
    parser.add_argument('--quality', type=int, choices=range(1, 7), default=3, help="Quality level from 1 to 6")
    args = parser.parse_args()

    zed = ZEDInterface(args.quality)
    if args.mode == 'stream':
        zed.start_streaming(args.ip)
    elif args.mode == 'record':
        zed.record_locally(args.duration)
