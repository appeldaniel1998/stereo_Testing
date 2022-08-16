import os

import cv2
import pyzed.sl as sl


def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
    zed.open(init_params)
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()

    while True:
        # Grab an image, a RuntimeParameters object must be given to grab()
        zed.grab(runtime_parameters)
        # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
        print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(),
                                                                             timestamp.get_milliseconds()))

        os.system("python yolov5-master/detect.py --weights '../best' --conf 0.3 --source 0 --data yolov5-master/data/data.yaml --project 'pin-box' --name 'detectTest'")

        cv2.imshow("video", img)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    # Close the camera
    zed.close()


if __name__ == "__main__":
    main()
