import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal

zed_list = []
left_list = []
depth_list = []
timestamp_list = []
thread_list = []
stop_signal = False
left_ind = 0
right_ind = 0
left_img = None
right_img = None


def signal_handler(signal, frame):
    global stop_signal
    stop_signal = True
    time.sleep(0.5)
    exit()


def stitch_images():
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind

    # if left_img is not None and right_img is not None:
    while True:
        # time.sleep(2)
        left_img_ = cv2.imread("left_.jpg")
        right_img_ = cv2.imread("right_.jpg")
        try:
            left_img_ = cv2.resize(left_img_, (0, 0), fx=0.4, fy=0.4)
            right_img_ = cv2.resize(right_img_, (0, 0), fx=0.4, fy=0.4)
            stitch_list = [left_img_, right_img_]
            print("[INFO] stitching images...")
            stitcher = cv2.Stitcher_create()

            status, stitched = stitcher.stitch(stitch_list)

            # if the status is '0', then OpenCV successfully performed image
            # stitching
            if status == 0:
                # write the output stitched image to disk
                # cv2.imwrite("test.jpg", stitched)
                # display the output stitched image to our screen
                cv2.imshow("Stitched", stitched)
                cv2.waitKey(1)
            # otherwise the stitching failed, likely due to not enough keypoints)
            # being detected
            else:
                print("[INFO] image stitching failed ({})".format(status))
        except Exception:
            pass


def grab_run(index, pos):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind
    global right_img
    global left_img

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
            if pos == 'left':
                cv2.imwrite("left_.jpg", left_list[index].get_data())
                left_img = cv2.imread("left_.jpg")
            else:
                cv2.imwrite("right_.jpg", left_list[index].get_data())
                right_img = cv2.imread("right_.jpg")

        # if cv2.waitKey(62):
        #     if pos == 'left':
        #         cv2.imwrite("DATA2/image_" + str(left_ind) + "_" + str(pos) + ".jpg", left_list[index].get_data())
        #         left_ind += 1
        #     else:
        #         cv2.imwrite("DATA2/image_" + str(right_ind) + "_" + str(pos) + ".jpg", left_list[index].get_data())
        #         right_ind += 1
        time.sleep(0.001)  # 1ms
    zed_list[index].close()


def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list
    signal.signal(signal.SIGINT, signal_handler)

    print("Running...")
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    # List and open cameras
    name_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)
        name_list.append("ZED {}".format(cam.serial_number))
        print("Opening {}".format(name_list[index]))
        zed_list.append(sl.Camera())
        left_list.append(sl.Mat())
        depth_list.append(sl.Mat())
        timestamp_list.append(0)
        last_ts_list.append(0)
        status = zed_list[index].open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()
        index = index + 1

    # Start camera threads
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            pos = ''
            if index == 0:
                pos = 'right'
            else:
                pos = 'left'
            thread_list.append(threading.Thread(target=grab_run, args=(index, pos,)))
            thread_list[index].start()

    stitch_thread = threading.Thread(target=stitch_images)
    stitch_thread.start()

    # Display camera images
    key = ''
    while key != 113:  # for 'q' key
        for index in range(0, len(zed_list)):
            if zed_list[index].is_opened():
                if timestamp_list[index] > last_ts_list[index]:
                    cv2.imshow(name_list[index], left_list[index].get_data())
                    x = round(depth_list[index].get_width() / 2)
                    y = round(depth_list[index].get_height() / 2)
                    err, depth_value = depth_list[index].get_value(x, y)
                    if np.isfinite(depth_value):
                        print("{} depth at center: {}MM".format(name_list[index], round(depth_value)))
                    last_ts_list[index] = timestamp_list[index]
        key = cv2.waitKey(10)
    cv2.destroyAllWindows()

    # Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")


if __name__ == "__main__":
    main()
