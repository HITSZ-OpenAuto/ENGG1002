import cv2
import numpy as np
from robomaster import robot

low_green = np.array([45, 70, 70])
up_green = np.array([77, 255, 255])

Kpy = 1.0
Kpx = 0.4
Kpz = 50


def process_img_open(src_binary, iter=1):
    # 开：先腐蚀，再膨胀 开运算可以用来消除小黑点，在纤细点处分离物体、平滑较大物体的边界的 同时并不明显改变其面积。
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(src_binary, op=cv2.MORPH_OPEN, kernel=kernel, iterations=iter)
    return opening


def process_img_close(src_binary, iter=1):
    # 闭：先膨胀，再腐蚀 闭运算可以用来排除小黑洞。
    kernel = np.ones((3, 3), np.uint8)
    closeing = cv2.morphologyEx(src_binary, op=cv2.MORPH_CLOSE, kernel=kernel, iterations=iter)
    return closeing


def process_img_ERODE(src_birnary, iteration=1):
    kernel = np.ones((3, 3), np.uint8)
    closing = cv2.morphologyEx(src_birnary, op=cv2.MORPH_ERODE, kernel=kernel, iterations=iteration)
    return closing


def process_img_DILATE(src_birnary, iteration=1):
    kernel = np.ones((3, 3), np.uint8)
    closing = cv2.morphologyEx(src_birnary, op=cv2.MORPH_ERODE, kernel=kernel, iterations=iteration)
    return closing


def findcontours(src, src_plot):
    contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        _area = cv2.contourArea((contours[idx]))
        if len(contour) >= 5:
            box = center_idx, size_idx, angle_idx = cv2.fitEllipse(contours[idx])
            if max(size_idx[0], size_idx[1]) < min(size_idx[0], size_idx[1]) * 1.3:
                center_idx = np.nan_to_num(center_idx, nan=0.0, posinf=1e3, neginf=-1e3)
                size_idx = np.nan_to_num(size_idx, nan=0.0, posinf=1e3, neginf=-1e3)
                print(f"this is center_id:{center_idx}")
                print(f"this is size_id:{size_idx}")
                center_idx = np.uint16(center_idx)
                size_idx = np.uint16(size_idx)
                cv2.drawContours(src_plot, contours, idx, (0, 0, 255), 3)
                box_array = np.array(box, dtype=object)
                box_array[1] = np.nan_to_num(box_array[1], nan=0.0, posinf=1e3, neginf=-1e3)
                print(f"this is box_array:{box_array}")
                box = tuple(box_array)
                print(f"this is box:{box}")
                cv2.ellipse(src_plot, box, (255, 0, 0), 2)
                cv2.circle((src_plot), center_idx, 1, (0, 0, 255), 3)
                cv2.putText(
                    src_plot,
                    "(%d,%d)" % (center_idx[0], center_idx[1]),
                    center_idx,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
                x = center_idx[0] / 640
                _y = center_idx[1] / 360
                diameter = np.mean(size_idx)
                d = diameter / 40

                error = 1 - d
                if error <= 0:
                    error = 0
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                else:
                    ep_chassis.drive_speed(x=Kpx * error, y=Kpy * (x - 0.5), z=Kpz * (x - 0.5))
                # print(x, y)
                # ep_chassis.drive_speed(x=0, y=Kpy * (x - 0.5), z=0)
                # ep_chassis.drive_speed(x=Kpx * (1-d), y=0, z=0)
                print(Kpx * error)
                # print(diameter, "\n")


name = "D:/class_test/img.jpg"
if __name__ == "__main__":  # python 主函数入口
    ep_robot = robot.Robot()  # 创建 robot 类的实例对象 ep_robot
    # 初始化 ep_robot 机器人，conn_type 指定机器人连接模式，ap 表示 wifi 直连模式，sta 表示 wifi 组网模式，rndis 表示 USB 连接模式
    ep_robot.initialize(conn_type="ap")
    ep_robot.set_robot_mode(mode="GIMBAL_LEAD")
    # 定义机器人云台对象
    ep_chassis = ep_robot.chassis

    ep_camera = ep_robot.camera  # 创建 ep_camera 对象

    # 调用 camera 对象的 start_video_stream 方法，开始获取视频流
    # display 参数指定是否显示获取到的视频流，resolution 参数指定视频的尺寸大小，支持 360p，540p，720p
    ep_camera.start_video_stream(display=False, resolution="360p")
    for i in range(0, 2000):
        # while(1): # 循环读取 200 帧图像数据
        img = ep_camera.read_cv2_image()  # 调用 camera 对象的 read_cv_image 方法，获取一帧图像
        img_color = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_green = cv2.inRange(img_color, lowerb=low_green, upperb=up_green)
        img_green = process_img_ERODE(img_green, 1)
        img_green = process_img_DILATE(img_green, 1)
        # img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite(name, img_color) # 调用 cv2 的 imshow 方法，将该帧图像显示在 Robot 窗口中
        img_green = process_img_open(img_green, 5)
        is_black = not cv2.countNonZero(img_green)
        if is_black:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            continue
        findcontours(img_green, img)
        cv2.imshow("Robot", img)
        cv2.imshow("Robot.green", img_green)
        # cv2.waitKey(1)  # 调用 cv2 的 waitKey 方法，刷新图像显示
        if cv2.waitKey(1) == 27:
            # 停止麦轮运动
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            ep_robot.close()  # 释放 ep_robot 对象相关资源

    cv2.destroyAllWindows()  # 程序结束前，销毁所有窗口
    ep_camera.stop_video_stream()  # 停止获取视频流

    # 停止麦轮运动
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ep_robot.close()  # 释放 ep_robot 对象相关资源
