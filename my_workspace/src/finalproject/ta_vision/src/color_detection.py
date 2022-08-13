#!/usr/bin/python
#import ta_vision

from cv2 import ROTATE_90_CLOCKWISE
from vision.camera import Camera
from color_detection import ColorDetection

import cv2 as cv
import rospy
import time

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

# centroid_Bool = False
lower_threshold_outter = (144, 215, 123)
upper_threshold_outter = (158, 255, 166)

lower_threshold_middle = (0, 120, 130)
upper_threshold_middle = (110, 255, 255)

lower_threshold_inner = (54, 220, 0)
upper_threshold_inner = (64, 255, 255)


# lower_threshold = (160, 190, 220)
# upper_threshold = (180, 230, 255)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(30) # 15 FPS
        cam_bool_outter_pub = rospy.Publisher("camera/bool/outter", String, queue_size=10)
        cam_bool_middle_pub = rospy.Publisher("camera/bool/middle", String, queue_size=10)
        cam_bool_inner_pub = rospy.Publisher("camera/bool/inner", String, queue_size=10)
        
        cam_val_outter_pub = rospy.Publisher("camera/data/outter", PointStamped, queue_size=10)
        cam_val_middle_pub = rospy.Publisher("camera/data/middle", PointStamped, queue_size=10)
        cam_val_inner_pub = rospy.Publisher("camera/data/inner", PointStamped, queue_size=10)



        cap = Camera(port=5600)
        cd_outter = ColorDetection(lower_threshold_outter, upper_threshold_outter)
        cd_middle = ColorDetection(lower_threshold_middle, upper_threshold_middle)
        cd_inner = ColorDetection(lower_threshold_inner, upper_threshold_inner)

        rospy.loginfo("Wait for camera capture..")
        frame = cap.capture()

        centroid_bool_outter = String()
        centroid_bool_middle = String()
        centroid_bool_inner = String()

        while frame is None and not rospy.is_shutdown():
            rate.sleep()
            frame = cap.capture()
        rospy.loginfo("Frame captured!")
        
        fps = 16.0
        t = time.time()
        

        while not rospy.is_shutdown():
            frame_start = cap.capture()
            frame = cv.GaussianBlur(frame_start, (9,9), 0)
            frame = cv.rotate(frame, ROTATE_90_CLOCKWISE) #camera vertical
            t_cap = rospy.Time.now()
            mask = cd_outter.update(frame)
            mask = cd_middle.update(frame)
            mask = cd_inner.update(frame)
            
######################### outter marker
            # if cd_outter.centroid:
            #     (cX, cY) = cd_outter.centroid
            #     centroid = PointStamped()
            #     centroid.point.x = cX - 240.0# / 1.15384615         #480
            #     centroid.point.y = cY - 320.0# / 1.53846154        #640
            #     centroid.point.y = -centroid.point.y
            #     centroid.header.stamp = t_cap
            #     cam_val_outter_pub.publish(centroid)
            # if cd_outter.has_centroid:
            #     centroid_bool_outter.data = "true"
            #     # cv.circle(frame, cd_outter.centroid, 8, (255, 0, 0), -1)
            # else:
            #     centroid_bool_outter.data = "false"
            # cam_bool_outter_pub.publish(centroid_bool_outter)

            
######################### middle marker
            if cd_middle.centroid:
                
                (cX, cY) = cd_middle.centroid
                centroid = PointStamped()
                centroid.point.x = cX - 240.0           #480
                centroid.point.y = cY - 320.0           #640
                centroid.point.y = -centroid.point.y
                centroid.header.stamp = t_cap
                cam_val_middle_pub.publish(centroid)
            if cd_middle.has_centroid:
                centroid_bool_middle.data = "true"
                cv.circle(frame, cd_middle.centroid, 5, (255, 0, 0), -1)
            else:
                centroid_bool_middle.data = "false"
            cam_bool_middle_pub.publish(centroid_bool_middle)

######################### inner marker
            if cd_inner.centroid:
                
                (cX, cY) = cd_inner.centroid
                centroid = PointStamped()
                centroid.point.x = cX - 240.0           #480
                centroid.point.y = cY - 320.0           #640
                centroid.point.y = -centroid.point.y
                centroid.header.stamp = t_cap
                cam_val_inner_pub.publish(centroid)
            if cd_inner.has_centroid:
                centroid_bool_inner.data = "true"
                cv.circle(frame, cd_inner.centroid, 2, (0, 0, 255), -1)
            else:
                centroid_bool_inner.data = "false"
            cam_bool_inner_pub.publish(centroid_bool_inner)
            
            # cv.putText(frame, "fps: %.1f" % fps, (10, 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, 127, 2)
            
            #print(fps)
            # cv.imshow("Frame", frame)

            key = cv.waitKey(15)
            if key == 27:
                break

            fps =  0.9 * fps + 0.1 * 1 / (time.time() - t)
            t = time.time()
            # time.sleep(0.3)
            rate.sleep()


    except rospy.ROSInterruptException:
        pass

    if cap is not None:
        cap.close()
    cv.destroyAllWindows()
