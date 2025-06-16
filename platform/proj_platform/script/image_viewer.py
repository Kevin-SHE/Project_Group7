# #!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# class ImageViewer:
#     def __init__(self):
#         self.bridge = CvBridge()
#         rospy.init_node('object_sorting_image_viewer', anonymous=True)
#         self.image_sub = rospy.Subscriber('/object_sorting/image_result', Image, self.callback)
#         rospy.loginfo("已订阅 /object_sorting/image_result 图像话题。")

#     def callback(self, msg):
#         try:
#             # 将ROS图像消息转为OpenCV格式
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             # 显示图像
#             cv2.imshow("Object Sorting Image", cv_image)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q'):
#                  rospy.signal_shutdown("用户请求退出")
#         except CvBridgeError as e:
#             rospy.logerr(f"图像转换错误: {e}")

#     def run(self):
#         while not rospy.is_shutdown():
#             rospy.sleep(0.1)
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     try:
#         viewer = ImageViewer()
#         viewer.run()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('object_sorting_image_viewer', anonymous=True)
        self.image_sub = rospy.Subscriber('/object_sorting/image_result', Image, self.callback)
        rospy.loginfo("已订阅 /object_sorting/image_result 图像话题。")
        self.output_path = "/home/ubuntu/pro/test/img/latest.jpg"  # 图像保存路径

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(self.output_path, cv_image)  # 保存为图像文件
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        viewer = ImageViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
