#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == '__main__':
    rospy.loginfo("main")

    rospy.loginfo("sample_rosserial_publisher node")
    pub = rospy.Publisher('/test/multi_array_pc2arduino', Float32MultiArray, queue_size=10)
    rospy.init_node('multi_array_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    try:
        while not rospy.is_shutdown():
            arr = Float32MultiArray()

            # 送信するデータ
            # arr.data = [2.0, 6.0, 10.0]                # arduinoの確保したリストよりも短い
            arr.data = [2.0, 4.0, 6.0, 8.0, 10.0]
            # arr.data = [10.0, 9.0, 8.0, 7.0, 6.0, 4.0] # arduinoの確保したリストよりも長い
            pub.publish(arr)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
