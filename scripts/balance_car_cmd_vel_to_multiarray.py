#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def publish_float_array():
    # ROSノードの初期化
    rospy.init_node('float_array_publisher', anonymous=True)

    # パブリッシャーの作成（std_msgs/Float32MultiArray型のメッセージをpublishする）
    pub = rospy.Publisher('float_array_topic', Float32MultiArray, queue_size=10)

    # ROSのループレート
    rate = rospy.Rate(1)  # 1Hzの周期でループ

    while not rospy.is_shutdown():
        # Float32MultiArrayメッセージを作成
        float_array_msg = Float32MultiArray()
        float_array_msg.data = [1.0, 2.0, 3.0, 4.0]  # 任意のデータを設定（ここではサンプルとして1.0, 2.0, 3.0, 4.0を設定）

        # メッセージをpublish
        pub.publish(float_array_msg)
        rospy.loginfo("Published: {}".format(float_array_msg.data))

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_float_array()
    except rospy.ROSInterruptException:
        pass
