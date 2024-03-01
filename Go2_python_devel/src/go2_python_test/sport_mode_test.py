#!/usr/bin/env python
import rospy
from unitree_api.msg import Request
import std_msgs.msg

def publish_sport_request():
    # ROSノードの初期化
    rospy.init_node('sport_request_publisher', anonymous=True)

    # パブリッシャーの作成
    pub = rospy.Publisher('/api/sport/request', Request, queue_size=10)

    # レートの設定（例: 10Hz）
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # メッセージの作成
        msg = Request()
        req.header.identity.api_id = 1004

        # メッセージの公開
        pub.publish(msg)

        # レートに従って待機
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sport_request()
    except rospy.ROSInterruptException:
        pass
