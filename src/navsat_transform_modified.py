#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import pyproj
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class FakeNavsatTransform:
    def __init__(self):
        rospy.init_node("antobot_aprilSlam_integration")

        self.utm_origin_set = False
        self.utm_easting = 0.0
        self.utm_northing = 0.0
        self.utm_zone = ''
        self.is_northern = True

        # Set yaw offset in degrees (robot's heading in ENU frame)
        self.yaw_offset_deg = 98.5  # <-- Change this to match your IMU heading at startup = 95 for demo bag
        self.yaw_offset_rad = math.radians(self.yaw_offset_deg)

        self.map_frame = "map"
        self.utm_frame = "utm"
        self.base_link_frame = "base_link"

        self.tf_listener = tf.TransformListener()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.gps_sub = rospy.Subscriber("/antobot_gps", NavSatFix, self.gps_callback, queue_size=1)
        self.gps_filtered_pub = rospy.Publisher("/gps/filtered", NavSatFix, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("fake_navsat_transform node started...")

    def gps_callback(self, msg):
        if self.utm_origin_set:
            return

        # Convert GPS to UTM
        self.utm_zone = self.get_utm_zone(msg.longitude)
        self.is_northern = msg.latitude >= 0
        self.utm_proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=not self.is_northern)
        utm_easting, utm_northing = self.utm_proj(msg.longitude, msg.latitude)

        now = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_link_frame, now, rospy.Duration(1.0))
            (trans, _) = self.tf_listener.lookupTransform(self.map_frame, self.base_link_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get map→base_link transform during GPS initialization.")
            return

        # To align base_link with GPS after yaw rotation,
        # we must rotate base_link's offset into yaw-aligned map frame (i.e., apply inverse rotation)
        cos_yaw = math.cos(-self.yaw_offset_rad)
        sin_yaw = math.sin(-self.yaw_offset_rad)

        x = trans[0]
        y = trans[1]

        rotated_x = cos_yaw * x - sin_yaw * y
        rotated_y = sin_yaw * x + cos_yaw * y

        # Then offset UTM by that rotated base_link position
        self.utm_easting = utm_easting - rotated_x
        self.utm_northing = utm_northing - rotated_y

        rospy.loginfo(f"UTM origin aligned at: easting={self.utm_easting}, northing={self.utm_northing}, zone={self.utm_zone}")
        self.broadcast_static_tf()
        self.utm_origin_set = True
        self.gps_sub.unregister()

    def broadcast_static_tf(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = self.utm_frame
        static_tf.child_frame_id = self.map_frame

        static_tf.transform.translation.x = -self.utm_easting
        static_tf.transform.translation.y = -self.utm_northing
        static_tf.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.yaw_offset_rad)
        static_tf.transform.rotation.x = quat[0]
        static_tf.transform.rotation.y = quat[1]
        static_tf.transform.rotation.z = quat[2]
        static_tf.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(static_tf)
        rospy.loginfo(f"Published static transform utm → map with yaw offset: {self.yaw_offset_deg}°")

    def get_utm_zone(self, lon):
        return int((lon + 180) / 6) + 1

    def timer_callback(self, event):
        if not self.utm_origin_set:
            return

        try:
            (trans, _) = self.tf_listener.lookupTransform(self.map_frame, self.base_link_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "Waiting for transform map → base_link")
            return

        # Rotate base_link position back into UTM
        x = trans[0]
        y = trans[1]
        cos_yaw = math.cos(-self.yaw_offset_rad)
        sin_yaw = math.sin(-self.yaw_offset_rad)

        rotated_x = cos_yaw * x - sin_yaw * y
        rotated_y = sin_yaw * x + cos_yaw * y

        easting = self.utm_easting + rotated_x
        northing = self.utm_northing + rotated_y

        # Convert back to lat/lon
        lon, lat = self.utm_proj(easting, northing, inverse=True)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = self.utm_frame
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = trans[2]
        gps_msg.status.status = 0
        gps_msg.status.service = 1

        self.gps_filtered_pub.publish(gps_msg)

if __name__ == '__main__':
    try:
        FakeNavsatTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
