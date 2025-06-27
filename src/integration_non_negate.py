#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import pyproj
import math
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class FakeNavsatTransform:
    def __init__(self):
        rospy.init_node("antobot_aprilSlam_integration")

        self.utm_origin_set = False
        self.utm_easting = 0.0
        self.utm_northing = 0.0
        self.utm_zone = ''
        self.is_northern = True
        self.yaw_offset_rad = None  # Will be set from IMU
        self.imu_yaw_ready = False
        self.map_enu_heading = None

        self.map_frame = "map"
        self.utm_frame = "utm"
        self.base_link_frame = "base_link"

        self.tf_listener = tf.TransformListener()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.imu_sub = rospy.Subscriber("/imu/data_corrected", Imu, self.imu_callback, queue_size=1)
        self.gps_sub = rospy.Subscriber("/antobot_gps", NavSatFix, self.gps_callback, queue_size=1)
        self.gps_filtered_pub = rospy.Publisher("/gps/filtered", NavSatFix, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("antobot_aprilSlam_integration node started...")
        
    def normalize_angle(self, angle_rad):
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def imu_callback(self, msg):
        if self.imu_yaw_ready:
            return
        orientation_q = msg.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        
        self.yaw_offset_rad = self.normalize_angle(yaw)


        raw_yaw_deg = math.degrees(yaw)
        rospy.loginfo(f"IMU yaw (raw): {raw_yaw_deg:.2f}°, used as offset.")

        self.imu_yaw_ready = True

    def gps_callback(self, msg):
        if self.utm_origin_set or not self.imu_yaw_ready:
            return

        # Convert GPS to UTM
        self.utm_zone = self.get_utm_zone(msg.longitude)
        self.is_northern = msg.latitude >= 0
        self.utm_proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=not self.is_northern)
        utm_easting, utm_northing = self.utm_proj(msg.longitude, msg.latitude)

        # Correct for IMU yaw
        cos_yaw = math.cos(self.yaw_offset_rad)
        sin_yaw = math.sin(self.yaw_offset_rad)

        # Correct for sensor offset (0.5m forward from base_link)
        gps_offset_robot_frame = [0.5, 0.0]
        offset_x = cos_yaw * gps_offset_robot_frame[0]
        offset_y = sin_yaw * gps_offset_robot_frame[0]
        utm_easting -= offset_x
        utm_northing -= offset_y

        # Get current base_link pose in map
        now = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_link_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_link_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get map→base_link transform during GPS initialization.")
            return
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(rot)

        # Yaw is the rotation around Z axis in radians
        yaw_deg = math.degrees(yaw)

        rospy.loginfo(f"Yaw between {self.map_frame} → {self.base_link_frame}: {yaw_deg:.2f}°")

        # Rotate map→base_link translation into ENU
        x = trans[0]
        y = trans[1]
        delta_yaw = self.normalize_angle(self.yaw_offset_rad - yaw)
        cos_yaw_n = math.cos(delta_yaw)
        sin_yaw_n = math.sin(delta_yaw)
        rotated_x = cos_yaw_n * x - sin_yaw_n * y
        rotated_y = sin_yaw_n * x + cos_yaw_n * y

        # Set corrected origin
        self.utm_easting = utm_easting - rotated_x
        self.utm_northing = utm_northing - rotated_y

        rospy.loginfo(f"UTM origin aligned at: easting={self.utm_easting}, northing={self.utm_northing}, zone={self.utm_zone}")
        rospy.loginfo(f"Initial yaw from IMU: {math.degrees(self.yaw_offset_rad):.2f}°")

        self.broadcast_static_tf(yaw)
        self.utm_origin_set = True
        self.gps_sub.unregister()

    def broadcast_static_tf(self,yaw_base_map):
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = self.utm_frame
        static_tf.child_frame_id = self.map_frame

        static_tf.transform.translation.x = -self.utm_easting
        static_tf.transform.translation.y = -self.utm_northing
        static_tf.transform.translation.z = 0.0

        #quat = quaternion_from_euler(0, 0, self.yaw_offset_rad-yaw)
        delta_yaw = self.normalize_angle(self.yaw_offset_rad - yaw_base_map)
        self.map_enu_heading = delta_yaw
        quat = quaternion_from_euler(0, 0, delta_yaw)
        rospy.loginfo(f"TF heading utm->map: {math.degrees(delta_yaw):.2f}°")
        static_tf.transform.rotation.x = quat[0]
        static_tf.transform.rotation.y = quat[1]
        static_tf.transform.rotation.z = quat[2]
        static_tf.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(static_tf)
        rospy.loginfo("Published static transform utm → map")

    def get_utm_zone(self, lon):
        return int((lon + 180) / 6) + 1

    def timer_callback(self, event):
        if not self.utm_origin_set:
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_link_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "Waiting for transform map → base_link")
            return

        # Rotate base_link position back into UTM
        x = trans[0]
        y = trans[1]
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(rot)
        print(self.map_enu_heading, yaw,self.yaw_offset_rad)
        # Yaw is the rotation around Z axis in radians
        yaw_deg = math.degrees(yaw)
        print(x,y)
        
        cos_yaw = math.cos(self.map_enu_heading)
        sin_yaw = math.sin(self.map_enu_heading)

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
