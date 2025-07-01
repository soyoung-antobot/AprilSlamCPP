#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from pyproj import Transformer, CRS
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
import math
from math import radians, sin, cos, sqrt, atan2
import numpy as np

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = radians(lat1), radians(lat2)
    d_phi = radians(lat2 - lat1)
    d_lambda = radians(lon2 - lon1)

    a = sin(d_phi/2)**2 + cos(phi1)*cos(phi2)*sin(d_lambda/2)**2
    c = 2*atan2(sqrt(a), sqrt(1-a))
    return R * c

def rotate_point(x, y, theta):
    """Rotate point (x, y) by theta radians."""
    c, s = np.cos(theta), np.sin(theta)
    x_new = c * x - s * y
    y_new = s * x + c * y
    return x_new, y_new


class StaticUTMToMapBroadcaster:
    def __init__(self):
        rospy.init_node('static_utm_to_map_broadcaster')

        rospy.Subscriber('/antobot_gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu/data_corrected', Imu, self.imu_callback)
        self.gps_filtered_pub = rospy.Publisher("/gps/filtered", NavSatFix, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        self.utm_coords = None
        self.imu_yaw = None
        self.transform_sent = False

        self.transformer_to_utm = None
        self.transformer_to_geo = None
        self.epsg_code = None
        self.utm_to_map_transform = None

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = yaw

    def gps_callback(self, msg):
        if msg.status.status < 0:
            return  # Ignore invalid GPS

        lat = msg.latitude
        lon = msg.longitude

        if self.transformer_to_utm is None:
            zone_number = int((lon + 180) / 6) + 1
            is_northern = lat >= 0
            self.epsg_code = 32600 + zone_number if is_northern else 32700 + zone_number

            utm_crs = CRS.from_epsg(self.epsg_code)
            self.transformer_to_utm = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
            self.transformer_to_geo = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
            rospy.loginfo(f"Initialized UTM transformer with EPSG:{self.epsg_code}")

        easting, northing = self.transformer_to_utm.transform(lon, lat)
        self.utm_coords = (easting, northing)

        self.try_publish_static_transform()

        if self.transform_sent:
            try:
                tf_map_base = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
                robot_map_x = tf_map_base.transform.translation.x
                robot_map_y = tf_map_base.transform.translation.y
                rospy.loginfo(f"Robot pose in map: x={robot_map_x:.3f}, y={robot_map_y:.3f}")

                # Extract static utm -> map transform
                tf_utm_map = self.utm_to_map_transform
                tx = tf_utm_map.transform.translation.x
                ty = tf_utm_map.transform.translation.y
                q = tf_utm_map.transform.rotation
                _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

                # Rotate the map position into the UTM frame
                rotated_x, rotated_y = rotate_point(robot_map_x, robot_map_y, yaw)

                # Then add translation to get full base_link UTM position
                utm_origin_x = self.utm_to_map_transform.transform.translation.x
                utm_origin_y = self.utm_to_map_transform.transform.translation.y

                base_utm_x = utm_origin_x + rotated_x
                base_utm_y = utm_origin_y + rotated_y


                lon_dbg, lat_dbg = self.transformer_to_geo.transform(base_utm_x, base_utm_y)
                rospy.loginfo(f"[DEBUG] Updated GPS Fix: ({lat:.8f}, {lon:.8f}) | Computed base_link UTM: ({base_utm_x:.2f}, {base_utm_y:.2f}) -> ({lat_dbg:.8f}, {lon_dbg:.8f})")

                dist = haversine(lat, lon, lat_dbg, lon_dbg)
                rospy.loginfo(f"[DEBUG] Distance between GPS and base_link positions: {dist:.2f} meters")

                # Publish filtered GPS
                filtered_msg = NavSatFix()
                filtered_msg.header.stamp = rospy.Time.now()
                filtered_msg.header.frame_id = "base_link"
                filtered_msg.latitude = lat_dbg
                filtered_msg.longitude = lon_dbg
                filtered_msg.status.status = msg.status.status
                filtered_msg.status.service = msg.status.service
                filtered_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.gps_filtered_pub.publish(filtered_msg)

            except Exception as e:
                rospy.logwarn(f"Could not compute UTM base_link for debugging/publishing: {e}")

    def try_publish_static_transform(self):
        if self.transform_sent or self.utm_coords is None or self.imu_yaw is None:
            return

        try:
            tf_map_base = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return

        # Robot's current pose in map frame
        robot_map_x = tf_map_base.transform.translation.x
        robot_map_y = tf_map_base.transform.translation.y
        rot = tf_map_base.transform.rotation
        quat = [rot.x, rot.y, rot.z, rot.w]
        roll, pitch, yaw_in_map = tf.transformations.euler_from_quaternion(quat)

        rospy.loginfo(f"[DEBUG] base_link heading in map: {math.degrees(yaw_in_map):.2f} degrees")

        print("------------------------")

        # Correct yaw: the difference between UTM (IMU) heading and map frame heading
        yaw_offset = self.imu_yaw - yaw_in_map  #

        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)

        # Get GPS UTM
        gps_utm_x, gps_utm_y = self.utm_coords

        # Rotate robot's position in map frame into UTM frame
        dx = cos_yaw * robot_map_x - sin_yaw * robot_map_y
        dy = sin_yaw * robot_map_x + cos_yaw * robot_map_y

        map_origin_utm_x = gps_utm_x - dx
        map_origin_utm_y = gps_utm_y - dy

        # Apply the yaw offset for map frame
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw_offset)


        # Prepare static transform
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "utm"
        t.child_frame_id = "map"
        t.transform.translation.x = map_origin_utm_x
        t.transform.translation.y = map_origin_utm_y
        t.transform.translation.z = 0.0

        #quat = tf.transformations.quaternion_from_euler(0, 0, self.imu_yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.static_br.sendTransform(t)
        self.utm_to_map_transform = t
        self.transform_sent = True

        rospy.loginfo(f"Published static utm->map transform with yaw offset {math.degrees(self.imu_yaw):.3f} degrees")
        rospy.loginfo(f"Map origin in UTM: ({map_origin_utm_x:.2f}, {map_origin_utm_y:.2f})")

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


if __name__ == '__main__':
    try:
        node = StaticUTMToMapBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
