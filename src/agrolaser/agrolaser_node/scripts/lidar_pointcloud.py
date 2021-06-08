#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import tf2_ros as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import time
#import pcl_ros


class LidarPointCloud:
    def __init__(self):
        rospy.init_node('lidar_pointcloud_node')
        self.source_frame = rospy.get_param('~lidar_frame', 'lms400_base')
        self.target_frame = rospy.get_param('~odom_frame', 'odom')
        self.lp = lg.LaserProjection()
        self.pc_pub = rospy.Publisher("/laserMapping/laser_points", PointCloud2, queue_size=2)
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(12))
        self.listener = tf2.TransformListener(self.tf_buffer)

        rospy.Subscriber("/laserMapping/laser_scan", LaserScan, self.scan_cb, queue_size=2)

    def scan_cb(self, msg):

        lookup_time = rospy.Time.now()
        timeout = 1.0
        pc2_msg = self.lp.projectLaser(msg)
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame,  lookup_time,
                                                    rospy.Duration(timeout))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(pc2_msg, trans)
        self.pc_pub.publish(cloud_out)



if __name__ == '__main__':
    lidar_pointcloud = LidarPointCloud()
    rospy.spin()
