#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import ros_numpy
import tables
from datetime import datetime
import signal
import os

from agrolaser_node.srv import ProjectService, ProjectServiceResponse


class LidarPointTable(tables.IsDescription):
    x = tables.Float32Col(pos=1)
    y = tables.Float32Col(pos=2)
    z = tables.Float32Col(pos=3)
    intensity = tables.Float32Col(pos=4)
    index = tables.UInt32Col(pos=5)


class ScanTable(tables.IsDescription):
    ts = tables.Time64Col(pos=1)
    ranges = tables.Float32Col(700, pos=2)
    intensities = tables.Float32Col(700, pos=3)


class OdometryTable(tables.IsDescription):
    ts = tables.Time64Col(pos=0)
    tx = tables.Float32Col(pos=1)
    ty = tables.Float32Col(pos=2)
    tz = tables.Float32Col(pos=3)
    qw = tables.Float32Col(pos=4)
    qx = tables.Float32Col(pos=5)
    qy = tables.Float32Col(pos=6)
    qz = tables.Float32Col(pos=7)


class LidarRecorder:
    """
    Records the LiDAR point cloud to HDF5.
    The file are created according to the project name and named with the timestamp.
    To create a new file, publish the project name to /project/name topic as String
    To stop recording, send a single character to /project/name
    """

    def __init__(self, root_folder="/data/h5_files", pc2_topic="/laserMapping/laser_points",
                 scan_topic="/laserMapping/laser_scan", odom_topic="/mavros/odometry/in"):

        self.file_open = False
        self.filename = ""
        self.h5_file = None
        self.point_table = None
        self.scan_table = None
        self.odom_table = None
        self.root_folder = root_folder
        try:
            if not os.path.exists(self.root_folder):
                os.makedirs(self.root_folder)
        except Exception as ex:
            pass
        rospy.Subscriber(pc2_topic, PointCloud2, self.cloud_cb, queue_size=50)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_cb, queue_size=50)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber("/project/name", String, self.project_cb, queue_size=2)
        rospy.init_node('lidar_recorder')
        self.service = rospy.Service('project_service', ProjectService, self.handle_projects)

    def handle_projects(self, req):
        request_string = req.request_string
        project_string = req.project
        project_path = os.path.join(self.root_folder, project_string)
        resp = ProjectServiceResponse()
        print(req)
        if request_string == "list":
            try:
                dirs = next(os.walk(self.root_folder))[1]
                resp.list_strings = dirs
                resp.response_string = "OK"
            except Exception as ex:
                resp.response_string = "error"
        elif request_string == "create":
            print("Creating: " + project_path)
            try:
                if not os.path.exists(project_path):
                    os.makedirs(project_path)
                    resp.response_string = "OK"
                else:
                    resp.response_string = "exists"
            except Exception as ex:
                resp.response_string = "error"
        elif request_string == "start":
            try:
                if not os.path.exists(project_path):
                    os.makedirs(project_path)
                self.create_new_file(project_string)
                resp.response_string = "OK"
            except Exception as ex:
                resp.response_string = "error"
        elif request_string == "stop":
            try:
                self.shutdown_node()
                resp.response_string = "OK"
            except Exception as ex:
                resp.response_string = "error"
        return resp

    def create_new_file(self, project_name):
        self.filename = "{project}_{ts}.h5".format(project=project_name,
                                                   ts=datetime.now().strftime("%Y%m%d_%H%M%S"))
        try:
            if not os.path.exists(os.path.join(self.root_folder, project_name)):
                os.makedirs(os.path.join(self.root_folder, project_name))
        except Exception as ex:
            pass
        self.filename = os.path.join(self.root_folder, project_name, self.filename)
        filters = tables.Filters(complevel=1, complib='blosc:lz4', fletcher32=True)
        self.h5_file = tables.open_file(self.filename, mode='w', filters=filters)
        self.point_table = self.h5_file.create_table(self.h5_file.root, 'points', LidarPointTable, "PointCloud")
        self.scan_table = self.h5_file.create_table(self.h5_file.root, 'scans', ScanTable, "Scans")
        self.odom_table = self.h5_file.create_table(self.h5_file.root, 'odom', OdometryTable, "Odometry")
        self.file_open = True
        rospy.loginfo("Recording point cloud to {}".format(self.filename))

    def cloud_cb(self, pc2_msg):
        if self.file_open:
            point_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
            self.point_table.append(point_array)

    def scan_cb(self, scan_msg):
        if self.file_open:
            self.scan_table.append([(scan_msg.header.stamp.to_nsec(), scan_msg.ranges, scan_msg.intensities)])

    def odom_cb(self, odom_msg):
        if self.file_open:
            self.odom_table.append([(odom_msg.header.stamp.to_nsec(),
                                     odom_msg.pose.pose.position.x,
                                     odom_msg.pose.pose.position.y,
                                     odom_msg.pose.pose.position.z,
                                     odom_msg.pose.pose.orientation.w,
                                     odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z)])

    def project_cb(self, string_msg):
        if len(string_msg.data) > 1:
            self.create_new_file(string_msg.data)
        else:
            self.shutdown_node()

    def shutdown_node(self):
        if self.file_open:
            rospy.loginfo("Stopped recording point cloud to {}".format(self.filename))
            self.h5_file.flush()
            self.h5_file.close()
            self.file_open = False


def close_files():
    print("Control+C detected!")
    lidar_csv.shutdown_node()


if __name__ == '__main__':
    lidar_csv = LidarRecorder()
    rospy.spin()
    rospy.on_shutdown(close_files)
    signal.signal(signal.SIGINT, close_files)
