#! /usr/bin/env python3

import rospy

class Utils(object):

    @staticmethod
    def ros_time_to_ns(time):
        k_s_to_ns = 1e9
        return time.secs * k_s_to_ns + time.nsecs

    @staticmethod
    def ts_ns_to_ros_time(ts_ns):
      k_ns_per_s = 1e9;
      ros_timestamp_sec = ts_ns / k_ns_per_s;
      ros_timestamp_nsec = ts_ns - (ros_timestamp_sec * k_ns_per_s);
      return rospy.Time(ros_timestamp_sec, ros_timestamp_nsec)
