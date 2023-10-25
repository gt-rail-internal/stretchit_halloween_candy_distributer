#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def transform_listener():
    rospy.init_node('transform_listener', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)  # Set the rate to 10 Hz
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            transform = tf_buffer.lookup_transform('base_link', 'candy3', now, rospy.Duration(1.0))
            rospy.loginfo('Translation: %s\nRotation: %s' % (transform.transform.translation, transform.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not get transform from 'base_link' to 'candy3': %s" % e)
            continue

        rate.sleep()

if __name__ == '__main__':
    transform_listener()
