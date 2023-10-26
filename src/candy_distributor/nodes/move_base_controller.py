# ros node to have a service that takes a linear displacement from the base_link to the candy3 frame and oputs cmd_vel to move the robot to that position
import rospy
from geometry_msgs.msg import Twist
from candy_distributor.srv import move_base_srv, move_base_srvResponse
from tf2_ros import TransformListener, Buffer

class MoveBaseController:
    def __init__(self) -> None:
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.move_base_service = rospy.Service('move_base', move_base_srv, self.move_base)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.kp = 0.5


    def move_base(self, req):
        # des_pos = req.distance
        rospy.loginfo("Moving to candy3")
        while(True):
            # get trasnform from base_link to candy3
            try:
                transform = self.tf_buffer.lookup_transform('base_link', 'candy3', rospy.Time(0), rospy.Duration(1.0))
            except Exception as e:
                rospy.logwarn("Could not get transform from 'base_link' to 'candy3': %s" % e)
                continue
            # get the distance from the transform
            des_pos = transform.transform.translation.x
            # if the distance is less than 0.1m then stop
            if des_pos < 0.01:
                rospy.loginfo("des_pos: %s" % des_pos)
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                self.cmd_vel_pub.publish(cmd_vel)
                break
            # else publish a cmd_vel to move the robot
            cmd_vel = Twist()
            cmd_vel.linear.x = self.kp * des_pos
            rospy.loginfo("cmd_vel: %s" % cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("Reached candy3")
        return move_base_srvResponse(False)
    
def main():
    rospy.init_node('move_base_controller', anonymous=True)
    move_base_controller = MoveBaseController()
    rospy.spin()

if __name__ == '__main__':
    main()

