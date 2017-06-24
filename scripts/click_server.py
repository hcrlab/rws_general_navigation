#!/usr/bin/env python
# license removed for brevity
import rospy
import position_manager

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String

class Click_Server():
    Go_To_Name_Sub    = None
    Go_To_Pose_Sub    = None
    Delete_Name_Sub   = None
    Add_Pose_Sub      = None
    Save_Current_Sub  = None 
    Manager = None
    Go_To_XY_Sub = None
    def __init__(self):
        print("Started init (Clicker)")
        self.Go_To_Name_Sub  = rospy.Subscriber("web_navigation_click/go_to_name", String, self.go_to_name)
        self.Go_To_Pose_Sub  = rospy.Subscriber("web_navigation_click/go_to_pose", Pose, self.go_to_pose) 
        self.Delete_Name_Sub = rospy.Subscriber("web_navigation_click/delete_name", String, self.delete_name)
        #Placeholder Sub. Need to create message type with Name AND Pose
        self.Add_Pose_Sub    = rospy.Subscriber("web_navigation_click/add_pose", String, self.add_pose)
        #This might be redundant with Add_Pose_Sub, since web navigation can just publish current position as a pose,
        #but might be more convenient to do this, and lets me debug 
        self.Save_Current_Sub = rospy.Subscriber("web_navigation_click/save_current", String, self.save_current)
        self.Go_To_XY_Sub     = rospy.Subscriber("web_navigation_click/go_to_point" , Point,  self.go_to_point)
        print "Made Subscribers"
        self.Manager = position_manager.Navigator()  
        
    def go_to_name(self, msg):
        print "(Clicker) Go to name: "+msg.data
        self.Manager.publish_saved_position(msg.data)

    def go_to_pose(self, msg):
        print "(Clicker) Go to pose:", msg
        self.Manager.publish_PoseStamped(self.Manager.make_PoseStamped_from_Pose(msg.data))

    def delete_name(self, msg):
        print "(Clicker) Delete name: "+msg.data
        self.Manager.delete_position(msg.data)

    def add_pose(self, msg):
        print "[TODO] (Clicker) Add pose with name:", msg
        #Placeholder action. Need to create message type with Name AND Pose
        self.Manager.save_position(self.Manager.make_PoseStamped_xy(0, 0), msg.data)

    def save_current(self, msg):
        print "(Clicker) Saving current pose with name:", msg
        self.Manager.save_current_position(msg.data)

    def go_to_point(self, msg):
        print "(Clicker) Going to point: (",msg.x,",",msg.y,")"
        self.Manager.publish_position_xy(msg.x, msg.y)

if __name__ == '__main__':
    try:
        rospy.init_node("web_navigation_click_server")
        wonderfully_descriptive_name = Click_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
