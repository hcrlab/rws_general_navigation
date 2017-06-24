#!/usr/bin/env python
# license removed for brevity
"""Provides a class that interfaces with MongoDB and the robot. When run, acts as a python command line interface."""
import rospy
import actionlib

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy

from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from web_navigation.msg import MapPoseNames, NameList

class Navigator():
    def __init__(self):
        print ("Started init (Navigator)")
        #Latched position topic for keeping web up to date.
        self.Latched_Positions = rospy.Publisher("/web_navigation/latched_position_names", NameList, latch=True, queue_size = 1)
        
        self.last_recieved_position = None
        #Keeps track of last known position
        self.sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.last_position_callback)
        #ActionClient for publishing navigation targets
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #Free to change rws_nav_position to whatever
        self.Mongo_Storer = MessageStoreProxy(database='rws_nav_positions', 
                                              collection='web_positions') 
        self.update_web()
    
    def last_position_callback(self, data):
        self.last_recieved_position = data
    
    def display_current_position(self):
        print(self.last_recieved_position)

    #Makes a PoseStamped with x, y, and w.    
    def make_PoseStamped_xy(self, x, y, w=1.0):
        """Makes a PoseStamped with position x, y, and rotation w."""  
        #Wrong way to construct things - use default MoveBaseGoal() then MoveBaseGoal.target_pose.pose.x = 1...
        h = Header(seq=1, stamp = rospy.Time.now(), frame_id = "map")
        p = Point(float(x), float(y), 0.0)
        q = Quaternion(0.0, 0.0, 0.0, w)
        return PoseStamped(header = h, pose = Pose(position = p, orientation = q))
        
    def make_PoseStamped_from_Pose(self, p):
        """Adds a header to the Pose p."""
        #Wrong way to construct things - use default MoveBaseGoal() then MoveBaseGoal.target_pose.pose.x = 1...
        h = Header(seq=1, stamp = rospy.Time.now(), frame_id = "map")
        return PoseStamped(header = h, pose = p)

    def save_position(self, position, name):
        if self.get_position(name):
            print("Updating position: "+name)
            self.Mongo_Storer.update_named(name, position)   
        else:
            print("Inserting new position: "+name)
            temp_id = self.Mongo_Storer.insert_named(name, position)
            self.Mongo_Storer.update_named(name, position, meta = {"_id" : temp_id})
        self.update_web()

    def get_position(self, name):
        """Returns a PoseStamped saved position"""
        return self.Mongo_Storer.query_named(name, "geometry_msgs/PoseStamped")[0] 
    
    def get_id(self, name):
        """Returns id of position named name in database"""
        print self.Mongo_Storer.query_named(name, "geometry_msgs/PoseStamped")[1]["_id"]
        return self.Mongo_Storer.query_named(name, "geometry_msgs/PoseStamped")[1]["_id"].__str__()
 
    def save_current_position(self, name): 
        """If known, saves the robot's current position to the database"""
        if(self.last_recieved_position != None):
            new_pose = PoseStamped(header = self.last_recieved_position.header, pose = self.last_recieved_position.pose.pose)
            self.save_position(new_pose, name)
        else:
            print("Current position not known! Launch amcl/robot?")
    
    def publish_PoseStamped(self, pose):
        """Sends a navigation goal with pose"""
        print"Publishing pose:",pose
        to_pub = MoveBaseGoal(target_pose = pose)        
        self.goal_client.send_goal(to_pub)

    def publish_saved_position(self, name):
        """Sends a navigation goal with the pose saved as name"""
        to_pub = self.get_position(name)
        if to_pub:
            self.publish_PoseStamped(to_pub)
    
    def publish_position_xy(self, x, y):
        """Sends a navigation goal with the position (x, y)"""
        self.publish_PoseStamped(self.make_PoseStamped_xy(x, y))

    def read_saved_position(self, name):
        """Outputs the stored position named name to console"""
        print self.get_position(name)

    def delete_position(self, name):
        if self.get_position(name):
            print("Deleting: "+name)
            print "ID: ",self.get_id(name) 
            self.Mongo_Storer.delete(self.get_id(name))
            self.update_web()
        else:
            print("'"+name+"' is not in database!")

    def list_positions(self):
        print "Positions:"
        positions = self.Mongo_Storer.query("geometry_msgs/PoseStamped")
        for p in positions:
            print p[1]["name"]+":"
            pos = p[0]
            #           Heh
            print "\t(",pos.pose.position.x,",",pos.pose.position.y,")"

    def update_web(self):
        """Publishes to a latched topic to keep the web interface updated. Called whenever the stored positions are changed."""
        position_list = [p for p, m in self.Mongo_Storer.query("geometry_msgs/PoseStamped")]
        position_name_list = [m["name"] for p, m in self.Mongo_Storer.query("geometry_msgs/PoseStamped")]
        print "Updating latched names..."
        self.Latched_Positions.publish(NameList(position_name_list))

def get_name():
    return raw_input("Name of position: ")
if __name__ == '__main__':
    try:
        test = MapPoseNames()
        rospy.init_node("web_navigation_test")
        n = Navigator()
        while True:
            print("Options:")
            print("\t(D)isplay robots current position")
            print("\t(S)ave robots current position")
            print("\t(R)ead a saved position")
            print("\t(O)utput a saved position to the robot")
            print("\t(P)ublish a position from console to the robot")
            print("\t(X)emove a saved position")
            print("\t(L)ist the saved positions")
            inp = raw_input("Choice:")
            if(inp.upper() == "D"):
                n.display_current_position()
            elif(inp.upper() == "P"):
                #try:
                    x = input("X pos:")
                    y = input("Y pos:")
                    n.publish_position_xy(x, y)
                #except:
                    #print("input is annoying")
            elif(inp.upper() == "S"):
                n.save_current_position(get_name())
            elif(inp.upper() == "R"):
                n.read_saved_position(get_name())
            elif(inp.upper() == "O"):
                n.publish_saved_position(get_name())
            elif(inp.upper() == "X"):
                n.delete_position(get_name())
            elif(inp.upper() == "L"):
                n.list_positions()
            else:
                print("Unknown choice!") 
    except rospy.ROSInterruptException:
        pass
