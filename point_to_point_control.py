import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion


import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import getch
#import webots
#from webots.msg import frame
import tf
import tf.transformations as tft
import tf2_ros
from tf2_msgs.msg import TFMessage

class mysub:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        #self.sub = rospy.Subscriber('frames', frame, self.call_frame)
        #self.tfsub = rospy.Subscriber('tf', TFMessage, self.call_tf)
        self.listener = tf.TransformListener()
        
        # mode between moving and rotating, moving is 0
        self.mode = 0
        
        # array of waypoints
        self.windex = 0; # keeps track of which point we are going to
        
        # og world: traj 1
        self.xway = [4,-3,-3,2,2,-0.5,-0.5]
        self.yway = [2.5,2.5,-3.5,-3.5,0.5,0.5,-3.5]
        self.thetaway = [1.57,3.14,-1.57,0,1.57,3.14,-1.57,0]
        
        # goal location and orientation
        self.x_goal = self.xway[self.windex]
        self.y_goal = self.yway[self.windex]
        self.theta_goal = self.thetaway[1]
        self.theta_curr = self.thetaway[0]
        
        #self.theta_goal = 3.14
        #self.theta_curr = 1.57 #keep heading
        self.theta_error = 0;
        
        # initialize position and orientation
        self.x = 0
        self.y = 0
        self.theta = 0
       


    def call_frame(self,data):
        self.pose = data.pose
        #position = data.pose.position
        #orientation = data.pose.orientation
        
        
    def create_twist(self,vel):
        #create a twist message from a velocity vector
        move_cmd = Twist()
        move_cmd.linear.x = vel[0]
        move_cmd.linear.y = vel[1]
        move_cmd.linear.z = vel[2]

        move_cmd.angular.x = vel[3]
        move_cmd.angular.y = vel[4]
        move_cmd.angular.z = vel[5]
    
        return move_cmd

    def run(self):
        #sub_pose = rospy.Subscriber('tf', Pose, pose)
        print("run")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/world', '/imu', rospy.Time(0))
                #link_names = self.listener.getFrameStrings()
                
                self.x = trans[0]
                self.y = trans[1]
                euler = tft.euler_from_quaternion(rot)
                self.theta = euler[2]
                
                print("theta ", self.theta)
                print("x ", self.x)
                print("y ", self.y)
                #print("done")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("error")
            
         
            #self.sub = rospy.Subscriber('frames', frame, self.call_frame)
            
            print("x_goal ", self.x_goal)
            print("y_goal ", self.y_goal)
            print("theta_goal (this is for upcoming rotation or current) ", self.theta_goal)
            print("desired heading ", self.theta_curr)

            if self.mode == 0: #going forward
                print("go straight")
                diff = [self.x - self.x_goal,self.y - self.y_goal] 
                #print("diff ", diff)
                #print("norm")
                print(np.linalg.norm(diff))
                if np.linalg.norm(diff) > 0.1: # go straight
                    self.theta_error = self.theta - self.theta_curr;
                    print("theta error ", self.theta_error)
                    msg = self.create_twist([6,0,0,0,0,20*(self.theta - self.theta_curr)]);
                    self.pub.publish(msg)   
                else: #stop and call rotate state
                    print("done going straight")
                    self.mode = 1
                    msg = self.create_twist([0,0,0,0,0,0]) 
                    self.pub.publish(msg)      
                             
            if self.mode == 1: #we are rotating
                if np.linalg.norm([self.theta - self.theta_goal]) > 0.1: #rotate
                    print("rotate")
                    msg = self.create_twist([0,0,0,0,0,-30])
                    self.pub.publish(msg)   
                else: #stop rotating and call straight state
                    print("stop rotating")
                    self.mode = 0
                    
                    if self.windex == 6:
                        self.mode = 2
                    else:
                        self.windex += 1
                        
                    self.x_goal = self.xway[self.windex]
                    self.y_goal = self.yway[self.windex]
                    self.theta_goal = self.thetaway[self.windex+1]
                    self.theta_curr = self.thetaway[self.windex]
                    
                    #if (self.theta_goal == 3.14):
                    #    print("here cause theta_goal was 3.14")
                    #    self.theta_goal = -1.57
                    #else:
                    #    print("change theta goal")
                    #    self.theta_goal += 1.57
                    #    
                    #if (self.theta_curr == 3.14):
                    #    self.theta_curr = -1.57
                    #else:
                    #    self.theta_curr += 1.57
                        
                    msg = self.create_twist([0,0,0,0,0,0])
                    self.pub.publish(msg)   
                    
            if self.mode == 2: #idle
                msg = self.create_twist([0,0,0,0,0,0])
                self.pub.publish(msg)  
                    
            rate.sleep()


if __name__ == "__main__":

    node = mysub()
    node.run()
    
