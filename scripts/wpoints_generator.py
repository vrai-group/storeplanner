#!/usr/bin/env python

import rospy
import rospkg
import tf
import sys
import argparse
import actionlib
import time 
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo
import message_filters
from utils import Utils
import visual_tests
from cameras import CamerasParams

class trajectoryGenerator:

    def __init__(self, args, debug_mode):

        self.task_num = args[1]
        self.store_name = args[2]
        self.num_trajectory = args[3]
        self.map_name = args[4]

        debug_mode = bool()
        if args[5] == 'True' or args[5] == 'true':
            debug_mode = True
        elif args[5] == 'False' or args[5] == 'false':
            debug_mode = False

        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path('storeplanner') + '/'

        self.utils = Utils(self.store_name, self.num_trajectory, self.map_name)
        self.bridge = CvBridge()

        #creates folders needed to store the final acquisitions
        self.utils.dir_exists( self.base_path + 'acquisitions/' + self.store_name + '/' + self.num_trajectory)

        #shelves
        self.utils.parse_shelves(self.base_path + 'models/' + self.store_name + '/shelves/shelves.json')
        self.shelves = self.utils.get_shelves()

        #customer trajectory
        self.full_trajectory = self.utils.load_trajectory() #TODO make rospack only used here

        self.map_shift = rospy.get_param("map_shift")
       
        self.robot_radius = rospy.get_param("robot_radius")
        self.patch_sz = 30 #based on the robot radius (must be divisible by 2)
        self.patch_len = 10 #(must be divisible by 2)
        
        self.px_robot_radius = self.utils.meters2pixels(self.robot_radius,0)[0]

        #repulsive areas
        self.utils.calculate_repulsive_areas(self.patch_sz)
        self.repulsive_areas = self.utils.get_repulsive_areas()

        #crossroads
        self.crossroads = self.utils.parse_crossroads(self.base_path + 'models/' + self.store_name + '/shelves/crossroads.json')
        self.crossroads = self.utils.get_crossroads()

        self.goal_cnt = 0
       
        if debug_mode == True :
            
            #performing all the steps to send goal but just for visualization except the map shift
            self.utils.set_map_image(self.task_num) #to check if the given goal is good 
            map_image = self.utils.get_map_image()
            check_img = self.utils.gray2bgr(map_image)
            img = self.utils.gray2bgr(map_image)

            #shelves check
            for shelf in self.shelves:
                # if shelf.id == "8" or shelf.id == "1":
                check_img = visual_tests.draw_shelf(img,shelf)
            #repulsive areas check
            for rep_area in self.repulsive_areas:
                # if rep_area.id == "8" or rep_area.id == "1":
                check_img = visual_tests.draw_roi(img,rep_area)

            #crossroads check
            for crossroad in self.crossroads:
                check_img = visual_tests.draw_point(check_img,crossroad,(0,0,255),5)
            
            for position in self.full_trajectory:
                i,j = self.utils.map2image(position[0],position[1])
                query_area = self.utils.find_query_shelf((i,j)) #getting rep area, not shelf
                #since we have a query shelf for each desired position, we can associate a dictionary
                #with key the goal counter (append 0,0 if query_area i s 0). This is TODO, for now we iterate
                #over all of them (does not affect planner since offline)
                if query_area.x != 0:
                    # check_img = visual_tests.draw_arrow(check_img,(i,j),query_area.center,(0,255,0))
                    
                    if self.utils.is_inside_a_repulsive_area(self.repulsive_areas,(i,j)):
                        valid_waypoint = self.utils.push_waypoint((i,j),query_area,self.patch_sz)
                        object_position = (i,j)
                        # check_img = visual_tests.draw_point(check_img,valid_waypoint,(0,255,0))
                        # check_img = visual_tests.draw_point(check_img,object_position,(255,0,0))
                    else:
                        valid_waypoint = (i,j)
                        object_position = query_area.center
                        # check_img = visual_tests.draw_point(check_img,valid_waypoint,(0,255,0))
                        # check_img = visual_tests.draw_point(check_img,object_position,(255,0,0))
               
                    # check_img = visual_tests.draw_arrow(check_img,valid_waypoint,object_position,(0,0,255))
            self.utils.show_img_and_wait_key("Shelves, Rep. Area and Crossroads", check_img) 

            self.utils.save_image(self.base_path + '/visual_map_check.jpg', check_img)
            #The drawings are the following:
            # at each waypoint of the trajectory we have a blue circle representing the robot size
            # a yellow square showing the area on which we ask if the waypoint is good or needs to be refined
            # arrow (for now only pointing at 0 degrees) to show the robot heading
            #if the point is bad, additional squares and arrows are shown
            goal_cnt = 0
            for position in self.full_trajectory:
                i,j = self.utils.map2image(position[0],position[1])
                   
                new_img_pt = self.utils.find_feasible_point((i,j),self.patch_sz,self.patch_len,goal_cnt)
                robot_pose, object_pose = self.utils.get_object_direction()
                
                img = visual_tests.draw_patch(img,new_img_pt,self.patch_sz,(0,255,0))
                # img = visual_tests.draw_robot(img,new_img_pt,self.px_robot_radius,(255,0,0))
                # img = visual_tests.draw_arrow(img,new_img_pt,(new_img_pt[0]+25,new_img_pt[1]),(0,255,0))
                #this happens if shelves are too far
                if robot_pose != 0 and object_pose != 0 :
                    img = visual_tests.draw_arrow(img,robot_pose,object_pose,(255,0,0))
                if( i!=new_img_pt[0] or j != new_img_pt[1] ):
                    # img = visual_tests.draw_patch(img,(i,j),self.patch_sz,(0,0,255))
                    # img = visual_tests.draw_robot(img,(i,j),self.px_robot_radius,(255,0,0))
                    # img = visual_tests.draw_arrow(img,(i,j),(i+25,j),(0,0,255))
                    if robot_pose != 0 and object_pose != 0 :
                        img = visual_tests.draw_arrow(img,robot_pose,object_pose,(255,0,0))
                         
                goal_cnt = goal_cnt + 1
                # self.utils.show_img_and_wait_key("recursive", img) 
            
            self.utils.show_img_and_wait_key("Patches",img)
            self.utils.save_image(self.base_path + '/patches.jpg', img)
            # visual_tests.show_good_bad_points(self.full_trajectory, int(self.map_source))
            # visual_tests.check_traj_correspondences(self.full_trajectory, filename, int(self.map_source) ) #TO REVIEW (occhio a quando moltiplico i ratio, da togliere)
            # visual_tests.check_feasibility() #stessa cosa della funzione sopra

            sys.exit(0)
    
        #ROS interface
        #subscribers
        #get cameras info
        #TODO parametrize them into a .yaml file
        self.tl_cam_rgb_info = rospy.wait_for_message("/tl_rgbd_camera/rgb/camera_info", CameraInfo)
        self.tr_cam_rgb_info = rospy.wait_for_message("/tr_rgbd_camera/rgb/camera_info", CameraInfo)
        self.ml_cam_rgb_info = rospy.wait_for_message("/ml_rgbd_camera/rgb/camera_info", CameraInfo)
        self.mr_cam_rgb_info = rospy.wait_for_message("/mr_rgbd_camera/rgb/camera_info", CameraInfo)
        self.bl_cam_rgb_info = rospy.wait_for_message("/bl_rgbd_camera/rgb/camera_info", CameraInfo)
        self.br_cam_rgb_info = rospy.wait_for_message("/br_rgbd_camera/rgb/camera_info", CameraInfo)
    
        # self.tl_rgb_img_sub = rospy.Subscriber("/tl_rgbd_camera/rgb/image_raw",Image, self.tl_rgb_img_cb)
        # self.tr_rgb_img_sub = rospy.Subscriber("/tr_rgbd_camera/rgb/image_raw",Image, self.tr_rgb_img_cb)
        # self.ml_rgb_img_sub = rospy.Subscriber("/ml_rgbd_camera/rgb/image_raw",Image, self.ml_rgb_img_cb)
        # self.mr_rgb_img_sub = rospy.Subscriber("/mr_rgbd_camera/rgb/image_raw",Image, self.mr_rgb_img_cb)
        # self.bl_rgb_img_sub = rospy.Subscriber("/bl_rgbd_camera/rgb/image_raw",Image, self.bl_rgb_img_cb)
        # self.br_rgb_img_sub = rospy.Subscriber("/br_rgbd_camera/rgb/image_raw",Image, self.br_rgb_img_cb)

        #synchronize cameras image by message_filters 
        self.tl_rgb_img_sub = message_filters.Subscriber("/tl_rgbd_camera/rgb/image_raw",Image)
        self.tr_rgb_img_sub = message_filters.Subscriber("/tr_rgbd_camera/rgb/image_raw",Image)
        self.ml_rgb_img_sub = message_filters.Subscriber("/ml_rgbd_camera/rgb/image_raw",Image)
        self.mr_rgb_img_sub = message_filters.Subscriber("/mr_rgbd_camera/rgb/image_raw",Image)
        self.bl_rgb_img_sub = message_filters.Subscriber("/bl_rgbd_camera/rgb/image_raw",Image)
        self.br_rgb_img_sub = message_filters.Subscriber("/br_rgbd_camera/rgb/image_raw",Image)

        self.ts_left_cameras = message_filters.TimeSynchronizer([self.tl_rgb_img_sub, self.ml_rgb_img_sub, self.bl_rgb_img_sub], 10)
        self.ts_left_cameras.registerCallback(self.left_cameras_cb)
        self.ts_right_cameras = message_filters.TimeSynchronizer([self.tr_rgb_img_sub, self.mr_rgb_img_sub, self.br_rgb_img_sub], 10)
        self.ts_right_cameras.registerCallback(self.right_cameras_cb)

        self.tl_rgb_img_msg = Image()
        self.tr_rgb_img_msg = Image()
        self.ml_rgb_img_msg = Image()
        self.mr_rgb_img_msg = Image()
        self.bl_rgb_img_msg = Image()
        self.br_rgb_img_msg = Image()

        self.left_capture_time = 0
        self.right_capture_time = 0

        self.pose_sub = rospy.Subscriber("/robot_pose",PoseWithCovarianceStamped, self.pose_cb) #try gazebo ground truth eventually
        self.pose = PoseWithCovarianceStamped()
        self.laser_scan_sub = rospy.Subscriber("/scan",LaserScan, self.laser_cb)
        self.laser_msg = LaserScan()

        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel",Twist, queue_size=1)

        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = self.client.wait_for_server()
        if not wait:
            rospy.signal_shutdown("Could not find move_base Action Server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end


    def run(self):
        
        self.utils.set_map_image(self.task_num) #to check if the given goal is good 

        map_img = self.utils.gray2bgr(self.utils.get_map_image())

        #######################
        ## OLD METHOD (patch search)
        #######################
        
        # img = self.utils.gray2bgr(self.utils.get_map_image())
        # for position in self.full_trajectory:
        #     #this double transformation is required
        #   i,j = self.utils.map2image(position[0],position[1])
        #   new_img_pt = self.utils.find_feasible_point(img,(i,j),self.patch_sz, self.patch_len,self.goal_cnt)
        #   robot_pose, object_pose = self.utils.get_object_direction()
        #   x, y = self.utils.image2map(new_img_pt[0],new_img_pt[1])
        #   x, y = self.utils.shift_goal((new_goal_x,new_goal_y),self.map_shift)

        #   self.send_waypoint(x, y)
        #   self.goal_cnt = self.goal_cnt + 1
                
        #   time.sleep(1.0)
        #   self.align()
        #   time.sleep(1.0)
        #   self.capture() 

        #######################
        ## NEW METHOD (repulsive areas)
        #######################
            
        ###TODO ALL OFFLINE, THEN SEND ONLY GOOD WAYPOINTS!

        if self.task_num == 1:

            # for position in self.full_trajectory:
                # i,j = self.utils.map2image(position[0],position[1])
            i,j = self.utils.map2image(6.7 ,15.0)
        
            
            query_area = self.utils.find_query_shelf((i,j)) #getting rep area, not shelf
            #since we have a query shelf for each desired position, we can associate a dictionary
            #with key the goal counter (append 0,0 if query_area i s 0). This is TODO, for now we iterate
            #over all of them (does not affect planner since offline)
            if query_area.x != 0: #if the generated waypoint from CNN is in good we send it, otherwise we simply skip it
                valid_waypoint = (0,0)
                if self.utils.is_inside_a_repulsive_area(self.repulsive_areas,(i,j)):
                    valid_waypoint = self.utils.push_waypoint((i,j),query_area,self.patch_sz)
                    object_position = (i,j)
                else:
                    valid_waypoint = (i,j)
                    object_position = query_area.center

                x, y = self.utils.image2map(valid_waypoint[0],valid_waypoint[1])
                x, y = self.utils.shift_goal((x,y),self.map_shift)
                object_position = self.utils.image2map(object_position[0],object_position[1])
                object_position = self.utils.shift_goal(object_position,self.map_shift)

                self.send_waypoint(x, y)
                self.goal_cnt = self.goal_cnt + 1

                capture_side = self.align_robot(object_position)
                self.capture(capture_side, query_area) 
    
            rospy.signal_shutdown("Reached last goal, shutting down wpoint generator node")
            rospy.logdebug("Reached last goal, shutting down wpoint generator node")

        else:
            for crossroad in self.crossroads:
                print(crossroad)

    def send_waypoint(self, x, y):
        print("Sending waypoint!")
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        #send the current goal
        self.client.send_goal(self.goal,done_cb=self.done_cb,feedback_cb=self.feedback_cb) #it is possible to add the others callbacks directly from here
        
        result = self.client.wait_for_result() #blocking execution until some result comes

        if not result:
            rospy.logerr("Something went wrong with the waypoint number: !", self.goal_cnt)
            rospy.signal_shutdown("Something went wrong with the waypoint number: !", self.goal_cnt)
        else:
            return self.client.get_result()

    def align_robot(self,object_position):
        rospy.logdebug("Aligning robot.")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        #get pose and turning direction sign
        curr_x = self.pose.pose.pose.position.x
        curr_y = self.pose.pose.pose.position.y
        curr_yaw = self.pose.pose.pose.orientation.z

        epsilon = 0.02 #around 1 degree

        theta = Utils.get_robot_obj_angle(curr_x,curr_y,curr_yaw,object_position)

        w_direction = 1 if abs(theta) >= Utils.pi()/2 else -1

        msg.angular.z = w_direction*0.2 #moving very slowly

        while not rospy.is_shutdown() and  abs(abs(theta) - Utils.pi()/2) > epsilon:
            self.cmd_vel_pub.publish(msg)
            curr_yaw = self.pose.pose.pose.orientation.z
            theta = Utils.get_robot_obj_angle(curr_x,curr_y,curr_yaw,object_position)

        capture_side = "left" if w_direction == -1 else "right"

        return capture_side
        
    def capture(self, capture_side, query_area):

        path = self.base_path + 'acquisitions/' + self.store_name + '/' + str(self.num_trajectory) + '/'
        self.utils.dir_exists( path + str(self.goal_cnt) )

        capture_time = 0
        top_cam_info = 0
        middle_cam_info = 0
        bottom_cam_info = 0
        
        #save images
        if capture_side == "left":
            capture_time = self.left_capture_time
            self.utils.save_image( path + str(self.goal_cnt) + '/top_rgb.jpg', self.tl_rgb_img)   
            self.utils.save_image( path + str(self.goal_cnt) + '/middle_rgb.jpg', self.ml_rgb_img)  
            self.utils.save_image(path + str(self.goal_cnt) + '/bottom_rgb.jpg', self.bl_rgb_img) 
            top_cam_info = self.tl_cam_rgb_info
            middle_cam_info = self.ml_cam_rgb_info
            bottom_cam_info = self.bl_cam_rgb_info

        if capture_side == "right":
            capture_time = self.right_capture_time
            self.utils.save_image( path + str(self.goal_cnt) + '/top_rgb.jpg', self.tr_rgb_img)   
            self.utils.save_image( path + str(self.goal_cnt) + '/middle_rgb.jpg', self.mr_rgb_img)  
            self.utils.save_image(path + str(self.goal_cnt) + '/bottom_rgb.jpg', self.br_rgb_img) 
            top_cam_info = self.tr_cam_rgb_info
            middle_cam_info = self.mr_cam_rgb_info
            bottom_cam_info = self.br_cam_rgb_info



        #save pose and shelf data
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        yaw = self.pose.pose.pose.orientation.z
        x,y = self.utils.shift_goal((x,y),self.map_shift,-1) #restore goal position wrt map origin
        self.utils.save_pose_metadata(path + str(self.goal_cnt) + '/pose.yaml',capture_time,x,y,yaw,capture_side,query_area.id)
        self.utils.save_camera_metadata(path + str(self.goal_cnt) + '/top_camera.yaml',capture_time,top_cam_info)
        self.utils.save_camera_metadata(path + str(self.goal_cnt) + '/middle_camera.yaml',capture_time,middle_cam_info)
        self.utils.save_camera_metadata(path + str(self.goal_cnt) + '/bottom_camera.yaml',capture_time,bottom_cam_info)

        rospy.logdebug("Saved waypoint number " + str(self.goal_cnt) + " images and pose.")



    def feedback_cb(self,feedback): #returns the robot position
        # rospy.loginfo("Feedback:%s" % str(feedback))
        a = 3

    def done_cb(self, status, result):
        #status gives info on preemptions etc on the goal
        print("status: ", status)
        print("result:" ,result)

    def laser_cb(self,data):
        self.laser_msg = data

    def pose_cb(self,data):
        self.pose = data

    def right_cameras_cb(self, top_img, middle_img, bottom_img):

        self.right_capture_time = top_img.header.stamp

        try:
            self.tr_rgb_img = self.bridge.imgmsg_to_cv2(top_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.mr_rgb_img = self.bridge.imgmsg_to_cv2(middle_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.br_rgb_img = self.bridge.imgmsg_to_cv2(bottom_img, "bgr8")
        except CvBridgeError as e:
            print(e)


    def left_cameras_cb(self, top_img, middle_img, bottom_img): 

        self.left_capture_time = top_img.header.stamp

        try:
            self.tl_rgb_img = self.bridge.imgmsg_to_cv2(top_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.ml_rgb_img = self.bridge.imgmsg_to_cv2(middle_img, "bgr8")
        except CvBridgeError as e:
            print(e) 
     
        try:
            self.bl_rgb_img = self.bridge.imgmsg_to_cv2(bottom_img, "bgr8")
        except CvBridgeError as e:
            print(e) 

def main():
    
    args = rospy.myargv()
    
    if len(args) != 6:
        Utils.print_usage(1)
    rospy.init_node('wpoints_generator', anonymous=True)
    
    robot_navigation = trajectoryGenerator(args, True)
    robot_navigation.run()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
