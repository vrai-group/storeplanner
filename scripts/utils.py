#!/usr/bin/env python

import json
import sys
import os.path
import rospy
import rospkg
import cv2
import numpy as np
import math 
from shelf import Shelf
from cameras import CamerasParams

class Utils:

    def __init__(self, store_name='', traj_num='', map_name=''): #to put in config eventually

        self.store_name = store_name
        self.traj_num = traj_num
        self.map_name = map_name

        self.map_source = 1 #always considering maps produced by blender

        self.map_image = []

        self.store_width = 0.0
        self.store_height = 0.0
        self.img_width = 0
        self.img_height = 0
        self.m_px_ratio_x = 0.0
        self.m_px_ratio_y = 0.0
        self.maps_x_ratio = 0.0
        self.maps_y_ratio = 0.0 

        self.robot_pose = 0
        self.object_pose = 0

        self.set_map_params() #with harcoded dimensions (TODO parametrize them based on store in the future)

        self.customer_traj = list()
        self.shelves = list()
        self.repulsive_areas = list()
        self.crossroads = list()
        self.forbidden_area = 0



    @staticmethod
    def print_usage(exit_code=0):
        print '''Usage: rosrun storeplanner wpoints_generator.py <task_num> <store_name> <traj_num> 

        - <task_num> the task you want to perform with the robot:
                      1 -> customer behaviour mimic 
                      2 -> customer avoidance

        - <store_name> the store name you want to inspecting under 
                       trajectories/ directory

        - <traj_num> number of the trajectory under trajectories/<store_name> 
                     you want to perform.
          Assumption: trajectories are saved as json files

        - <map_name> the name of the source map under maps/<store_name>/

        - <debug_mode> enable visualization simulating waypoints on the map
                       or send real waypoints to the robot
          
        Example: rosrun storeplanner wpoints_generator.py store_2 1 blender_map False
        '''

        sys.exit(exit_code)

    def set_map_params(self):
        #trajectory image params
        traj_planimetry_width = 42.5
        traj_planimetry_height = 32.4
        traj_img_width = 1602
        traj_img_height = 1210

        #blender generated image params        
        blender_store_width = 45.0
        blender_store_height = 34.6
        blender_img_width = 1192
        blender_img_height = 918

        #correct trajectory dimensions
        traj_m_px_ratio_x = traj_planimetry_width / (traj_img_width-22-30) #have to correct them
        traj_m_px_ratio_y = traj_planimetry_height / (traj_img_height-24-26)
        traj_store_width = traj_img_width * traj_m_px_ratio_x #42.5
        traj_store_height = traj_img_height * traj_m_px_ratio_y #32.4 

        if self.map_source == 0: #setting params based on trajectory
            self.store_width = traj_store_width
            self.store_height = traj_store_height
            self.img_width = traj_img_width
            self.img_height = traj_img_height
            self.m_px_ratio_x = traj_m_px_ratio_x
            self.m_px_ratio_y = traj_m_px_ratio_y

            self.maps_x_ratio = 1.0
            self.maps_y_ratio = 1.0

        elif self.map_source == 1: #setting params based on blender
            self.store_width = blender_store_width
            self.store_height = blender_store_height
            self.img_width = blender_img_width
            self.img_height = blender_img_height
            self.m_px_ratio_x = blender_store_width / blender_img_width #TODO retrieve it from map yaml
            self.m_px_ratio_y = blender_store_height / blender_img_height #TODO retrieve it from map yaml

            #TODO (for each store) proportions. Given a goal x,y in the trajectory image
            #we have to transform it for the blender map
            self.maps_x_ratio = blender_store_width / traj_store_width 
            self.maps_y_ratio = blender_store_height / traj_store_height
            
 
        else:
            rospy.signal_shutdown("Wrong map source param!")

    ##################################
    ##SHELVES/FORBIDDEN AREAS UTILS
    ##################################

    #TODO in init
    #extracts shelves from file and convert them into image coordinates
    def parse_shelves(self,filepath):
        if os.path.exists(filepath):
            with open(filepath,'rb') as json_file:
                data = json.load(json_file)
        else:
            rospy.signal_shutdown('Could not find shelves file.\nCheck file path.')
        
        for shelf in data['shelves']:
            p1_x,p1_y = self.map2image(shelf['x'],shelf['y'])
            w,h = self.meters2pixels(shelf['w'],shelf['h'])
            img_shelf = Shelf(shelf['id'],p1_x,p1_y,shelf['z'],w,h)
            self.shelves.append(img_shelf)
            
    #TODO in init
    def calculate_repulsive_areas(self,patch_sz):
    
        for shelf in self.shelves:
            d_min = 0
            if shelf.z > CamerasParams.robot_height: #make sure we take the shelf top part
                d_min = (shelf.z - CamerasParams.top_camera_height)/math.tan(CamerasParams.vfov/2)
            else: #make sure we take the shelf bottom part
                d_min = (CamerasParams.bottom_camera_height)/math.tan(CamerasParams.vfov/2)
            
            d_min_px = self.map2image(d_min,0)[0]

            if d_min_px < patch_sz:
                d_min_px = patch_sz
             
            eps = 0
            d = d_min_px + eps #to have some margin eventually
   
            repulsive_shelf = Shelf(shelf.id,shelf.x-d,shelf.y-d,shelf.z,shelf.w+2*d,shelf.h+2*d) #basicallly enhancing the shelves
            self.repulsive_areas.append(repulsive_shelf)

    def is_inside_a_repulsive_area(self,regions,pt):
        for region in regions:
            if region.x <= pt[0] <= region.x+region.w and region.y <= pt[1] <= region.y+region.h:
                return True       
        return False

    #since shelves and repulsive areas share the same center and id, we get the query shelf by using the repulsive areas
    def find_query_shelf(self,goal):
        dist_to_shelf_center = 99999
        query_shelf = Shelf()

        for rep_area in self.repulsive_areas:
            dist = self.calc_eucl_dist(rep_area.center,goal)
            if  dist < dist_to_shelf_center:
                query_shelf = rep_area
                dist_to_shelf_center = dist
        
        if dist_to_shelf_center > 150: #the generated wpoint is too far/wrong, thus not scanning
            query_shelf = Shelf()

        return query_shelf
    
    def push_waypoint(self,goal,rep_area,patch_sz):
        boundaries = list()
    
        #as [left_boundary,right_boundary,top_boundary,bottom_boundary] if vertical to reduce bias
        l = (rep_area.x,goal[1])
        r = (rep_area.x + rep_area.w,goal[1])
        t = (goal[0],rep_area.y)
        b = (goal[0],rep_area.y + rep_area.h)
        boundaries = [l,r,t,b]
 
        #the capture position is too close wrt patch size, we need to move to AT LEAST patch_sz
        # if rep_area.z < 2:#TODO change rep area to be at least patch_sz
        #     l2 = (rep_area.x - int(patch_sz/4),goal[1]) 
        #     r2 = (rep_area.x + rep_area.w + int(patch_sz/4),goal[1])
        #     t2 = (goal[0],rep_area.y- int(patch_sz/4))
        #     b2 = (goal[0],rep_area.y + rep_area.h + int(patch_sz/4))
        #     boundaries = [l2,r2,t2,b2]

        dists = list()
        for boundary in boundaries:
            dists.append(self.calc_eucl_dist(goal,boundary))
            
        sorted_dists_idx = sorted(range(len(dists)), key=lambda k: dists[k])

        for idx in sorted_dists_idx:
            new_goal = boundaries[idx]
            patch = self.map_image[new_goal[1]-patch_sz/2:new_goal[1]+patch_sz/2,new_goal[0]-patch_sz/2:new_goal[0]+patch_sz/2]
            if(self.is_good_spot(patch)):
                return new_goal
        print("Could not find a feasible position for this waypoint. Aborting scanning")
        return 0,0

    def get_repulsive_areas(self):
        return self.repulsive_areas 

    def get_shelves(self):
        return self.shelves

    ##################################
    ##TRAJECTORY UTILS
    ##################################

 #TODO in init instead of in wpoint generator
    def load_trajectory(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('storeplanner') + '/trajectories/' + self.store_name + '/' + self.traj_num + '.json'

        if os.path.exists(filepath):
            with open(filepath) as f:
                data = json.load(f)
        else:
            rospy.signal_shutdown('Could not find the trajectory file.\nCheck file path.')

        return self.transform_waypoints(data["points"])
 #TODO in init instead of in wpoint generator
    def transform_waypoints(self, store_waypoints):
        
        for coord in store_waypoints: #dict type
            new_coord = self.tranform_position( coord['x'], coord['y'])
            self.customer_traj.append( new_coord )

        return self.customer_traj
 #TODO in init instead of in wpoint generator
    def correct_trajectory(self,traj, map_shift):
        corr_traj = list()
        x,y,Y = map_shift.split() 
        
        for pose in traj: #not acting on Yaw
            corr_pose = (pose[0]+float(x), pose[1]+float(y))
            corr_traj.append(corr_pose)
        return corr_traj
        #TODO in init 
    def tranform_position(self, x_traj, y_traj):

        x = self.maps_x_ratio * x_traj
        y = self.maps_y_ratio * y_traj

        x_map, y_map = self.map2image_meters(x,y) 
            
        if x_map >= self.store_width or y_map >= self.store_height:
            rospy.signal_shutdown('Trajectory out of store map. Aborting')

        if x_map <= 0 or y_map <= 0:
            rospy.signal_shutdown('Negative position. Aborting')

        return ( x_map, y_map )


    ##################################
    ##CROSSROADS UTILS
    ##################################

    def parse_crossroads(self,filepath):
        if os.path.exists(filepath):
            with open(filepath,'rb') as json_file:
                data = json.load(json_file)
        else:
            rospy.signal_shutdown('Could not find crossroads file.\nCheck file path.')
        
        for crossroad in data['crossroads']:
            x,y = self.map2image(crossroad['x'],crossroad['y'])
            self.crossroads.append((x,y))

    def get_crossroads(self):
        return self.crossroads

    
    ##################################
    ## COORDINATES UTILS
    ##################################

    def pixels2meters(self,x_px,y_px):
        x_m = x_px * self.m_px_ratio_x
        y_m = y_px * self.m_px_ratio_y

        return x_m,y_m

    def meters2pixels(self,x_m,y_m):
        x_px = int(x_m / self.m_px_ratio_x)
        y_px = int(y_m / self.m_px_ratio_y)

        return x_px,y_px

    #changes coordinates of a point in meter from image to map coordinates
    def image2map(self,p_x,p_y):
        x_map = p_x * self.m_px_ratio_x
        y_map = self.store_height - p_y * self.m_px_ratio_y

        return x_map,y_map
    #changes coordinates of a point in pixels from map to image coordinates
    def map2image(self,p_x,p_y):
        x_img = int(p_x / self.m_px_ratio_x)
        y_img = self.img_height - int(p_y / self.m_px_ratio_y)

        return x_img,y_img

    #change reference (invert y axis) of a point in pixels
    def map2image_pixels(self,p_x,p_y):
        x = p_x
        y = self.img_height - p_y

        return x,y
    #change reference (invert y axis) of a point in meters
    def map2image_meters(self,p_x,p_y):
        x = p_x
        y = self.store_height - p_y
       
        return x,y

    def shift_goal(self,waypoint,map_shift,sign=1):
        x,y,Y = map_shift.split() 
        corr_pose = (waypoint[0]+sign*float(x), waypoint[1]+sign*float(y))
        return corr_pose

    ##################################
    ## OLD APPROACH FEASIBLE POINTS UTILS
    ##################################
    
    #this function returns 2 positions: the interested object on the shelf and the robot position in a free area
    def find_feasible_point(self, curr_goal, patch_sz, patch_len, iter):
        initial_patch = self.map_image[curr_goal[1]-patch_sz/2:curr_goal[1]+patch_sz/2,curr_goal[0]-patch_sz/2:curr_goal[0]+patch_sz/2]
        mean = np.average(initial_patch)
        new_x = 0
        new_y = 0
        #the waypoint is good, meaning is not on a shelf -> we modify only the object position (closest)
        if mean == 255:
            self.robot_pose = curr_goal
            shelf_found = False
            #we first try to look around for a shelf in the 4 canonical directions
            for slide_cnt in range(1,80): 
                shelf_found, x_shelf, y_shelf = self.search_closest_shelf(curr_goal, patch_len, slide_cnt)
                if shelf_found:
                    self.object_pose = (x_shelf, y_shelf)
                    break
            #if they are too far, we look for the closest point (here we capture also corners)
            if shelf_found == False: #after 100 pixel, if we do not find a small line we seek for the closest point
                for slide_cnt in range(1,200): 
                    point_found, x_shelf_pt, y_shelf_pt = self.search_closest_point( curr_goal, slide_cnt)
                    if point_found:
                        self.object_pose = (x_shelf_pt, y_shelf_pt)
                        break
            return curr_goal #already a valid point
        #here we fo the same thing, however we need to modify also the robot position
        #get a good robot position
        for slide_cnt in range(1,100): 
            good_spot_found,new_x,new_y = self.slide_patch( curr_goal, patch_sz, slide_cnt)
            if good_spot_found:
                self.robot_pose = (new_x,new_y) 
                # self.object_pose = curr_goal
                break
        shelf_found = False
         #now we look for a good object position (as above) but with the new robot position
        for slide_cnt in range(1,80): 
            shelf_found, x_shelf, y_shelf = self.search_closest_shelf(self.robot_pose, patch_len, slide_cnt)
            if shelf_found:
                self.object_pose = (x_shelf, y_shelf)
                break
        
        if shelf_found == False:
            for slide_cnt in range(1,200): 
                point_found, x_shelf_pt, y_shelf_pt = self.search_closest_point( self.robot_pose, slide_cnt)
                if point_found:
                    self.object_pose = (x_shelf_pt, y_shelf_pt)
                    break
        return new_x,new_y

    def slide_patch(self,goal, patch_sz, it):
        i = goal[0]
        j = goal[1]
        up_patch = self.map_image[j-patch_sz/2-it:j+patch_sz/2-it,i-patch_sz/2:i+patch_sz/2]
        down_patch = self.map_image[j-patch_sz/2+it:j+patch_sz/2+it,i-patch_sz/2:i+patch_sz/2]
        left_patch = self.map_image[j-patch_sz/2:j+patch_sz/2,i-patch_sz/2-it:i+patch_sz/2-it]
        right_patch = self.map_image[j-patch_sz/2:j+patch_sz/2,i-patch_sz/2+it:i+patch_sz/2+it]

        if self.is_good_spot(up_patch):
            return 1,i,j-it
        if self.is_good_spot(down_patch):
            return 1,i,j+it
        if self.is_good_spot(left_patch):
            return 1,i-it,j
        if self.is_good_spot(right_patch):
            return 1,i+it,j
        return 0,0,0

    def is_good_spot(self,patch):
        return np.average(patch) == 255

    def search_closest_shelf(self, goal, patch_len, it):
        #this searches the closest shelf with increasing line patches positions
        # -----------
        # | ------- |
        # | |     | |
        # | |  .  | |
        # | |     | |
        # | ------- |
        # -----------
        
        i = goal[0]
        j = goal[1]
        #TODO also for patch above
        if j-it > 0:
            up_line = self.map_image[j-it,i-patch_len/2:i+patch_len/2]
            if self.is_closest_shelf(up_line):
                return 1,i,j-it
        if j+it < self.map_image.shape[0]:
            down_line = self.map_image[j+it,i-patch_len/2:i+patch_len/2]
            if self.is_closest_shelf(down_line):        
                return 1,i,j+it
        if i-it > 0:
            left_line = self.map_image[j-patch_len/2:j+patch_len/2,i-it]
            if self.is_closest_shelf(left_line):
                return 1,i-it,j
        if i+it < self.map_image.shape[1]:  
            right_line = self.map_image[j-patch_len/2:j+patch_len/2,i+it]
            if self.is_closest_shelf(right_line):
                return 1,i+it,j

        return 0,0,0
    
    def search_closest_point(self, goal, it):
        #make frame around as before
        i = goal[0]
        j = goal[1]
        #TODO take the CLOSEST (euclidian) wrt the center and not the first as right now
        if j-it > 0:
            up_line = self.map_image[j-it,i-it:i+it]
            shift = 0
            for x in up_line:
                if x == 0:
                    return 1,i-it+shift,j-it
                else:
                    shift = shift + 1

        if j+it < self.map_image.shape[0]:
            down_line = self.map_image[j+it,i-it:i+it]
            shift = 0
            for x in down_line:
                if x == 0:
                    return 1,i-it+shift,j+it
                else:
                    shift = shift + 1

        if i-it > 0:
            left_line = self.map_image[j-it:j+it,i-it]
            shift = 0
            for y in left_line:
                if y == 0:
                    return 1,i-it,j-it+shift
                else:
                    shift = shift + 1

        if i+it < self.map_image.shape[1]:  
            right_line = self.map_image[j-it:j+it,i+it]
            shift = 0
            for y in right_line:
                if y == 0:
                    return 1,i+it,j-it+shift
                else:
                    shift = shift + 1

        return 0,0,0

    def is_closest_shelf(self,patch):
        return np.average(patch) == 0

    def get_ratios(self):
        return self.m_px_ratio_x, self.m_px_ratio_y

    def get_object_direction(self):
        return self.robot_pose,self.object_pose

    ##################################
    ## MAP UTILS
    ##################################

    def set_map_image(self, task):

        rospack = rospkg.RosPack()
        filepath = rospack.get_path('storeplanner')
        filename = filepath + '/maps/' + self.store_name + '/' + self.map_name + '/map.pgm'

        if task == 1:
            self.map_image = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)    
        else:
            curr_map = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)
            min_x, min_y, max_x, max_y = self.calc_forbidden_area(curr_map) 
            #this is done for the planner if for some reason a bad wpoint is sent to move_base
            curr_map[min_y:max_y,min_x:max_x] = 0
            self.map_image = curr_map
            self.forbidden_area = (min_x, min_y, max_x, max_y)
        

    def get_map_image(self):
        return self.map_image

    ##################################
    ## FORBIDDEN AREA
    ##################################

    def calc_forbidden_area(self, curr_map):

        min_x, min_y = self.map2image(self.customer_traj[0][0],self.customer_traj[0][1])
        max_x, max_y = self.map2image(self.customer_traj[0][0],self.customer_traj[0][1])
        
        for position in self.customer_traj:
            x, y = self.map2image(position[0],position[1])

            if x < min_x:
                min_x = x
            if x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y

        return min_x,min_y,max_x,max_y

    def is_forbidden_point(self,point):
        x_min = self.forbidden_area[0]
        y_min = self.forbidden_area[1]
        x_max = self.forbidden_area[2]
        y_max = self.forbidden_area[3]

        if x_min <= point[0] <= x_max and y_min <= point[1] <= y_max:
            return True
        else:
            return False

    def get_forbidden_area(self):
        return self.forbidden_area

    ##################################
    ## OPENCV UTILS
    ##################################

    def show_img_and_wait_key(self,window_name,img):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
        cv2.imshow(window_name, img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

    def read_image(self,filename, coding = 1):
        return cv2.imread(filename,1)

    def save_image(self,filename,image):
        cv2.imwrite(filename,image)

    def gray2bgr(self,image):
        return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

    def bgr2gray(self,image):
        return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    ##################################
    ## MISCELLANEOUS UTILS
    ##################################  

    def save_pose_metadata(self,path,time,x,y,yaw,side,shelf_id):
        f = open(path, "w")
        f.write('# pose.yaml file')
        f.write('\n\n')
        f.write('shelf_id: ' + str(shelf_id) + '\n')
        f.write('capture_time: ' + str(time.secs) + '.' + str(time.nsecs) + '\n')
        f.write('capture_side: ' + str(side) + '\n')
        f.write('x_robot: ' + str(x) + '\n')
        f.write('y_robot: ' + str(y) + '\n')
        f.write('yaw_robot: ' + str(yaw) )
        f.close()

    def save_camera_metadata(self,path,time,cam_info):
        f = open(path, "w")
        f.write('# cameras.yaml file')
        f.write('\n\n')
        f.write('capture_time: ' + str(time.secs) + '.' + str(time.nsecs) + '\n')
        f.write('height: ' + str(cam_info.height) + '\n')
        f.write('width: ' + str(cam_info.width) + '\n')
        f.write('distorton coefficients: ' + str(cam_info.D) + '\n')
        f.write('camera matrix: ' + str(cam_info.K) + '\n')
        f.write('rectification matrix: ' + str(cam_info.R) + '\n') #only for stereo, so not used
        f.close()

    def dir_exists(self,path):
        if os.path.exists(path):
            return True
        else:
            return self.create_dir(path)

    def create_dir(self,path):
        return os.makedirs(path)
    @staticmethod
    def deg2rad(angle_deg):
        return math.radians(angle_deg)
    @staticmethod
    def rad2deg(angle_rad):
        return math.degrees(angle_rad)
    @staticmethod
    def wrap_deg_yaw(theta): 
        return theta - 360 if theta > 0 else theta + 360
    @staticmethod
    def wrap_rad_yaw(theta): 
        return theta - 2*self.pi()  if theta > 0 else theta + 2*self.pi()

    def calc_eucl_dist(self,p1,p2):
       return math.sqrt((p2[0] - p1[0])**2 + (p1[1] - p2[1])**2)
    @staticmethod
    def get_robot_obj_angle(robot_x,robot_y,robot_yaw,obj_pos):
        return math.atan2(obj_pos[1]-robot_y,obj_pos[0]-robot_x)-robot_yaw
    @staticmethod
    def pi():
        return math.pi
