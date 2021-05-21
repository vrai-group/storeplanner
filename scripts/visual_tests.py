#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2
import numpy as np
from utils import Utils

#TODO: parametrize map path (need to add arg)

###### VISUAL TESTS (trajectories and recovery)
def check_traj_correspondences( robot_trajectory, filename, map_source):

    utils = Utils(filename, map_source)
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('storeplanner') + '/trajectories/' + filename
    blender_filename = rospack.get_path('storeplanner') + '/maps/store_2/trial/map.pgm' #to parametrize per store

    filename = os.path.splitext(filepath)[0]+'.jpg'
    robot_stops_img = cv2.imread(filename,1)
    robot_stops_blender_img = cv2.imread(blender_filename,1)

    for pose in robot_trajectory:

        x = utils.maps_x_ratio * pose[0]
        y = utils.maps_y_ratio * pose[1]

        if utils.map_source == 0:
            i,j = utils.map2image(pose[0],pose[1])   
            robot_stops_img = cv2.circle(robot_stops_img, (i,j), radius=5, color=(0, 0, 255), thickness=-1)
        elif utils.map_source == 1:
            i,j = utils.map2image(x,y)
            robot_stops_blender_img = cv2.circle(robot_stops_blender_img, (i,j), radius=3, color=(0, 0, 255), thickness=-1)


    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=3, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=3, color=(0, 0, 255), thickness=-1)

    if utils.map_source == 0:
        utils.show_img_and_wait_key("Original Trajectories",robot_stops_img)
    elif utils.map_source == 1:
        utils.show_img_and_wait_key("Trajectories on blender map",robot_stops_blender_img)


def check_feasibility():
    utils = Utils()

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('storeplanner') + '/maps/store_2/trial/map.pgm' #to parametrize per store

    img_rgb = cv2.imread(filepath,cv2.IMREAD_COLOR)
    img = cv2.imread(filepath,cv2.IMREAD_GRAYSCALE)

    #desired waypoints
    inside_points = [(150,355), (600,735), (680,200),(500,500)]

    for point in inside_points:
        cv2.circle(img_rgb, point, radius=3, color=(0, 0, 255), thickness=2)
        new_point = utils.find_closest_goal(img, point)
        cv2.circle(img_rgb, new_point, radius=3, color=(0, 255, 0), thickness=2)

    utils.show_img_and_wait_key("Feasibility", img_rgb)


def show_good_bad_points(waypoints,map_source):
    utils = Utils()
    averages = []
    ksize = 4 #for each side
    rospack = rospkg.RosPack()
    filename = rospack.get_path('storeplanner') + '/maps/store_2/trial/map.pgm' #to parametrize per store
    img = cv2.imread(filename,cv2.IMREAD_COLOR)

    for waypoint in waypoints:
        # x = utils.maps_x_ratio * waypoint[0]
        # y = utils.maps_y_ratio * waypoint[1]

        i,j = utils.map2image(waypoint[0],waypoint[1])

        draw_arrow(img,(i,j),(i+20,j),0)

    utils.show_img_and_wait_key("Wpoints goodness", img)

#drawing functions
def draw_point(img,pt,color=(0,0,255),thickness=3):
    return cv2.circle(img,pt,thickness,color,-1)

def draw_robot(img,pt,radius,color):
    return cv2.circle(img,pt,int(radius),color,2)

def draw_arrow(img, pt1, pt2, color):
    return cv2.arrowedLine(img, pt1, pt2, color,2)

def draw_patch(img, pt, psize,color):
    pt1 = (int(pt[0]-psize/2),int(pt[1]-psize/2))
    pt2 = (int(pt[0]+psize/2),int(pt[1]+psize/2))
    return cv2.rectangle(img,pt1,pt2,color,-1)

def draw_shelf(img,shelf):
    cv2.putText(img,shelf.id,(shelf.x+int(shelf.w/2)-10,shelf.y+int(shelf.h/2)+10),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
    return cv2.rectangle(img,(shelf.x,shelf.y),(shelf.x+shelf.w,shelf.y+shelf.h),(0,0,255,0.5),2)

def draw_roi(img,shelf):
    return cv2.rectangle(img,(shelf.x,shelf.y),(shelf.x+shelf.w,shelf.y+shelf.h),(0,255,0,0.5),2)


