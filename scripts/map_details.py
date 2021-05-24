import argparse
import rospkg
import json
import cv2
import numpy as np
import os.path

class mapDetails:

  def __init__(self, args):

    self.shelves = list()
    self.shelves_h = list()
    self.capture_stops = list()

    rospack = rospkg.RosPack()
    base_path = rospack.get_path('storeplanner')
    map_filename = base_path + '/maps/' + args['store'] + '/' + args['map'] + '/map.pgm'

    if not os.path.exists(map_filename):
      print('Could not find map file.\nCheck file path.')
      sys.exit(1)

    self.details_path = base_path + '/models/' + args['store'] + '/details/'

    if not os.path.exists(self.details_path):
      os.makedirs(self.details_path)

    self.img = cv2.imread(map_filename,cv2.IMREAD_COLOR)

  def run(self):
 
    ans = raw_input("Do you want to select the shelves? [y/n]\n")
    
    if ans == 'y' or ans == 'yes':

      cv2.namedWindow("Shelves selector",cv2.WINDOW_NORMAL)
      cv2.resizeWindow("Shelves selector", self.img.shape[1], self.img.shape[0])
      self.shelves = cv2.selectROIs("Shelves selector",self.img)
      cv2.destroyWindow("Shelves selector")
      ans2 = raw_input("Do you want to give shelves height? (default 2.0 meters for all) [y/n]\n")

      if ans2 == 'y' or ans2 == 'yes':
        self.shelves_h = self.calc_shelves_height()
      else:
        self.shelves_h = [2.0] * len(self.shelves)

      self.save_shelves()
      
    #the capture stops should be given to the robot as it will do the inventary
    ans = raw_input("Do you want to select the capture stops? [y/n]\n")

    if ans == 'y' or ans == 'yes':
      cv2.namedWindow("Capture Stops selector",cv2.WINDOW_NORMAL)
      cv2.resizeWindow("Capture Stops selector", self.img.shape[1], self.img.shape[0])
      cv2.setMouseCallback("Capture Stops selector",self.mouse_callback)

      while(True):
        cv2.imshow("Capture Stops selector",self.img)
        k = cv2.waitKey() & 0xFF
        if k == 27: #esc
            break
      cv2.destroyWindow("Capture Stops selector")
      
      self.save_capture_stops()

  def calc_shelves_height(self):

    shelves_h = list()

    for shelf in self.shelves:
      temp_img = self.img.copy()
      (x, y, w, h) = shelf
      cv2.rectangle(temp_img,(x,y),(x + w,y + h),(0,0,255),2)
      cv2.namedWindow("Shelf Height calculator",cv2.WINDOW_NORMAL)
      cv2.resizeWindow("Shelf Height calculator", int(self.img.shape[1]/2), int(self.img.shape[0]/2))
      cv2.imshow("Shelf Height calculator",temp_img)
      cv2.waitKey(300)
      height = raw_input("Enter height for this shelf: ")
      shelves_h.append(float(height))
      cv2.destroyWindow("Shelf Height calculator")

    return shelves_h
      
  def save_shelves(self):

    filename = self.details_path + 'shelves.json'

    shelf_cnt = 0
    data = {}
    data['shelves'] = []

    for shelf in self.shelves:
      shelf_cnt = shelf_cnt + 1
      (x, y, w, h) = shelf
      data['shelves'].append({
        'id': shelf_cnt,
        'x': int(x),
        'y': int(y),
        'z': float(self.shelves_h[shelf_cnt-1]),
        'w': int(w),
        'h': int(h)
    })
    with open(filename, 'w') as outfile:
      json.dump(data, outfile)

  def save_capture_stops(self):

    filename = self.details_path + 'capture_stops.json'

    stop_cnt = 0
    data = {}
    data['capture_stops'] = []
    
    for stop in self.capture_stops:
      stop_cnt = stop_cnt + 1
      (x, y) = stop
      data['capture_stops'].append({
        'id': stop_cnt,
        'x': int(x),
        'y': int(y)
    })
    with open(filename, 'w') as outfile:
      json.dump(data, outfile)

  def mouse_callback(self,event,x,y,flags,param):

    if event == cv2.EVENT_LBUTTONDBLCLK:
        self.img = cv2.circle(self.img,(x,y),3,(0,0,255),-1)
        cv2.imshow("Capture Stops selector",self.img)
        self.capture_stops.append((x,y))


def main():

  parser = argparse.ArgumentParser()
  parser.add_argument('--store', type=str, default='store_2', help='store name / folder')
  parser.add_argument('--map', type=str, default='blender_map', help='map source / folder')
  args = parser.parse_args()
  args_ = vars(args)

  map_details = mapDetails(args_)
  map_details.run()


if __name__ == '__main__':
    main()
