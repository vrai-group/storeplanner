import argparse
import rospkg
import json
import cv2
import numpy as np
import os.path

class mapDetails:

  def __init__(self, args):

    self.shelves = list()
    self.crossroads = list()

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
 
    ans = raw_input("DO you want to select the shelves? [y/n]")
    
    if ans == 'y' or ans == 'yes':
      cv2.namedWindow("Shelves selector",cv2.WINDOW_NORMAL)
      cv2.resizeWindow("Shelves selector", self.img.shape[1], self.img.shape[0])
      self.shelves = cv2.selectROIs("Shelves selector",self.img)
      self.save_shelves()
      cv2.destroyWindow("Shelves selector")

    ans = raw_input("DO you want to select the crossroads? [y/n]")

    if ans == 'y' or ans == 'yes':
      cv2.namedWindow("Crossroads selector",cv2.WINDOW_NORMAL)
      cv2.resizeWindow("Crossroads selector", self.img.shape[1], self.img.shape[0])
      cv2.setMouseCallback("Crossroads selector",self.mouse_callback)

      while(True):
        cv2.imshow("Crossroads selector",self.img)
        k = cv2.waitKey() & 0xFF
        if k == 27: #esc
            break
      cv2.destroyWindow("Crossroads selector")
      
      self.save_crossroads()
      
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
        'z': 2.0, #TODO, selection of height
        'w': int(w),
        'h': int(h)
    })
    with open(filename, 'w') as outfile:
      json.dump(data, outfile)

  def save_crossroads(self):

    filename = self.details_path + 'crossroads.json'

    crossroad_cnt = 0
    data = {}
    data['crossroads'] = []
    
    for crossroad in self.crossroads:
      crossroad_cnt = crossroad_cnt + 1
      (x, y) = crossroad
      data['crossroads'].append({
        'id': crossroad_cnt,
        'x': int(x),
        'y': int(y)
    })
    with open(filename, 'w') as outfile:
      json.dump(data, outfile)

  def mouse_callback(self,event,x,y,flags,param):

    if event == cv2.EVENT_LBUTTONDBLCLK:
        self.img = cv2.circle(self.img,(x,y),3,(0,0,255),-1)
        cv2.imshow("Crossroads selector",self.img)
        self.crossroads.append((x,y))


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
