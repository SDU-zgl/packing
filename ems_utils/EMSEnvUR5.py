from typing import Optional, Union, Tuple, List
import numpy as np
import random

from .Ems import Ems
from .components import Cube
from .components import Bin_container
from .show import CubePlot






class EMSEnv:

    def __init__(self, bin_size=[100, 100, 120]):
        #bin_size单位需要和给定item的单位一致
        self.ems = Ems(bin_size)  #10*10*10
        self.plot = CubePlot(bin_size)
        self.bin = Bin_container(bin_size[0],bin_size[1],bin_size[2])
        self.bin_size = bin_size

    def step(self, box: List[float]):
        cube = Cube(box)
        s = self.ems.get_suggestion(cube)
        loc = s[0]
        cube_ = s[1]
        if loc is None:
            # print('final cube num:',self.bin.get_cubes_num())
            # print('final space ratio:',self.bin.get_filling_ratio())
            return None, 0, False, {'uti':self.bin.get_filling_ratio(),'num':self.bin.get_cubes_num()}

        task = {'loc': loc, 'cube': cube}
        ### cal new ems ###
        self.ems.confirm_cube2ems(task)
        self.plot.update(cube_.size, loc.point1.size, cube.expand, delay=0.5)
        ### bin add cube ###
        self.bin.add_cubes(cube)
        ### cal rotation ###
        rotation = 1
        if abs(cube.x - cube_.x) > 0.0001:
            rotation = 2
        return [loc.point1.size[0] + cube_.size[0] / 2,    ## 不加偏差expand也行 ##
                loc.point1.size[1] + cube_.size[1] / 2,
                loc.point1.z + cube_.z, rotation], 0, False, {'uti':self.bin.get_filling_ratio(),'num':self.bin.get_cubes_num()}

    def virtual_step(self, box: List[float]):
        cube = Cube(box)
        s = self.ems.get_suggestion(cube)
        loc = s[0]
        cube_ = s[1]
        if loc is None:
            # print('final cube num:',self.bin.get_cubes_num())
            # print('final space ratio:',self.bin.get_filling_ratio())
            return None, 0, False, {'uti':self.bin.get_filling_ratio(),'num':self.bin.get_cubes_num()}

        # task = {'loc': loc, 'cube': cube}
        # ### cal new ems ###
        # self.ems.confirm_cube2ems(task)
        # self.plot.update(cube_.size, loc.point1.size, cube.expand, delay=0.5)
        # ### bin add cube ###
        # self.bin.add_cubes(cube)
        ### cal rotation ###
        rotation = 1
        if abs(cube.x - cube_.x) > 0.0001:
            rotation = 2
        return [loc.point1.size[0] + cube_.size[0] / 2,    ## 不加偏差expand也行 ##
                loc.point1.size[1] + cube_.size[1] / 2,
                loc.point1.z + cube_.z, rotation], 0, False, {'uti':self.bin.get_filling_ratio(),'num':self.bin.get_cubes_num()}


    def reset(self, *, seed: Optional[int] = None, return_info: bool = False, options: Optional[dict] = None):
        del self.ems
        self.ems = Ems(self.bin_size)
        self.plot.clear()
        self.bin = Bin_container(self.bin_size[0],self.bin_size[1],self.bin_size[2])
        return [0, 0, 0]

    def render(self, mode="human"):
        pass






# def main():
#     env = EMSEnv()
#     env.reset()

#     ### 100*100*100的仿真格子中预测，并加偏差，真实放置时坐标都除2 ###
#     # preset_list = [[2.6, 1.6, 2], [2.8, 1.8, 2], [3.5, 2.3, 2], [3.9, 2.7, 2], [4.2, 2.8, 2.2], [4.7, 3.2, 2.6], [4, 4, 2], [3, 3, 2], [4, 4, 1], [3, 3, 1],[3.5,2,2.3]]
#     preset_list = [[7, 6, 4], [7, 6, 6], [7, 7, 4], [7, 7, 6], [8, 6, 4], [8, 6, 6], [8, 8, 4], [8, 8, 6], [8, 8, 8], [10, 6, 4], [10, 6, 6], [10, 8, 4], [10, 8, 6], [10, 8, 8]]

#     workspace_limits = np.asarray([[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]])
#     ur5 = Robot(workspace_limits)
#     # ur5 = Real_binpacking_Env()
#     # ur5.reset()
#     while True:
#         cube_size = ur5.get_random_current_item()
#         print('current item size: ', cube_size)
#         # index = np.random.randint(0, 14)
#         cube = Cube(cube_size)#(preset_list[index])
#         action, _, _, _ = env.step(cube.size)
#         print('place action: ',action)
#         if action is None:
#             break
#         # if index == 12:
#         #     cube.z = 1.75
#         print(action)
#         ur5.step_place_action(action, cube.z)

def update_heightmap(height_map, item_size, place_position): # [x,y]
    #这部分更新height_map没有考虑到不完全重合的情况
    place_position = np.round(np.array(place_position)).astype(np.int64)
    height_map[int(place_position[1]-item_size[1]/2.0):int(place_position[1]+item_size[1]/2.0), int(place_position[0]-item_size[0]/2.0):int(place_position[0]+item_size[0]/2.0)] += item_size[2]
    return height_map

def detect_region_from_mask(mask):
    grasp_uv_centers = []
    cnt_list = []
    bbox_list = [] #[x,y,w,h]
    # find grasp solution on binary mask of depth
    ret, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        # exclude the noise point
        if w>=20 and h>= 20:
            # find the rotation angle
            rect = cv2.minAreaRect(cnt)
            grasp_uv_centers.append([int(rect[0][0]),int(rect[0][1])])
            cnt_list.append(np.int0(cv2.boxPoints(rect)))
            bbox_list.append([x,y,w,h])

    return grasp_uv_centers, cnt_list, bbox_list


def calculate_flatten_score(before_heightmap,after_heightmap,cube_size):
    
    max_value = np.max(after_heightmap)
    return np.sum(max_value - after_heightmap)

    # return cube_size[0]*cube_size[1]*cube_size[2]


    # possible_height = [0,20,30,40,50,60,70,80,90,100,110]
    
    # for h in possible_height:
    #     binary_mask = np.zeros((150,150))
    #     binary_mask[np.where((before_heightmap - h)==0)] = 255
        
    # binary_mask = np.zeros((150,150))

    


def main():
    ####################
    bin_heightmap = np.zeros((100,100))
    ####################
    
    env = EMSEnv()
    env.reset()
    item_size_set = [(52, 42.5, 21),(38, 37, 20),(51, 33, 20.5),(52, 42, 30),
             (51, 32.5, 30),(52, 43, 30.5),(52, 42.5, 21),(42.5, 41.5, 40),(43.5, 42.5, 30),
             (43, 41.5, 20),(43, 41, 30.5),(36.5, 32.5, 20.5),(42, 32, 30),(42.5, 42.5, 39.5),
             (42, 32.5, 30),(52, 33, 30),(51, 42, 40)]
    import random
    #random.shuffle(item_size_set)
    item_size_set.sort(key=lambda position: position[0]*position[1],reverse=True)
    ### 100*100*100的仿真格子中预测，并加偏差，真实放置时坐标都除2 ###
    # preset_list = [[2.6, 1.6, 2], [2.8, 1.8, 2], [3.5, 2.3, 2], [3.9, 2.7, 2], [4.2, 2.8, 2.2], [4.7, 3.2, 2.6], [4, 4, 2], [3, 3, 2], [4, 4, 1], [3, 3, 1],[3.5,2,2.3]]
    preset_list = [[7, 6, 4], [7, 6, 6], [7, 7, 4], [7, 7, 6], [8, 6, 4], [8, 6, 6], [8, 8, 4], [8, 8, 6], [8, 8, 8], [10, 6, 4], [10, 6, 6], [10, 8, 4], [10, 8, 6], [10, 8, 8]]

    #workspace_limits = np.asarray([[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]])
    #ur5 = Robot(workspace_limits)
    # ur5 = Real_binpacking_Env()
    # ur5.reset()
    #ur5.load_five_item_at_start()
    # while True:
    #     cube_size = ur5.select_one_item_from_five_items()#ur5.select_max_item_from_five_items()#ur5.select_one_item_from_five_items()
    #     print('current item size: ', cube_size)
    #     # index = np.random.randint(0, 14)
    #     cube = Cube(cube_size)#(preset_list[index])
    #     action, _, _, _ = env.step(cube.size)
    #     print('place action: ',action)
    #     if action is None:
    #         break
    #     # if index == 12:
    #     #     cube.z = 1.75
    #     bin_heightmap = update_heightmap(bin_heightmap, cube_size,[action[0],action[1]])
    #     #ur5.step_place_action(action, cube.z)
    for item in item_size_set:
        cube_size = item
        print('current item size: ', cube_size)
        cube = Cube(cube_size)#(preset_list[index])
        action, _, _, info = env.step(cube.size)
        print('place action: ',action,'info',info)
        if action is None:
            break
         # if index == 12:
         #     cube.z = 1.75
        bin_heightmap = update_heightmap(bin_heightmap, cube_size,[action[0],action[1]])

if __name__ == '__main__':
    main()
    import time
    time.sleep(60)                                                                                                                                           
    #main_with_selection()
