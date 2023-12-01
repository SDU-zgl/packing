from .components import Cube, Point, Space, space_intersect, embody, combine_ems_y, combine_ems_x
import numpy as np
from typing import List, Dict, Union, Tuple
from .container import Container
from .show import CubePlot
import time


class Ems(Container):
    def __init__(self, size: List[float], low_bound: Union[List[float], float] = 0.5):
        super().__init__(size)
        if not isinstance(low_bound, list):
            self.low_bound = [low_bound for _ in range(3)]
        else:
            if not len(low_bound) == 3:
                assert "The bounds length of the list type must be 3"
            self.low_bound = low_bound

    def get_all_ems(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        pre_position = []
        for ems in self.space_list:
            for c in cube:     ### cube 定义了_getitem_方法，直接调用返回一个cube list(x,y对调)
                if ems.can_accommodate(c):
                    for s in self.cal_cube2ems(c, ems):
                        pre_position.append((ems, s))
        return pre_position

    def get_suggestion(self, cube: Cube):
        all_suggestion = self.heuristic_method_2(cube)
        #all_suggestion = self.heuristic_method_1(cube)
        #all_suggestion = self.heuristic_method_2xxxaaa(cube)
        #all_suggestion = self.heuristic_method_2xaxa(cube)
        #all_suggestion = self.heuristic_method_2xaxa(cube)
        return all_suggestion

    @staticmethod
    #### 计算ems内部的四个放置位置，加入了expand偏差 ###
    def cal_cube2ems(cube: Cube, space: Space):
        sug = [Space(Point([space.point1.x, space.point1.y, space.point1.z]),
                     Point([space.point1.x + cube.x, space.point1.y + cube.y, space.point1.z + cube.z])),
               Space(Point([space.point2.x - cube.x, space.point1.y, space.point1.z]),
                     Point([space.point2.x, space.point1.y + cube.y, space.point1.z + cube.z])),
               Space(Point([space.point1.x, space.point2.y - cube.y, space.point1.z]),
                     Point([space.point1.x + cube.x, space.point2.y, space.point1.z + cube.z])),
               Space(Point([space.point2.x - cube.x, space.point2.y - cube.y, space.point1.z]),
                     Point([space.point2.x, space.point2.y, space.point1.z + cube.z]))]
        return sug

    @staticmethod
    def cal_ems(ems: Space, loc: Space):

        new_ems = [Space(Point([loc.point2.x, ems.point1.y, ems.point1.z]), ems.point2),  
                   Space(ems.point1, Point([loc.point1.x, ems.point2.y, ems.point2.z])), 
                   Space(ems.point1, Point([ems.point2.x, loc.point1.y, ems.point2.z])),  
                   Space(Point([ems.point1.x, loc.point2.y, ems.point1.z]), ems.point2),
                   Space(Point([ems.point1.x, ems.point1.y, loc.point2.z]), ems.point2)]   ## 上方 ##
        
        return new_ems

    
    
    def cal_safe_cube2ems(self, cube: Cube, space: Space):    
        ems_list = [Space(Point([space.point1.x, space.point1.y, space.point1.z]),
                            Point([space.point1.x + cube.x, space.point1.y + cube.y, space.point1.z + cube.z])),
                    Space(Point([space.point2.x - cube.x, space.point1.y, space.point1.z]),
                            Point([space.point2.x, space.point1.y + cube.y, space.point1.z + cube.z])),
                    Space(Point([space.point1.x, space.point2.y - cube.y, space.point1.z]),
                            Point([space.point1.x + cube.x, space.point2.y, space.point1.z + cube.z])),
                    Space(Point([space.point2.x - cube.x, space.point2.y - cube.y, space.point1.z]),
                            Point([space.point2.x, space.point2.y, space.point1.z + cube.z]))]
        
        ### 计算接触面积，排除悬空点 ###
        max_area = 0
        new_ems_list = []
        for ems in ems_list:
            contact_area_ratio = self.cal_intersection_ratio(ems)
            if contact_area_ratio >= 0.75:#0.65:
                new_ems_list.append(ems)
                ####
                if max_area < contact_area_ratio:
                    max_area = contact_area_ratio
                    max_area_ems = ems
                ####
                # print('ratio',self.cal_intersection_ratio(ems))
                # print('ems_area',ems.get_bottom_area())
        if len(new_ems_list) == 0:
            return None, None
        return new_ems_list, max_area_ems

    def cal_intersection_ratio(self,ems:Space):
        if ems.point1.z == 0:
            return 1
        area = 0
        for cube_s in self.cube_space_list:
            if cube_s.point2.z == ems.point1.z:  ##可能支撑
                if not (cube_s.point1.x >= ems.point2.x or cube_s.point2.x <= ems.point1.x \
                        or cube_s.point1.y >= ems.point2.y or cube_s.point2.y <= ems.point1.y):   ### 相交
                    intersection_x = min(cube_s.point2.x,ems.point2.x) - max(cube_s.point1.x,ems.point1.x)
                    intersection_y = min(cube_s.point2.y,ems.point2.y) - max(cube_s.point1.y,ems.point1.y)
                    intersection = intersection_x*intersection_y
                    area = area + intersection
        ratio = area/ems.get_bottom_area()
        return ratio

    def confirm_cube2ems(self, task: Dict):
        loc: Space = task['loc']
        self.add_cube(task['cube'])
        self.add_cube_space(task['loc'])

        del_list = []
        cur_len = len(self.space_list)

        ### 产生新的ems ###
        for emsID in range(cur_len):
            ems = self.space_list[emsID]
            ins = space_intersect(ems, loc)   ###与loc空间相交的ems空间，会产生新的ems空间。其中loc一定位于一个ems空间里，可能与其他的ems相交。对于相交的，保留相交的在ems内部的Loc空间计算新ems###
            if ins:
                new_ems = self.cal_ems(ems, loc)   #new_ems = self.cal_ems(ems, ins)
                for ne in new_ems:
                    if ne.can_accommodate(Cube(self.low_bound)):
                        self.space_list.append(ne)
                del_list.append(ems) 

        ### 删除与loc相交的ems ###     
        for dl in del_list:
            if dl in self.space_list:
                self.space_list.remove(dl)
        self.space_list.sort(key=lambda x: - x.get_volume()) ## ems体积从打到小排列？ 负的 
        del_list.clear()

        #### 去掉包含的ems ###
        for i in range(len(self.space_list)):
            for j in range(i + 1, len(self.space_list)):
                if embody(self.space_list[i], self.space_list[j]):
                    del_list.append(self.space_list[j])    
        for dl in del_list:
            if dl in self.space_list:
                self.space_list.remove(dl)

        # #### 合并ems ###
        # for i in range(len(self.space_list)):
        #     for j in range(i + 1, len(self.space_list)):
        #         com_ems_y =  combine_ems_y(self.space_list[i], self.space_list[j])
        #         com_ems_x =  combine_ems_x(self.space_list[i], self.space_list[j])
        #         if com_ems_y:
        #             if com_ems_y.can_accommodate(Cube(self.low_bound)):
        #                 self.space_list.append(com_ems_y)
        #         if com_ems_x:
        #             if com_ems_x.can_accommodate(Cube(self.low_bound)):
        #                 self.space_list.append(com_ems_x)                    

        # #### 去掉包含的ems ###
        # for i in range(len(self.space_list)):
        #     for j in range(i + 1, len(self.space_list)):
        #         if embody(self.space_list[i], self.space_list[j]):
        #             del_list.append(self.space_list[j])
        # for dl in del_list:
        #     if dl in self.space_list:
        #         self.space_list.remove(dl)


    def draw_ems(self, plot: CubePlot):
        pc = []
        for s in self.space_list:
            pc.append(plot.update(s.capacity, s.point1.size, color='green', delay=0.5, alpha=0.5))
        for p in pc:
            plot.remove(p)

    def heuristic_method_1(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        max_score = 0
        best_placement = None
        cube_dir = None
        for ems in self.space_list:
            for c in cube:  ## 旋转xy两个方向 ##
                if ems.can_accommodate(c):  ### 通过体积判断cube能否放入EMS中 ###
                    for s in self.cal_cube2ems(c, ems):    #### 每个EMS内部，有四个角可以放置cube,计算每个EMSN内的4个可行性位置s，计算时已加入偏差 ###
                        x_rate = cube.x / ems.capacity[0]
                        y_rate = cube.y / ems.capacity[1]
                        current_score = max(x_rate, y_rate)

                        ### 根据在EMS所占面积打分 ###
                        # current_score = x_rate*y_rate
                        # if 0.8 > current_score > 0.6:    
                        #     current_score = 0.5
                        if current_score > max_score:
                            max_score = current_score
                            best_placement = s
                            cube_dir = c
        return best_placement, cube_dir   #### postion---Space类型， cube_dir---计算xy旋转？

    def heuristic_method_3(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        max_score = 0
        best_placement = None
        cube_dir = None
        for ems in self.space_list:
            for c in cube:  ## 旋转xy两个方向 ##
                if ems.can_accommodate(c): 
                    cor_ems = self.cal_safe_cube2ems(c, ems)
                    if cor_ems:
                        best_placement = cor_ems[0]
                        cube_dir = c
                        return best_placement, cube_dir
                        # for s in cor_ems:    
                        #     x_rate = cube.x / ems.capacity[0]
                        #     y_rate = cube.y / ems.capacity[1]
                        #     current_score = max(x_rate, y_rate)

                            
                        #     # ### 根据在EMS所占体积积打分 ###
                        #     # x_rate = cube.x / ems.capacity[0]
                        #     # y_rate = cube.y / ems.capacity[1]
                        #     # z_rate = cube.z / ems.capacity[2]
                        #     # current_score = x_rate*y_rate*z_rate

                        #     # ### 根据在EMS所占面积打分 ###
                        #     # x_rate = cube.x / ems.capacity[0]
                        #     # y_rate = cube.y / ems.capacity[1]
                        #     # current_score = x_rate*y_rate
                        #     # if 0.8 > current_score > 0.6:    
                        #     #     current_score = 0.5
                        #     if current_score > max_score:
                        #         max_score = current_score
                        #         best_placement = s
                        #         cube_dir = c
        return best_placement, cube_dir   #### postion---Space类型， cube_dir---计算xy旋转？

    # 先铺满底部
    def heuristic_method_2xaxa(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        ##################################
        min_z = 10000
        for tmp in self.space_list:
            if (tmp.point1.z < min_z) and (tmp.can_accommodate(cube[0]) or tmp.can_accommodate(cube[1])):
                min_z = tmp.point1.z

        mini_space_list = []
        # min_z = self.space_list[0].point1.z
        print('min_z: ', min_z)
        for tmp in self.space_list:
            if tmp.point1.z == min_z:
                mini_space_list.append(tmp)
        print('min_z item count: ', len(mini_space_list))
        ##################################
        max_score = 0
        best_placement = None
        cube_dir = None
        for ems in mini_space_list:#self.space_list:
            for c in cube:  ## 旋转xy两个方向 ##
                if ems.can_accommodate(c): 
                    # print('1111111111')
                    cor_ems, _ = self.cal_safe_cube2ems(c, ems)
                    if cor_ems:
                        # print('22222222222')
                        # best_placement = cor_ems[0]
                        # cube_dir = c
                        # return best_placement, cube_dir
                        for s in cor_ems:    
                            x_rate = cube.x / ems.capacity[0]
                            y_rate = cube.y / ems.capacity[1]
                            current_score = max(x_rate, y_rate)

                            
                            # ### 根据在EMS所占体积积打分 ###
                            # x_rate = cube.x / ems.capacity[0]
                            # y_rate = cube.y / ems.capacity[1]
                            # z_rate = cube.z / ems.capacity[2]
                            # current_score = x_rate*y_rate*z_rate

                            # ### 根据在EMS所占面积打分 ###
                            # x_rate = cube.x / ems.capacity[0]
                            # y_rate = cube.y / ems.capacity[1]
                            # current_score = x_rate*y_rate
                            # if 0.8 > current_score > 0.6:    
                            #     current_score = 0.5
                            if current_score > max_score:
                                max_score = current_score
                                best_placement = s
                                cube_dir = c

        return best_placement, cube_dir   #### postion---Space类型， cube_dir---计算xy旋转？
    
    
    ## 先放左下角
    def heuristic_method_2(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        max_score = 0
        best_placement = None
        cube_dir = None
        for ems in self.space_list:
            for c in cube:  ## 旋转xy两个方向 ##
                if ems.can_accommodate(c): 
                    cor_ems,_ = self.cal_safe_cube2ems(c, ems)
                    if cor_ems:
                        for s in cor_ems:    
                            x_rate = cube.x / ems.capacity[0]
                            y_rate = cube.y / ems.capacity[1]
                            current_score = max(x_rate, y_rate)

                            
                            # ### 根据在EMS所占体积积打分 ###
                            # x_rate = cube.x / ems.capacity[0]
                            # y_rate = cube.y / ems.capacity[1]
                            # z_rate = cube.z / ems.capacity[2]
                            # current_score = x_rate*y_rate*z_rate

                            # ### 根据在EMS所占面积打分 ###
                            # x_rate = cube.x / ems.capacity[0]
                            # y_rate = cube.y / ems.capacity[1]
                            # current_score = x_rate*y_rate
                            # if 0.8 > current_score > 0.6:    
                            #     current_score = 0.5

                            if current_score > max_score:
                                max_score = current_score
                                best_placement = s
                                cube_dir = c

        return best_placement, cube_dir   #### postion---Space类型， cube_dir---计算xy旋转？


    ## 先放左下角，并且先放接触面积大的
    def heuristic_method_2xxxaaa(self, cube: Cube):
        self.space_list.sort(key=lambda x: x.point1.z)
        max_score = 0
        best_placement = None
        cube_dir = None
        for ems in self.space_list:
            for c in cube:  ## 旋转xy两个方向 ##
                if ems.can_accommodate(c): 
                    cor_ems, max_ems = self.cal_safe_cube2ems(c, ems)
                    if cor_ems:
                        best_placement = max_ems
                        cube_dir = c


                        # for s in cor_ems:    
                        #     x_rate = cube.x / ems.capacity[0]
                        #     y_rate = cube.y / ems.capacity[1]
                        #     current_score = max(x_rate, y_rate)

                            
                        #     # ### 根据在EMS所占体积积打分 ###
                        #     # x_rate = cube.x / ems.capacity[0]
                        #     # y_rate = cube.y / ems.capacity[1]
                        #     # z_rate = cube.z / ems.capacity[2]
                        #     # current_score = x_rate*y_rate*z_rate

                        #     # ### 根据在EMS所占面积打分 ###
                        #     # x_rate = cube.x / ems.capacity[0]
                        #     # y_rate = cube.y / ems.capacity[1]
                        #     # current_score = x_rate*y_rate
                        #     # if 0.8 > current_score > 0.6:    
                        #     #     current_score = 0.5

                        #     if current_score > max_score:
                        #         max_score = current_score
                        #         best_placement = s
                        #         cube_dir = c

        return best_placement, cube_dir   #### postion---Space类型， cube_dir---计算xy旋转？
                    


def main():
    plot = CubePlot([40, 40, 35])
    ems = Ems([40, 40, 35])
    while True:
        cube = Cube(expand=0.1)
        s = ems.get_suggestion(cube)
        if len(s) == 0:
            break
        loc = s[0]
        cube_ = s[1]
        if loc is None:
            break
        task = {'loc': loc, 'cube': cube}
        print('cube with size: ', cube.size, ', place at: ', loc.get_space())
        ems.confirm_cube2ems(task)
        plot.update(cube_.size, loc.point1.size, cube.expand, delay=0.1)
        # ems.draw_ems(plot)
    print(ems.get_used_ratio())


if __name__ == '__main__':
    # np.random.seed(5)
    main()
