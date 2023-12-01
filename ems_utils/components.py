from typing import Any, Dict, List, Tuple, Union
import numpy as np
import itertools


class Cube:
    # preset_list = [[2.6, 1.6, 1.8], [2.8, 1.8, 2], [3.5, 2.3, 2], [3.9, 3.7, 2.1], [4.2, 2.8, 2.3], [4.7, 3.2, 2.6]]
    # preset_list = [[2.6, 1.6, 2], [2.8, 1.8, 2], [3.5, 2.3, 2], [3.9, 3.7, 2], [4.2, 2.8, 2], [4.7, 3.2, 2]]
    
    # preset_list = [[2.6, 1.6, 1], [2.8, 1.8, 2], [3.5, 2.3, 2], [3.9, 3.7, 3], [4.2, 2.8, 3], [4.7, 3.2, 3]]
    preset_list = [[7, 6, 4], [7, 6, 6], [7, 7, 4], [7, 7, 6], [8, 6, 4], [8, 6, 6], [8, 8, 4], [8, 8, 6], [8, 8, 8], [10, 6, 4], [10, 6, 6], [10, 8, 4], [10, 8, 6], [10, 8, 8]]


    def __init__(self, size: List[float] = None, expand=0.2): #expand=0.5
        if size is None:
            # size = [np.random.randint(2, 5) for i in range(3)]
            # size = self.preset_list[np.random.randint(0, 3)]
            size = self.preset_list[np.random.randint(0, 14)]#14
            # size = [1.8, 1.8, 2]
        if not len(size) == 3:
            assert 'cube must be 3D'
        self.expand = expand
        self.size = size
        self.x = size[0] + 2 * self.expand
        self.y = size[1] + 2 * self.expand
        self.z = size[2]
        # self.perm = itertools.permutations(self.size)
        # self.perm = list(set(self.perm))
        self.perm = [[self.x, self.y, self.z], [self.y, self.x, self.z]]
        # self.perm = [[self.x, self.y, self.z]]

    def get_volume(self):
        return self.size[0] * self.size[1] * self.size[2]

    def __len__(self):
        return len(self.perm)

    def __getitem__(self, item):
        return Cube(list(self.perm[item]), expand=0)


class Point:
    def __init__(self, size: List[float] = None):
        if not len(size) == 3:
            assert 'point must be 3D coordinates'
        if size is None:
            size = [0, 0, 0]
        self.size = size
        self.x = size[0]
        self.y = size[1]
        self.z = size[2]

    def get_point(self):
        return self.size


class Space:
    def __init__(self, point1: Point, point2: Point):
        self.point1 = point1
        self.point2 = point2
        self.capacity = [p2 - p1 for p1, p2 in zip(self.point1.size, self.point2.size)]

    def can_accommodate(self, cube: Cube) -> bool:
        for c, s in zip(self.capacity, cube.size):
            if c < s:
                return False
        return True

    def get_space(self):
        return self.point1.get_point() + self.point2.get_point()

    def get_volume(self):
        return self.capacity[0] * self.capacity[1] * self.capacity[2]
    
    def get_bottom_area(self):
        return self.capacity[0] * self.capacity[1]


class Bin_container:
    def __init__(self, length=10, width=10, height=10):
        """Constructor for Bin"""
        self.length = length
        self.width = width
        self.height = height
        self.size = (length,width,height)
        self.total_cubes = 0  # number of total items in one bin
        self.cubes = []  # item in one bin 
        self.unplaced_cubes = []

    def add_cubes(self, cube: Cube ):
        self.cubes.append(cube)
  
    def get_volume(self):
        return self.length * self.height * self.width

    def get_filling_ratio(self):
        total_filling_volume = 0
        total_filling_ratio = 0

        for cube in self.cubes:
            total_filling_volume += cube.get_volume()

        total_filling_ratio = total_filling_volume / self.get_volume()
        return total_filling_ratio

    def get_cubes_num(self):
        return len(self.cubes)

### 判断ems与Loc相交 ###    
def space_intersect(space1: Space, space2: Space, low_bound=0.1):
    x_ems1, y_ems1, z_ems1, x_ems2, y_ems2, z_ems2 = space1.point1.size + space1.point2.size
    x_emp1, y_emp1, z_emp1, x_emp2, y_emp2, z_emp2 = space2.point1.size + space2.point2.size

    ### ems空间不变，将loc超出ems的部分删除，只保留在ems内的部分，如果有删过头的，会返回none ###  -----> 什么作用 ?
    if x_ems1 > x_emp1:
        x_emp1 = x_ems1
    if y_ems1 > y_emp1:
        y_emp1 = y_ems1
    if z_ems1 > z_emp1:
        z_emp1 = z_ems1
    if x_ems2 < x_emp2:
        x_emp2 = x_ems2
    if y_ems2 < y_emp2:
        y_emp2 = y_ems2
    if z_ems2 < z_emp2:
        z_emp2 = z_ems2

    if x_emp1 > x_emp2:
        x_emp1 = x_emp2
    if y_emp1 > y_emp2:
        y_emp1 = y_emp2
    if z_emp1 > z_emp2:
        z_emp1 = z_emp2
    if abs(x_emp1 - x_emp2) < low_bound \
            or abs(y_emp1 - y_emp2) < low_bound \
            or abs(z_emp1 - z_emp2) < low_bound:
        return None
    return Space(Point([x_emp1, y_emp1, z_emp1]), Point([x_emp2, y_emp2, z_emp2]))

### 删除相互包含的ems ###
def embody(space1: Space, space2: Space):
    if space1.point1.x <= space2.point1.x:
        if space1.point1.y <= space2.point1.y:
            if space1.point1.z <= space2.point1.z:
                if space1.point2.x >= space2.point2.x:
                    if space1.point2.y >= space2.point2.y:
                        if space1.point2.z >= space2.point2.z:
                            return True
    return False

### 合并Ems ###
def combine_ems_y(space1: Space, space2: Space,low_bound=0.1):
    if space1.point1.z == space2.point1.z:   ### point2.z一定相等？ 为容器最高点
        if not ((space1.point1.y > space2.point2.y) or (space1.point2.y < space2.point1.y)):
        ### 按y接触合并 ： ###
            x1 = max(space1.point1.x , space2.point1.x)
            x2 = min(space1.point2.x , space2.point2.x)
            y1 = min(space1.point1.y , space2.point1.y)
            y2 = max(space1.point2.y , space2.point2.y)
            z1 = space1.point1.z
            z2 = space2.point2.z
        ### 判断是否合并错误 ###
            if x1 > x2 :
                x1 = x2 
            if y1 > y2 :
                y1 = y2
            if z1 > z2 :
                z1 = z2
            if abs(x1 - x2) < low_bound or abs(y1 - y2) < low_bound or abs(z1 - z2) < low_bound:
                return None
            return Space(Point([x1,y1,z1]), Point([x2,y2,z2]))
        return None
    return None

def combine_ems_x(space1: Space, space2: Space,low_bound=0.1):
    if space1.point1.z == space2.point1.z:   ### point2.z一定相等？ 为容器最高点
        if not ((space1.point1.x > space2.point2.x) or (space1.point2.x < space2.point1.x)):
        ### 按X接触合并 ： ###
            x1 = min(space1.point1.x , space2.point1.x)
            x2 = max(space1.point2.x , space2.point2.x)
            y1 = max(space1.point1.y , space2.point1.y)
            y2 = min(space1.point2.y , space2.point2.y)
            z1 = space1.point1.z
            z2 = space2.point2.z
        ### 判断是否合并错误 ###
            if x1 > x2 :
                x1 = x2 
            if y1 > y2 :
                y1 = y2
            if z1 > z2 :
                z1 = z2
            if abs(x1 - x2) < low_bound or abs(y1 - y2) < low_bound or abs(z1 - z2) < low_bound:
                return None
            return Space(Point([x1,y1,z1]), Point([x2,y2,z2]))
        return None
    return None

# def main():
#     cube = Cube([2, 2, 2])
#     for i in cube:
#         print(i.size)


# if __name__ == '__main__':
#     main()
