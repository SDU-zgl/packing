from typing import List
from abc import abstractmethod
from .components import Cube, Space, Point


class Container:
    def __init__(self, size: List[float]):
        self.size = size
        self.cube_list = []
        self.cube_space_list = []
        self.used_volume = 0
        self.volume = size[0] * size[1] * size[2]
        self.space_list = []
        self.space_list.append(Space(Point([0, 0, 0]), Point(self.size)))
        self.point_dict = {}

    @abstractmethod
    def get_suggestion(self, cube: Cube):
        pass

    def add_cube(self, cube: Cube):
        self.cube_list.append(cube)
        self.used_volume += cube.get_volume()

    def add_cube_space(self,loc:Space):
        self.cube_space_list.append(loc)

    def get_used_ratio(self):
        return self.used_volume / self.volume

