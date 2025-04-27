import math
import heapq
from dataTypes import Point, Polygon, VisibilityGraph, PathResult
from typing import List

class ShortestPathFinder:
    def __init__(self, obstacles: List[Polygon], start_point: Point, end_point: Point):
        self.obstacles = obstacles
        self.start_point = start_point
        self.end_point = end_point
        
        self.all_points = [start_point, end_point]
        self.point_numbers = {start_point: 0, end_point: 1}
        
        next_number = 2
        for shape in obstacles:
            for corner in shape:
                self.all_points.append(corner)
                self.point_numbers[corner] = next_number
                next_number += 1
        
        self.shape_edges = set()
        for shape in obstacles:
            total_corners = len(shape)
            for i in range(total_corners):
                corner1 = shape[i]
                corner2 = shape[(i + 1) % total_corners]
                small_num = min(self.point_numbers[corner1], self.point_numbers[corner2])
                big_num = max(self.point_numbers[corner1], self.point_numbers[corner2])
                self.shape_edges.add((small_num, big_num))
    
    def can_see(self, point1_num: int, point2_num: int) -> bool:
        point1 = self.all_points[point1_num]
        point2 = self.all_points[point2_num]
        
        if (min(point1_num, point2_num), max(point1_num, point2_num)) in self.shape_edges:
            return True
        
        for shape in self.obstacles:
            total_corners = len(shape)
            for i in range(total_corners):
                shape_point1 = shape[i]
                shape_point2 = shape[(i + 1) % total_corners]
                
                if ((point1[0] == shape_point1[0] and point1[1] == shape_point1[1]) or 
                    (point1[0] == shape_point2[0] and point1[1] == shape_point2[1]) or 
                    (point2[0] == shape_point1[0] and point2[1] == shape_point1[1]) or 
                    (point2[0] == shape_point2[0] and point2[1] == shape_point2[1])):
                    continue
                
                if self.lines_cross(point1, point2, shape_point1, shape_point2):
                    return False
        
        return True
    
    def lines_cross(self, p1: Point, p2: Point, q1: Point, q2: Point) -> bool:
        def get_turn(p: Point, q: Point, r: Point) -> int:
            result = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if abs(result) < 1e-10:
                return 0
            return 1 if result > 0 else 2
        
        def is_between(p: Point, q: Point, r: Point) -> bool:
            return (q[0] <= max(p[0], r[0]) + 1e-10 and q[0] >= min(p[0], r[0]) - 1e-10 and
                    q[1] <= max(p[1], r[1]) + 1e-10 and q[1] >= min(p[1], r[1]) - 1e-10)
        
        turn1 = get_turn(p1, p2, q1)
        turn2 = get_turn(p1, p2, q2)
        turn3 = get_turn(q1, q2, p1)
        turn4 = get_turn(q1, q2, p2)
        
        if turn1 != turn2 and turn3 != turn4:
            return True
        
        if turn1 == 0 and is_between(p1, q1, p2): return True
        if turn2 == 0 and is_between(p1, q2, p2): return True
        if turn3 == 0 and is_between(q1, p1, q2): return True
        if turn4 == 0 and is_between(q1, p2, q2): return True
        
        return False
    
    def make_map(self) -> VisibilityGraph:
        connections = {i: {} for i in range(len(self.all_points))}
        
        for i in range(len(self.all_points)):
            for j in range(i + 1, len(self.all_points)):
                if self.can_see(i, j):
                    point1 = self.all_points[i]
                    point2 = self.all_points[j]
                    dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
                    
                    connections[i][j] = dist
                    connections[j][i] = dist
        
        return connections
    
    def find_path(self) -> PathResult:
        connections = self.make_map()
        
        start_num = 0
        end_num = 1
        
        distances = {i: float('inf') for i in range(len(self.all_points))}
        distances[start_num] = 0
        
        to_check = [(0, start_num)]
        
        came_from = {i: None for i in range(len(self.all_points))}
        
        while to_check:
            current_dist, current = heapq.heappop(to_check)
            
            if current == end_num:
                break
            
            if current_dist > distances[current]:
                continue
            
            for next_point, weight in connections[current].items():
                new_dist = current_dist + weight
                
                if new_dist < distances[next_point]:
                    distances[next_point] = new_dist
                    came_from[next_point] = current
                    heapq.heappush(to_check, (new_dist, next_point))
        
        if distances[end_num] == float('inf'):
            return [], float('inf')
        
        path = []
        current = end_num
        while current is not None:
            path.append(self.all_points[current])
            current = came_from[current]
        
        path.reverse()
        return path, distances[end_num]

def make_bigger(shape: Polygon, size: float = 1.0) -> Polygon:
    bigger_shape = []
    total_corners = len(shape)
    
    for i in range(total_corners):
        p1 = shape[i]
        p2 = shape[(i + 1) % total_corners]
        
        line = (p2[0] - p1[0], p2[1] - p1[1])
        length = math.sqrt(line[0]**2 + line[1]**2)
        
        if length > 0:
            outward = (-line[1] / length, line[0] / length)
        else:
            outward = (0, 0)
        
        move = (outward[0] * size, outward[1] * size)
        bigger_shape.append((p1[0] + move[0], p1[1] + move[1]))
    
    return bigger_shape

class RobotPathFinder:
    def __init__(self, obstacles: List[Polygon], start: Point, end: Point):
        self.obstacles = obstacles
        self.start = start
        self.end = end
        
        self.bigger_obstacles = [make_bigger(shape) for shape in obstacles]
        self.point_finder = ShortestPathFinder(self.bigger_obstacles, start, end)
    
    def find_path(self) -> PathResult:
        return self.point_finder.find_path()

def find_robot_path(obstacles: List[Polygon], start: Point, end: Point) -> PathResult:
    finder = RobotPathFinder(obstacles, start, end)
    return finder.find_path()

if __name__ == "__main__":
    shapes = [
        [(1, 1), (1, 3), (3, 3), (3, 1)],
        [(5, 2), (7, 4), (9, 2), (7, 0)]
    ]
    
    start = (0, 0)
    end = (10, 3)
    
    path, total_distance = find_robot_path(shapes, start, end)
    
    print("Robot's Path:")
    for i, point in enumerate(path):
        print(f"Stop {i}: {point}")
    print(f"Total distance: {total_distance}")