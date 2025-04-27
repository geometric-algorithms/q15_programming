import math
import heapq
from dataTypes import Point, Edge, Polygon, VisibilityGraph, DistanceMap, PreviousMap, PathResult

class ShortestPathFinder:
    def __init__(self, obstacles: list[Polygon], start_point: Point, end_point: Point):
        self.obstacles = obstacles
        self.start_point = start_point
        self.end_point = end_point
        
        self.all_points = [start_point, end_point]
        self.point_to_index = {start_point: 0, end_point: 1}
        
        current_index = 2
        for obstacle in obstacles:
            for point in obstacle:
                self.all_points.append(point)
                self.point_to_index[point] = current_index
                current_index += 1
        
        self.edges_of_obstacles = set()
        for obstacle in obstacles:
            number_of_points = len(obstacle) 
            for i in range(number_of_points):
                point1 = obstacle[i]
                point2 = obstacle[(i + 1) % number_of_points]
                edge = (min(self.point_to_index[point1], self.point_to_index[point2]), 
                       max(self.point_to_index[point1], self.point_to_index[point2]))
                self.edges_of_obstacles.add(edge)
    
    def can_points_see_each_other(self, index1: int, index2: int) -> bool:
        point1 = self.all_points[index1]
        point2 = self.all_points[index2]
        
        if (min(index1, index2), max(index1, index2)) in self.edges_of_obstacles:
            return True
        
        for obstacle in self.obstacles:
            number_of_points = len(obstacle)
            for i in range(number_of_points):
                obstacle_point1 = obstacle[i]
                obstacle_point2 = obstacle[(i + 1) % number_of_points]
                
                if ((point1[0] == obstacle_point1[0] and point1[1] == obstacle_point1[1]) or 
                    (point1[0] == obstacle_point2[0] and point1[1] == obstacle_point2[1]) or 
                    (point2[0] == obstacle_point1[0] and point2[1] == obstacle_point1[1]) or 
                    (point2[0] == obstacle_point2[0] and point2[1] == obstacle_point2[1])):
                    continue
                
                if self.do_lines_cross(point1, point2, obstacle_point1, obstacle_point2):
                    return False
        
        return True

    def do_lines_cross(self, point1: Point, point2: Point, obstacle_point1: Point, obstacle_point2: Point) -> bool:
        def check_direction(p: Point, q: Point, r: Point) -> int:
            result = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if abs(result) < 1e-10:
                return 0
            return 1 if result > 0 else 2
        
        def is_point_on_line(p: Point, q: Point, r: Point) -> bool:
            return (q[0] <= max(p[0], r[0]) + 1e-10 and q[0] >= min(p[0], r[0]) - 1e-10 and
                    q[1] <= max(p[1], r[1]) + 1e-10 and q[1] >= min(p[1], r[1]) - 1e-10)
        
        dir1 = check_direction(point1, point2, obstacle_point1)
        dir2 = check_direction(point1, point2, obstacle_point2)
        dir3 = check_direction(obstacle_point1, obstacle_point2, point1)
        dir4 = check_direction(obstacle_point1, obstacle_point2, point2)
        
        if dir1 != dir2 and dir3 != dir4:
            return True
        
        if dir1 == 0 and is_point_on_line(point1, obstacle_point1, point2): return True
        if dir2 == 0 and is_point_on_line(point1, obstacle_point2, point2): return True
        if dir3 == 0 and is_point_on_line(obstacle_point1, point1, obstacle_point2): return True
        if dir4 == 0 and is_point_on_line(obstacle_point1, point2, obstacle_point2): return True
        
        if ((point1[0] == obstacle_point1[0] and point1[1] == obstacle_point1[1]) or 
            (point1[0] == obstacle_point2[0] and point1[1] == obstacle_point2[1]) or 
            (point2[0] == obstacle_point1[0] and point2[1] == obstacle_point1[1]) or 
            (point2[0] == obstacle_point2[0] and point2[1] == obstacle_point2[1])):
            return False
        
        return False

    def make_visibility_graph(self) -> VisibilityGraph:
        graph: VisibilityGraph = {i: {} for i in range(len(self.all_points))}
        
        for i in range(len(self.all_points)):
            for j in range(i + 1, len(self.all_points)):
                if self.can_points_see_each_other(i, j):
                    p1 = self.all_points[i]
                    p2 = self.all_points[j]
                    distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                    
                    graph[i][j] = distance
                    graph[j][i] = distance
        
        return graph
    
    def find_shortest_path(self) -> PathResult:
        graph = self.make_visibility_graph()
        
        start_idx = 0
        end_idx = 1
        
        distances = {i: float('inf') for i in range(len(self.all_points))}
        distances[start_idx] = 0
        
        to_visit = [(0, start_idx)]
        
        came_from = {i: None for i in range(len(self.all_points))}
        
        while to_visit:
            current_dist, current = heapq.heappop(to_visit)
            
            if current == end_idx:
                break
            
            if current_dist > distances[current]:
                continue
            
            for next_point, dist in graph[current].items():
                total_dist = current_dist + dist
                
                if total_dist < distances[next_point]:
                    distances[next_point] = total_dist
                    came_from[next_point] = current
                    heapq.heappush(to_visit, (total_dist, next_point))
        
        if distances[end_idx] == float('inf'):
            return [], float('inf')
        
        path = []
        current = end_idx
        while current is not None:
            path.append(self.all_points[current])
            current = came_from[current]
        
        path.reverse()
        
        return path, distances[end_idx]

def compute_shortest_path(obstacles: list[Polygon], start: Point, goal: Point) -> PathResult:
    finder = ShortestPathFinder(obstacles, start, goal)
    return finder.find_shortest_path()