from typing import List, Tuple, Dict, Set

Point = Tuple[float, float]
Edge = Tuple[int, int]
Polygon = List[Point] 

VisibilityGraph = Dict[int, Dict[int, float]]
DistanceMap = Dict[int, float]
PreviousMap = Dict[int, int] 

PathResult = Tuple[List[Point], float]