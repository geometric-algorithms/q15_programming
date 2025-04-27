import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as PlotPolygon, Circle
from matplotlib.collections import PatchCollection
import os
from typing import List
from dataTypes import Point, Polygon
from main import ShortestPathFinder
from unitDisc import RobotPathFinder, make_bigger, find_robot_path

def visualize_unit_disk_path(obstacles: List[Polygon], start: Point, goal: Point, 
                            title: str = "Unit Disk Robot Path Planning", 
                            save_path: str = None, show_plot: bool = True):
    
    path, distance = find_robot_path(obstacles, start, goal)
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    original_patches = []
    for polygon in obstacles:
        original_patches.append(PlotPolygon(np.array(polygon)))
    
    p_original = PatchCollection(original_patches, alpha=0.3, facecolor='gray', edgecolor='black', linewidth=1.5)
    ax.add_collection(p_original)
    
    expanded_patches = []
    for polygon in obstacles:
        expanded = make_bigger(polygon)
        expanded_patches.append(PlotPolygon(np.array(expanded)))
    
    p_expanded = PatchCollection(expanded_patches, alpha=0.2, facecolor='red', edgecolor='red', linewidth=1, linestyle='--')
    ax.add_collection(p_expanded)
    
    ax.plot(start[0], start[1], 'go', markersize=12, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')
    
    start_circle = Circle(start, radius=1, fill=True, alpha=0.3, color='green')
    goal_circle = Circle(goal, radius=1, fill=True, alpha=0.3, color='red')
    ax.add_patch(start_circle)
    ax.add_patch(goal_circle)
     
    if path:
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        
        ax.plot(path_x, path_y, 'b-', linewidth=3, label='Shortest Path')
        
        if len(path) > 2:
            waypoints_x = path_x[1:-1]
            waypoints_y = path_y[1:-1]
            ax.plot(waypoints_x, waypoints_y, 'yo', markersize=8, label='Waypoints')
            
            for i in range(1, len(path) - 1):
                waypoint_circle = Circle(path[i], radius=1, fill=True, alpha=0.1, color='blue')
                ax.add_patch(waypoint_circle)
    
    all_points = [point for polygon in obstacles for point in polygon] + [start, goal]
    min_x = min(p[0] for p in all_points) - 2
    max_x = max(p[0] for p in all_points) + 2
    min_y = min(p[1] for p in all_points) - 2
    max_y = max(p[1] for p in all_points) + 2
    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_title(f"{title}\nDistance: {distance:.2f}", fontsize=16)
    ax.set_xlabel('X', fontsize=14)
    ax.set_ylabel('Y', fontsize=14)
    
    ax.plot([], [], 'gray', linewidth=2, label='Original Obstacles')
    ax.plot([], [], 'r--', linewidth=2, label='Expanded Obstacles')
    ax.plot([], [], 'b-', linewidth=3, label='Path')
    ax.legend(loc='upper right', fontsize=12)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show_plot:
        plt.tight_layout()
        plt.show()
    else:
        plt.close(fig)

def run_test_cases():
    os.makedirs("unit_disk_plots", exist_ok=True)
    
    obstacles1 = [
        [(1, 1), (1, 3), (3, 3), (3, 1)],
        [(5, 2), (7, 4), (9, 2), (7, 0)]
    ]
    start1 = (0, 0)
    goal1 = (10, 3)
    
    visualize_unit_disk_path(
        obstacles1, start1, goal1,
        title="Test Case 1: Simple scenario with two obstacles",
        save_path="unit_disk_plots/test_case_1.png"
    )
    
    obstacles2 = [
        [(2, 0), (2, 4), (4, 4), (4, 0)],
        [(6, 6), (6, 10), (8, 10), (8, 6)],
        [(10, 0), (10, 4), (12, 4), (12, 0)]
    ]
    start2 = (0, 2)
    goal2 = (14, 8)
    
    visualize_unit_disk_path(
        obstacles2, start2, goal2,
        title="Test Case 2: Narrow passage between obstacles",
        save_path="unit_disk_plots/test_case_2.png"
    )
    
    obstacles3 = [
        [(2, 2), (2, 5), (4, 5), (4, 2)],
        [(6, 1), (7, 3), (9, 3), (10, 1), (8, 0)],
        [(5, 6), (6, 8), (8, 8), (9, 6), (7, 5)],
        [(12, 4), (13, 6), (15, 6), (16, 4), (14, 3)]
    ]
    start3 = (1, 1)
    goal3 = (15, 7)
    
    visualize_unit_disk_path(
        obstacles3, start3, goal3,
        title="Test Case 3: Complex environment",
        save_path="unit_disk_plots/test_case_3.png"
    )
    
    print("All test cases completed. Results saved in unit_disk_plots directory.")

def create_comparison_plot():
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    
    test_cases = [
        {
            "title": "Simple scenario",
            "obstacles": [
                [(1, 1), (1, 3), (3, 3), (3, 1)],
                [(5, 2), (7, 4), (9, 2), (7, 0)]
            ],
            "start": (0, 0),
            "goal": (10, 3)
        },
        {
            "title": "Narrow passage",
            "obstacles": [
                [(2, 0), (2, 4), (4, 4), (4, 0)],
                [(6, 6), (6, 10), (8, 10), (8, 6)],
                [(10, 0), (10, 4), (12, 4), (12, 0)]
            ],
            "start": (0, 2),
            "goal": (14, 8)
        },
        {
            "title": "Complex environment",
            "obstacles": [
                [(2, 2), (2, 5), (4, 5), (4, 2)],
                [(6, 1), (7, 3), (9, 3), (10, 1), (8, 0)],
                [(5, 6), (6, 8), (8, 8), (9, 6), (7, 5)],
                [(12, 4), (13, 6), (15, 6), (16, 4), (14, 3)]
            ],
            "start": (1, 1),
            "goal": (15, 7)
        }
    ]
    
    for i, test_case in enumerate(test_cases):
        ax = axs[i]
        
        obstacles = test_case["obstacles"]
        start = test_case["start"]
        goal = test_case["goal"]
        
        path, distance = find_robot_path(obstacles, start, goal)
        
        original_patches = []
        for polygon in obstacles:
            original_patches.append(PlotPolygon(np.array(polygon)))
        
        p_original = PatchCollection(original_patches, alpha=0.3, facecolor='gray', edgecolor='black', linewidth=1.5)
        ax.add_collection(p_original)
        
        expanded_patches = []
        for polygon in obstacles:
            expanded = make_bigger(polygon)
            expanded_patches.append(PlotPolygon(np.array(expanded)))
        
        p_expanded = PatchCollection(expanded_patches, alpha=0.2, facecolor='red', edgecolor='red', linewidth=1, linestyle='--')
        ax.add_collection(p_expanded)
        
        ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
        
        start_circle = Circle(start, radius=1, fill=True, alpha=0.3, color='green')
        goal_circle = Circle(goal, radius=1, fill=True, alpha=0.3, color='red')
        ax.add_patch(start_circle)
        ax.add_patch(goal_circle)
        
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            ax.plot(path_x, path_y, 'b-', linewidth=2)
            
            if len(path) > 2:
                waypoints_x = path_x[1:-1]
                waypoints_y = path_y[1:-1]
                ax.plot(waypoints_x, waypoints_y, 'yo', markersize=6)
        
        all_points = [point for polygon in obstacles for point in polygon] + [start, goal]
        min_x = min(p[0] for p in all_points) - 2
        max_x = max(p[0] for p in all_points) + 2
        min_y = min(p[1] for p in all_points) - 2
        max_y = max(p[1] for p in all_points) + 2
        
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title(f"{test_case['title']}\nDistance: {distance:.2f}", fontsize=12)
        
        if i == 0:
            ax.plot([], [], 'gray', linewidth=2, label='Original')
            ax.plot([], [], 'r--', linewidth=1, label='Expanded')
            ax.plot([], [], 'b-', linewidth=2, label='Path')
            ax.legend(loc='upper right', fontsize=10)
    
    plt.suptitle("Unit Disk Robot Path Planning", fontsize=16)
    
    os.makedirs("unit_disk_plots", exist_ok=True)
    plt.savefig("unit_disk_plots/comparison.png", dpi=300, bbox_inches='tight')
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()
    
    print("Comparison plot saved to unit_disk_plots/comparison.png")

if __name__ == "__main__":
    run_test_cases()
    create_comparison_plot()