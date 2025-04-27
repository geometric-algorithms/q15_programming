import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as PlotPolygon
from matplotlib.collections import PatchCollection
import sys
import os
from dataTypes import Point, Polygon
from main import ShortestPathFinder, compute_shortest_path

def visualize_path(obstacles, start, goal, path=None, title="Path Visualization", 
                   show_visibility=True, save_path=None, show_plot=True):
    fig, ax = plt.subplots(figsize=(12, 10))
    
    patches = []
    for polygon in obstacles:
        patches.append(PlotPolygon(np.array(polygon)))
    
    p = PatchCollection(patches, alpha=0.4, facecolor='gray', edgecolor='black', linewidth=2)
    ax.add_collection(p)
    
    ax.plot(start[0], start[1], 'go', markersize=12, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')
    
    if path:
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        
        ax.plot(path_x, path_y, 'b-', linewidth=3, label='Shortest Path')
        
        if len(path) > 2:
            waypoints_x = path_x[1:-1]
            waypoints_y = path_y[1:-1]
            ax.plot(waypoints_x, waypoints_y, 'yo', markersize=8, label='Waypoints')
    
    if show_visibility and path:
        path_finder = ShortestPathFinder(obstacles, start, goal)
        # Use make_visibility_graph instead of build_visibility_graph
        visibility_graph = path_finder.make_visibility_graph()
        
        # Make sure we're using the correct attribute names
        for i in range(len(path_finder.all_points)):
            for j in visibility_graph[i]:
                if j > i:
                    p1 = path_finder.all_points[i]
                    p2 = path_finder.all_points[j]
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'c-', alpha=0.3, linewidth=1)
    
    all_points = [point for polygon in obstacles for point in polygon] + [start, goal]
    min_x = min(p[0] for p in all_points) - 1
    max_x = max(p[0] for p in all_points) + 1
    min_y = min(p[1] for p in all_points) - 1
    max_y = max(p[1] for p in all_points) + 1
    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_title(title, fontsize=16)
    ax.set_xlabel('X', fontsize=14)
    ax.set_ylabel('Y', fontsize=14)
    ax.legend(loc='upper right', fontsize=12)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show_plot:
        plt.tight_layout()
        plt.show()
    else:
        plt.close(fig)

def plot_test_case(test_num, show_visibility=True):
    test_cases = {
        1: {
            "title": "Test Case 1: Multiple Triangular Obstacles",
            "obstacles": [
                [(2, 1), (1, 3), (3, 3)],
                [(4, 0), (4, 2), (6, 1)],
                [(5, 4), (7, 6), (7, 2)],
                [(8, 3), (10, 4), (9, 1)]
            ],
            "start": (0, 0),
            "goal": (12, 5)
        },
        2: {
            "title": "Test Case 2: Complex scenario with multiple obstacles",
            "obstacles": [
                [(2, 2), (2, 5), (4, 5), (4, 2)],
                [(6, 1), (7, 3), (9, 3), (10, 1), (8, 0)],
                [(5, 6), (6, 8), (8, 8), (9, 6), (7, 5)],
                [(12, 4), (13, 6), (15, 6), (16, 4), (14, 3)]
            ],
            "start": (1 , 1 ),
            "goal": (10, 8)
        },
        3: {
            "title": "Test Case 3: Narrow passage between obstacles",
            "obstacles": [
                [(2, 0), (2, 4), (4, 4), (4, 0)],
                [(6, 6), (6, 10), (8, 10), (8, 6)],
                [(10, 0), (10, 4), (12, 4), (12, 0)]
            ],
            "start": (0, 2),
            "goal": (14, 8)
        },
        4: {
            "title": "Test Case 4: Maze-like environment",
            "obstacles": [
                [(2, 0), (2, 8), (3, 8), (3, 0)],
                [(5, 2), (5, 10), (6, 10), (6, 2)],
                [(8, 0), (8, 8), (9, 8), (9, 0)],
                [(11, 2), (11, 10), (12, 10), (12, 2)],
                [(14, 0), (14, 8), (15, 8), (15, 0)]
            ],
            "start": (0, 5),
            "goal": (17, 5)
        },
        5: {
            "title": "Test Case 5: No feasible path",
            "obstacles": [
                [(2, 0), (2, 10), (3, 10), (3, 0)],
                [(3, 9), (15, 9), (15, 10), (3, 10)],
                [(15, 0), (15, 10), (16, 10), (16, 0)],
                [(3, 0), (15, 0), (15, 1), (3, 1)]
            ],
            "start": (5, 5),
            "goal": (20, 5)
        }
    }
    
    if test_num not in test_cases:
        print(f"Error: Test case {test_num} not found.")
        return
    
    test_case = test_cases[test_num]
    
    path, distance = compute_shortest_path(
        test_case["obstacles"], 
        test_case["start"], 
        test_case["goal"]
    )
    
    os.makedirs("plots", exist_ok=True)
    
    visualize_path(
        test_case["obstacles"],
        test_case["start"],
        test_case["goal"],
        path,
        title=f"{test_case['title']}\nDistance: {distance:.2f}",
        show_visibility=show_visibility,
        save_path=f"plots/test_case_{test_num}.png"
    )
    
    print(f"Test case {test_num} plotted and saved to plots/test_case_{test_num}.png")
    print(f"Path length: {len(path)} points, Total distance: {distance:.2f}")

def plot_all_test_cases(show_visibility=True):
    for i in range(1, 6):
        plot_test_case(i, show_visibility)

def create_comparison_plot():
    fig, axs = plt.subplots(2, 3, figsize=(18, 12))
    axs = axs.flatten()
    
    test_cases = {
        1: {
            "title": "Test Case 1: Multiple Triangular Obstacles",
            "obstacles": [
                [(2, 1), (1, 3), (3, 3)],
                [(4, 0), (4, 2), (6, 1)],
                [(5, 4), (7, 6), (7, 2)],
                [(8, 3), (10, 4), (9, 1)]
            ],
            "start": (0, 0),
            "goal": (12, 5)
        },
        2: {
            "title": "Test Case 2: Complex obstacles",
            "obstacles": [
                [(2, 2), (2, 5), (4, 5), (4, 2)],
                [(6, 1), (7, 3), (9, 3), (10, 1), (8, 0)],
                [(5, 6), (6, 8), (8, 8), (9, 6), (7, 5)],
                [(12, 4), (13, 6), (15, 6), (16, 4), (14, 3)]
            ],
            "start": (1, 1),
            "goal": (10, 8)
        },
        3: {
            "title": "Test Case 3: Narrow passage",
            "obstacles": [
                [(2, 0), (2, 4), (4, 4), (4, 0)],
                [(6, 6), (6, 10), (8, 10), (8, 6)],
                [(10, 0), (10, 4), (12, 4), (12, 0)]
            ],
            "start": (0, 2),
            "goal": (14, 8)
        },
        4: {
            "title": "Test Case 4: Maze-like",
            "obstacles": [
                [(2, 0), (2, 8), (3, 8), (3, 0)],
                [(5, 2), (5, 10), (6, 10), (6, 2)],
                [(8, 0), (8, 8), (9, 8), (9, 0)],
                [(11, 2), (11, 10), (12, 10), (12, 2)],
                [(14, 0), (14, 8), (15, 8), (15, 0)]
            ],
            "start": (0, 5),
            "goal": (17, 5)
        },
        5: {
            "title": "Test Case 5: No feasible path",
            "obstacles": [
                [(2, 0), (2, 10), (3, 10), (3, 0)],
                [(3, 9), (15, 9), (15, 10), (3, 10)],
                [(15, 0), (15, 10), (16, 10), (16, 0)],
                [(3, 0), (15, 0), (15, 1), (3, 1)]
            ],
            "start": (5, 5),
            "goal": (20, 5)
        }
    }
    
    for i in range(1, 6):
        test_case = test_cases[i]
        ax = axs[i-1]
        
        path, distance = compute_shortest_path(
            test_case["obstacles"],     
            test_case["start"], 
            test_case["goal"]
        )
        
        patches = []
        for polygon in test_case["obstacles"]:
            patches.append(PlotPolygon(np.array(polygon)))
        
        p = PatchCollection(patches, alpha=0.4, facecolor='gray', edgecolor='black', linewidth=1.5)
        ax.add_collection(p)
        
        ax.plot(test_case["start"][0], test_case["start"][1], 'go', markersize=8, label='Start')
        ax.plot(test_case["goal"][0], test_case["goal"][1], 'ro', markersize=8, label='Goal')
        
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            
            ax.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
            
            if len(path) > 2:
                waypoints_x = path_x[1:-1]
                waypoints_y = path_y[1:-1]
                ax.plot(waypoints_x, waypoints_y, 'yo', markersize=5, label='Waypoints')
        
        all_points = [point for polygon in test_case["obstacles"] for point in polygon] + [test_case["start"], test_case["goal"]]
        min_x = min(p[0] for p in all_points) - 1
        max_x = max(p[0] for p in all_points) + 1
        min_y = min(p[1] for p in all_points) - 1
        max_y = max(p[1] for p in all_points) + 1
        
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title(f"{test_case['title']}\nDistance: {distance:.2f}", fontsize=12)
        
        if i == 1:
            ax.legend(loc='upper right', fontsize=10)
    
    axs[5].axis('off')
    
    plt.suptitle("Comparison of Shortest Paths Using Visibility Graphs", fontsize=18)
    
    os.makedirs("plots", exist_ok=True)
    plt.savefig("plots/comparison.png", dpi=300, bbox_inches='tight')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
    
    print("Comparison plot saved to plots/comparison.png")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        plot_all_test_cases()
    elif len(sys.argv) == 2:
        if sys.argv[1] == "all":
            plot_all_test_cases()
        elif sys.argv[1] == "comparison":
            create_comparison_plot()
        else:
            try:
                test_num = int(sys.argv[1])
                plot_test_case(test_num)
            except ValueError:
                print("Error: Invalid test case number. Please provide a number between 1 and 5.")
    elif len(sys.argv) == 3 and sys.argv[1] == "novis":
        try:
            test_num = int(sys.argv[2])
            plot_test_case(test_num, show_visibility=False)
        except ValueError:
            print("Error: Invalid test case number. Please provide a number between 1 and 5.")
    else:
        print("Usage:")
        print("  python plot.py                  # Plot all test cases")
        print("  python plot.py all              # Plot all test cases")
        print("  python plot.py <test_num>       # Plot specific test case (1-5)")
        print("  python plot.py novis <test_num> # Plot without visibility graph")
        print("  python plot.py comparison       # Create comparison plot of all test cases")