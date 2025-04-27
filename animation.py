import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Define the original polygon (square)
polygon = np.array([(1, 1), (4, 1), (4, 4), (1, 4)])

# Function to compute offset polygon correctly
def make_bigger(shape, size=1.0):
    total_corners = len(shape)
    expanded_edges = []
    
    # Calculate offset edges
    for i in range(total_corners):
        p1 = shape[i]
        p2 = shape[(i + 1) % total_corners]
        
        # Calculate the edge vector
        edge = p2 - p1
        length = np.sqrt(np.sum(edge**2))
        
        # Calculate the outward normal vector
        if length > 0:
            outward = np.array([-edge[1], edge[0]]) / length
        else:
            outward = np.array([0, 0])
        
        # Offset the edge along the normal
        offset_p1 = p1 + outward * size
        offset_p2 = p2 + outward * size
        
        expanded_edges.append((offset_p1, offset_p2))
    
    # Connect the offset edges at their intersections
    bigger_shape = []
    for i in range(total_corners):
        line1 = expanded_edges[i]
        line2 = expanded_edges[(i + 1) % total_corners]
        
        # Find intersection of adjacent offset edges
        intersection = line_intersection(line1[0], line1[1], line2[0], line2[1])
        if intersection is not None:
            bigger_shape.append(intersection)
        else:
            # Fallback if no intersection (rare case)
            bigger_shape.append(line1[1])
    
    return np.array(bigger_shape)

# Function to find intersection of two line segments
def line_intersection(p1, p2, p3, p4):
    # Convert to homogeneous coordinates
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
    # Calculate determinants
    D = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    if abs(D) < 1e-10:  # Lines are parallel
        return None
    
    # Calculate intersection point
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / D
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / D
    
    return np.array([px, py])

# Setup plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)
ax.set_aspect('equal')
ax.set_title('Polygon Expansion Animation')

# Plot original polygon
poly_closed = np.vstack([polygon, polygon[0]])  # Close the polygon
original_poly, = ax.plot(poly_closed[:, 0], poly_closed[:, 1], 'b-', label='Original Polygon')

# Plot expanded polygon (initially empty)
expanded_poly, = ax.plot([], [], 'r-', label='Expanded Polygon')

# Animation function
size_max = 1.0
frames = 100
def animate(frame):
    size = size_max * frame / frames
    bigger = make_bigger(polygon, size)
    
    # Close the polygon by appending the first point
    bigger_closed = np.vstack([bigger, bigger[0]])
    
    # Update the expanded polygon data
    expanded_poly.set_data(bigger_closed[:, 0], bigger_closed[:, 1])
    return expanded_poly,

# Add legend
ax.legend()

# Create animation
ani = FuncAnimation(fig, animate, frames=frames+1, interval=50, blit=True)
plt.grid(True)
plt.show()