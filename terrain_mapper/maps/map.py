import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.interpolate import griddata
import cv2

# --- Load CSV ---
csv_path = '/home/mocha1410/ros2_ws/src/terrain_mapper/terrain_mapper/maps/terrain_map.csv'

x_vals = []
y_vals = []
height_vals = []

with open(csv_path, newline='') as csvfile:
    
    reader = csv.DictReader(csvfile)
    for row in reader:
        x_vals.append(float(row['x']))
        y_vals.append(float(row['y']))
        height_vals.append(float(row['z']))

x = np.array(x_vals)
y = np.array(y_vals)
z = np.array(height_vals)

# --- Step 1: Remove statistical outliers ---
def remove_outliers(x, y, z, k=30, threshold=2.0):
    from sklearn.neighbors import NearestNeighbors
    pts = np.column_stack((x, y, z))
    nbrs = NearestNeighbors(n_neighbors=k).fit(pts)
    distances, _ = nbrs.kneighbors(pts)
    mean_distances = distances[:, 1:].mean(axis=1)
    std = mean_distances.std()
    mean = mean_distances.mean()
    mask = mean_distances < (mean + threshold * std)
    return x[mask], y[mask], z[mask]

x, y, z = remove_outliers(x, y, z)

# --- Step 2: Grid interpolation ---
grid_res = 0.01  # 1cm resolution
xi = np.arange(min(x), max(x), grid_res)
yi = np.arange(min(y), max(y), grid_res)
xi, yi = np.meshgrid(xi, yi)
zi = griddata((x, y), z, (xi, yi), method='cubic')

# --- Step 3: Bilateral smoothing (only on valid data) ---
zi_masked = np.nan_to_num(zi, nan=np.nanmean(zi))  # Replace NaNs before smoothing
zi_smoothed = cv2.bilateralFilter(zi_masked.astype(np.float32), d=9, sigmaColor=0.5, sigmaSpace=5.0)

# --- Plotting ---
plt.figure(figsize=(10, 8))
t = plt.contourf(xi, yi, zi_smoothed, levels=20, cmap='terrain')
plt.colorbar(t, label='Height (m)')

# --- Convex Hull outline ---
points = np.column_stack((x, y))
hull = ConvexHull(points)
plt.plot(points[hull.vertices, 0], points[hull.vertices, 1], 'k--', lw=1.5, label='Scan Boundary')

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Smoothed Terrain Map (Denoised & Interpolated)")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.show()
