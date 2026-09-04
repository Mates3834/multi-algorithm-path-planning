import matplotlib.pyplot as plt
import numpy as np
from src.environment.grid_map import GridMap
from src.planners.grid_search import astar
from src.smoothing.spline_smoother import smooth_path,collision_free
from src.tracking.pure_pursuit import track_path,tracking_rmse

grid=GridMap.demo()
path=np.asarray(astar(grid,(5,5),(74,52)),float)
smooth=smooth_path(path)
ref=smooth if collision_free(grid,smooth) else path
traj=track_path(ref)

print("Tracking RMSE:",tracking_rmse(ref,traj))

plt.figure()
plt.imshow(grid.occ,cmap="gray_r",origin="lower")
plt.plot(path[:,0],path[:,1],"--",label="A* raw")
plt.plot(ref[:,0],ref[:,1],label="Reference")
plt.plot(traj[:,0],traj[:,1],label="Tracked")
plt.xlabel("x")
plt.ylabel("y")
plt.title("Path Planning and Tracking")
plt.legend()
plt.grid(True)
plt.show()
