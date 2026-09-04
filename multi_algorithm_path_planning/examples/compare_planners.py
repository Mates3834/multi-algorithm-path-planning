import time
import numpy as np
from src.environment.grid_map import GridMap
from src.planners.grid_search import astar,dijkstra
from src.planners.rrt_star import rrt_star
from src.planners.hybrid_astar import hybrid_astar
from src.smoothing.spline_smoother import smooth_path,collision_free
from src.evaluation.metrics import summarize

grid=GridMap.demo()
start=(5,5); goal=(74,52)

planners={
    "Dijkstra": lambda: np.asarray(dijkstra(grid,start,goal),float),
    "A*": lambda: np.asarray(astar(grid,start,goal),float),
    "RRT*": lambda: rrt_star(grid,start,goal),
    "Hybrid A*": lambda: hybrid_astar(grid,(start[0],start[1],0.0),goal),
}

for name,fn in planners.items():
    t0=time.perf_counter()
    path=fn()
    elapsed=time.perf_counter()-t0
    if path is None:
        print(name,"FAILED")
        continue

    smooth=smooth_path(path)
    if collision_free(grid,smooth):
        eval_path=smooth
        mode="smoothed"
    else:
        eval_path=path
        mode="raw (smoothing rejected due to collision)"

    print("\n",name,mode)
    for k,v in summarize(grid,eval_path,elapsed).items():
        print(f"{k}: {v:.5g}")
