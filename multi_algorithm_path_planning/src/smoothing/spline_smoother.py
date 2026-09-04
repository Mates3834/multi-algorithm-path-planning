import numpy as np
from scipy.interpolate import splprep, splev


def smooth_path(path, samples=250, smoothing=2.0):
    path = np.asarray(path, dtype=float)
    if len(path) < 4:
        return path
    tck, _ = splprep([path[:,0], path[:,1]], s=smoothing, k=min(3,len(path)-1))
    u = np.linspace(0,1,samples)
    x,y = splev(u,tck)
    return np.c_[x,y]


def collision_free(grid, path):
    return all(grid.is_free(x,y) for x,y in np.asarray(path))
