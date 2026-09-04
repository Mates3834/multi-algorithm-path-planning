import numpy as np
from scipy.ndimage import distance_transform_edt


def path_length(path):
    p=np.asarray(path)
    return float(np.sum(np.linalg.norm(np.diff(p,axis=0),axis=1)))


def curvature(path):
    p=np.asarray(path,dtype=float)
    if len(p)<3:
        return np.zeros(len(p))
    dx=np.gradient(p[:,0]); dy=np.gradient(p[:,1])
    ddx=np.gradient(dx); ddy=np.gradient(dy)
    den=(dx*dx+dy*dy)**1.5 + 1e-9
    return (dx*ddy-dy*ddx)/den


def smoothness_cost(path):
    p=np.asarray(path,dtype=float)
    if len(p)<3: return 0.0
    second=np.diff(p,n=2,axis=0)
    return float(np.sum(np.linalg.norm(second,axis=1)**2))


def min_clearance(grid,path):
    free = 1-grid.occ
    dist = distance_transform_edt(free)
    vals=[]
    for x,y in np.asarray(path):
        ix=int(np.clip(round(x),0,grid.width-1))
        iy=int(np.clip(round(y),0,grid.height-1))
        vals.append(dist[iy,ix])
    return float(np.min(vals))


def summarize(grid,path,planning_time):
    k=curvature(path)
    return {
        "path_length": path_length(path),
        "planning_time_s": float(planning_time),
        "min_clearance_cells": min_clearance(grid,path),
        "max_abs_curvature": float(np.max(np.abs(k))) if len(k) else 0.0,
        "smoothness_cost": smoothness_cost(path),
    }
