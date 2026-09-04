import heapq
import math
import numpy as np


def hybrid_astar(grid, start, goal, heading_bins=24,
                 step=2.0, turn_rates=(-0.35, 0.0, 0.35)):
    """
    Lightweight heading-aware lattice planner inspired by Hybrid A*.

    State = (x, y, heading). Continuous motion primitives are discretized
    into occupancy-grid / heading bins. This is intentionally a compact
    educational implementation, not a full production Hybrid A* planner.
    """
    sx, sy, sth = start
    gx, gy = goal

    def key(x,y,th):
        b = int(((th%(2*math.pi))/(2*math.pi))*heading_bins) % heading_bins
        return (int(round(x)), int(round(y)), b)

    def heuristic(x,y):
        return math.hypot(gx-x, gy-y)

    k0 = key(sx,sy,sth)
    pq=[(heuristic(sx,sy),0.0,sx,sy,sth)]
    best={k0:0.0}
    parent={}
    state_of={k0:(sx,sy,sth)}

    while pq:
        _,g,x,y,th=heapq.heappop(pq)
        k=key(x,y,th)
        if g > best.get(k,float("inf")) + 1e-9:
            continue
        if heuristic(x,y) < 2.5:
            path=[(x,y)]
            while k in parent:
                k=parent[k]
                s=state_of[k]
                path.append((s[0],s[1]))
            path.reverse()
            path.append((gx,gy))
            return np.asarray(path)

        for w in turn_rates:
            nth = th + w
            nx = x + step*math.cos(nth)
            ny = y + step*math.sin(nth)
            if not grid.is_free(nx,ny):
                continue
            nk=key(nx,ny,nth)
            ng=g+step+0.4*abs(w)
            if ng < best.get(nk,float("inf")):
                best[nk]=ng
                parent[nk]=k
                state_of[nk]=(nx,ny,nth)
                heapq.heappush(pq,(ng+heuristic(nx,ny),ng,nx,ny,nth))
    return None
