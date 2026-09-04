import math
import numpy as np


def _segment_free(grid, a, b, samples=25):
    for t in np.linspace(0.0, 1.0, samples):
        x = a[0] + t*(b[0]-a[0])
        y = a[1] + t*(b[1]-a[1])
        if not grid.is_free(x, y):
            return False
    return True


def rrt_star(grid, start, goal, iterations=2500, step_size=4.0,
             goal_radius=4.0, rewire_radius=7.0, seed=2):
    rng = np.random.default_rng(seed)
    nodes = [np.array(start, dtype=float)]
    parent = [-1]
    cost = [0.0]

    for _ in range(iterations):
        if rng.random() < 0.08:
            sample = np.array(goal, dtype=float)
        else:
            sample = np.array([rng.uniform(0, grid.width-1),
                               rng.uniform(0, grid.height-1)])

        d = [np.linalg.norm(n-sample) for n in nodes]
        i_near = int(np.argmin(d))
        direction = sample - nodes[i_near]
        L = np.linalg.norm(direction)
        if L < 1e-9:
            continue
        new = nodes[i_near] + direction/L * min(step_size, L)
        if not grid.is_free(*new) or not _segment_free(grid, nodes[i_near], new):
            continue

        near_ids = [i for i,n in enumerate(nodes)
                    if np.linalg.norm(n-new) <= rewire_radius]
        best_parent = i_near
        best_cost = cost[i_near] + np.linalg.norm(new-nodes[i_near])

        for j in near_ids:
            c = cost[j] + np.linalg.norm(new-nodes[j])
            if c < best_cost and _segment_free(grid, nodes[j], new):
                best_parent, best_cost = j, c

        nodes.append(new)
        parent.append(best_parent)
        cost.append(best_cost)
        new_id = len(nodes)-1

        for j in near_ids:
            c = best_cost + np.linalg.norm(nodes[j]-new)
            if c < cost[j] and _segment_free(grid, new, nodes[j]):
                parent[j] = new_id
                cost[j] = c

        if np.linalg.norm(new-np.array(goal)) <= goal_radius and \
           _segment_free(grid, new, np.array(goal)):
            nodes.append(np.array(goal,dtype=float))
            parent.append(new_id)
            cost.append(best_cost + np.linalg.norm(new-np.array(goal)))
            idx = len(nodes)-1
            path=[]
            while idx != -1:
                path.append(nodes[idx].copy())
                idx = parent[idx]
            return np.asarray(path[::-1])

    return None
