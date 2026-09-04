import heapq
import math


def _reconstruct(parent, goal):
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    return path[::-1]


def dijkstra(grid, start, goal):
    pq = [(0.0, start)]
    cost = {start: 0.0}
    parent = {}
    while pq:
        g, cur = heapq.heappop(pq)
        if cur == goal:
            return _reconstruct(parent, goal)
        if g > cost[cur]:
            continue
        for nb in grid.neighbors8(cur):
            step = math.hypot(nb[0]-cur[0], nb[1]-cur[1])
            ng = g + step
            if ng < cost.get(nb, float("inf")):
                cost[nb] = ng
                parent[nb] = cur
                heapq.heappush(pq, (ng, nb))
    return None


def astar(grid, start, goal):
    h = lambda n: math.hypot(goal[0]-n[0], goal[1]-n[1])
    pq = [(h(start), 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    while pq:
        _, g, cur = heapq.heappop(pq)
        if cur == goal:
            return _reconstruct(parent, goal)
        if g > cost[cur]:
            continue
        for nb in grid.neighbors8(cur):
            step = math.hypot(nb[0]-cur[0], nb[1]-cur[1])
            ng = g + step
            if ng < cost.get(nb, float("inf")):
                cost[nb] = ng
                parent[nb] = cur
                heapq.heappush(pq, (ng+h(nb), ng, nb))
    return None
