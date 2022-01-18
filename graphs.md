# Graphs

## Graph and Tree Traversals

### Depth-First Search (DFS)

```python
def dfs(node, graph, visited):
  # process node
  visited.add(node)
  for neighbor in graph(node):
    dfs(neighbor, graph, visited)
```

* **Time complexity**: $(E+V)$
* **Space complexity**: $O(V)$

### Breadth-First Search (BFS)

```python
def bfs(start, graph):
  queue = deque([start])
  visited = set([start])
  while queue:
    u = queue.popleft()
    if u not in visited:
      visited.add(u)
      # process u
      for v in graph[u]:
          queue.append(v)
```

* **Time complexity**: $O(E+V)$
* **Space complexity**: $O(V)$

## Disjoint Set

```python
class DisjointSet:
  def __init__(self, n):
    self.parent = [*range(n)]

  def find(self, u):
    if self.parent[u] != u:
      self.parent[u] = self.root(self.parent[u])
    return self.parent[u]

  def union(self, u, v):
    rootU, rootV = self.root(u), self.root(v)
    if rootU != rootV:
      self.parent[rootU] = rootV
      return True
    return False
```

* **Time complexity**: the construction is $O(n)$ and both *find* and *union* are $O(log(n))$
* **Space complexity**: $O(n)$

There is an optimization called *Union by Rank* where the *find* and *union* are almost $O(1)$. The idea is to keep track of the depth (or rank) of each tree and on the *union* operation use the set with higer rank as parent of the other.

## Minimum Spanning Tree

### Prim's Algorithm

```python
def prim(graph):
  treeWeight = 0
  heap = [(0, 0)] # w, u
  visited = set()
  while heap:
    w, u = heapq.heappop(heap)
    if u in visited: continue
    visited.add(u)
    treeWeight += w
    for v, w in graph(u):
      if v in visited: continue
      heapq.heappush(heap, (w, v))
  return treeWeight
```

* **Time complexity**: $O(E * log(V))$ with a binary heap.
* **Space complexity**: $O(V)$

### Kruskal's Algorithm

```python
def kruskal(graph):
  dset = DisjointSet(len(graph))
  edges = []
  for u in range(n):
    for v, w in graph(u):
      edges.append((w, u, v))
  edges.sort()
  treeWeight = 0
  for w, u, v in edges:
    if dset.union(u,v):
      treeWeight += w
      n -= 1
      if n == 1:
        return treeWeight
```

* **Time complexity**: $O(E * log(E))$
* **Space complexity**: $O(V)$

**Note:** Kruskal's algorithm implementation is a bit larger than Prim's because it requires the implementation of a Disjoint Set

## Shortest Path

### Dijkstra's Algorithm

Computes the shortest path from one node to the rest on a graph without negative weights.

```python
def dijkstra(start, graph):
  heap = [(start, k)]
  dist = {}
  while heap:
    d, u = heapq.heappop(heap)
    if u not in dist:
      dist[u] = d
      for v, w in neighbors[u]:
        if v not in dist:
          heapq.heappush(heap, (d + w, v))
  return dist
```

* **Time complexity**: $O(E * log(V))$ with a binary heap.
* **Space complexity**: $O(V)$

### Bellman-Ford Algorithm

Computes the shortest path from one node to the rest on a graph, negative weights without negative cycles allowed.

```python
def bellmanFord(start, k, n, edges):
  dist = [inf] * n
  dist[start] = 0
  for _ in range(n-1):
    for u, v, w in edges:
      dist[v] = min(dist[u] + w, dist[v])
  return dist
```

* **Time complexity**: $O(V * E))$ with a binary heap
* **Space complexity**: $O(V)$

**Notes:**

- The algorithm can be improved by stopping if there weren't any changes on a given iteration of the main loop.
- It can be modified to detect if there are cycles of negative cost, with an extra iteration at the end, if there are changes then there is a cycle of negative costs.
- And it can be modified to compute the shortest lengths in k steps with an extra array to store the results of the previous iteration.

### Shortest Path Faster Algorithm (SPFA)

Same as Bellman-Ford but faster (same time and **Space complexity** though). It can't detect negative cost cycles.

```python
def spfa(start, graph):
  dist = [inf] * n
  dist[start] = 0
  queue = collections.deque([start])
  inQueue = set([start])
  while queue:
    u = queue.popleft()
    cost = dist[u-1]
    inQueue.remove(u)
    for v, w in graph[u]:
      if dist[v] > cost + w:
        dist[v] = cost + w
        if v not in inQueue:
          inQueue.add(v)
          queue.append(v)
```

* **Time complexity**: $O(V * E))$ with a binary heap
* **Space complexity**: $O(V)$

### Floyd-Warshall algorithm

Shorthest path between all pair of vertices. Negative weights without negative cycles allowed.

```python
def floydWarshall(n, edges:
  dist = [[inf for _ in range(n)] for _ in range(n)]
  for u, v, w in edges:
    dist[u][v] = w
  for i in range(n):
    dist[i][i] = 0
  for k in range(n):
    for i in range(n):
      for j in range(n):
        dist[i][j] = min(dist[i][j], dist[i][k]+dist[k][j])
  return dist
```

* **Time complexity**: $O(V^3))$
* **Space complexity**: $O(V^2)$
