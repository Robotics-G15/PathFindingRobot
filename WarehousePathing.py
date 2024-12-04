# This class represents a directed graph using adjacency list representation
class Graph:
    def __init__(self, V: int):  # Constructor
        self.V = V
        self.Node_pos = {}
        self.adj = [[] for _ in range(V+2)]

    def addEdge(self, u: int, v: int, w: int):
        self.adj[u].append([v, w])
        self.adj[v].append([u, w])

    # Finds the node with the minimum distance in the priority queue
    def minDistanceNode(self, queue, dist):
        min_distance = float('inf')
        min_node = -1
        for node in queue:
            if dist[node] < min_distance:
                min_distance = dist[node]
                min_node = node
        return min_node

    # Prints the shortest path from src to goal
    def shortestPath(self, src, goal):
        # Initialize distances and previous node list
        dist = [float('inf')] * self.V
        dist[src] = 0
        prev = [None] * self.V  # To store the shortest path tree

        # Priority queue represented as a list of nodes
        queue = list(range(self.V))

        while queue:
            # Get the node with the minimum distance
            u = self.minDistanceNode(queue, dist)
            queue.remove(u)

            # If we reached the goal, we can stop
            if u == goal:
                break

            # Update distances for adjacent vertices
            for v, weight in self.adj[u]:
                if v in queue and dist[v] > dist[u] + weight:
                    dist[v] = dist[u] + weight
                    prev[v] = u  # Store the path

        # Reconstruct the path from src to goal
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = prev[current]
        path = path[::-1]  # Reverse the path

        # Print the shortest path and its distance
        if dist[goal] == float('inf'):
            print(f"No path from {src} to {goal}")
        else:
            print(f"Shortest path from {src} to {goal}: {' -> '.join(map(str, path))}")
            print(f"Distance: {dist[goal]}")
        return path
    
    def Update_Graph_Weight(self, Path, weight):
        for i in range(len(Path)-1):
            for j in range(len(self.adj[Path[i]])):
                if self.adj[Path[i]][j][0] == Path[i+1]:
                    self.adj[Path[i]][j][1] += weight
                    break
            
    def connect_matrix(self, width, height, width_weight, height_weight):
        width_increment = 0
        for j in range(height):
            for i in range(width-1):
                self.addEdge(i+width_increment+1, i+1+width_increment+1, width_weight)
            width_increment += width
            
        height_increment = 0
        for j in range(height-1):
            for i in range(width):
                self.addEdge(i+height_increment+1, i+width+height_increment+1, height_weight)
            height_increment += width

        self.addEdge(0, 1, 1)
        self.addEdge(width*height+1,  width*height, 1)

        n = 0
        for x in range(height):
            for y in range(width):
                self.Node_pos[(-x, y)] = (n)
                n += 1
                
    def get_current_node(self, x, y):
        return self.Node_pos[(x, y)]
        