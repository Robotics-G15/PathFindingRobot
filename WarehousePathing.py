# This class represents a directed graph using adjacency list representation
class Graph:
    def __init__(self, V):  # Constructor
        self.V = V
        self.adj = [[] for _ in range(V)]

    def addEdge(self, u, v, w):
        self.adj[u].append((v, w))
        self.adj[v].append((u, w))

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
    def connect_matrix(self, width, height, width_weight, height_weight):
        width_increment = 0
        for j in range(height):
            for i in range(width-1):
                self.addEdge(i+width_increment, i+1+width_increment, width_weight)
            width_increment += width
            
        height_increment = 0
        for j in range(height-1):
            for i in range(width):
                self.addEdge(i+height_increment, i+width+height_increment, height_weight)
            height_increment += width

# Driver code
if __name__ == "__main__":
    # Create the graph
    
	width = 10
	height = 4

	V = width*height
	Warehouse_floor = Graph(V)
	Warehouse_floor.connect_matrix(width, height, 1, 3)
	SkyHook = Graph(V)
	SkyHook.connect_matrix(width, height, 1, 1)

	print(Warehouse_floor.adj)
	Warehouse_floor.shortestPath(0, 24)