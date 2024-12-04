import rclpy
from rclpy.node import Node
import sys
from new_interfaces.srv import JobBoardT
from new_interfaces.srv import JobBoardC
from new_interfaces.srv import TaxiAval
from new_interfaces.srv import Registry
from pyrobosim.planning.actions import TaskAction, TaskPlan
from rclpy.action import ActionClient
from pyrobosim_msgs.action import ExecuteTaskAction, ExecuteTaskPlan
from pyrobosim_msgs.msg import TaskAction, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState

# This class represents a directed graph using adjacency list representation
class Graph:
    def __init__(self, V: int):  # Constructor
        self.V = V
        self.Node_pos = {}
        self.adj = [[] for _ in range(V)]

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
                self.addEdge(i+width_increment, i+1+width_increment, width_weight)
            width_increment += width
            
        height_increment = 0
        for j in range(height-1):
            for i in range(width):
                self.addEdge(i+height_increment, i+width+height_increment, height_weight)
            height_increment += width

        n = 0
        for x in range(height):
            for y in range(width):
                self.Node_pos[(x, y)] = (n)
                n += 1
                
    def get_current_node(self, x, y):
        return self.Node_pos[(x, y)]
        

class Taxi_bot(Node):
    
    def __init__(self,name, Warehouse_floor ,item_ID = None, Goal_Node = None, Node = 0, X_pos = 0, Y_pos = 0, orientation = 0, Shelf_destination = None):
        super().__init__('taxi_service')
        #Service to pick up the item 
        self.srv = self.create_service(JobBoardT, f'pickItem_{name}', self.pickTaxi_callback)
        #Service to show if taxi is avaliable or not
        self.srv = self.create_service(TaxiAval, "avalaible_T", self.showAvlb_taxi)
        #Register existance to hive 
        self.cli = self.create_client(Registry, 'register_hive')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting....')
        self.req = Registry.Request()
        self.action_client = ActionClient(self, ExecuteTaskAction, "execute_action")
        self.plan_client = ActionClient(self, ExecuteTaskPlan, "execute_task_plan")
        self.world_client = self.create_client(RequestWorldState, "request_world_state")
        while rclpy.ok() and not self.world_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("Waiting foe world state...")
            future = self.world_client.call_async(RequestWorldState.Request())
            rclpy.spin_until_future_complete(self, future)
        self.name = name
        self.avaliable = True
        self.item_ID = item_ID
        self.Goal_Node = Goal_Node
        self.Node = Node
        self.X_pos = X_pos
        self.Y_pos = Y_pos
        self.orientation = orientation
        self.Shelf_destination = Shelf_destination
        self.warehouse = Warehouse_floor
        
        self.start(Warehouse_floor)
        self.register()
        self.startGoing("banana", 0, 1)

    #Register that the taxi of name exist
    def register(self):
        self.req.name = str(self.name)
        future = self.cli.call_async(self.req)
        def when_finished(_future):
            self.get_logger().info(f'Register {future.result()}')
        future.add_done_callback(when_finished)
        return future
        
    def send_action(self, goal):
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal)

    def send_plan(self, goal):
        print(goal)
        self.plan_client.wait_for_server()

        #while not self.plan_client.wait_for_server(timeout_sec=1.0):
            #self.get_logger().info('Service not avaliable, waiting....')
        goal_future = self.plan_client.send_goal_async(goal)
        print(goal_future.result())
        #print(goal_future.execution_result())
        def when_finished(_goal_future):
            self.get_logger().info(f'Register {goal_future.execution_result()}')
        #goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("NOt accepted")
            return

    #Service to transport items to given location 
    def pickTaxi_callback(self, request, response):
        #respond with taxi name in order to be put back in avalaible taxi list
        response.sum = str(request.name)
        #INdicate that taxi is occupied
        self.updateTaxi(self.name, False)
        #Pritng incoming request
        self.get_logger().info('Incoming request\n items:%s name:%s location:%s shelfId: %dTaken care by %s' 
            % (request.item, request.name, request.location, request.shelf, self.name))

        #Begin travel to location
        location = request.location
        self.startGoing(request.item, location, request.shelf)
        #Indicate that taxi is no longer occupied
        self.updateTaxi(self.name, True)
        return response

    #update avaliability attribute of taxi
    def updateTaxi(self, name, avaliable):
        self.avaliable = avaliable

    #when hive requires transport check if this taxi is avalaible
    def showAvlb_taxi(self, request, response):
        avaliable = []
        if self.avaliable == True:
            avaliable.append(self.name)
        else:
            #let other taxis get the job if not avaliable 
            time.sleep(20)


        response.avaliable = avaliable
        self.get_logger().info('%s avaliable: %s' % (request.name, response.avaliable))
        return response
        
        
    def start(self, Warehouse_floor):
        self.spawn_taxi() # Spawn taxi

    def put_items(self, item, shelf_ID):
        shelf = "shelf"+str(shelf_ID)
        name = "Taxi-Bot"+ str(self.name[4])
        actions = [TaskAction(
            type="navigate",
            source_location="Warehouse",
            target_location="Item_spawn",
        ), TaskAction(type="detect", object=item),
        TaskAction(type="pick", object=item),
        TaskAction(type="navigate",
            source_location="Item_spawn",target_location= shelf,
        ), TaskAction(type="place", object=item),
        TaskAction(type="navigate", target_location="Item_spawn")]
        
        goal = ExecuteTaskPlan.Goal()
        goal.plan = TaskPlan(robot=name, actions=actions)
        self.send_plan(goal)

    def get_items(self, item, shelf_ID):
        shelf = "shelf"+str(shelf_ID)
        name = "Taxi-Bot"+ self.name[4]
        actions = [
        TaskAction(
            type="navigate",
            source_location="Warehouse",
            target_location=shelf,
        )]
        actions.append(TaskAction("detect", object=item))
        actions.append(TaskAction("pick", object=item))
        actions.append(TaskAction("navigate",target_location= "Delivery",
        ))
        goal = ExecuteTaskPlan.Goal()
        actions.append(TaskAction("place", object=item))
        goal.plan = TaskPlan(robot= name, actions=actions)
        self.send_plan(goal)


    def startGoing(self, item, location, shelf_ID):
        #need to make location into node
        ##cahnge location to string put/get
        self.put_items(item, shelf_ID)
        #self.get_items(item, shelf_ID)
        '''
        shelf = "Delivery"
        action = [TaskAction(robot="Taxi-Bot1", type="navigate", target_location=shelf), TaskAction(robot="Taxi-Bot1",type="navigate", target_location="shelf2")]
        goal = ExecuteTaskPlan.Goal()
        name = 'Taxi-Bot1'
        goal.plan = TaskPlan(robot=name, actions=action)
        self.send_plan(goal)'''
        #Path = self.get_path(Warehouse_floor, +1) # get path to node
        #self.go_to_goal(Path) # Go to shelf
        #self.Arrived_at_Goal() # PUBLISH TO SUBSCRIBER

        self.arrivedToClaw(item, location, shelf_ID)
        #self.store_item() # PUBLISH TO SUBSCRIBER
        return None


    #communication between claw and taxi
    #sending item, self location 
    #the claw is identified by shelf id which calls only the claw with that id
    def arrivedToClaw(self, item, location, shelf_ID):
        #Claw service
        self.cli = self.create_client(JobBoardC, f'takeFromTaxi{shelf_ID}')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req = JobBoardC.Request()
        self.req.name = 'put'
        self.req.item = item
        self.req.location = location
        #call asynchronously
        future = self.cli.call_async(self.req)
        #function which returns the responce when claw returned it
        def when_finished(_future):
            self.get_logger().info(f'Claw located at: {future.result()}')

        future.add_done_callback(when_finished)
        return future
        
    def Return(self, Warehouse_floor): ##Should only run on Hive permisions
        self.Node, self.Goal_Node = self.Goal_Node, self.Node # Swap Start and Finish
        Path = self.get_path(Warehouse_floor, -1) # get return path
        self.go_to_goal(Path) # Return To start
        self.delete_Taxi() # Delete taxi
        
        
    def get_path(self, Warehouse_floor, weight):
        Path = Warehouse_floor.shortestPath(self.Node, self.Goal_Node)
        Warehouse_floor.Update_Graph_Weight(Path, weight)
        return Path
        
    def spawn_taxi(self):
        #ROS COMMAND TO SPAWN A TAXI
        #Should always spawn on Node (0)
        Cmd_vel = f"Spawn Taxi_bot at  Node(0)"
        print(Cmd_vel)
        
    def go_to_goal(self, Path):
        #generate cmd to output to terminal
        # temp cmd_vel 
        Cmd_vel = ""
        for i in range(len(Path)-1):
            cur = [self.X_pos, self.Y_pos]
            self.check_direction((Path[i] - Path[i+1]))
            Cmd_vel = f"Move from Node {cur} -> Node {[self.X_pos, self.Y_pos]} at orientation {self.orientation}"
            print(Cmd_vel)
            
    def check_direction(self, direction):
        if direction == -1:
            self.update_X_pos(-1)
            self.update_orientation(180) # Moving left           
        elif direction == 1:
            self.update_X_pos(+1)
            self.update_orientation(0) # Moving right            
        elif direction == -10: 
            self.update_Y_pos(+1)
            self.update_orientation(90) # Moving upward            
        elif direction == 10: 
            self.update_Y_pos(-1)
            self.update_orientation(270) # Moving downward 
        else:
            print("Error")
            
    def update_X_pos(self, New_X_pos):
        self.X_pos += New_X_pos
        
    def update_Y_pos(self, New_Y_pos):
        self.Y_pos += New_Y_pos
            
    def update_orientation(self, New_orientation):
        self.orientation = New_orientation
        
    def Arrived_at_Goal(self):
        #PUBLSISH TO CLAW
        pass
    
    def get_item(self, item):
        Shelf_ID, Shelf_pos = Shelf.search_shelves(item) # recieve topic command
        Path = Graph.Warehouse_floor.shortestPath(self, Graph.get_current_node((self.X_pos, self.Y_pos)), Shelf_ID)
        self.go_to_goal(Path)
        self.item = Shelf.shelves.get(Shelf_pos) # publish to hive
            
    def store_item(self):
        #ROS Publishes to claw That item is stored
        #Shelf.Shelf_position = Shelf_destination
        self.item = None
        
    def Check_Tasks(self):
        #CHECKS ROS TOPIC TO SEE IF MORE TASKS ARE TO BE COMPLETED
        pass
        
    def delete_Taxi(self):
        #ROS COMMAND TO /KIll A TAXI
        #taxi_bot.Taxis.remove(self)
        pass
    
def main():
    width = 10
    height = 4

    V = width*height
    Warehouse_floor = Graph(V)
    Warehouse_floor.connect_matrix(width, height, 1, 3)
    rclpy.init()
    #create taxi with name consisting of inputted number
    name = sys.argv[1]
    taxi = Taxi_bot(f"Taxi{name}",Warehouse_floor)
    rclpy.spin(taxi)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    '''
    Goal_Node = 15
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)
    Taxi_bot("Item_ID", Goal_Node, Warehouse_floor)'''
