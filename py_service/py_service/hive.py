#from example_interfaces.srv import AddTwoInts
#from new_interfaces.srv import ShipmentList
from new_interfaces.srv import JobBoardT
from new_interfaces.srv import JobBoardC
from new_interfaces.msg import JobBoardN
from new_interfaces.msg import ShipmentList

import rclpy 
from rclpy.node import Node

class Shelf:
    def __init__(self):
        self.x, self.y = self.getCoordinates()
        self.width = 10
        self.height = 10
        self.contents = []
        self.allocate_contents()

    #populating avaliable shelve space
    def allocate_contents(self):
        for i in range(0, self.width):
            self.contents.append([])
            for l in range (0, self.height):
                self.contents[i].append([None])

    #obtain position of the shelve in the warehouse
    def getCoordinates(self):
        x = 10
        y = 10
        return x, y
    
    def get(self, index):
        x = int(index[0])
        y = int(index[1])
        item = self.contents[x][y]
        self.contents[x][y] = [None]
        return item
    
    #For use with claw 
    # Add an item to a given index, if it is empty.
    # If no index is given, find and use an empty place to put the item.
    def put(self, item, index):
        x = int(index[0])
        y = int(index[1])
        if index:
            if self.contents[x][y] != [None]:
                #in case the space is occupied find new one
                if self.find_free() != None:
                    self.put(item, self.find_free())
                else:
                    return False
            else:
                self.contents[x][y] = item
                return True                                     
        else:
            for i in range(0,len(self.contents)):
                if self.contents[x][y] == [None]:
                    self.contents[x][y] = item
                    return True
            return False


    # View the contents of the shelf
    def view_contents(self):
        return self.contents
    
    #function checking if there is any space
    def isEmpty(self):
        if self.find_free() == None:
            return False
        else:
            return True

    #define location of free spot in a shelve
    def find_free(self):
        for i in range(0, len(self.contents)):
            for l in range(0, len(self.contents[i])):
                if self.contents[i][l] == [None]:
                    return (i, l)
                
    #define location and number of item in a shelve                
    def find_item(self, item):
        item_count = 0
        location = []
        for i in range(0, len(self.contents)):
            for l in range(0, len(self.contents[i])):
                if self.contents[i][l] == item:
                    location.append([i, l])
                    item_count += 1

        return item_count, location
    
    def capacity(self):
        free_spaces = 0
        for i in range(0, len(self.contents)):
            for l in range(0, len(self.contents[i])):
                if self.contents[i][l] == [None]:
                    free_spaces += 1
        return free_spaces
                
    #find how many spots are avaliable and return its location
    def find_Nofree(self):
        free_spaces = 0
        location = []
        for i in range(0, len(self.contents)):
            for l in range(0, len(self.contents[i])):
                if self.contents[i][l] == [None]:
                    location.append([i, l])
                    free_spaces += 1

        return free_spaces, location
    
    def locate_shelves(self):
        return self.x, self.y
 

class Hive(Node):
    def __init__(self, Max_storage):
        super().__init__('hive_service')
        #self.srv = self.create_service(ShipmentList, 'shipment_list',self.shipment_callback)
        self.subscription = self.create_subscription(ShipmentList, 'shipment', self.listener_callback, 10)
        self.subscription
        #self.srv = self.create_service(ShipmentList, 'delivery_list',self.delivery_callback)
        '''self.cli = self.create_client(JobBoardT, 'pickItem_taxi')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req = JobBoardT.Request()
        self.cli = self.create_client(JobBoardC, 'pickItem_claw')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req = JobBoardC.Request()'''
        self.publisher_ = self.create_publisher(JobBoardN, 'item', 10)
        self.i = None
        self.Max_storage = Max_storage
        self.location = (0, 0)
        self.shelves = []
        self.makeShelves()
        self.name = None

    def listener_callback(self, msg):
        listQ= msg.location
        listItem = msg.data
        q = []
        w = []
        for i in listQ:
            q.append(i)
        for p in listItem:
            w.append(p)
        #print(self.shelf_location[0])
        self.get_logger().info(f'I heard %s {msg.location}' %msg.data)
        print("lists",listQ, listItem)
        print("my",q, w)
        self.distribute_Shipment(w, q)

    
    def timer_callback(self, item, location):
        msg = JobBoardN()
        msg.data = '%s' %(item)
        msg.location = location
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing %s {msg.location}' %msg.data)

    #create shelves to occupy max storage of hive
    def makeShelves(self):
        self.shelves = []
        for i in range(0, self.Max_storage//100):
            shelve = Shelf()
            self.shelves.append(shelve)

    #allocate shipment to free shelves
    def allocateShipment(self, shipmentQ, item):
        noItem = 0
        for i in range(0, len(self.shelves)):
            no, locat = self.shelves[i].find_Nofree()
            if no >= shipmentQ[noItem]:
                self.sendLocationPut(i, locat, item[noItem],shipmentQ[noItem])
                shipmentQ[noItem] -= no
                print(i,no,  item[noItem], shipmentQ[noItem], i)
                noItem += 1
                if noItem == len(shipmentQ):
                    break
            else:
                self.sendLocationPut(i, locat, item[noItem], shipmentQ[noItem])
                shipmentQ[noItem] -= no
                print(i,no, item[noItem], shipmentQ[noItem])

        return True

    def fulfillDelivery(self, deliveryQ, item):
        noItem = 0
        for i in range(0, len(self.shelves)):
            no, locat = self.shelves[i].find_item(item[noItem])
            if no >= deliveryQ[noItem]:
                self.sendLocationGet(i, locat, item[noItem],deliveryQ[noItem])
                deliveryQ[noItem] -= no
                print(i,no,  item[noItem], deliveryQ[noItem], i)
                noItem += 1
                if noItem == len(deliveryQ):
                    break
            else:
                self.sendLocationGet(i, locat, item[noItem], deliveryQ[noItem])
                deliveryQ[noItem] -= no
                print(i,no, item[noItem], deliveryQ[noItem])

        return True
                
    def sendLocationTaxi(self, item, goal):
        self.req_1.name = 'name'
        self.req_1.item = item
        print(goal)
        self.req_1.location = goal
        return self.cli_1.call_async(self.req_1)

    def sendLocationClaw(self, item, shelf_l):
        self.req_2.name = 'name'
        self.req_2.item = item
        self.req_2.location = shelf_l
        future = self.cli_2.call_async(self.req_2)
        response = future.result()
        if response == None:
            response = self.sendLocationClaw(item, shelf_l)

        return response

    def send_request(self, item, shelf_l):
        self.req.name = 'name'
        self.req.item = item
        self.req.location = shelf_l
        future = self.cli.call_async(self.req)
        print(future.result())
        return self.cli.call_async(self.req)

    #get location of a shelve and sent to taxi
    #send location to the claw
    def sendLocationPut(self, shelf_ID, location, item, noItems):
        shelf_location = self.shelves[shelf_ID].locate_shelves()
        for i in range(0, len(location)):
            #print(self.shelves[no].view_contents())
            if noItems-1 >= i:
                #claw_location = self.sendLocationClaw(item, shelf_location)
                #self.send_request("cow", [0,0])
                #self.main()
                self.timer_callback(item, shelf_location)
                #rclpy.spin(self)
                '''future = self.send_request(item, shelf_location)
                self.get_logger().info('result %s' %(future.result()))
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info('this %s' %(response.sum))
                future = self.send_request("cow", (0,0))
                self.get_logger().info('result %s' %(future.result()))
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info(
                    'Result of add-two_ints: for %s' %
                    (response.sum))
                #self.sendLocationTaxi(item, claw_location )'''
                self.shelves[shelf_ID].put(item, location[i])
            else:
                return
            #self.sendLocationClaw(i, item)

        
    #sends location of a shelve to the taxi
    def sendLocationGet(self, shelf_ID, location, item, quantity):
        shelf_location = self.shelves[shelf_ID].locate_shelves()
        for i in range(0, len(location)):
            #can introduce taxi verification
            if quantity-1 >= i:
                self.sendLocationTaxi(item, shelf_location)
                self.shelves[shelf_ID].get(location[i])
            else:
                return
            #self.sendlocationClaw(i, item)
        
    def capacity(self):
        capacity = 0
        for i in range(0, len(self.shelves)):
            capacity += self.shelves[i].capacity()
            #no, loc = self.shelves[i].find_item("Bananas")
            #capacity += no

        return capacity


    def distribute_priority(self, shipment, shipment_quantity):
        #init
        shipment_total = 0
        shipment_proportional = []

        #get shipment total
        shipment_total = sum(shipment_quantity)

        #get shipment proportions
        for i in range(len(shipment)):
            shipment_proportional.append(shipment_total//shipment_quantity[i])

        #priority sort # proportional 
        # might need to use merge if 1000's of items
        for i in range(len(shipment)):
            for j in range(len(shipment)-1):
                if shipment_proportional[j] > shipment_proportional[j+1]:
                    shipment[j], shipment[j+1] = shipment[j+1], shipment[j]
                    shipment_quantity[j], shipment_quantity[j+1] = shipment_quantity[j+1], shipment_quantity[j]
                    shipment_proportional[j], shipment_proportional[j+1] = shipment_proportional[j+1], shipment_proportional[j]

        print(shipment)
        print(shipment_proportional)
        print(shipment_quantity)

        # for every shipment_proportional total [1,2,3] = 6
        # 1 robot for item 1, 2 robots for item 2, 3 robots for item 3 
        return shipment, shipment_quantity
        

    def distribute_Shipment(self, listItems, listQuantity ):
        listItems, listQuantity = self.distribute_priority(listItems, listQuantity)
        result = self.allocateShipment(listQuantity, listItems)
        return result
        

    def shipment_callback(self, request, response):
        self.get_logger().info('%s, %d' %(request.item[0], request.q[0]))
        #print(len(request.item))
        quantity = request.q
        items = request.item
        response.done = self.distribute_Shipment(items, quantity)
        return response

    def delivery_callback(self, request, response):
        self.get_logger().info('%s, %d' %(request.item[0], request.q[0]))
        #print(len(request.item))
        quantity = request.q
        items = request.item
        response.done = self.fulfillDelivery(quantity, items)
        return response
    

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b +request.c
        self.get_logger().info('Incoming request\na: %d  %s %s' %(request.q, request.item, response.done))
    
        return response

    def main(self):
        future_1 = self.send_request("cow", (0,0))
        self.get_logger().info('result %s' %(future_1.result()))
        rclpy.spin_until_future_complete(self, future_1)
        response_1 = future_1.result()
        self.get_logger().info(
            'Result of add-two_ints: for %s' %
            (response_1.sum))
        

    '''def main(self):
        try:
            future_1 = self.send_request("cow", (0,0))
            self.get_logger().info('result %s' %(future_1.result()))
            #rclpy.spin_until_future_complete(self, future_1)
            response_1 = future_1.result()
        except Exception as e:
            self.get_logger().info(
                'Result of add-two_ints: for %s' %
                (response_1.sum))
        rclpy.spin_once(self, timeout_sec=0.5)'''

    def now(self):
        #self.main()
        self.timer_callback()
    
def main():
    rclpy.init()
    hive = Hive(10000)
    #hive.main()
    #items = ["banana", "cow"]
    #quantity = [3, 4]
    #response = hive.distribute_Shipment(items, quantity)
    '''future = hive.send_request("cow", (0,0))
    hive.get_logger().info('result %s' %(future.result()))
    rclpy.spin_until_future_complete(hive, future)
    response = future.result()
    hive.get_logger().info(
        'Result of add-two_ints: for %s' %
        (response.sum))'''
    

    rclpy.spin(hive)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
