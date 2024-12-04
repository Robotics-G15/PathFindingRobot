


from new_interfaces.srv import JobBoardT
from new_interfaces.srv import JobBoardC
from new_interfaces.srv import ShipmentList
from new_interfaces.srv import TaxiAval
from new_interfaces.srv import Registry
import rclpy 
from rclpy.node import Node
import time


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

    #define of free spot in a shelve
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

#     def listener_callback(self, msg):
#         listQ= msg.location
#         listItem = msg.data
#         q = []
#         w = []
#         for i in listQ:
#             q.append(i)
#         for p in listItem:
#             w.append(p)
#         #print(self.shelf_location[0])
#         self.get_logger().info(f'I heard %s {msg.location}' %msg.data)
#         print("lists",listQ, listItem)
#         print("my",q, w)
#         self.distribute_Shipment(w, q)

        #Recieving shipment from client items and their quantity
        self.srv = self.create_service(ShipmentList, 'shipment_list',self.shipment_callback)
        #Fulfilling delivery from client items and their quantity
        self.srv = self.create_service(ShipmentList, 'delivery_list',self.delivery_callback)
        #Taxi registry
        self.srv = self.create_service(Registry, 'register_hive', self.registry_callback)

        #Taxi avaliability service
        self.cli_2 = self.create_client(TaxiAval, "avalaible_T")
        while not self.cli_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req_2 = TaxiAval.Request()

        self.Max_storage = Max_storage
        self.location = (0, 0)
        self.shelves = []
        self.taxi = []
        #List of avaliable taxis
        self.avaliable = []
        self.makeShelves()

    #Register all created Taxis 
    def registry_callback(self, request, response):
        self.taxi.append(request.name)
        self.avaliable.append(request.name)
        self.get_logger().info(f'I register {request.name}')
        response.approve = "True"
        print(self.taxi)
        return response

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
            if no != 0:
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

#     def fulfillDelivery(self, deliveryQ, item):
#         noItem = 0
#         for i in range(0, len(self.shelves)):
#             no, locat = self.shelves[i].find_item(item[noItem])
#             if no >= deliveryQ[noItem]:
#                 self.sendLocationGet(i, locat, item[noItem],deliveryQ[noItem])
#                 deliveryQ[noItem] -= no
#                 print(i,no,  item[noItem], deliveryQ[noItem], i)
#                 noItem += 1
#                 if noItem == len(deliveryQ):
#                     break
#             else:
#                 self.sendLocationGet(i, locat, item[noItem], deliveryQ[noItem])
#                 deliveryQ[noItem] -= no
#                 print(i,no, item[noItem], deliveryQ[noItem])

#         return True
                
#     def sendLocationTaxi(self, item, goal):
#         self.req_1.name = 'name'
#         self.req_1.item = item
#         print(goal)
#         self.req_1.location = goal
#         return self.cli_1.call_async(self.req_1)

#     def sendLocationClaw(self, item, shelf_l):
#         self.req_2.name = 'name'
#         self.req_2.item = item
#         self.req_2.location = shelf_l
#         future = self.cli_2.call_async(self.req_2)
#         response = future.result()
#         if response == None:
#             response = self.sendLocationClaw(item, shelf_l)

        return response
#     def response_callback(self, future):
#         if future.result() is not None:
#             result = future.result()
#             self.get_logger().info(f'REsult recieved {result}')
#         else:
#             self.get_logger().error("goal rejected")

#     def send_request(self, item, shelf_l):
#         self.req.name = 'name'
#         self.req.item = item
#         self.req.location = shelf_l
#         future = self.cli.call_async(self.req)
#         print(future.result())
#         def when_finished(_future):
#             self.get_logger().info(f'hshys{future.result()}')
#         future.add_done_callback(when_finished)
#         return future

    #get location of a shelve and sent to taxi
    #send location to the cla

        
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
        
=======
            #Find avalaibel location and number of spaces in shelves
            no, locat = self.shelves[i].find_Nofree()
            if no != 0:
                if no >= shipmentQ[noItem]:
                    #Notify claw, assign taxi and put item on a shelve
                    self.sendLocationPut(i, locat, item[noItem],shipmentQ[noItem])
                    shipmentQ[noItem] -= no
                    #move to another item
                    noItem += 1
                    #if all items have been allocated
                    if noItem == len(shipmentQ):
                        break
                else:
                    self.sendLocationPut(i, locat, item[noItem], shipmentQ[noItem])
                    shipmentQ[noItem] -= no

        return True

    #get required items from the shelves
    def fulfillDelivery(self, deliveryQ, item):
        noItem = 0
        for i in range(0, len(self.shelves)):
            #check how many required items are on the shelve
            no, locat = self.shelves[i].find_item(item[noItem])
            if no != 0:
                if no >= deliveryQ[noItem]:
                    #Notify claw, assign taxi and get item off the shelve
                    self.sendLocationGet(i, locat, item[noItem],deliveryQ[noItem])
                    deliveryQ[noItem] -= no
                    #move to another item
                    noItem += 1
                    #if all required items have been found
                    if noItem == len(deliveryQ):
                        break
                else:
                    self.sendLocationGet(i, locat, item[noItem], deliveryQ[noItem])
                    deliveryQ[noItem] -= no

        return True

    #send shelve location to the claw
    def send_requestClawPut(self, item, location, shelf_ID):
        #Claw service
        self.cli = self.create_client(JobBoardC, f'pickItem_claw{shelf_ID}')
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
            self.get_logger().info(f'Claw located at:{future.result()}')
            #call taxi with the location of the claw
            self.allocateTaxi(item, future.result().sum, shelf_ID)

        future.add_done_callback(when_finished)
        return future

    #send shelve location to the claw
    def send_requestClawGet(self, item, shelf_l, shelf_ID):
        #Claw service
        self.cli = self.create_client(JobBoardC, f'pickItem_claw{shelf_ID}')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req = JobBoardC.Request()
        self.req.name = 'get'
        self.req.item = item
        self.req.location = shelf_l
        #call asynchronously
        future = self.cli.call_async(self.req)
        #function which returns the responce when claw returned it
        def when_finished(_future):
            self.get_logger().info(f'Claw located at: {future.result()}')
            #call taxi with the location of the claw
            self.allocateTaxi(item, future.result().sum, shelf_ID)

        future.add_done_callback(when_finished)
        return future

    #send details to the taxi
    def giveTaxi(self, name, item, location, shelf_ID):
        #establsh connection with the avaliable taxi 
        self.cli_1 = self.create_client(JobBoardT, f'pickItem_{name}')
        while not self.cli_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting...')
        self.req_1 = JobBoardT.Request()
        self.req_1.name = name
        self.req_1.item = item
        self.req_1.location = location
        self.req_1.shelf = shelf_ID

        future = self.cli_1.call_async(self.req_1)
        #get the response when taxi returned results
        def when_finished(_future):
            self.get_logger().info(f'{item} was give to : {future.result().sum}')
            #add taxi back to list of avaliable taxis
            self.avaliable.append(name)
        future.add_done_callback(when_finished)
        return future

    #send information about pickup to the taxi
    def allocateTaxi(self, item, location, shelf_ID):
        #check if any taxis are avalaible
        if len(self.avaliable) != 0:
            #send claw location to the taxi
            self.giveTaxi(self.avaliable[0], item, location, shelf_ID)
            #delete taxi from avaliable
            self.avaliable.pop(0)
        else:
            #Find first avaliable taxi
            self.getAvaliableT(item, location, shelf_ID)

    #assign first avaliable taxi
    #when taxi is not avaliable it will wait allowing other avaliable taxis to fulfill the service
    def getAvaliableT(self, item, location, shelf_ID):
        self.req_2.name = item
        #call all taxis
        future = self.cli_2.call_async(self.req_2)
        #function to get the results when returned
        def when_finished(_future):
            self.get_logger().info(f'Avaliable taxi found: {future.result()}')
            name = future.result().avaliable[0]
            #send details to the avaliable taxi
            self.giveTaxi(name, item, location, shelf_ID)

        future.add_done_callback(when_finished)
        return future

    #get location of a shelve and sent to claw
    def sendLocationPut(self, shelf_ID, location, item, noItems):
        for i in range(0, len(location)):
            #allocate items depending on the number of given locations
            if noItems-1 >= i:
                #send shlef location to the claw
                self.send_requestClawPut(item, tuple(location[i]), shelf_ID)
                #put item into a virtual shelve
                self.shelves[shelf_ID].put(item, location[i])
            else:
                return

        
    #sends location of a shelve to the claw
    def sendLocationGet(self, shelf_ID, location, item, quantity):
        for i in range(0, len(location)):
            if quantity-1 >= i:
                self.send_requestClawGet(item, tuple(location[i]), shelf_ID)
                #get item from virtual shelve
                self.shelves[shelf_ID].get(location[i])
            else:
                return
        
    #CHeck how much space is left on the shelves
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
        
    #Prioritise items with larger quanity and allocate shelfs & taxis

    def distribute_Shipment(self, listItems, listQuantity ):
        listItems, listQuantity = self.distribute_priority(listItems, listQuantity)
        result = self.
        (listQuantity, listItems)
        return result
        


    #Recieve shippment call with items
    def shipment_callback(self, request, response):
        self.get_logger().info('Shippemnt of items:%s, in quantity: %d' %(request.item[0], request.q[0]))
        quantity = request.q
        items = request.item
        response.done = self.distribute_Shipment(items, quantity)
        return response


    #get delivery of items with their quantity
    def delivery_callback(self, request, response):
        self.get_logger().info('Delivery of %s, %d' %(request.item[0], request.q[0]))
        quantity = request.q
        items = request.item
        response.done = self.fulfillDelivery(quantity, items)
        return response
    

#     def add_three_ints_callback(self, request, response):
#         response.sum = request.a + request.b +request.c
#         self.get_logger().info('Incoming request\na: %d  %s %s' %(request.q, request.item, response.done))
    
#         return response

#     def main(self):
#         future_1 = self.send_request("cow", (0,0))
#         self.get_logger().info('result %s' %(future_1.result()))
#         rclpy.spin_until_future_complete(self, future_1)
#         response_1 = future_1.result()
#         self.get_logger().info(
#             'Result of add-two_ints: for %s' %
#             (response_1.sum))
        

#     '''def main(self):
#         try:
#             future_1 = self.send_request("cow", (0,0))
#             self.get_logger().info('result %s' %(future_1.result()))
#             #rclpy.spin_until_future_complete(self, future_1)
#             response_1 = future_1.result()
#         except Exception as e:
#             self.get_logger().info(
#                 'Result of add-two_ints: for %s' %
#                 (response_1.sum))
#         rclpy.spin_once(self, timeout_sec=0.5)'''

#     def now(self):
#         #self.main()
#         self.timer_callback()
    
def main():
    rclpy.init()
    hive = Hive(10000)
#     #hive.main()
#     #items = ["banana", "cow"]
#     #quantity = [3, 4]
#     #response = hive.distribute_Shipment(items, quantity)
#     '''future = hive.send_request("cow", (0,0))
#     hive.get_logger().info('result %s' %(future.result()))
#     rclpy.spin_until_future_complete(hive, future)
#     response = future.result()
#     hive.get_logger().info(
#         'Result of add-two_ints: for %s' %
#         (response.sum))'''
    


    rclpy.spin(hive)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
