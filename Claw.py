               

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
        loaction = []
        for i in range(0, len(self.contents)):
            for l in range(0, len(self.contents[i])):
                if self.contents[i][l] == item:
                    loaction.append([i, l])
                    item_count += 1

        return item_count, loaction
                
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
 

'''she = Shelf()
she.contents[0][0] = [1]
she.contents[0][1] = [1]
#print(she.find_Nofree())
#print(she.find_free())
#print(she.view_contents())
freeSpace = she.find_free()
print(freeSpace)
print(she.put("Banana", (0, 1)))
print(she.get(freeSpace))'''

class Hive(Shelf):

    def __init__(self, Max_storage):
        self.Max_storage = Max_storage
        #store all shelves avaliable
        self.location = (0, 0)
        self.shelves = []
        self.makeShelves()

    #create shelves to occupy max storage of hive
    def makeShelves(self):
        self.shelves = []
        for i in range(0, self.Max_storage//100):
            shelve = Shelf()
            self.shelves.append(shelve)

    #allocate shipment to free shelves
    def allocateShipment(self, shipmentQ, item):
        for i in range(0, len(self.shelves)):
            no, locat = self.shelves[i].find_Nofree()
            if no >= shipmentQ:
                self.sendLocationPut(i, locat, item)
                return no
            else:
                shipmentQ -= no
                self.sendLocationPut(i, locat, item)

    def fulfillDelivery(self, deliveryQ, item):
        print(item)
        for i in range(0, len(self.shelves)):
            no, locat = self.shelves[i].find_item(item)
            if no >= deliveryQ:
                self.sendLocationGet(i, locat, item)
                return no
            else:
                deliveryQ -= no
                self.sendLocationGet(i, locat, item)

    def sendLocationTaxi(self, origin, goal):
        #print(origin, goal)
        return 0

    #get location of a shelve and sent to taxi
    #send location to the claw
    def sendLocationPut(self, no, location, item):
        shelf_location = self.shelves[no].locate_shelves()
        for i in location:
            #print(self.shelves[no].view_contents())
            self.sendLocationTaxi(self.location, shelf_location )
            self.shelves[no].put(item, i)
            #self.sendLocationClaw(i)
        
    #sends location of a shelve to the taxi
    def sendLocationGet(self, no, location, item):
        shelf_location = self.shelves[no].locate_shelves()
        for i in location:
            #can introduce taxi verification
            self.sendLocationTaxi(shelf_location, self.location )
            self.shelves[no].get(i)
            #self.sendlocationClaw(i)
        

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
                if shipment_proportional[j] < shipment_proportional[j+1]:
                    shipment[j], shipment[j+1] = shipment[j+1], shipment[j]
                    shipment_quantity[j], shipment_quantity[j+1] = shipment_quantity[j+1], shipment_quantity[j]
                    shipment_proportional[j], shipment_proportional[j+1] = shipment_proportional[j+1], shipment_proportional[j]

        print(shipment)
        print(shipment_proportional)

        # for every shipment_proportional total [1,2,3] = 6
        # 1 robot for item 1, 2 robots for item 2, 3 robots for item 3 
        return shipment_proportional
        
hive = Hive(10000)
hive.allocateShipment(10, "Bananas")
hive.fulfillDelivery(10, "Bananas")
#hive.distribute_priority(["Bananas", "Kiwi", "Motor Oil"], [1020, 3000, 1020])