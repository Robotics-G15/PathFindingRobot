class Shelve:
    #Class to track capacity of each shelve and direct the claw
    #needs location of each shelve
    def __init__(self):
        self.posx = 10
        self.posy = 10
        #list representing all spaces in shelves
        self.log = []
        self.allocate_log()

    #function checking if there is any space
    def isEmpty(self):
        if self.find_free() == None:
            return False
        else:
            return True
    
    #populating avaliable shelve space
    def allocate_log(self):
        for i in range(0, self.posx):
            self.log.append([])
            for l in range (0, self.posy):
                self.log[i].append([0])

    #directing claw into correct space, adding item to the shelve
    def allocate_shelves(self, location):
        x = int(location[0])
        y = int(location[1])
        self.log[x][y] = [1]

    #define location of free spot in a shelve
    def find_free(self):
        for i in range(0, len(self.log)):
            for l in range(0, len(self.log[i])):
                if self.log[i][l] == [0]:
                    return (i, l)
                
    #find how many spots are avaliable and return its location
    def find_Nofree(self):
        free_spaces = 0
        loaction = []
        for i in range(0, len(self.log)):
            for l in range(0, len(self.log[i])):
                if self.log[i][l] == [0]:
                    loaction.append([i, l])
                    free_spaces += 1

        return free_spaces, loaction
                

#she = Shelve()
#she.log[0][0] = [1]
#she.log[0][1] = [1]
#print(she.find_Nofree())


class Hive(Shelve):

    def __init__(self, Max_storage):
        self.Max_storage = Max_storage
        #store all shelves avaliable
        self.shelves = []
        self.makeShelves()

    #create shelves to occupy max storage of hive
    def makeShelves(self):
        self.shelves = []
        for i in range(0, self.Max_storage//100):
            shelve = Shelve()
            self.shelves.append(shelve)

    #allocate shipment to free shelves
    def allocateShipment(self, shipmentQ):
        for i in range(0, len(self.shelves)):
            no, locat = self.shelves[i].find_Nofree()
            if no >= shipmentQ:
                self.sendLocation(i, locat)
                return no
            else:
                shipmentQ -= no
                self.sendLocation(i, locat)

    #sends location of an item to the claw
    #should send location of shelve to taxi
    def sendLocation(self, no, location):
        for i in location:
            #sendLocationTaxi(i)
            self.shelves[no].allocate_shelves(i)
            return 0

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
hive.allocateShipment(100)
#hive.distribute_priority(["Bananas", "Kiwi", "Motor Oil"], [1020, 3000, 1020])