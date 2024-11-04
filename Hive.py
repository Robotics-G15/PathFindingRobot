class Hive:

    def __init__(self, Max_storage):
        self.Max_storage = Max_storage

    def distribute_priority(self, shipment, shipment_quantity):
        #init
        shipment_total = 0
        shipment_proportional = []

        #get shipment total
        for i in range(len(shipment_quantity)):
            shipment_total += shipment_quantity[i]

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
hive.distribute_priority(["Bananas", "Kiwi", "Motor Oil"], [1020, 3000, 1020])