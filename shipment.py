import sys
from new_interfaces.srv import ShipmentList

import rclpy 
from rclpy.node import Node
import yaml
import time
import sys


class Shipment(Node):
    def __init__(self):
        super().__init__('shipment_client')
        #Create client for the hive service to recieve shipment
        self.cli = self.create_client(ShipmentList, 'shipment_list')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again')
        self.req = ShipmentList.Request()
    

    #SEnd request to the hive with item names and quantity
    #input should have format of item1,item2 quantity1,quantity2
    #where item list and quantity are separated by a single space
    def send_request(self):
        #format the input
        items_input = sys.argv[1]
        items = []
        items = items_input.split(",")
        q_input = sys.argv[2]
        qua = []
        qua = q_input.split(",")
        for i in range(0, len(qua)):
            qua[i] = int(qua[i])
        self.req.item = items
        self.req.q = qua
        return self.cli.call_async(self.req)
    
    def item_spawner(self, item, quantity):
        self.spawn_req.item = item
        self.spawn_req.quantity = quantity
        return self.cli.call_async(self.spawn_req)
    

    
def main():
    rclpy.init()
    ship = Shipment()
    #send request to the hive
    future = ship.send_request()
    #handle response
    ship.get_logger().info('result %s' %(future.result()))

    try:
        while rclpy.ok():
            item_name = (ship.items)
            quantity = 1
            cmd.vel = f"item, quantity"
            print(cmd.vel)
            time.sleep(5)
    except KeyboardInterrupt:
        ship.get_logger().info('Shipment process interrupted.')


    rclpy.spin_until_future_complete(ship, future)
    response = future.result()
    ship.get_logger().info(
        'Result for %s %s, %s' %
        ((sys.argv[1]), (sys.argv[2]),response.done))
    ship.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
