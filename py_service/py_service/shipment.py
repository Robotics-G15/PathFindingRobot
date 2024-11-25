import sys
#from example_interfaces.srv import AddTwoInts
from new_interfaces.srv import ShipmentList
from new_interfaces.msg import ShipmentList

import rclpy 
from rclpy.node import Node

class Shipment(Node):
    def __init__(self):
        super().__init__('shipment_client')
        self.publisher_ = self.create_publisher(ShipmentList, 'shipment', 10)
        #self.cli = self.create_client(ShipmentList, 'shipment_list')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
         #   self.get_logger().info('service not avaliable, waiting again')
        #self.req = ShipmentList.Request()
    
    def timer_callback(self, item, location):
        msg = ShipmentList()
        msg.data = item.split(',')
        location = location.split(',')
        for i in range(len(location)):
            location[i] = int(location[i])
        msg.location = location
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing %s {msg.location}' %msg.data)

    def send_request(self):
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
    
def main():
    rclpy.init()
    ship = Shipment()
    ship.timer_callback(sys.argv[1], sys.argv[2])
    '''future = ship.send_request()
    ship.get_logger().info('result %s' %(future.result()))
    rclpy.spin_until_future_complete(ship, future)
    response = future.result()
    ship.get_logger().info(
        'Result of add-two_ints: for %s %s, %s' %
        ((sys.argv[1]), (sys.argv[2]),response.done))
    '''
    ship.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
