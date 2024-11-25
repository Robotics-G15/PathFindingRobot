import sys
#from example_interfaces.srv import AddTwoInts
from new_interfaces.srv import ShipmentList

import rclpy 
from rclpy.node import Node

class Delivery(Node):
    def __init__(self):
        super().__init__('delivery_client')
        self.cli = self.create_client(ShipmentList, 'delivery_list')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again')
        self.req = ShipmentList.Request()
    
    
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
    delivery = Delivery()
    future = delivery.send_request()
    delivery.get_logger().info('result %s' %(future.result()))
    rclpy.spin_until_future_complete(delivery, future)
    response = future.result()
    delivery.get_logger().info(
        'Result of add-two_ints: for %s %s, %s' %
        ((sys.argv[1]), (sys.argv[2]),response.done))
    
    delivery.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
