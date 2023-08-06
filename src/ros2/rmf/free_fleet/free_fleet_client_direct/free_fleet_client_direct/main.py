import rclpy
import threading
from .dds.dds_communicator import DDS_communicator 
from.dds import messages
from  . import client_direct as ffc


def main(args=None):
    # test=DDS_communicator(42,"test",messages.FreeFleetData_SlideDrawerRequest)
    # while True:
    #     test.publish(message= messages.FreeFleetData_SlideDrawerRequest("test","col",2,2,True,True))
    #     answer =test.get_next()
    #     if(answer is not None):
    #         print( answer.fleet_name)
    #     print("")
    rclpy.init(args=args)
    ff_client = ffc.free_fleet_client_direct()
    rclpy.spin(ff_client)
    ff_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()