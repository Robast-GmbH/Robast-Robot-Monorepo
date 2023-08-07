
import messages
from dds_communicator import DDS_communicator  

print("sss")

test=DDS_communicator(42,"slide_drawer_request",messages.FreeFleetData_SlideDrawerRequest)
while True:
    
    test.publish(message= messages.FreeFleetData_SlideDrawerRequest("test","col",2,2,True,True))
