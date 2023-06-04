#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int64
from morai_msgs.msg import EgoVehicleStatus

class Ego():

    def __init__(self) :

        rospy.init_node("Ego_gps")
        SubEgo = rospy.Subscriber("/Ego_topic",EgoVehicleStatus,self.Callback)
        
        self.Pubmsg = rospy.Publisher('/sector',Int64,queue_size = 3)
        self.EgoData = EgoVehicleStatus()
        
        print(self.EgoData)
        rospy.spin()
        pass

    
    def Callback(self,data):
        self.EgoData.position = data.position
        position = [self.EgoData.position.x,self.EgoData.position.y]
        

        if 0<= position[0] <= 1.0 and -5.6<= position[1] <= -5.2 :
            sector = -1
        elif -7.20716 <= position[0] <= 2.153631 and -5.604280 <= position[1] <= -5.290197:
            sector = 1
        elif 3.79796 <= position[0] <= 7.665473 and -5.657542 <= position[1] <= -3.498897:
            sector = 2
        elif 7.1649327 <= position[0] <= 7.4233531 and -3.174717 <= position[1] <= 1.498442888:
            sector = 3
        elif 7.1443076 <= position[0] <= 9.021918 and 1.82776844 <= position[1] <= 4.5027089:
            sector = 4
        elif 9.584747 <= position[0] <= 12.383953 and  3.08582639 <= position[1] <= 4.5179562 :
            sector = 5
        elif 12.0928077 <= position[0] <= 12.3852863 and 1.6152038 <= position[1] <= 2.9787034:
            sector = 6
        elif 13.6311550 <= position[0] <= 14.04518 and  -1.3936691284 <= position[1] <= -0.672400116:
            sector = 7
        elif 14.8219060 <= position[0] <= 19.6635532 and -1.28312397 <= position[1] <= 3.2881116:
            sector = 8
        elif  -6.00000 <= position[0] <= 16.89175 and  5.27467 <= position[1] <= 5.60317:
            sector = 9
        
        else: 
            sector = 0
        print(f"x : {position[0]}, y : {position[1]}")
        print(f"sector : {sector}")

        self.Pubmsg.publish(sector)
        pass

if __name__ == "__main__":
    ego = Ego()