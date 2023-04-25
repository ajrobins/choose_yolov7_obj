#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from master_pkg.srv import SetObjectClass, SetObjectClassResponse, GetObjectClasses, GetObjectClassesResponse
# from std_srvs.srv import Trigger
class objClass:

    def __init__(self):
        
        #initialise node
        rospy.init_node('choose_object_node')
        self.num = 0

        #get object class parameter from command line
        self.object_class = rospy.get_param('~object_class', '1')

        #get publishers and subscribers
        self.pub_object_class = rospy.Publisher("object_class", Int8, queue_size=10)
        self.get_classes_srv = rospy.Service("get_object_classes", GetObjectClasses, self.get_object_classes)

        self.set_class_srv = rospy.Service("set_object_class", SetObjectClass, self.change_object_class)

        self.pub_object_class.publish("self.object_class")
            # wait for valid simulation time
        while not rospy.Time.now():
            pass
        
        

        # set rate for publishing
        self.rate = rospy.Rate(10)


    def get_object_classes(self, data):
        classes = ["0: Ground Coffee",
                "1: Banana",
                "2: Pitcher",
                "3: Bleach bottle",
                "4: Bowl",
                "5: Mug",
                "6: Power drill",
                "7: Wood block",
                "8: Scissors",
                "9: Marker pen",
                "10: Small clamp",
                "11: Cheez It crackers",
                "12: Big clamp",
                "13: Foam Brick",
                "14: Domino sugar box",
                "15: Campbells tomato soup",
                "16: Mustard Bottle",
                "17: Tuna can",
                "18: Jello Box - Chocolate",
                "19: Jello Box - Strawberry",
                "20: Spam can"]
        response = []
        for i in classes:
            response.append(i)
        return GetObjectClassesResponse(response)
    
    def change_object_class(self, data):
        object_class = data.object_class
        # print(int(object_class))
    # Check  new_class is valid int between 1-21
        if isinstance(object_class, int) & 1 <= object_class <= 21:
            self.object_class = object_class
            rospy.loginfo(f"Object class set to {self.object_class}")
            self.pub_object_class.publish(self.object_class)
            return SetObjectClassResponse(True, "New object class recieved. Initiate search...")
        else:
            rospy.logwarn("Invalid object class. Please choose an integer between 0 and 20...")
            return SetObjectClassResponse(False, "Object class not valid. Please choose an integer between 0 and 20")



    def run(self):

        while not rospy.is_shutdown():
            self.pub_object_class.publish(self.object_class)
            self.rate.sleep()

def main():
    
    obj = objClass()
    obj.run()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()  