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


    def get_object_classes(self):
        return GetObjectClassesResponse("""0: Ground Coffee\n 1: Banana\n 2: Pitcher\n 3: Bleach bottle\n 
        4: Bowl\n 5: Mug\n 6: Power drill\n 7: Wood block\n 
        8: Scissors\n 9: Marker pen\n 10: Small clamp\n 11: Cheez It crackers\n 
        12: Big clamp\n 13: Foam Brick\n 14: Domino sugar box\n 15: Campbells tomato soup\n 
        16: Mustard Bottle\n 17: Tuna can\n 18: Jello Box - Chocolate\n 19: Jello Box - Strawberry\n 20: Spam can\n""")
    
    
    def change_object_class(self, new_class):
    # Check  new_class is valid int between 1-21
        if isinstance(new_class, int) & 1 <= new_class <= 21:
            self.object_class = new_class
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