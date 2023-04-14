#!/usr/bin/env python
import rospy
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Int8
class FilterObject:

    def __init__(self):
        
        #initialise node
        rospy.init_node('object_filter_node')
        self.num = 0

        #get object class parameter from command line
        self.object_class = rospy.get_param('~object_class', '1')

        #get publishers and subscribers
        self.pub_filtered = rospy.Publisher("Filtered_detection", Detection2DArray, queue_size=10)
        self.pub_object = rospy.Publisher("object_class", Int8, queue_size=10)
        rospy.Subscriber("/yolov7/yolov7", Detection2DArray, self.callback)

            # wait for valid simulation time
        while not rospy.Time.now():
            pass
        
        # set rate for publishing
        self.rate = rospy.Rate(10)

    def callback(self, data):

        #Filter out all object detection data apart from data for object we are looking for
        filtered_detections = Detection2DArray()
        filtered_detections.header = data.header
        for object in data.detections:
            for result in object.results:
                #print("result id =", result.id)
               # print("class id = ", self.object_class, "\n")
                resultID = result.id
                classID = self.object_class
                if int(resultID) == int(classID):
                    # print("FOUND")
                    filtered_detections.detections.append(object)
                    # print("Object ", self.num, "data found")
                    self.num = self.num + 1
                    #print(filtered_detections.detections)
                    break
        #publish the data only if specified class is detected
        if filtered_detections.detections:
           # print(filtered_detections)
            self.pub_filtered.publish(filtered_detections)
            # print("Publishing filtered detections...\n")



def main():
    
    FilterObject()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()  