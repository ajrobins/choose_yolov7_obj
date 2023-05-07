#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import sqlite3
import os
from master_pkg.srv import findObject
from gazebo_msgs.srv import GetModelState
obj_data = []

class ObjectDetector:
    def __init__(self):

        rospy.init_node('findObject_experiment_node', anonymous=True)

        self.world_name = rospy.get_param("world")
        self.model_name = rospy.get_param("model")

         # Setup database
        self.create_database()

    def create_database(self):
        # Connect to db
        connectDB = sqlite3.connect(os.path.join(os.path.dirname(__file__), "experiment_data.db"))
        cursor = connectDB.cursor()

        # Create table
        cursor.execute('''CREATE TABLE IF NOT EXISTS object_data
                            (
                                object_class INTEGER,
                                success INTEGER,
                                probability INTEGER,
                                time_in_secs INTEGER,
                                x REAL,
                                y REAL,
                                z REAL,
                                false_detections INTEGER, 
                                frame_id TEXT,
                                world TEXT,
                                model TEXT)
                            ''')
        connectDB.commit()
        cursor.close()
        connectDB.close()
    def get_ground_truth(self, class_id):
        #wait for ground truth service to come available
        rospy.wait_for_service("/get_ground_truth")
        try:
            db_proxy = rospy.ServiceProxy("/get_ground_truth", GetModelState)
            response = db_proxy(model_name=str(class_id), relative_entity_name='')

            if response.success is True:
                # print("Got ground truth for detection")
                # print(response)
                ground_truth = PoseStamped()
                ground_truth.pose.position = response.pose.position
                return ground_truth
            else:
                rospy.logerr("Could not get ground truth for object detection...")
            
        except rospy.ServiceException as e:
            print("db srv failed...: {}".format(e))

    def insert_object_data(self, obj_data):
        # Connect to db
        connectDB = sqlite3.connect(os.path.join(os.path.dirname(__file__), "experiment_data.db"))
        cursor = connectDB.cursor()

        # Insert object data into database
        obj_data.append(self.world_name)
        obj_data.append(self.model_name)
        print(obj_data)
        cursor.execute("INSERT INTO object_data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                             obj_data)
        connectDB.commit()
        cursor.close()
        connectDB.close()

    def run(self):
       
        print("test")
        # Wait for the /find_object service to become available
        rospy.wait_for_service('/find_object')
        find_object = rospy.ServiceProxy('/find_object', findObject)

        # List of objects to detect
        objects_to_detect = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 16, 17, 18, 19, 20]
        for obj in objects_to_detect:
            # Call the /find_object service
            try:
                rospy.loginfo("Sending Object Request over /find_object")
                response = find_object(obj)
                rospy.loginfo("Object detected: %s", response)
                obj_data.append([obj, response.success, response.probability, response.time_in_secs,
                            response.x, response.y, response.z, response.false_detections, response.frame_id])
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s", e)

            rospy.Rate(10).sleep()


if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        for detection in obj_data:
            detector.insert_object_data(detection)
        pass
