#!/usr/bin/env python3
import rospy
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from geometry_msgs.msg import Pose2D
from beginner_tutorials.msg import Detection, DetectionArr
from arena import *

scene = Scene(host="arenaxr.org", scene="person_tracker")

class Person:
    def __init__(self, id):
        self.pinata = GLTF(
            object_id=id,
            position=([0,0,0]),
            scale=(1, 1, 1),
            persist=True,
            url=PINATA_MODEL_PATH
        )

        self.isTracked = True;
        scene.add_object(self.pinata)

# tracks all people in the scene
people = []

# assets for program
PINATA_MODEL_PATH="https://www.dropbox.com/s/a7fm7tcvybhh5rj/pinata.glb?dl=0"

people.append(Person(0))

def callback(data):
    print("Tracking {} people".format(len(people)))
    assert isinstance(data, DetectionArr)
    objects = data.objects
    num_objects = len(objects)
    if num_objects > 0:
        x = objects[0].x
        y = objects[0].y
        z = objects[0].z
        people[0].pinata.update_attributes(position=[x, y, z])
        scene.update_object(people[0].pinata)
#    for i in range(0, num_objects):
#        x = objects[i].x
#        y = objects[i].y
#        z = objects[i].z
#        if i >= len(people):
#            people.append(Person(i))
#            print("new person detected")
#        people[i].pinata.update_attributes(position=[x, y, z])
#        scene.update_object(people[i].pinata)
#        print("person at x:{}, y:{}, z:{}".format(x, y, z))
#        print("{} people detected".format(num_objects))
#    for j in range(num_objects, len(people)):
#        print("Removing 1 person")
#        people[j].pinata.update_attributes(position=[0, -1000, 0])
#        scene.update_object(people[j].pinata)
#    print('-------')

def main():
    rospy.init_node('arena', anonymous=True)
    #rospy.Subscriber("detectnet/detections", Detection2DArray, callback)
    rospy.Subscriber("/detectionprocessor/detections", DetectionArr, callback)
    print("successfully subscribed")
    rospy.spin()


if __name__ == '__main__':
    main()


