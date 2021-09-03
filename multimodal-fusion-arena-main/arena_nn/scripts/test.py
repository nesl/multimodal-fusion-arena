#!/usr/bin/env python3
import rospy
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from geometry_msgs.msg import Pose2D

from arena import *

# location of pinata
pinata_loc = [0,0,0]

# assets for program
PINATA_MODEL_PATH="https://www.dropbox.com/s/a7fm7tcvybhh5rj/pinata.glb?dl=0"

scene = Scene(host="arenaxr.org", scene="person_tracker")
pinata = GLTF(
            object_id="pinata",
            position=(pinata_loc),
            scale=(1, 1, 1),
            persist=True,
            url=PINATA_MODEL_PATH
            )

scene.add_object(pinata)
print("pinata created")
#@scene.run_once
#def make_pinata():
#    global pinata, pinata_loc
    

def callback(data):
    assert isinstance(data, Detection2DArray)
    new_x = -1
    new_y = -1
    people_counter = 0;
    for detection in data.detections:
        if detection.results[0].score > 0.7:
            people_counter += 1
            x = detection.bbox.center.x / 50
            y = (720 - detection.bbox.center.y) / 50
            temp_loc_x = pinata_loc[0]
            if new_x == -1 or abs(x - temp_loc_x) < abs (new_x - temp_loc_x):
                new_x = x
                new_y = y
    
    print("{} people detected".format(people_counter))
    if new_x != -1:
        print("x:{}, y:{}".format(new_x,new_y))
        pinata_loc[0] = new_x
        pinata_loc[1] = new_y
        pinata.update_attributes(position=pinata_loc)
        scene.update_object(pinata)	

#    for detection in data.detections:
#    	x = detection.bbox.center.x;
#    	y = detection.bbox.center.y;
#    	print("x:{}, y:{}".format(x,y))
#    print("1 frame")
    
def main():
    rospy.init_node('arena', anonymous=True)
    rospy.Subscriber("detectnet/detections", Detection2DArray, callback)
    print("successfully subscribed")
    rospy.spin()


if __name__ == '__main__':
    main()

