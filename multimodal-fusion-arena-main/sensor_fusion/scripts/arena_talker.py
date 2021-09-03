#!/usr/bin/env python3
import rospy
from NESLMessages.msg import PersonArr, Person, NeslCoord
from arena import *

boxes = []

NESL_LAB_PATH = "https://www.dropbox.com/s/i8ypy60wa1b7wm5/Conf%20room%20v2.glb?dl=0"

scene = Scene(host="arenaxr.org", scene="person_tracker")


def callback(data):
    personArr = data.personArr
    num_people_in_scene = len(personArr)
    print("Tracking {} people".format(num_people_in_scene))
    while len(boxes) < num_people_in_scene:
        boxes.append(Box(scale=Scale(0.5, 2, 0.5), color=(0, 0, 255)))
    for i in range(num_people_in_scene):
        boxes[i].update_attributes(position=[
                                   personArr[i].personCoord.x, personArr[i].personCoord.y, personArr[i].personCoord.z], scale=(personArr[i].bbx, personArr[i].bby, personArr[i].bbz))
        if personArr[i].talking == True:
            boxes[i].update_attributes(color=(255, 0, 0))
        else:
            boxes[i].update_attributes(color=(0, 0, 255))

    for i in range(num_people_in_scene, len(boxes)):
        boxes[i].update_attributes(position=[0, -1000, 0])
    for box in boxes:
        scene.update_object(box)


def main():
    scene.add_object(GLTF(
        object_id="lab",
        position=(0, 0.6, 0),
        rotation=(0, 90, 0),
        scale=(1, 1, 1),
        persist=True,
        url=NESL_LAB_PATH
    ))
    scene.add_object(Box(object_id="x_axis", position=(
        2.5, 0, 0), scale=(5, 0.01, 0.01), color=(255, 0, 0), persist=True))
    scene.add_object(Box(object_id="y_axis", position=(
        0, 2.5, 0), scale=(0.01, 5, 0.01), color=(0, 255, 0), persist=True))
    scene.add_object(Box(object_id="z_axis", position=(
        0, 0, 2.5), scale=(0.01, 0.01, 5), color=(0, 0, 255), persist=True))
    rospy.init_node('arena', anonymous=True)
    rospy.Subscriber("/NESL/combined", PersonArr, callback)
    rospy.spin()
    #scene.add_object(Box(position=(0, 0, 0), scale=(1, 2, 1)))
    #scene.add_object(Line(path=((0, 0, 0), (1, 0, 0)), color=(255, 255, 255)))


if __name__ == '__main__':
    main()
