#!/usr/bin/env python
# license removed for brevity
import os

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import argparse
import glob
import cv2
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description='Evaluator.')
parser.add_argument('-f', '--folder', type=str, required=True,
                    help='Path to folder with the images.')
parser.add_argument('-of', '--output_folder', type=str, required=True,
                    help='Path to the folder in which to put the txt files.')
args = parser.parse_args()
folder = args.folder

# setup ros stuff
pub = rospy.Publisher('/image', Image, queue_size=1)
rospy.init_node('evaluator')
bridge = CvBridge()

rospy.sleep(1)

# list all images in folder
image_paths = glob.glob(folder + '/*.jpg')

for image_path in image_paths:

    print('Analysing image ' + image_path)
    image = cv2.imread(image_path)

    image_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')

    pub.publish(image_msg)
    rospy.sleep(0.5)

    cv2.imshow('evaluator', image)
    cv2.waitKey(50)

    print('waiting for detections message ... ')
    detections_msg = rospy.wait_for_message('/detections', Detection2DArray)
    print('detections received')

    text_filename = os.path.basename(image_path)
    text_filename = args.output_folder + '/' + os.path.splitext(text_filename)[0] + '.txt'
    print(text_filename)
    f = open(text_filename, "w")

    print('There are ' + str(len(detections_msg.detections)) + ' detections')
    for detection_idx, detection in enumerate(detections_msg.detections):
        # print('Analysing detection ' + str(detection_idx))

        w = detection.bbox.size_x
        h = detection.bbox.size_y
        x = detection.bbox.center.x - int(w / 2)
        y = detection.bbox.center.y - int(h / 2)

        if detection.results[0].id == 1:
            category = 'Tomato'
        else:
            category = 'Unknown'

        score = detection.results[0].score

        f.write(category + ' ' + str(score) + ' ' + str(int(x)) + ' '
                + str(int(y)) + ' ' + str(int(w)) + ' ' + str(int(h)) + '\n')

    print('Press a key for next image ... ')
    cv2.waitKey(5)
    f.close()

#     #!/usr/bin/env python
# import rospy
# from std_msgs.msg import String
#
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#
# def listener():
#
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber("chatter", String, callback)
#
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#
# if __name__ == '__main__':
#     listener()
