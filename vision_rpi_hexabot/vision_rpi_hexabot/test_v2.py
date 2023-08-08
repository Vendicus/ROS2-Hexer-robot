import cv2 
import os

# dependencies paths
main_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
file_to_process_path = os.path.join(main_dir_path,'resource','plane.jpg')
label_file = os.path.join(main_dir_path, 'resource', 'labels.txt')

# AI model paths
conifg_net_file = '/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
model_weights = '/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/frozen_inference_graph.pb'

# model build
detector = cv2.dnn.DetectionModel(model_weights, conifg_net_file)
detector.setInputSize(320,320) # image size to use on detection 320x320
detector.setInputScale(1.0/127.5) # scale = 1, doc
detector.setInputMean((127.5, 127.5, 127.5)) # shape build as :(3, 320, 320, 1) so 3 shapes 
detector.setInputSwapRB(True)

# image read
img = cv2.imread(file_to_process_path)

# data for rectangle
label_ID = [[]]
label_ID, conf, bbox = detector.detect(img, confThreshold = 0.5)
print(label_ID, bbox)

#segregate in array all labels
with open(label_file,'rt') as f:
    label = f.read().rstrip('\n').split('\n')


#creating boxes and labels on vision
for class_ID, conifdence, border in zip(label_ID.flatten(),conf.flatten(),bbox):
    cv2.rectangle(img,border,color=(55,255,0),thickness=2)
    cv2.putText(img,label[class_ID-1].upper() + str(" ID: ")+str(class_ID),(border[0],border[3]+120), cv2.FONT_HERSHEY_TRIPLEX,1,(55,255,100),1)
    cv2.putText(img,str("PROB: ") + str(round(conifdence, 2)),(border[0] + 10,border[1]+30), cv2.FONT_HERSHEY_TRIPLEX,1,(55,255,100),1)

#segregate in array all labels
with open(label_file,'rt') as f:
    label = f.read().rstrip('\n').split('\n')


cv2.imshow("output",img)
cv2.waitKey(0)