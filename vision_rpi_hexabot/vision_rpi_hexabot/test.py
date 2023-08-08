import tflite_runtime.interpreter as tflite 
import cv2 
#interpreter = tflite.Interpreter(model_path="/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/lite-model_ssd_mobilenet_v1_100_320_uint8_nms_1.tflite")
import numpy as np
import os

#loading image to process
main_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
file_to_process_path = os.path.join(main_dir_path,'resource','plane.jpg')
label_file = os.path.join(main_dir_path, 'resource', 'labels.txt')

#loading labels for object recognition
label = open(label_file,"r")
label_array = label.readlines()

#load tensorflow and allocate its path
interpreter = tflite.Interpreter(model_path = "/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/lite-model_ssd_mobilenet_v1_100_320_uint8_nms_1.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# opencv technics to resize and optimize image
camera_img = cv2.imread(file_to_process_path)
file_to_process_path = cv2.resize(camera_img,(320,320))
file_to_process_path = file_to_process_path.reshape(1, 320, 320, 3)
file_to_process_path = file_to_process_path.astype(np.uint8)

# Test the model on input data.
input_shape = input_details[0]['shape']
input_data = np.array(file_to_process_path, dtype=np.uint8)
interpreter.set_tensor(input_details[0]['index'], input_data)

#predictor of objects
interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_data = interpreter.get_tensor(output_details[1]['index'])

# the end results
print(output_data)
print(label_array[int(output_data[0][0])])