
![IMG_20230810_115807](https://github.com/Vendicus/ROS2-Hexer-robot/assets/119676540/4b9171fd-d469-4a6f-a85f-2f3a48ee90b4)
<p align=center> HEXER v.1.00 during walking.</p>

# Intelligent walking hexapod robot with computer vision
Intelligent RPi 4 based spider robot with computer vision and motion planning. It's full project created by student of AGH in krakow in Poland as his diploma Bachelor engineer work. Created with Middleware ROS2 on Linux Ubuntu 22.05. The robot itself is able to detect ground (by force sensors installed in its feet) and move on terrain with little obstacles. It calculates motion plannig autonomously and is controlled by console controllers (Pygame supported). Operator can move it forward, backward or tourn it.

For now version 1.00 is avaible but i am currently working on much more stable version 2.00 (with services, actions and some basic computation by RPI 4b).

It contains two sections, one for central computer for operator and one section for RPI 4b, which is used as minicomputer of robot.

Warning: Central Computer Main Controller use multithreading executors in ROS2, so if you want to use that code, you need to have multithreaded CPU ! (at least 3 threads)


![IMG_20230810_115832](https://github.com/Vendicus/ROS2-Hexer-robot/assets/119676540/5f98e8f9-7055-4235-b04d-a6c150050d12)
<p align=center> HEXER v.1.00, up view</p>

This robot has onboard widerange rpi camera to detect objects in real time by using ssd-mobilenetv3 AI model with COCO library. Model and camera view is controlled by OpenCV. It can detect about 100 objects in real time (20-30 fps) by pipeline in ROS2. The Raspberry PI sends camera images (with changed resolution to save systems resource and work with more fps) directly to main computer. Central computer calculates and detect objects sent by Rpi and show view to operator in real time.

![AI_vision](https://github.com/Vendicus/ROS2-Hexer-robot/assets/119676540/903743ba-260d-451b-ad88-b83e77cc337b)
<p align=center> HEXER v.1.00, objects detection by vision system.</p>

</br>
<h1 align=center>LIST OF CONTENTS</h1>
</br>

# Central Computer :
</br>

## Nodes:
### main: used to controll all systems.
- Main Controller
- Main Controller v2 (work in progress)

### controllers of legs during stance - Used to calculate Bezier points and Inverse kinematics ( look at submodules to see details) for legs to stay in one position
- stand (initial process)
- stand in loop ld (left down leg)
- stand in loop lu (left up leg)
- stand in loop lm (left middle leg)
- stand in loop rm (right middle leg)
- stand in loop rd (right down leg)
- stand in loop ru (right up leg)

### controllers of legs during walk - Used to calculate Bezier points and Inverse kinematics ( look at submodules to see details) for legs during move and tourning. All walking is divided to two phases during walk for each leg.
- leg left down phase one 
- leg left down phase two
- leg left mid phase one
- leg left mid phase two
- leg left up phase one
- leg left up phase two
- leg right down phase one 
- leg right down phase two
- leg right mid phase one 
- leg right mid phase two
- leg right up phase one
- leg right uo phase two

### test nodes - used for experiments and learning process
- servo subscriber
- test
- ziper way

## Submodules : Important modules, used practically by all nodes.
- bezier - calculates bezier points in 2D for Z axis.
- forward kinematics - use for getting actual position of gripper (in this project feet).
- inverse kinematics optimalisation - the most important module. Calculates array of points for algorithm to get from one position to another.
- left tourning down leg- module used to calculate movement of feet when on ground (to turn robot)
- left tourning middle leg
- left tourning right leg
- right tourning down leg
- right tourning middle leg
- right tourning up leg
- smooth test- used to testing smooth steps of robot
- tourning- used to test tourning alghoritm
- walking- testing module

## Hexapod messages - custom created msg for ROS2 nodes
- hexapod msg folder

</br>

# Raspberry PI 4b 
</br>

## Nodes:
### main nodes:
- servo subscriber
- servo sub v2 (work in progress)
- camera pub - used to send data from camera in resolution 320x320

### test nodes:
- test
- publisher




