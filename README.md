# Intelligent walking hexapod robot with computer vision
Intelligent RPi 4 based spider robot with computer vision and motion planning. It's full project created by student of AGH in krakow in Poland as his diploma Bachelor engineer work. Created with Middleware ROS2 on Linux Ubuntu 22.05.

It contains two sections, one for central computer for operator and one section for RPI 4b, which is used as minicomputer of robot.

Warning: Central Computer Main Controller use multithreading executors in ROS2, so if you want to use that code, you need to have multithreaded CPU ! (at least 3 threads)

# Central Computer :
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

# Raspberry PI 4b 
## Nodes:
### main nodes:
- servo subscriber
- servo sub v2 (work in progress)
- camera pub - used to send data from camera in resolution 320x320

### test nodes:
- test
- publisher




