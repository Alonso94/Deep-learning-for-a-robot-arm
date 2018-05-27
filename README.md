# Deep learning for a robot arm
Implement one of the Reinforcement learning algorithms (DDPG Deep Deterministic Ploicy Gradients), to control a robotic arm.
The goal of the project is to map **features** from a camera mounted on the robot to **motor commands** in a **end to end** way.

### Presentation:
A presentation about the project can be found here:
#### [DLRA1.pdf](https://github.com/Alonso94/Deep-learning-for-a-robot-arm/blob/master/DLRA1.pdf)

### Used tools and technologies :
* Python 2.7
* TensorFlow 1.5.1
* OpenCV
* Vrep RemoteAPI
* Pyserial

### Missions:
* **Reaching object in the work space:**

 Environment file: [armenv_real.py](https://github.com/Alonso94/Deep-learning-for-a-robot-arm/blob/master/armenv_real.py)
 
 Algorithm implementaion: [real-ddpg.py](https://github.com/Alonso94/Deep-learning-for-a-robot-arm/blob/master/real-ddpg.py)

* **Follow a black line:**

 Environment file: [mission_armenv_real.py](https://github.com/Alonso94/Deep-learning-for-a-robot-arm/blob/master/mission_armenv_real.py) 
 
 Algorithm implementaion: [mission_real-ddpg.py](https://github.com/Alonso94/Deep-learning-for-a-robot-arm/blob/master/mission_real-ddpg.py)
