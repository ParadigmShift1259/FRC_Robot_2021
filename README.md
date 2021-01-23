## Setting up FRC Python
Team #1259 Paradigm Shift

The robot.py in this project sets up a robot with 2 Spark controllers, an XboxController, and a CTRE Pigeon IMU gyro. You can see official documentation [here](https://robotpy.readthedocs.io/en/stable/index.html)

Requirements:
- python 3 64-bit. You can check this by typing py into the console and reading the first line
    - Install python [here](https://www.python.org/downloads/)
- pip version 21.0. You can check this with ```py -m pip -V```
    - To update, you can run ```py -m pip install --upgrade pip```


#### 1. RobotPy on your Computer
Make sure you're connected to the internet.
Install robotpy and its third party components for computer development. Enter each separately and wait for them to finish.
```sh
py -3 -m pip install robotpy
pip3 install -U robotpy[all]
```

#### 2. Python + RobotPy to the roboRIO
Make sure you're connected to the internet.
Download python, robotpy, and its components for your roboRIO.
```sh
py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer download robotpy[all]
```
Make sure you're connected to your robot's radio. The python download does take some time, so expect a minute or two of waiting.
```
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer install robotpy
py -3 -m robotpy_installer install robotpy[all]
```
#### 3. Deploying to the Robot
Make sure you run this command in terminal in this project.
Make sure you're connected to your robot's radio.
While deploying, you should see the robot code turn red, then back green again.
The console should output you with any print() or errors in your project.
```
py -3 robot.py deploy --nc
```
