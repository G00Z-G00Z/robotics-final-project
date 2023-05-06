# Robotics homework

# Introduction

Make a simulation of a UR5 in CoppeliaSim, using python and ZeroMQ Remote API. 

![Robot UR5](./img/ur5.png)

- [ ] Investigate [Denavit-Hartenberg parameters](https://www.mdpi.com/2218-6581/11/6/137) of the UR5 
- [ ] Install dependecies: `pyzmq, cbor, numpy, argparse, time, textwrap`
- [ ] Make a `venv`
- [ ] Follow the steps in [Procedures](#procedures)

# Submission 

- [ ] **Upload your CoppeliaSim scene to Blackboard**. This should be ready for testing and be configure to be interfaced from Python using the Remote API.
- [ ] **Upload your Jupyter Notebook to Blackboard**. It is expected to run properly when tested using Python 3.10.
- [ ] **Include a demo video (10min approx.), face recording is need, where you explain steps from 3 to 16.**
- [ ] Add your personal conclusions at the end of your Jupyter Notebook.
- [ ] What would you like to learn/add to this assignment if you had to complete it just for fun and for the sole intention of learning more about this topic/simulator?
- [ ] Add references.

# Procedures

1. Work on the CoppeliSim simulator to become familiar with the virtualised UR5 robot.
2. From the Scene hierarchy window, identify the links and joints that are part of the UR5.
3. Configure the Python Child Script.
4. Open your Jupyter Notebook.
5. Import the required Python libraries.
6. Create a client to get connected to the zmqRemoteApi from CoppeliaSim.
7. Get the handles of joints and links.
8. Start simulation.
9. Define the rotation angle of each joint.
10. Implement the forward-kinematics equations to predict the robot’s end-effector position. Feel free to use the homogeneous matrix representation or the Denavit-Hartenberg description first then the homogeneous matrix representation.
11. Print the robot’s end effector position with respect to the robot’s base coordinate system.
12. Move the robot by applying the instruction sim.setJointTargetPosition(jointx, anglex) to every joint of the UR5 robot. jointx and anglex were defined in step 9.
13. Get the actual position of the robot’s end-effector.
14. Compare the predicted and the actual robot’s end-effector positions.
15. Discuss your results in detail.
16. Set all joints to a 0o rotation angle.
17. Repeat steps 9 to 15 according to the following Table 1 (NB. Add as many code cells to your Jupyter Notebook as needed in order to test the following joints configurations):

| Run | θ1  | θ2   | θ3  | θ4  | θ5  | Predicted position (X,Y,Z) | Actual position (X,Y,Z) |
| --- | --- | ---  | --- | --- | --- | ---                        | ---                     |
| 1   | 20° | 30°  | 15° | 5°  | 45° |                            |                         |
| 2   | 0°  | −90° | 90° | 0°  | 90° |                            |                         |
| 3   | 0°  | 10°  | 20° | 30° | 40° |                            |                         |

# Developers

## Jupyter lab

To run jupyter lab, run the command: `./jupyter.sh`

## Venv

```bash
python -m venv final-exam-venv
source ./final-exam-venv/bin/activate
pip install -r requirements.txt
```

