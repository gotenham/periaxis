# Periaxis
*Designing a kinetic sculpture of a robotic hand rotating a globe*

This is a personal project inspired by Mark Setrakian's work; I've been interested in doing another project using [ROS](https://www.ros.org/) for a while, and thought the inverse kinematics challenge of this sculpture sounded interesting to solve. Below is the design process I've taken to get to where the project currently sits:

The name is a portmanteau of periapsis (the closest point to the earth in an orbit) and axis (as in spinning on an axis).

## Design

The hand will use 5 fingers to hold and rotate a sphere (earth globe). To rotate the sphere the fingers must trace a circle on a horizontal plane, the sequence of each leg is offset so the sphere is always contacted by at least three fingers in a triangular pattern.

Illustration of timing:
![StepSequence](https://github.com/gotenham/periaxis/assets/40827722/b899385a-a847-4438-a2d5-3e1a63a19e26)

The ROS configuration will include seperate nodes for Gait Control (master clock), leg Inverse Kinematic Solver, and Servo Driver SDK; the initial design for the ROS system is below:

![image](https://github.com/gotenham/periaxis/assets/40827722/f7a554e9-8029-49e5-9ed4-75a6da031c31)

ROS Topic Publisher Subscriber map:

![Screenshot at 2022-06-18 16-14-02](https://github.com/gotenham/periaxis/assets/40827722/d247e2df-09df-400a-b23a-88ed1037ec13)

A prototype was designed to better understand the inverse kinematics of the leg, each joint is asigned a frame of reference so matrix transformations can be constructed to convert coordinates them:

![Frames](https://github.com/gotenham/periaxis/assets/40827722/041c30cd-e3e1-4b0f-8732-6eeeae83a030)

Design of fingers and encapsulated servos:

![image](https://github.com/gotenham/periaxis/assets/40827722/18651be7-1fc8-48c1-aa5d-6af9307de450)

![1](https://github.com/gotenham/periaxis/assets/40827722/e8a2576b-d650-44b3-b750-8a5e031b2957)
![2](https://github.com/gotenham/periaxis/assets/40827722/73c55d32-d493-4b3f-ac67-9f070293c5bf)
![3](https://github.com/gotenham/periaxis/assets/40827722/2ce5f90e-0ac6-454b-9213-3a62e1632cbb)


![image](https://github.com/gotenham/periaxis/assets/40827722/173ad446-b793-496c-aa71-93519072643e)

![image](https://github.com/gotenham/periaxis/assets/40827722/fc6f8372-2ddb-492c-a863-386480ea1e16)

![image](https://github.com/gotenham/periaxis/assets/40827722/eee5b170-b65f-45c8-b479-f9387d5b312b)

![image](https://github.com/gotenham/periaxis/assets/40827722/06c6b08d-63a3-4173-9dec-7fc9ff528726)
![image](https://github.com/gotenham/periaxis/assets/40827722/5d356f3e-03d6-4a57-8d20-64289b5da071)

![image](https://github.com/gotenham/periaxis/assets/40827722/6bcdd7f1-773c-4987-8832-744de26550db)
