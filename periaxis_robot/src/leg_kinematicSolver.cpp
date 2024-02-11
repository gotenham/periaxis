
//  This C++ code manages a node which calculates leg angle inverse kinematics from a desired base frame goal coordinate. The node 
//  subscribes to periaxis_robot/legGoalCoord and publishes joint angles to dynamixel/set_position. If the dynamixel/get_position service 
//  is avaiable and two inverse kinematic solutions have been found, the node will try to select the solutions with the 
//  shortest path from the current joint position.
#include <ros/ros.h> //ros library
#include <Eigen/Dense> //basic matrix and vector functions
#include <math.h> //library for math functions
#include <string>
#include "periaxis_robot/legGoalCoord.h" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
#include "periaxis_robot/SetPosition.h" //message file for published dynamixel set position topic message; generated from msg/SetPosition.msg
#include "sensor_msgs/JointState.h" //ros standard message file for publishing joint states
//#include "periaxis_robot/GetPosition.h" //service file for dynamixel get position service request; generated from srv/GetPosition.srv

//pull defines from math.h for pi (M_PI)
#define _USE_MATH_DEFINES
//Comment out below to stop debug outputs to node terminal
//#define _DEBUG

//Output format for converting Eigen matrix in comma seperated vector for printing
const Eigen::IOFormat FlatCommaVector(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "{", "}");

const double xrad2deg = (180/M_PI); //used to convert radians to degrees
//const ros::Duration publishPeriodJS_sec = ros::Duration(0.2); //Period (sec) for sending new joint state topic messages to update virtual model of robot current configuration

bool GetPosition_ServiceAvailable = 0; //tracks if this service is available

int getParam(std::string paramName){ 
//#This function returns the specified parameter values from the parameter file / server 
    int myValue = 0;
    bool success = ros::param::get(paramName, myValue);
     
    if(success){
        ROS_INFO_STREAM("Getting parameter " << paramName << " as " << myValue);
    } else {
        ROS_ERROR_STREAM("Error getting " << paramName);
    }
    
    return myValue;
}

//_______________________________________________________
class LegManager {
    private:

        //clock times used to track publishing for JointState messages for virtual model of robot current configuration
        ros::Time CLK_publish_JS = ros::Time::now(); //initialise publish time to current time
        ros::Time CLK_now = ros::Time::now();

        ros::Subscriber legGoalCoord_sub; //subscriber for leg coord message type topic
        ros::Publisher ServoSetPosition_pub; //publisher for servo set command topic
        ros::Publisher JointStates_pub; //publisher for joint states for robot virtual visualiser
        //ros::ServiceClient ServoGetPosition_client; //client for calling getPosition service for DMXL servos
        
        //Setup type def aliases for custom matrix for int and double data types
        typedef Eigen::Matrix<int, 5, 3> Matrix53i;
        //typedef Eigen::Matrix<double, 5, 3> Matrix53d;
        Matrix53i legServoIDMatrix = Matrix53i::Constant(0); //Holds the servo IDs which are assigned to each leg number [row: legNumber(1-5); column: DMXL servo unique ID (base, mid, end)]
        Matrix53i lastCommand = Matrix53i::Constant(-1); //matrix for last numeric angle commands sent to leg DMXL servos [row: legNumber(1-5); column: last numeric joint goal (base, mid, end]; initialised to -1

    public:
        LegManager(ros::NodeHandle *nh) {
            //Setup subscriber object for 'periaxis_robot/legGoalCoord' topic, recieved messages invoke invKinematicSolver function
            legGoalCoord_sub = nh->subscribe<periaxis_robot::legGoalCoord>("periaxis_robot/legGoalCoord", 1000, [=](auto &msg) {invKinematicSolver(*msg);});
            
            //Setup publisher for sending position command topics to dynamixel sdk
            ServoSetPosition_pub = nh->advertise<periaxis_robot::SetPosition>("dynamixel/set_position", 1000);
            
            //Publisher for joint state topics
            JointStates_pub = nh->advertise<sensor_msgs::JointState>("joint_states", 100);

            //TBD pull this info from parameter server:
            //Populate the lookup table for which servo IDs are assigned to each leg
            legServoIDMatrix << 1,  2,  3,
                                4,  5,  6,
                                7,  8,  9,
                                10, 11, 12,
                                13, 14, 15;

            /*
            //Check if the get_position service is available for selecting shortest path from current position to inverse kinematic solution 
            GetPosition_ServiceAvailable = ros::service::waitForService("dynamixel/get_position", 5000); //wait 5s for service available
            if(GetPosition_ServiceAvailable) {
                //if service is available, connect local client to it
                ServoGetPosition_client = nh->serviceClient<periaxis_robot::GetPosition>("dynamixel/get_position");
                
                ROS_INFO("Client connected to dynamixel/get_position service.\nNode ready...\n");
            } else {
                //
                ROS_INFO("Cannot connect to dynamixel/get_position service; continuing without consideration for 'nearest current angle' kinematic solution.\nNode ready...\n");
            }
            */
            
        }

        void invKinematicSolver(const periaxis_robot::legGoalCoord& msg) {
            //Setup set position message, one for each leg joint, this will be published as a topic for dynamixel sdk once kinematics are calculated
            periaxis_robot::SetPosition setPosCmd;
            //Setup topic msg object for publishing joint states for virtual robot visualiser
            sensor_msgs::JointState jointStateMsg;

            //Update current time to when the topic was recieved
            CLK_now = ros::Time::now();

            Eigen::Vector4d coord_base, coord_Frame0, coord_Frame1; //vector coords within each frame perspective
            double theta1, theta2_1, theta2_2, theta3_1, theta3_2; //our calculated angles (in rad) for the three joints, theta2 & 3 have two solutions

            //This is set here temporarily, plan is to eventually pull these values from the parameter server
            const double base_radius = 7.5, L1 = 2.3, L2 = 9.5, L3 = 14.3;
            const short int ServoMIN_J1 = 204, ServoMAX_J1 = 820; //max and min numeric limits physically imposed by the attached servo C joint hardware
            const short int ServoMIN_J2 = 181, ServoMAX_J2 = 757;
            const short int ServoMIN_J3 = 0, ServoMAX_J3 = 538; //different min max values for last fingure which has a wider range of motion

            //Calculate the legs angle in circle based on leg number (currently configured for 5 legs)
            short int legNum = msg.legNum;
            double legAngle;
            if(legNum-1==0){
                legAngle = 0; //exception for div 0 errors
            } else{
                legAngle = (legNum-1)*(2*M_PI)/5; //base frame rotation to align with corresponding leg
            }

            //Set the corresponding unique servo IDs which relate to the requested leg number
            setPosCmd.id1 = legServoIDMatrix((legNum-1),0);
            setPosCmd.id2 = legServoIDMatrix((legNum-1),1);
            setPosCmd.id3 = legServoIDMatrix((legNum-1),2);

            //Construct target coordinate vector within base frame from topic message
            coord_base << msg.x, msg.y, msg.z, 1;

            #ifdef _DEBUG 
                //Output debugging message to node terminal
                ROS_INFO_STREAM("Topic msg recieved on 'periaxis_robot/legGoalCoord' for leg " << legNum 
                                << " {" << (int)setPosCmd.id1  << ", " << (int)setPosCmd.id2 << ", " << (int)setPosCmd.id3 
                                << "} to target [" << msg.x << ", " << msg.y << ", " << msg.z << "]");
            #endif

            //Generate transformation matrix for base frame to leg frame 0 (ie @ face of joint 1 knuckle servo)
            Eigen::Matrix4d A_B0;
            A_B0 << cos(legAngle),     0,      sin(legAngle),     base_radius*sin(legAngle),
                    -sin(legAngle),    0,      cos(legAngle),     base_radius*cos(legAngle),
                    0,                 -1,      0,                0,
                    0,                 0,      0,                 1;
            
            //calculate base frame coord within leg frame 0
            coord_Frame0 = A_B0.inverse() * coord_base;

            ROS_INFO_STREAM("leg " << legNum << " BaseCoord: " << coord_base[0] << ", " << coord_base[1] << ", " << coord_base[2] << " frame0: " << coord_Frame0[0] << ", " << coord_Frame0[1] << ", " << coord_Frame0[2]);
              
            //Calculate joint 1 (knuckle) angle from frame 0 coords
            theta1 = atan2(coord_Frame0(0), -coord_Frame0(1)); //atan(x/-y)

            //track inverted knuckle angles to reverse finger joint angles
            bool knuckleInverted = false;

            //The knuckle can only rotate -90 to +90 deg, however the plane following the knuckle which the rest of the finger operates on 
            // can access full 360deg by inverting; therefore we can account for this by detecting knuckle angles outside this
            // range and use the top 180deg range to cover the lower 180 range aswell, with the fingers inverted.
            // Note there may be some strange behaviour around the knuckle -90 and +90 transition point (if it comes up) which is a kind of singularity
            if(theta1 > M_PI/2) {
                theta1 -= M_PI; //if greater than 90deg, use inverted position to achieve correct plane by subtracting 180deg
                knuckleInverted = true;
                #ifdef _DEBUG 
                    ROS_INFO_STREAM("##DEBUG## theta1 greater than 90deg");
                #endif
            } else if(theta1 < -M_PI/2) {
                theta1 += M_PI; //if less than than -90deg, use inverted position to achieve correct plane by adding 180deg
                knuckleInverted = true;
                #ifdef _DEBUG 
                    ROS_INFO_STREAM("##DEBUG## theta1 less than -90deg");
                #endif
            }
            
            //Build next transformation matrix from calculated theta1 to generate finger frame 2D plane for next calc
            Eigen::Matrix4d A_01;
            A_01 << cos(theta1-M_PI/2),     0,      sin(theta1-M_PI/2),     0,
                    sin(theta1-M_PI/2),     0,      -cos(theta1-M_PI/2),    0,
                    0,                      1,      0,                      L1,
                    0,                      0,      0,                      1;
            
            //calculate coord within leg frame 1
            coord_Frame1 = A_01.inverse() * coord_Frame0;
  
            #ifdef _DEBUG 
                //node terminal debugging output
                ROS_INFO_STREAM("Coord transform outputs are: \nbase:   [" << coord_base.transpose() <<  "]\nFrame0: [" << coord_Frame0.transpose() <<  "]\nFrame1: [" << coord_Frame1.transpose() << "]");
            #endif

            //pull x and y coords in frame 1 to calculate theta2 and theta3 solutions
            double F1_x, F1_y;
            F1_x = coord_Frame1(0);
            F1_y = coord_Frame1(1);
            
            //Coords in frame 1 are planar with the rest of the finger and rotate with the knuckle angle, we can now solve
            // a 2D problem to find theta2 and theta3;
            //(note flipped frame, from arm side profile, x is up, y horizontal and away from arm)
            //Solution to simultaneous equations in terms of theta2 & theta3 is derived from basic 2-link planar arm as:
            // frame1.y = L2*cos(theta2) + L3*cos(theta2 + theta3)
            // frame1.x = L2*sin(theta2) + L3*sin(theta2 + theta3)
            //Theta2 inverse kinematic solutions
            theta2_1 = -2*atan((sqrt(- pow(F1_x, 4) - 2*pow(F1_x, 2)*pow(F1_y, 2) + 2*pow(F1_x, 2)*pow(L2, 2) + 2*pow(F1_x, 2)*pow(L3, 2) - pow(F1_y, 4) + 2*pow(F1_y, 2)*pow(L2, 2) + 2*pow(F1_y, 2)*pow(L3, 2) - pow(L2, 4) + 2*pow(L2, 2)*pow(L3, 2) - pow(L3, 4)) + 2*F1_x*L2)/(pow(F1_x, 2) + pow(F1_y, 2) + 2*F1_y*L2 + pow(L2, 2) - pow(L3, 2)));
            theta2_2 = 2*atan((sqrt(- pow(F1_x, 4) - 2*pow(F1_x, 2)*pow(F1_y, 2) + 2*pow(F1_x, 2)*pow(L2, 2) + 2*pow(F1_x, 2)*pow(L3, 2) - pow(F1_y, 4) + 2*pow(F1_y, 2)*pow(L2, 2) + 2*pow(F1_y, 2)*pow(L3, 2) - pow(L2, 4) + 2*pow(L2, 2)*pow(L3, 2) - pow(L3, 4)) - 2*F1_x*L2)/(pow(F1_x, 2) + pow(F1_y, 2) + 2*F1_y*L2 + pow(L2, 2) - pow(L3, 2)));

            //Theta3 inverse kinematic solutions
            theta3_1 = 2*atan(sqrt((- pow(F1_x, 2) - pow(F1_y, 2) + pow(L2, 2) + 2*L2*L3 + pow(L3, 2))*(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)))/(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)));
            theta3_2 = -2*atan(sqrt((- pow(F1_x, 2) - pow(F1_y, 2) + pow(L2, 2) + 2*L2*L3 + pow(L3, 2))*(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)))/(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)));

            //Calculated angles are from the servo midway home position, ie 0deg is home and servo horn up;
            // Joint1: negative angle is CW, positive is CCW; Joint2&3: positive angle is CW, negative is CCW;
            // the servo.position input is 0-1024 numeric which represents 0-300deg rotation; with 150deg (512 numeric) being servo horn midway home position;
            // the following maps between these different ranges:
            short int theta1_numeric   = (1024.0/300)*(theta1*xrad2deg + 150);
            short int theta2_1_numeric = (1024.0/300)*(theta2_1*xrad2deg + 150);
            short int theta2_2_numeric = (1024.0/300)*(theta2_2*xrad2deg + 150);
            short int theta3_1_numeric = (1024.0/300)*(theta3_1*xrad2deg + 150);
            short int theta3_2_numeric = (1024.0/300)*(theta3_2*xrad2deg + 150);

            #ifdef _DEBUG 
                //DEBUGGING, can be commented out later on if needed
                ROS_INFO_STREAM("## DEBUG ##\ntheta1_numeric " << theta1_numeric << " angle: " << theta1*xrad2deg
                << "\ntheta2_1_numeric " << theta2_1_numeric << " angle: " << theta2_1*xrad2deg
                << "\ntheta2_2_numeric " << theta2_2_numeric << " angle: " << theta2_2*xrad2deg
                << "\ntheta3_1_numeric " << theta3_1_numeric << " angle: " << theta3_1*xrad2deg
                << "\ntheta3_2_numeric " << theta3_2_numeric << " angle: " << theta3_2*xrad2deg);
            #endif

            //Check if a real solution has been found for inverse kinematics
            bool invalid_S1 = isnan(theta2_1) || isnan(theta3_1); //solution set 1 S1 invalid due null
            bool invalid_S2 = isnan(theta2_2) || isnan(theta3_2); //solution set 2 S2 invalid due null
            //Validate theta 1 is within servo bounds
            bool invalid_theta1 = theta1_numeric < ServoMIN_J1 || theta1_numeric > ServoMAX_J1;

            //Validate kinematic solution 1 is within servo bounds
            if(theta2_1_numeric < ServoMIN_J2 || theta2_1_numeric > ServoMAX_J2 || theta3_1_numeric < ServoMIN_J3 || theta3_1_numeric > ServoMAX_J3) {
                invalid_S1 = true;
            }
            //Validate kinematic solution 2 is within servo bounds
            if(theta2_2_numeric < ServoMIN_J2 || theta2_2_numeric > ServoMAX_J2 || theta3_2_numeric < ServoMIN_J3 || theta3_2_numeric > ServoMAX_J3) {
                invalid_S2 = true;
            }
            
            //if either theta 1, or both kinematic solutions, are null or out of range; kinematic solution is unresolved
            // if this is the case code will jump to the 'else' of this if statement
            if( !(invalid_theta1 || invalid_S1 && invalid_S2) ) {
                //Get the previously sent commands, used to check closest kinematic solution if two are found, and to mute repeated commands being sent for same position request
                short int j1_previousPosition = lastCommand(legNum-1, 0); 
                short int j2_previousPosition = lastCommand(legNum-1, 1);
                short int j3_previousPosition = lastCommand(legNum-1, 2); 
                
                //If there are two solutions, pick the one closest the the previously sent command (only after previous command has been sent)
                if(!invalid_S1 && !invalid_S2 && lastCommand(legNum-1, 1) != -1 && lastCommand(legNum-1, 2) != -1 ) {

                    //Find which solution is closest to current joint configuration
                    short int deltaJ2S1 = abs(j2_previousPosition - theta2_1_numeric); //Joint 2 S1 solution
                    short int deltaJ3S1 = abs(j3_previousPosition - theta3_1_numeric); //Joint 3 S1 solution
                    
                    short int deltaJ2S2 = abs(j2_previousPosition - theta2_2_numeric); //Joint 2 S2 solution
                    short int deltaJ3S2 = abs(j3_previousPosition - theta3_2_numeric); //Joint 3 S2 solution

                    //Select the solution with the shortest path from current joint positions
                    if( (deltaJ2S1+deltaJ3S1) <= (deltaJ2S2+deltaJ3S2) ) {
                        //If solution 1 is valid and has a shorter or equal path to solution 2, invalidate solution 2
                        invalid_S2 = true;
                    } else if( (deltaJ2S1+deltaJ3S1) > (deltaJ2S2+deltaJ3S2)) {
                        // otherwise if solution 2 is valid and has a shorter path than solution 1, invalidate solution 1
                        invalid_S1 = true;
                    }
                }
                
                //-----------------------------
                /*
                //Set topic message data ready to be published               
                setPosCmd.position1 = (theta1_numeric * 0.2) + (j1_previousPosition * 0.8);
                //Select best kinematic solution:
                if(!invalid_S1) {
                    //S1 solution available
                    setPosCmd.position2 = (theta2_1_numeric * 0.2) + (j2_previousPosition * 0.8);
                    setPosCmd.position3 = (theta3_1_numeric * 0.2) + (j3_previousPosition * 0.8);

                    //Update last joint angle (rad) of solution 1 for updating the virtual robot model
                    jointStateMsg.position = {theta1, theta2_1, theta3_1};
                } else if(!invalid_S2) {
                    //S2 solution available
                    setPosCmd.position2 = (theta2_2_numeric * 0.2) + (j2_previousPosition * 0.8);
                    setPosCmd.position3 = (theta3_2_numeric * 0.2) + (j3_previousPosition * 0.8);

                    //Update last joint angle (rad) of solution 2 for updating the virtual robot model
                    jointStateMsg.position = {theta1, theta2_2, theta3_2};
                }
                */
                //---------------------------

                //Set topic message data ready to be published               
                setPosCmd.position1 = theta1_numeric;
                //Select best kinematic solution:
                if(!invalid_S1) {
                    //S1 solution available
                    setPosCmd.position2 = theta2_1_numeric;
                    setPosCmd.position3 = theta3_1_numeric;

                    //Update last joint angle (rad) of solution 1 for updating the virtual robot model
                    jointStateMsg.position = {theta1, theta2_1, theta3_1};
                } else if(!invalid_S2) {
                    //S2 solution available
                    setPosCmd.position2 = theta2_2_numeric;
                    setPosCmd.position3 = theta3_2_numeric;

                    //Update last joint angle (rad) of solution 2 for updating the virtual robot model
                    jointStateMsg.position = {theta1, theta2_2, theta3_2};
                }

                //If calculated position is the same as was previously sent to this servo, mute command by removing arm authority. 
                // Arming gives execute authority to the DMXL driver node
                (setPosCmd.position1 == j1_previousPosition) ? setPosCmd.arm1 = false : setPosCmd.arm1 = true;
                (setPosCmd.position2 == j2_previousPosition) ? setPosCmd.arm2 = false : setPosCmd.arm2 = true;
                (setPosCmd.position3 == j3_previousPosition) ? setPosCmd.arm3 = false : setPosCmd.arm3 = true;

                #ifdef _DEBUG 
                    ROS_INFO_STREAM("## DEBUG ## Last command matrix:\n" << lastCommand);
                #endif
                
                //Update relevent row in lastCommand to track previously sent messages
                lastCommand.row(legNum-1) << setPosCmd.position1, setPosCmd.position2, setPosCmd.position3;

                #ifdef _DEBUG 
                    ROS_INFO_STREAM("Inverse kinematics resolved angle, solution " << ((!invalid_S1) ? "S1" : "S2") << " selected:\ntheta1: " << theta1*xrad2deg << " theta1_numeric: " << setPosCmd.position1
                    << "\ntheta2: " << ((!invalid_S1) ? theta2_1*xrad2deg : theta2_2*xrad2deg) << " theta1_numeric: " << setPosCmd.position2
                    << "\ntheta3: " << ((!invalid_S1) ? theta3_1*xrad2deg : theta3_2*xrad2deg) << " theta1_numeric: " << setPosCmd.position3 << "\n");
                #endif
                
                //temporary condition while testing on single leg
                if(legNum==1 || legNum==2 || legNum==3 || legNum==4 || legNum==5) {
                    //Publish messages to servo command topic
                    ServoSetPosition_pub.publish(setPosCmd);
                }

                //Generate joint state model position for simulated model
                jointStateMsg.header.stamp = CLK_now;
                jointStateMsg.name = {"L" + std::to_string(legNum) + "_joint1",  "L" + std::to_string(legNum) + "_joint2",   "L" + std::to_string(legNum) + "_joint3"};
                JointStates_pub.publish(jointStateMsg);
                
                //ROS_INFO_STREAM(jointStateMsg.name[0] << ", " << jointStateMsg.name[1] << ", " << jointStateMsg.name[2]);

                /*
                //Find elapsed duration since last visualisation topic was sent
                ros::Duration elapsed = CLK_now - CLK_publish_JS;
                if(elapsed > publishPeriodJS_sec) {
                    //Upate last published time
                    CLK_publish_JS = CLK_now;

                    //VectorXd allTheta(lastJointTheta.size());
                    //ROS_INFO_STREAM("MSGPublish: " << lastJointTheta.format(FlatCommaVector)); // << "- \n" << lastJointTheta);

                    //std::vector<double> jointPositions(lastJointTheta.size());
                    //Matrix53d::Map(&jointPositions[0], lastJointTheta.size()) = lastJointTheta;

                    //std::vector<double> vec(lastJointTheta.data(), lastJointTheta.data() + lastJointTheta.size());

                    for (int i = 0; i < 15; i++) {
                        ROS_INFO_STREAM("VectorPos:" << i << " " << lastJointState[i]); // << "- \n" << lastJointTheta);
                    }
                    

                    //Eigen::Map<Matrix53d>(&jointPositions[0], lastJointTheta.size()) = lastJointTheta;
                    
                    // vector<double> vec(arr.cols()); 
                    // Map<RowVectorXd>(&vec[0], 1, mat.cols()) = mat.row(0);

                    //(lastJointTheta.data(), lastJointTheta.size());

                    
                    jointStateMsg.header.stamp = CLK_now;
                    jointStateMsg.name = {"L1_joint1", "L1_joint2", "L1_joint3"};
                    jointStateMsg.position = &lastJointState;
                    JointStates_pub.publish(jointStateMsg);
                    
                    jointStateMsg.header.stamp = CLK_now;
                    jointStateMsg.name = {"L1_joint1", "L1_joint2", "L1_joint3"};
                    jointStateMsg.position = {theta1, ((!invalid_S1) ? theta2_1 : theta2_2), ((!invalid_S1) ? theta3_1 : theta3_2)};
                    JointStates_pub.publish(jointStateMsg);
                    
                }
                */

            } else {
                //No valid solutions
                ROS_ERROR_STREAM("Cannot resolve kinematic solution, null angle or out of range detected.");
            }
        }

}; //end class definition
//_______________________________________________________

int main(int argc, char **argv) {
    //initialise a ROS node for our code and assign node name
    ros::init(argc, argv, "invKinematicSolver");
    //Start node 
    ros::NodeHandle nh;

    //message to node terminal
    ROS_INFO_STREAM("Starting invKinematicSolver node. Node subscribes to leg goal coords on periaxis_robot/legGoalCoord; joint outcomes are published to dynamixel/set_position");
    
    //Construct an object instance of LegManager class
    LegManager client{&nh};

    ros::Rate rate(1000); //rate of ros spin in Hz
    while(ros::ok()){
        //process queue
        ros::spin();
    }
}


