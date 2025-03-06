//  This C++ code manages a node which calculates leg angle inverse kinematics from a desired base frame goal coordinate. The node 
//  subscribes to periaxis_node/legGoalCoord and publishes joint angles to dynamixel/set_position. If the dynamixel/get_position service 
//  is avaiable and two inverse kinematic solutions have been found, the node will try to select the solutions with the 
//  shortest path from the current joint position.
#include "rclcpp/rclcpp.hpp" //ros 2 library

#include <Eigen/Dense> //basic matrix and vector functions
#include <math.h> //library for math functions
//#include <string>
//#include <sstream> // used for std::ostringstream to send ROS2 node logs
#include "periaxis_interfaces/msg/leg_goal_coord.hpp" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
#include "periaxis_interfaces/msg/set_position.hpp" //message file for published dynamixel set position topic message; generated from msg/SetPosition.msg
#include "periaxis_interfaces/srv/get_position.hpp" //service file for dynamixel get position service request; generated from srv/GetPosition.srv
#include "sensor_msgs/msg/joint_state.hpp" //ros 2 standard message file for publishing joint states
//pull defines from math.h for pi (M_PI)
#define _USE_MATH_DEFINES

//Comment out below to stop debug outputs to node terminal
//#define _DEBUG

//Output format for converting Eigen matrix in comma seperated vector for printing
//const Eigen::IOFormat FlatCommaVector(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "{", "}");

const double xrad2deg = (180/M_PI); //used to convert radians to degrees

// START CLASS DEF ///////////////////////////////////////////////////////////////////
class LegKinematics : public rclcpp::Node {
	public:
		// CLASS CONSTRUCTOR
		LegKinematics() : Node("kinematic_solver")
		{
			//TBD pull this info from parameter server:
            //Populate the lookup table for which servo IDs are assigned to each leg
            _legServoIDMatrix << 	1,  2,  3, // Leg #1
									4,  5,  6, // Leg #2
									7,  8,  9, // Leg #3
									10, 11, 12, // Leg #4
									13, 14, 15; // Leg #5
								
			//Starting ROS 2 Node log message
			RCLCPP_INFO(this->get_logger(), "Starting kinematic_solver node. Node subscribes to leg goal coords on periaxis_node/legGoalCoord; joint outcomes are published to dynamixel/set_position and joint_states.");
			
			//Setup subscriber object for 'periaxis_node/legGoalCoord' topic, recieved messages invoke invKinematicSolver function
			_leg_goal_coord_sub = this->create_subscription<periaxis_interfaces::msg::LegGoalCoord>( //ROS2
				"periaxis_node/legGoalCoord", //topic
				100, //msg sub queue
				[this](const periaxis_interfaces::msg::LegGoalCoord::SharedPtr msg) { //lamda funct handler for ROS 2 class member function call format
					this->invKinematicSolver(msg);  // Pass shared pointer directly
				}
			);

            //Setup publisher for sending position command topics to dynamixel sdk
			_servo_set_position_pub = this->create_publisher<periaxis_interfaces::msg::SetPosition>(
				"dynamixel/set_position",  //topic
				100); //msg pub queue
            
            //Publisher for joint state topics
			_joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>(
				"joint_states",  //topic
				100); //msg pub queue

            /*
            //Check if the get_position service is available for selecting shortest path from current position to inverse kinematic solution 
            GetPosition_ServiceAvailable = ros::service::waitForService("dynamixel/get_position", 5000); //wait 5s for service available
            if(GetPosition_ServiceAvailable) {
                //if service is available, connect local client to it
                ServoGetPosition_client = nh->serviceClient<periaxis_interfaces::GetPosition>("dynamixel/get_position");
                
                ROS_INFO("Client connected to dynamixel/get_position service.\nNode ready...\n");
            } else {
                //
                ROS_INFO("Cannot connect to dynamixel/get_position service; continuing without consideration for 'nearest current angle' kinematic solution.\nNode ready...\n");
            }
            */
			//////////////////////////////////////////////////////////

			RCLCPP_INFO(this->get_logger(), "Node initialise sucessfully and running..."); //ROS 2 log
		}

	private:
		// PRIVATE CLASS COMPONENTS
		// Declare subscription, publisher components as class private members
		rclcpp::Subscription<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr _leg_goal_coord_sub; //Subscription for leg coord message type topic
		rclcpp::Publisher<periaxis_interfaces::msg::SetPosition>::SharedPtr _servo_set_position_pub; //publisher for servo set command topic
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_states_pub; //publisher for joint states for robot virtual visualiser
        
        //Setup type def aliases for custom matrix for int data types
        typedef Eigen::Matrix<int, 5, 3> Matrix53i;
        Matrix53i _legServoIDMatrix = Matrix53i::Constant(0); //Holds the servo IDs which are assigned to each leg number [row: legNumber(1-5); column: DMXL servo unique ID leg(base, mid, tip)]
        Matrix53i _lastCommand = Matrix53i::Constant(-1); //matrix for last numeric angle commands sent to leg DMXL servos [row: legNumber(1-5); column: last numeric joint goal (base, mid, tip]; initialised to -1
		// NEXT SECTION

        //ROS2 Subscription callback function for leg goal coordinate messages
		void invKinematicSolver(const std::shared_ptr<periaxis_interfaces::msg::LegGoalCoord> &msg) 
		{
            //Setup set_position message containing all leg servo angle commands, published as a topic for dynamixel sdk once kinematics are solved
            periaxis_interfaces::msg::SetPosition setPosCmd;
            //Setup topic msg object for publishing joint states for virtual robot visualiser
            sensor_msgs::msg::JointState jointStateMsg;

            //Update current time to when the topic was recieved
			rclcpp::Time CLK_now = this->get_clock()->now(); //ROS2, use class reference to node to get current time for joint_state pub

            Eigen::Vector4d coord_base, coord_Frame0, coord_Frame1; //vector coords within each frame perspective
            double theta1, theta2_1, theta2_2, theta3_1, theta3_2; //our calculated angles (in rad) for the three joints, note that theta2 & 3 have two solutions

            //TBD TEMP SETUP, eventualy pull these values from the ROS parameter server
            const double base_radius = 75, L1 = 23, L2 = 95, L3 = 143; //leg link lengths in (mm)
            const short int ServoMIN_J1 = 204, ServoMAX_J1 = 820; //joint1 [base] servo numeric angle limits
            const short int ServoMIN_J2 = 181, ServoMAX_J2 = 757; //joint2 [mid] servo numeric angle limits
            const short int ServoMIN_J3 = 0, ServoMAX_J3 = 538; //joint3 [tip] servo numeric angle limits

            //Calculate the legs angle in circle based on leg number (curr config for 5 legs)
            int legNum = msg->leg_num;
            double legAngle;
            if(legNum-1==0){
                legAngle = 0; //exception for div 0 errors
            } else{
                legAngle = (legNum-1)*(2*M_PI)/5; //base frame rotation (rad) to align with corresponding leg
            }

            //Set the corresponding unique servo IDs which relate to the requested leg number
            setPosCmd.id1 = _legServoIDMatrix((legNum-1),0);
            setPosCmd.id2 = _legServoIDMatrix((legNum-1),1);
            setPosCmd.id3 = _legServoIDMatrix((legNum-1),2);

            #ifdef _DEBUG 
                //Output debug for leg num and tip goal coord recieved in msg, and the servo ID's the program currently associates to the leg
				RCLCPP_DEBUG_STREAM(this->get_logger(), 
					"Recieved msg on 'periaxis_node/legGoalCoord' for leg " << legNum 
                    << " with servo IDs {" << (int)setPosCmd.id1  << ", " << (int)setPosCmd.id2 << ", " << (int)setPosCmd.id3 
                    << "} to target tip coordinate [" << msg->coord.x << ", " << msg->coord.y << ", " << msg->coord.z << "]");
            #endif

            //Construct homogeneous target coord vector in base frame from recieved topic message
            coord_base << msg->coord.x, msg->coord.y, msg->coord.z, 1;
			
            //Generate transformation matrix for base frame to leg frame 0 (ie @ face of joint 1 knuckle servo)
            Eigen::Matrix4d A_B0;
            A_B0 << cos(legAngle),     0,      sin(legAngle),     base_radius*sin(legAngle),
                    -sin(legAngle),    0,      cos(legAngle),     base_radius*cos(legAngle),
                    0,                 -1,      0,                0,
                    0,                 0,      0,                 1;
            
            //calculate base frame coord within leg frame 0
            coord_Frame0 = A_B0.inverse() * coord_base;
	
			#ifdef _DEBUG 
				//send log to ROS2 node for coord in base frame and and rotation transformed to be above leg
				RCLCPP_DEBUG_STREAM(this->get_logger(), 
						"Leg " << legNum << " BaseCoord: " << coord_base[0] << ", " << coord_base[1] << ", " << coord_base[2] << " frame0: " << coord_Frame0[0] << ", " << coord_Frame0[1] << ", " << coord_Frame0[2]);
            #endif
			
			//Calculate joint 1 (knuckle) angle from frame 0 coords
            theta1 = atan2(coord_Frame0(0), -coord_Frame0(1)); //atan(x/-y)

            //track inverted knuckle angles to reverse finger joint angles
            bool knuckleInverted = false;

            //The knuckle can only rotate -90 to +90 deg, however the plane following the knuckle which the rest of the finger operates on 
            // can potentially access full 360deg by inverting; therefore we can account for this by detecting knuckle angles outside this
            // range and use the top 180deg range to cover the lower 180 range aswell, with the fingers inverted.
            // Note there may be some strange behaviour around the knuckle -90 and +90 transition point (if it comes up) which is a kind of singularity
            if(theta1 > M_PI/2) {
                theta1 -= M_PI; //if greater than 90deg, use inverted position to achieve correct plane by subtracting 180deg
                knuckleInverted = true;
                #ifdef _DEBUG 
                    //send log to ROS2 node for knuckle CCW triggered inversion
					RCLCPP_WARN(this->get_logger(), "Theta1 greater than CCW +90deg limit, inverting knuckle -180deg to maintain angle of leg joint plane");
                #endif
            } else if(theta1 < -M_PI/2) {
                theta1 += M_PI; //if less than than -90deg, use inverted position to achieve correct plane by adding 180deg
                knuckleInverted = true;
                #ifdef _DEBUG 
                    //send log to ROS2 node for knuckle CW triggered inversion
					RCLCPP_WARN(this->get_logger(), "Theta1 less than CW -90deg limit, inverting knuckle +180deg to maintain angle of leg joint plane");
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
                //send debug to ROS2 node for coordinate location relative to each joint frame 
				RCLCPP_DEBUG_STREAM(this->get_logger(), 
					"Coord transform outputs are: \nbase:   [" << coord_base.transpose() 
					<< "]\nFrame0: [" << coord_Frame0.transpose() 
					<< "]\nFrame1: [" << coord_Frame1.transpose() << "]");
			#endif

            //pull x and y coords in frame 1 to calculate theta2 and theta3 solutions
            double F1_x, F1_y;
            F1_x = coord_Frame1(0);
            F1_y = coord_Frame1(1);
            
            //Coords in frame 1 are planar with the rest of the finger and rotate with the knuckle angle, we can now solve
            // a 2D problem to find theta2 and theta3;
            // NOTE: flipped frame, from leg side-profile, x is up, y horizontal and aligned + to finger tip, z is frame 1 joint rotation
            //Solution to simultaneous equations in terms of theta2 & theta3 is derived from basic 2-link planar arm as:
            // frame1.y = L2*cos(theta2) + L3*cos(theta2 + theta3)
            // frame1.x = L2*sin(theta2) + L3*sin(theta2 + theta3)
            //Theta2 inverse kinematic solutions
            theta2_1 = -2*atan((sqrt(- pow(F1_x, 4) - 2*pow(F1_x, 2)*pow(F1_y, 2) + 2*pow(F1_x, 2)*pow(L2, 2) + 2*pow(F1_x, 2)*pow(L3, 2) - pow(F1_y, 4) + 2*pow(F1_y, 2)*pow(L2, 2) + 2*pow(F1_y, 2)*pow(L3, 2) - pow(L2, 4) + 2*pow(L2, 2)*pow(L3, 2) - pow(L3, 4)) + 2*F1_x*L2)/(pow(F1_x, 2) + pow(F1_y, 2) + 2*F1_y*L2 + pow(L2, 2) - pow(L3, 2)));
            theta2_2 = 2*atan((sqrt(- pow(F1_x, 4) - 2*pow(F1_x, 2)*pow(F1_y, 2) + 2*pow(F1_x, 2)*pow(L2, 2) + 2*pow(F1_x, 2)*pow(L3, 2) - pow(F1_y, 4) + 2*pow(F1_y, 2)*pow(L2, 2) + 2*pow(F1_y, 2)*pow(L3, 2) - pow(L2, 4) + 2*pow(L2, 2)*pow(L3, 2) - pow(L3, 4)) - 2*F1_x*L2)/(pow(F1_x, 2) + pow(F1_y, 2) + 2*F1_y*L2 + pow(L2, 2) - pow(L3, 2)));

            //Theta3 inverse kinematic solutions
            theta3_1 = 2*atan(sqrt((- pow(F1_x, 2) - pow(F1_y, 2) + pow(L2, 2) + 2*L2*L3 + pow(L3, 2))*(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)))/(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)));
            theta3_2 = -2*atan(sqrt((- pow(F1_x, 2) - pow(F1_y, 2) + pow(L2, 2) + 2*L2*L3 + pow(L3, 2))*(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)))/(pow(F1_x, 2) + pow(F1_y, 2) - pow(L2, 2) + 2*L2*L3 - pow(L3, 2)));

			// ?? Joint1: negative angle is CW, positive is CCW; Joint2 & 3: positive angle is CW, negative is CCW ??;
			//Calculated angles are from the servo midway home position, ie 0deg is home and servo horn up;
            // the servo.position input is 0-1024 numeric which represents 0-300deg rotation; with 150deg (512 numeric) being servo horn midway home position;
            // the following maps between these different ranges:
            short int theta1_numeric   = (1024.0/300)*(theta1*xrad2deg + 150);
            short int theta2_1_numeric = (1024.0/300)*(theta2_1*xrad2deg + 150);
            short int theta2_2_numeric = (1024.0/300)*(theta2_2*xrad2deg + 150);
            short int theta3_1_numeric = (1024.0/300)*(theta3_1*xrad2deg + 150);
            short int theta3_2_numeric = (1024.0/300)*(theta3_2*xrad2deg + 150);

            #ifdef _DEBUG 
                //send debug log to ROS2 node for leg servo inverse kinematic solutions, both numeric command and angle in radians
				RCLCPP_DEBUG_STREAM(this->get_logger(),
					"# Calculated Raw Joint Angle Solutions #\n{jointName_solutionNum,numericServoCmd,angleRad}\n"
					"theta1, "   << theta1_numeric   << ", " << theta1   * xrad2deg << "\n"
					"theta2_1, " << theta2_1_numeric << ", " << theta2_1 * xrad2deg << "\n"
					"theta2_2, " << theta2_2_numeric << ", " << theta2_2 * xrad2deg << "\n"
					"theta3_1, " << theta3_1_numeric << ", " << theta3_1 * xrad2deg << "\n"
					"theta3_2, " << theta3_2_numeric << ", " << theta3_2 * xrad2deg);
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
                short int j1_previousPosition = _lastCommand(legNum-1, 0); 
                short int j2_previousPosition = _lastCommand(legNum-1, 1);
                short int j3_previousPosition = _lastCommand(legNum-1, 2); 
                
                //If there are two solutions, pick the one closest the the previously sent command (only after previous command has been sent)
                if(!invalid_S1 && !invalid_S2 && _lastCommand(legNum-1, 1) != -1 && _lastCommand(legNum-1, 2) != -1 ) {

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

                //Set topic message data ready to publish           
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
                    //send log to ROS2 node for the PREVIOUS sent servo commands
					RCLCPP_DEBUG_STREAM(this->get_logger(), 
						"Last command matrix:\n" << _lastCommand);
                #endif
                
                //Update relevent row in _lastCommand to track previously sent messages
                _lastCommand.row(legNum-1) << setPosCmd.position1, setPosCmd.position2, setPosCmd.position3;

				#ifdef _DEBUG 
                    //send log to ROS2 node
					RCLCPP_DEBUG_STREAM(this->get_logger(), 
						">> InvKinematic Joint Angle <<"
						<< "\nSelected Solution:[" << ((!invalid_S1) ? "1" : "2") << "]"
						<< "\ntheta1_rad: " << theta1*xrad2deg << " theta1_numeric: " << setPosCmd.position1
						<< "\ntheta2_rad: " << ((!invalid_S1) ? theta2_1*xrad2deg : theta2_2*xrad2deg) << " theta1_numeric: " << setPosCmd.position2
						<< "\ntheta3_rad: " << ((!invalid_S1) ? theta3_1*xrad2deg : theta3_2*xrad2deg) << " theta1_numeric: " << setPosCmd.position3);
                #endif
                
                //TBD remove, for adding temporary condition while testing on single leg
                if(legNum==1 || legNum==2 || legNum==3 || legNum==4 || legNum==5) {
				//if(legNum==1) {
                    //Publish messages to servo command topic
					_servo_set_position_pub->publish(setPosCmd);
                }

                //Generate joint state model position for simulated model
                jointStateMsg.header.stamp = CLK_now;
                jointStateMsg.name = {"L" + std::to_string(legNum) + "_joint1",  "L" + std::to_string(legNum) + "_joint2",   "L" + std::to_string(legNum) + "_joint3"};
				
				//Publish messages to joint_states position topic (used for virtual visualisation of robot legs)
				_joint_states_pub->publish(jointStateMsg);
                
				#ifdef _DEBUG 
					//send DEBUG log to ROS2 node for published joint_states message
					RCLCPP_DEBUG_STREAM(this->get_logger(), 
							"JointStateMsg: " << jointStateMsg.name[0] << ", " << jointStateMsg.name[1] << ", " << jointStateMsg.name[2]);
				#endif
						
            } else {
                //No valid solutions log to ROS2 node
				RCLCPP_ERROR(this->get_logger(), "Cannot resolve kinematic solution, null angle or out of range detected.");
            }
        }

};
// END CLASS DEF ///////////////////////////////////////////////////////////////////


int main(int argc, char **argv) {
	
	//Initialise ROS 2 node
	rclcpp::init(argc, argv);
	//auto nh = rclcpp::Node::make_shared("invKinematicSolver"); //Start node 

    // //Construct an object instance of LegManager class
	// LegManager client(node);
	
	// Create a shared pointer to LegKinematics instance
	auto node = std::make_shared<LegKinematics>();

	// Spin the node to process subscription callbacks
	rclcpp::spin(node);
	
	// Shutdown ROS 2 node when finished
	rclcpp::shutdown();
	
	return 0;
}


