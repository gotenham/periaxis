
//  This C++ code manages a node which calculates the current coordinate of the point which the tip of each leg traces along the trace function
#include <ros/ros.h> //ros library
#include <Eigen/Dense> //basic matrix and vector functions
#include <math.h> //library for math functions
#include <chrono> //timer control functionality
#include "periaxis_robot/legGoalCoord.h" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
#include "periaxis_robot/GetPosition.h" //service file for dynamixel get position service request; generated from srv/GetPosition.srv
//#include "geometry_msgs/Point.h"
//#include "geometry_msgs/Quaternion.h"
#include "periaxis_robot/gaitParameters.h" //message file for updated gait parameters topic this node subscribes to

//pull defines from math.h for pi (M_PI)
#define _USE_MATH_DEFINES

const double xrad2deg = (180/M_PI); //used to convert radians to degrees

//Start and end of (2/5)*M_PI trace to have the trace path centred over y axis and equal for all legs to complete full circle
const double traceSTART = 0.943; // (3/10)*M_PI radians
const double traceEND = 2.199; // (7/10)*M_PI radians

const int publishPeriod_millis = 40; //Period for sending new coordinate topic messages

// MIN MAX Settings for incoming live parameter changes
const int cyclePeriod_millis_MIN = 2000;
const int cyclePeriod_millis_MAX = 20000;
const float traceRadius_MIN = 2;
const float traceRadius_MAX = 50;
const float push_ratio_MIN = 0.1;
const float push_ratio_MAX = 0.9;
const float step_offsetFactor_MIN = 1;
const float step_offsetFactor_MAX = 50;

// ### GLOBAL VARIABLES ### initial parameters for controlling gait and trace function parameters at startup
double cyclePeriod_millis = 8000; //Period for the complete PUSH & STEP cycle of a leg
double push_ratio = 0.6; //ratio leg should be pushing vs stepping (eg (2/5) as 40% stepping, 60% pushing)
double traceRadius = 10; //Radius of the circle which is traced by the legs
double step_offsetFactor = 10; //effects the tip of the legs + Y and - Z axis offset from the trace path during a cycle resetting step, higher is larger step height

bool reInitialise = false; //this controls when the leg class objects need to update their internal transforms and parameters after they have been updated from periaxis_robot/gaitParameters

//These are the two matrixes which determine the final position and orientation of the trace path function
Eigen::Vector3d traceOffset_trans = {0, 0, 15}; //translation offset for the frame which the trace function is drawn within        
Eigen::Matrix3d traceOffset_rot = Eigen::Matrix3d::Identity(); //rotation component of trace frame offset (currently set to no rotation)
//Eigen::Quaterniond traceOffset_rot = Eigen::Quterniond::Identity();
// ### END GV ### 

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
class GaitMaster {
    private:
        //Setup type def aliases for clock and milliseconds
        typedef std::chrono::high_resolution_clock CLK;
        typedef std::chrono::duration<float, std::milli> duration_milis;

        ros::Publisher legGoalCoord_pub; //publisher for leg coord message type topic

        //clock times used to track publishing and step rates
        CLK::time_point CLK_stepStart = CLK::now(); //initialise publish time to current time
        CLK::time_point CLK_publish = CLK::now();

        // ###KEY CLASS OBJECT VARIABLES ###
        int me_legNum = 0; //Leg number, assigned to object during initialisation
        double me_legZAngle_rad; //the world Z axis angle the trace function will be rotated by to be above this leg
        double me_legPhaseOffset_ratio; //the ratio (% from 0-1) of the legs walk cycle time which will be offset (put out of phase)
        double me_legPhaseOffset_count; //the number of legPhaseOffset_ratio offsets to apply when calculating the legs CLK_Master cycle phasing
        double me_cyclePeriod_millis; //holds the current cyclePeriod_millis, this is used to calculate the current cycle progress based on the previous cycle period after the cycle period has been updated 

        //Offset time to control phasing between each of the legs walk cycles; this is generated from: me_legPhaseOffset_count * (me_legPhaseOffset_ratio * cyclePeriod_millis)
        double legCycleOffset_millis = 0;

        //Track the current position in the circles cycle
        double PI_current_trace = 0;
        double PI_current_rot = 0;

        //Transformation matrix for trace frame offset for leg and A_traceFrame template for final combined homogeneous trace function transform matrix
        Eigen::Matrix3d legOffset_rot; //rotation matrix to align function with leg angle
        //homogeneous tranformation matrix represents a rotation FOLLOWED by a translation
        Eigen::Matrix4d A_traceFrame = Eigen::Matrix4d::Identity(); //final transformation matrix to be constructed; set to Identity matrix to make bottom row 0,0,0,1  

        //Tracks current trace position coords in the initial base frame (about 0,0,0); and in the transformed frame above the legs
        Eigen::Vector4d traceCoord_base, traceCoord_trans;

    public:
        GaitMaster(ros::NodeHandle *nh, int legNum, double legZAngle_rad, double legPhaseOffset_ratio, double legPhaseOffset_count) { //class constructor function
            //Setup publisher for sending position command topics to invKinematicSolver
            legGoalCoord_pub = nh->advertise<periaxis_robot::legGoalCoord>("periaxis_robot/legGoalCoord", 100);

            //initialise object variables
            me_legNum = legNum;
            me_legZAngle_rad = legZAngle_rad;
            me_legPhaseOffset_ratio = legPhaseOffset_ratio;
            me_legPhaseOffset_count = legPhaseOffset_count;

            double numCyclesAdjust = 0; //holds the number of cycle periods the modf normalisation has adjusted for
            //As the calculated phaseOffset_total can more than a single cycle period, we take only the decimal part of the calculation; eg offset of -120% is equivelant to -20% in current cycle
            double phaseOffset_normalised = std::modf((me_legPhaseOffset_ratio * me_legPhaseOffset_count), &numCyclesAdjust);

            //Use parameters to initialise transformation matrix and timing offsets for this specific leg;
            // note on the '1-': as we calculate the CLK_stepStart as a subtraction from the master clock, we need to invert the phaseOffset_normalised,
            // ie -40% offset equates to the leg being 60% of the way through the cycle
            this->initialiseLegParameters(1 - phaseOffset_normalised);
        }
        
        void GaitController(CLK::time_point *CLK_master, bool me_reInitialise) {
            
            //create publisher instance to publish topics to periaxis_robot/legGoalCoord
            periaxis_robot::legGoalCoord pub;

            //create milis duration object called 'elapsed' containing the current duration since the start of the legs cycle
            duration_milis elapsed = *CLK_master - CLK_stepStart;

            //current step cycle progress as a percentage
            double cycle_currentProgress = elapsed.count() / me_cyclePeriod_millis;

            //If new parameters have changed, re-initialise object variables with new values
            if(me_reInitialise) {
                CLK_stepStart = *CLK_master; //reset the legs clock to master in preperation for re-initialisation calculation
                this->initialiseLegParameters(cycle_currentProgress); //reinitialise timing with new parameters
            }

            if(cycle_currentProgress <= push_ratio) { //PUSHING PART OF CYCLE
                //Current leg tip trace location coordinates; point must travel from traceSTART to traceEND within (push_ratio * cyclePeriod_millis),
                // we use (cycle_currentProgress/push_ratio) to proportion that ratio back to a 0 - 100% range for controlling the
                // traces current position based on cycle_currentProgress
                PI_current_trace = traceSTART + ((cycle_currentProgress/push_ratio) * (traceEND - traceSTART));

                //ROS_INFO_STREAM("PUSHING: " << PI_current_trace << " cycle_currentProgress: " << cycle_currentProgress);

                //Calculate trace position in base frame
                traceCoord_base <<  traceRadius*cos(PI_current_trace), 
                                    traceRadius*sin(PI_current_trace),
                                    0,
                                    1;

            } else if(cycle_currentProgress > push_ratio && cycle_currentProgress < 1) { //STEPPING PART OF CYCLE
                 
                //Calculates the current percentage progress through the step cycle
                double STEP_progress =  1 - (1-cycle_currentProgress) / (1-push_ratio);

                //So that the stepping cycle has a soft begining and end, the current progress is adjusted using cosine from pi to 0, this converts
                // the 0-100% STEP_progress into a 100-0% progress which begins and ends slower than the middle of the step
                double PI_stepSoftener = cos(STEP_progress * M_PI)/2 + 0.5;

                //As PI_stepSoftener goes from 100-0%, our trace progresses from traceEND to traceSTART over the (1-push_ratio) time period to reset the push cycle
                PI_current_trace = traceSTART + PI_stepSoftener * (traceEND - traceSTART);

                //Calculate trace position at base frame; the step offset part of the equation modifies the y and z coordinate to
                // step away from the circle trace, increasing as the STEP trace position gets to its midway point, and then decreasing 
                // again to meet the circle trace to start the next PUSH cycle
                traceCoord_base <<  traceRadius*cos(PI_current_trace), 
                                    traceRadius*sin(PI_current_trace) + step_offsetFactor*(sin(PI_current_trace) - sin(traceSTART)),
                                    -step_offsetFactor*(sin(PI_current_trace) - sin(traceSTART)),
                                    1;

                //ROS_INFO_STREAM("STEPPING: " << PI_current_trace << " cycle_currentProgress: " << cycle_currentProgress);

            } else { //End of current step cycle, start of next cycle
                //reset clock counter
                CLK_stepStart = *CLK_master;
            }

            //create milis duration object for the current duration since publishing last topic message (no leg offset is required here so
            // we can reference the master clock directly)
            duration_milis publish_duration = *CLK_master - CLK_publish;

            if(publish_duration.count() > publishPeriod_millis) {
                
                //temp msg debugging
                ROS_INFO_STREAM("Leg " << me_legNum << " current progress " << cycle_currentProgress << ".");
                
                //Upate last published time
                CLK_publish = *CLK_master;

                //Calculate new coords of trace after A frame transform
                traceCoord_trans = A_traceFrame * traceCoord_base;

                //Set publish message variables with calculated coordinate target
                pub.legNum = me_legNum;
                pub.x = traceCoord_trans(0);
                pub.y = traceCoord_trans(1);
                pub.z = traceCoord_trans(2);

                //ROS_INFO_STREAM("Leg " << me_legNum << " current cycle progress " << round(cycle_currentProgress*100) << "% " << (cycle_currentProgress > push_ratio ? "STEPPING" : "PUSHING"));
                legGoalCoord_pub.publish(pub);
                
            }


        }

        void initialiseLegParameters(double currentCycleProgress) {
            
            //Calculate the new step start based on the current cycle progress
            CLK_stepStart -= std::chrono::milliseconds((int)(cyclePeriod_millis * currentCycleProgress));          

            //ROS_INFO_STREAM("Initialising leg " << me_legNum << " with inbound parameters."); // << traceOffset_trans(0) << ", " << traceOffset_trans(1) << ", " << traceOffset_trans(2));

            ROS_INFO_STREAM("Leg " << me_legNum << " progress "  <<  currentCycleProgress <<  " phaseoffset_total " << cyclePeriod_millis * currentCycleProgress << " " << std::chrono::duration_cast<std::chrono::milliseconds>(CLK::now() - CLK_stepStart).count() / cyclePeriod_millis);

            //Update the local version of me_cyclePeriod_millis to reflect new timing
            me_cyclePeriod_millis = cyclePeriod_millis;

            //Rotation matrix to place the trace over the corresponding leg
            legOffset_rot <<    cos(me_legZAngle_rad),  -sin(me_legZAngle_rad), 0,
                                sin(me_legZAngle_rad),  cos(me_legZAngle_rad),  0,
                                0,                      0,                      1;
            
            //Build the complete trace frame transformation matrix
            A_traceFrame.block<3,3>(0,0) << (traceOffset_rot * legOffset_rot); //combine both the trace frame rotation and the leg rotation (pre multiply traceOffset_rot as trace frame rotation is about fixed frame)
            A_traceFrame.block<3,1>(0,3) << traceOffset_trans;

        }

}; //end class definition
//_______________________________________________________

// 'periaxis_robot/gaitParameters' subscription callback function
void ParameterUpdate(const periaxis_robot::gaitParameters::ConstPtr& msg) {
    //Ensure the recieved msg has the corresponding ARMED bit true, this is to ensure variables are only updated from topics when they are intended
    // we also check if reInitialise is false so that any previously recieved updates are process first before a new parameter update is considered
    if(msg->arm_liveUpdate && !reInitialise) {
        
        //Check if the new parameters are within acceptable ranges before updating internal variables
        cyclePeriod_millis = (msg->cyclePeriod_millis >= cyclePeriod_millis_MIN && msg->cyclePeriod_millis <= cyclePeriod_millis_MAX) ? msg->cyclePeriod_millis : cyclePeriod_millis;
        traceRadius = (msg->traceRadius >= traceRadius_MIN && msg->traceRadius <= traceRadius_MAX) ? msg->traceRadius : traceRadius;
        push_ratio = (msg->push_ratio >= push_ratio_MIN && msg->push_ratio <= push_ratio_MAX) ? msg->push_ratio : push_ratio;
        step_offsetFactor = (msg->step_offsetFactor >= step_offsetFactor_MIN && msg->step_offsetFactor <= step_offsetFactor_MAX) ? msg->step_offsetFactor : step_offsetFactor;

        //Manage the trace frame translation 
        if (msg->traceOffset_trans.x != 0 || msg->traceOffset_trans.y != 0 || msg->traceOffset_trans.z != 0) {
            double trans_x = msg->traceOffset_trans.x;
            double trans_y = msg->traceOffset_trans.y;
            double trans_z = msg->traceOffset_trans.z;
            
            //Update the trace frame translation vector with the new parameters
            traceOffset_trans = {trans_x, trans_y, trans_z};
            //ROS_INFO_STREAM("ParameterUpdate " << traceOffset_trans(0) << ", " << traceOffset_trans(1) << ", " << traceOffset_trans(2));
        }

        //Manage the trace frame rotation
        if ((msg->traceOffset_rot.x != 0 || msg->traceOffset_rot.y != 0 || msg->traceOffset_rot.z != 0) && msg->traceOffset_rot.w != 0) {
            double Qrot_x = msg->traceOffset_rot.x;
            double Qrot_y = msg->traceOffset_rot.y;
            double Qrot_z = msg->traceOffset_rot.z;
            double Qrot_w = msg->traceOffset_rot.w;

            //Generate quaternion from input parameters
            Eigen::Quaterniond q(Qrot_w, Qrot_x, Qrot_y, Qrot_z);
            //Normalise quaternion coordinate components to unit vector (to account for non-normalised inbound vector)
            q.normalize();

            //Generate 3x3 rotation matrix from quaternion representation and load to trace frame rotation matrix
            traceOffset_rot = q.toRotationMatrix(); 
            ROS_INFO_STREAM("New frame rotation: " <<  Qrot_x << ", " << Qrot_y << ", " << Qrot_z << " w:" << Qrot_w <<  "\nQuaternion " << q << "\n" << traceOffset_rot;);
        }

        //trigger re-initialisation for leg parameters to account for new values
        reInitialise = true;

        // cyclePeriod_millis
        // traceRadius
        // geometry_msgs/Point traceOffset_trans
        // geometry_msgs/Quaternion traceOffset_rot
        // push_ratio
        // step_offsetFactor
    }
}


int main(int argc, char **argv) {
    //Setup type def aliases for clock and milliseconds
    typedef std::chrono::high_resolution_clock CLK;

    //Setup master clock pointer
    CLK::time_point CLK_master;

    //initialise a ROS node for our code and assign node name
    ros::init(argc, argv, "gait_controller");
    //Start node 
    ros::NodeHandle nh;

    //message to node terminal
    ROS_INFO_STREAM("Starting gait_controller node. Node controls position and timing of legs by publishing leg goal coords on periaxis_robot/legGoalCoord;");
    
    //zOffsetUnit is negative due leg numbers increment CW about Z
    double zOffsetUnit = -(2*M_PI)/5; //(circle / number of legs)

    //Construct object instances of GaitMaster class for each leg {NodeHandler, LegNumber, legZAngle_rad, PhaseOffsetRatio, NumberOfPhasesToApply}
    GaitMaster leg1{&nh, 1, 0*zOffsetUnit, 0.4, 0}; 
    GaitMaster leg2{&nh, 2, 1*zOffsetUnit, 0.4, 1};
    GaitMaster leg3{&nh, 3, 2*zOffsetUnit, 0.4, 2};
    GaitMaster leg4{&nh, 4, 3*zOffsetUnit, 0.4, 3};
    GaitMaster leg5{&nh, 5, 4*zOffsetUnit, 0.4, 4};

    //setup subscriber for gait parameter updates
    ros::Subscriber sub = nh.subscribe("periaxis_robot/gaitParameters", 10, ParameterUpdate);

    //to ensure all legs have their parameters re-initialised after a parameter callback is updated, we use AND with this loopStart variable to trigger the legs initialiseLegParameters script
    bool loopStart = false; 

    while(ros::ok()) {
        //update master clock with current time
        CLK_master = CLK::now();
        
        //Check if parameters have been updated since last loop, this ensures all legs get the update
        if(reInitialise) {
            loopStart = true;
        }

        //loop gait master controller, pointer to current master clock
        leg1.GaitController(&CLK_master, (loopStart && reInitialise)); 
        leg2.GaitController(&CLK_master, (loopStart && reInitialise)); 
        leg3.GaitController(&CLK_master, (loopStart && reInitialise)); 
        leg4.GaitController(&CLK_master, (loopStart && reInitialise)); 
        leg5.GaitController(&CLK_master, (loopStart && reInitialise)); 
        
        //Finish re-initialisation
        if(loopStart && reInitialise) {
            loopStart = false;
            reInitialise = false;
        }

        ros::spinOnce(); //Spin to process any new 'periaxis_robot/gaitParameters' subscription topics
    }

    return 0;
}

//OLD section of code for drawing a circular path which rotates about the horizontal by 45 deg each round
    /*if(elapsed.count() > (float)50) {
        
        CLK_stepStart = CLK_current;

        //Calculate trace position at base
        traceCoord_base << 5.5*cos(PI_current_trace), 5.5*sin(PI_current_trace), 0, 1;

        //Set transformation for new location of trace frame
        A_traceFrame <<   cos(PI_current_rot),    0,  sin(PI_current_rot),  0,
                            0,                      1,  0,                    14,
                            -sin(PI_current_rot),   0,  cos(PI_current_rot),  11.5,
                            0,              0,  0,            1;


        //Calculate new coords of trace after transform
        traceCoord_trans = A_traceFrame * traceCoord_base;

        pub.legNum = 1;
        pub.x = traceCoord_trans(0);
        pub.y = traceCoord_trans(1);
        pub.z = traceCoord_trans(2);

        legGoalCoord_pub.publish(pub);
        
        PI_current_trace += 0.1;
        //Reset to begining of cycle
        if(PI_current_trace >= 2*M_PI) {
            PI_current_trace = 0;

            PI_current_rot += M_PI/4;
            if(PI_current_rot >= 2*M_PI) {
                PI_current_rot = 0;
            }
        }
    }*/