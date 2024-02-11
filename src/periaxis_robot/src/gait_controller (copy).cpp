
//  This C++ code manages a node which calculates the current coordinate of the point which the tip of each leg traces along the trace function
#include <ros/ros.h> //ros library
#include <Eigen/Dense> //basic matrix and vector functions
#include <math.h> //library for math functions
#include <chrono> //timer control functionality
#include "periaxis_robot/legGoalCoord.h" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
#include "periaxis_robot/GetPosition.h" //service file for dynamixel get position service request; generated from srv/GetPosition.srv
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "periaxis_robot/gaitParameters.h" //message file for updated gait parameters topic this node subscribes to

//pull defines from math.h for pi (M_PI)
#define _USE_MATH_DEFINES

const double xrad2deg = (180/M_PI); //used to convert radians to degrees

//Start and end of (2/5)*M_PI trace to have the trace path centred over y axis
const double traceSTART = 0.943; // (3/10)*M_PI radians
const double traceEND = 2.199; // (7/10)*M_PI radians

double publishPeriod_millis = 50; //Period for sending new coordinate topic messages
double cyclePeriod_millis = 10000; //Period for the complete PUSH & STEP cycle of a leg
double push_ratio = 0.6; //ratio leg should be pushing vs stepping (eg (2/5) as 40% stepping, 60% pushing)

double traceRadius = 10; //Radius of the circle which is traced by the legs
double step_offsetFactor = 15; //effects the tip of the legs + Y and - Z axis offset from the trace path during a cycle resetting step, higher is larger step height

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

        //Offset time to control phasing between each of the legs walk cycles 
        double legCycleOffset_millis = 0;

        //Leg number, assigned to object during initialisation
        int me_legNum = 0;
        
        //Track the current position in the circles cycle
        double PI_current_trace = 0;
        double PI_current_rot = 0;

        //Transformation matrix for trace frame offset
        Eigen::Vector3d traceOffset_trans = {0, 0, 10}; //translation offset for the frame which the trace function is drawn within
        Eigen::Matrix3d traceOffset_rot = Eigen::Matrix3d::Identity(); //rotation component of trace frame offset (currently set to no rotation)
        Eigen::Matrix4d A_traceFrame = Eigen::Matrix4d::Identity(); //final transformation matrix to be constructed; set to Identity matrix to make bottom row 0,0,0,1
        
        //Tracks current trace position coords in the initial base frame (about 0,0,0); and in the transformed frame above the legs
        Eigen::Vector4d traceCoord_base, traceCoord_trans;

    public:
        GaitMaster(ros::NodeHandle *nh, int legNum, double legZAngle_rad, double legPhaseOffset_ratio, double legPhaseOffset_count) { //class constructor function
            /*  
            nh = Node handler pointer object will use to interact with ros
            legNum = Unique leg number from 1-5, used as identifier in msg to determine which servos to control
            legZAngle_rad = the world Z axis angle the trace function will be rotated by to be above the leg
            legPhaseOffset_ratio = the ratio (% from 0-1) of the legs walk cycle time which will be offset (put out of phase)
            legPhaseOffset_count = the number of legPhaseOffset_ratio offsets to apply when calculating the legs CLK_Master cycle phasing
            */

            //Setup publisher for sending position command topics to invKinematicSolver
            legGoalCoord_pub = nh->advertise<periaxis_robot::legGoalCoord>("periaxis_robot/legGoalCoord", 100);

            //initialise object leg number
            me_legNum = legNum;

            //Calculate the timing offset in ms for the leg based on the provided cycle offset ratio and the number of offsets to apply
            legCycleOffset_millis = legPhaseOffset_count * (legPhaseOffset_ratio * cyclePeriod_millis);
            
            //If the offset is larger than one cycle, we need to also offset the CLK_stepStart otherwise the leg will need to wait an
            // additional numCyclesAdjust cycles before resetting the CLK_stepStart time
            if(legCycleOffset_millis > cyclePeriod_millis) {
                int numCyclesAdjust = round(legCycleOffset_millis / cyclePeriod_millis); //find how many whole number cycles we need to offset by
                CLK_stepStart -= std::chrono::milliseconds((int)(cyclePeriod_millis * numCyclesAdjust));
            }

            //Rotation matrix to place the trace over the corresponding leg
            traceOffset_rot <<  cos(legZAngle_rad),  -sin(legZAngle_rad),   0,
                                sin(legZAngle_rad),  cos(legZAngle_rad),    0,
                                0,                   0,                     1;
            
            //Build the complete trace frame transformation matrix
            A_traceFrame.block<3,3>(0,0) << traceOffset_rot; 
            A_traceFrame.block<3,1>(0,3) << traceOffset_trans;
        }
        
        void GaitController(CLK::time_point *CLK_master) {
            //create publisher instance to publish topics to periaxis_robot/legGoalCoord
            periaxis_robot::legGoalCoord pub;

            //Calculate offset time for walk cycle phasing offset between legs based on master clock
            CLK::time_point CLK_current = *CLK_master - std::chrono::milliseconds((int)legCycleOffset_millis);

            //create milis duration object called 'elapsed' containing the current duration since the start of the legs cycle
            duration_milis elapsed = CLK_current - CLK_stepStart;

            //current step cycle progress as a percentage
            double cycle_currentProgress = elapsed.count() / cyclePeriod_millis;

            //During initialisation, the cycle phase offset may create a negative percentage if CLK_current is offset to before
            // the time CLK_stepStart was generated; if this is the case we adjust the cycle_currentProgress to its equivelant
            // in the previous cycle, eg -20% in current cycle is equivelant to 80% in the previous cycle
            if(cycle_currentProgress < 0) {
                cycle_currentProgress = (1-abs(cycle_currentProgress));
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
                CLK_stepStart = CLK_current;
            }

            //create milis duration object for the current duration since publishing last topic message (no leg offset is required here so
            // we can reference the master clock directly)
            duration_milis publish_duration = *CLK_master - CLK_publish;

            if(publish_duration.count() > publishPeriod_millis) {
                //Upate last published time
                CLK_publish = *CLK_master;

                //Calculate new coords of trace after A frame transform
                traceCoord_trans = A_traceFrame * traceCoord_base;

                //Set publish message variables with calculated coordinate target
                pub.legNum = me_legNum;
                pub.x = traceCoord_trans(0);
                pub.y = traceCoord_trans(1);
                pub.z = traceCoord_trans(2);

                ROS_INFO_STREAM("Leg " << me_legNum << " current cycle progress " << round(cycle_currentProgress*100) << " " << (cycle_currentProgress > push_ratio ? "STEPPING" : "PUSHING"));
                //ROS_INFO_STREAM("Pi " << PI_current_trace << " BaseCoord: " << traceCoord_base[0] << ", " << traceCoord_base[1] << ", " << traceCoord_base[2] << " NewCoord: " << traceCoord_trans[0] << ", " << traceCoord_trans[1] << ", " << traceCoord_trans[2]);
                legGoalCoord_pub.publish(pub);
                
            }

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
        }

}; //end class definition
//_______________________________________________________



void ParameterUpdate(const periaxis_robot::gaitParameters::ConstPtr& msg) {
    ROS_INFO_STREAM("Parameter Update " << cyclePeriod_millis << " for leg with new millis ");
}



int main(int argc, char **argv) {
    //Setup type def aliases for clock and milliseconds
    typedef std::chrono::high_resolution_clock CLK;

    //Setup master clock pointer
    CLK::time_point *CLK_master;

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
    //ros::Subscriber gaitParameterUpdate_sub;
    //gaitParameterUpdate_sub = nh.subscribe("periaxis_robot/gaitParameters", 10, ParameterUpdate);

    ros::Subscriber sub = nh.subscribe("periaxis_robot/gaitParameters", 1000, ParameterUpdate);

    while(ros::ok()) {
        //update master clock with current time
        //*CLK_master = CLK::now();

        //loop gait master controller, pointer to current master clock
        leg1.GaitController(CLK_master); 
        leg2.GaitController(CLK_master); 
        leg3.GaitController(CLK_master); 
        leg4.GaitController(CLK_master); 
        leg5.GaitController(CLK_master); 
        ros::spinOnce();
    }
    //ros::spinOnce();
    return 0;
}


