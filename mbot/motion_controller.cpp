#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include "maneuver_controller.h"

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers. 
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////

class StraightManeuverController : public ManeuverControllerBase
{
public:
    StraightManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float alpha = atan2(dy, dx) - pose.theta;
        float dist = sqrt(pow(dx,2) + pow(dy,2));
        float K_dist = 1.0 ;
              float K_w = 0.8;
        float K_v = 1.0;
        float K_b = 0.5;
        float w_sp = K_w * alpha;
        float v_sp = K_v * dist;
       
          if (v_sp < 0.15){
           v_sp = 0.15;
     }
       if (v_sp > 0.25){
         v_sp = 0.25;
     }
      // if (dx < 0.3){
       //   v_sp = 0.7*v_sp;

     // }
     // else if(dy < 0.3){
     //   v_sp = 0.7*v_sp;
   //  }
      
        
      
//     if (w_sp < 1.57){
  //        w_sp = 1.57;
    // }
      // if (w_sp > 2.6){
        // w_sp = 2.6;
    // 
       
      return {0, v_sp, w_sp};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        return ((fabs(pose.x - target.x) < 0.01) && (fabs(pose.y - target.y)  < 0.01));
    }
}; 

class TurnManeuverController : public ManeuverControllerBase
{
public:
    TurnManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float alpha = atan2(dy, dx) - pose.theta;
	float beta =target.theta - pose.theta;
        float K_w = 0.8;
	float K_b = 0.5;

        float w_sp = K_w * alpha + K_b * beta;

  //  if (w_sp < 1.57){
      //   w_sp = 1.57;
    // }
  //  if (w_sp > 3.1410){
  //      w_sp = 3.1410;
//     }


        return {0, 0, w_sp};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        return (fabs(angle_diff(pose.theta, target_heading)) < 0.03);
    }
};


class MotionController
{ 
public: 
    
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

	    time_offset = 0;
	    timesync_initialized_ = false;
    } 
    
    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void) 
    {
        mbot_motor_command_t cmd {now(), 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()) 
        {
            pose_xyt_t target = targets_.back();
            pose_xyt_t pose = currentPose();

            ///////  TODO: Add different states when adding maneuver controls /////// 
            if(state_ == TURN)
            { 
                if(turn_controller.target_reached(pose, target))
                {
                    // // rotate to final orientation
                    // float K_w = 1.0;
                    // float beta = target.theta - pose.theta;

                    // cmd = {0, 0, K_w * beta};

		            state_ = DRIVE;
                } 
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                }
            }
            else if(state_ == DRIVE) 
            {
                if(straight_controller.target_reached(pose, target))
                {
                    if(!assignNextTarget())
                    {
                        std::cout << "\rTarget Reached!";
                    }
                }
                else
                { 
                    cmd = straight_controller.get_command(pose, target);
                }
		    }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
		} 
        return cmd; 
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime << "\n"; 
    	for(auto pose : targets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << std::endl;

        assignNextTarget();

        //confirm that the path was received
        message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        pose_xyt_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        computeOdometryOffset(*pose);
    }
    
private:
    
    enum State
    {
        TURN,
        DRIVE
    };
    
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;
 
    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }
    
    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { targets_.pop_back(); }
        state_ = TURN; 
        return !targets_.empty();
    }
    
    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
         
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated; 
        odomToGlobalFrame_.theta = deltaTheta;
    }
    
    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        
        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            	mbot_motor_command_t cmd = controller.updateCommand();
            	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }
    
    return 0;
}
