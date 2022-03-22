#pragma once

#include <mutex>
#include <vector>

#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "tm_msgs/FeedbackState.h"

class Tm {
   public:
    Tm(ros::NodeHandle& nh);
    ~Tm();

    std::vector<double> getJointPosition();
    bool isMoving();
    void waitForIdle();

    int stopMotion();
    int gripperOpen();
    int gripperClose();

   private:
    std::vector<double> jointPos;
    std::vector<double> jointPre;
    void feedBackCallBack(const tm_msgs::FeedbackState::ConstPtr& info);

    std::mutex m;
    ros::Subscriber feedBackSubscriber;
    ros::ServiceClient eventClient;
    ros::ServiceClient ioClient;
};
