#include "tm.h"

#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "ros/node_handle.h"
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetEventRequest.h"

using namespace std;

Tm::Tm(ros::NodeHandle& nh) {
    this->feedBackSubscriber = nh.subscribe<tm_msgs::FeedbackState>(
        "/feedback_states", 1000, &Tm::feedBackCallBack, this);
    this->eventClient =
        nh.serviceClient<tm_msgs::SetEvent>("/tm_driver/set_event");

    this->ioClient = nh.serviceClient<tm_msgs::SetIO>("/tm_driver/set_io");

    jointPos.resize(6);
    jointPre.resize(6);
}

Tm::~Tm() {}

void Tm::feedBackCallBack(const tm_msgs::FeedbackState::ConstPtr& info) {
    this->jointPos = info->joint_pos;
}

std::vector<double> Tm::getJointPosition() {
    lock_guard<std::mutex> lock(m);
    return this->jointPos;
}

bool Tm::isMoving() {
    bool result = false;
    std::vector<double> jointNow = getJointPosition();

    for (uint i = 0; i < jointNow.size(); i++) {
        result = (abs(jointNow[i] - jointPre[i]) > 0.000001) ? true : result;
    }
    jointPre = jointNow;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return result;
}

void Tm::waitForIdle() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (true) {
        if (isMoving() != true) {
            break;
        }
    }
}
int Tm::stopMotion() {
    tm_msgs::SetEvent request;
    request.request.func = tm_msgs::SetEvent::Request::STOP;
    if (eventClient.call(request)) {
        if (request.response.ok) {
        } else
            ROS_WARN_STREAM("SetEvent to robot , but response not yet ok ");

    } else {
        ROS_ERROR_STREAM("Error SetEvent to robot");
        return -1;
    }
    return 0;
}

int Tm::gripperOpen() {
    tm_msgs::SetIO request;

    request.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    request.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
    request.request.pin = 0;
    request.request.state = tm_msgs::SetIO::Request::STATE_ON;
    // request.request.func = tm_msgs::SetEvent::Request::STOP;
    if (ioClient.call(request)) {
        if (request.response.ok) {
        } else
            ROS_WARN_STREAM("SetIO to robot , but response not yet ok ");

    } else {
        ROS_ERROR_STREAM("Error SetIO to robot");
        return -1;
    }
    return 0;
}

int Tm::gripperClose() {
    tm_msgs::SetIO request;

    request.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    request.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
    request.request.pin = 0;
    request.request.state = tm_msgs::SetIO::Request::STATE_OFF;
    // request.request.func = tm_msgs::SetEvent::Request::STOP;
    if (ioClient.call(request)) {
        if (request.response.ok) {
        } else
            ROS_WARN_STREAM("SetIO to robot , but response not yet ok ");

    } else {
        ROS_ERROR_STREAM("Error SetIO to robot");
        return -1;
    }
    return 0;
}
