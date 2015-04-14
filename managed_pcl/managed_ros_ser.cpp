// This is the main DLL file.

#include "stdafx.h"

#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Pose.h>
#include <windows.h>

#include "managed_ros_ser.h"
#include "utils.h"
#include <msclr\lock.h>

namespace unmanaged_ros_ser {
class PosPublisherImpl
{
public:
    PosPublisherImpl(std::string const& ros_master_ip)
        : ros_master_ip_(ros_master_ip)
        , nh_()
        , pose_msg_()
        , detected_obj_pose_pub_("detected_obj_pose", &pose_msg_)
    {
        pose_msg_.position.x = pose_msg_.position.y = pose_msg_.position.z = 0;
        pose_msg_.orientation.x = pose_msg_.orientation.y = pose_msg_.orientation.z = 0;
        pose_msg_.orientation.w = 1;
        nh_.initNode((char*)ros_master_ip_.c_str());
        nh_.advertise(detected_obj_pose_pub_);
    }

    void publishPose(double x, double y, double z) {
        pose_msg_.position.x = x;
        pose_msg_.position.y = y;
        pose_msg_.position.z = z;
        detected_obj_pose_pub_.publish(&pose_msg_);
    }

    void spinOnce() {
        nh_.spinOnce();
        Sleep(100);
    }

private:
    std::string const ros_master_ip_;
    ros::NodeHandle nh_;
    geometry_msgs::Pose pose_msg_;
    ros::Publisher detected_obj_pose_pub_;
    bool stop_requested_;
    bool stopped_;
};

}

namespace managed_ros_ser {

ref class PosPublisherImpl : public System::IDisposable
{
public:
    PosPublisherImpl(System::String^ ros_master_ip)
        : unmanaged_impl_(nullptr)
        , stop_requested_(false)
        , stopped_(true)
    {
        unmanaged_impl_ = new unmanaged_ros_ser::PosPublisherImpl(utils::toStdString(ros_master_ip));
    }
    !PosPublisherImpl() {
        stop();
        delete unmanaged_impl_;
        unmanaged_impl_ = nullptr;
    }
    ~PosPublisherImpl() {
        this->!PosPublisherImpl();
    }

    void publishPose(double x, double y, double z) {
        msclr::lock l(this);
        unmanaged_impl_->publishPose(x, y, z);
    }

    void spinOnce() {
        msclr::lock l(this);
        unmanaged_impl_->spinOnce();
    }

    void start() {
        stop_requested_ = false;
        stopped_ = false;
        while(!stop_requested_) {
            spinOnce();
        }
        stopped_ = true;
    }
    void stop() {
        stop_requested_ = true;
        while(!stopped_)
            Sleep(100);
    }

private:
    unmanaged_ros_ser::PosPublisherImpl* unmanaged_impl_;
    bool stop_requested_;
    bool stopped_;
};

PosPublisher::PosPublisher() : impl_(nullptr)
{
}

void PosPublisher::start(System::String^ ros_master_ip) {
    //assert(!isStarted());
    impl_ = gcnew PosPublisherImpl(ros_master_ip);
    System::Threading::Thread^ t = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(impl_, &PosPublisherImpl::start));
    t->Start();
}

bool PosPublisher::isStarted() {
    return impl_ != nullptr;
}

void PosPublisher::publishPose(double x, double y, double z) {
    //assert(isStarted());
    impl_->publishPose(x, y, z);
}

void PosPublisher::stop() {
    clean();
}

void PosPublisher::clean() {
    if(isStarted()) {
        delete impl_;
        impl_ = nullptr;
    }
}


} // namespace managed_ros_ser
