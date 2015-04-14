// managed_ros_ser.h

#pragma once

//using namespace System;

namespace managed_ros_ser {

    ref class PosPublisherImpl;

    public ref class PosPublisher : public System::IDisposable
    {
    public:
        PosPublisher();
        !PosPublisher() {
            clean();
        }
        ~PosPublisher() {
            this->!PosPublisher();
        }

        void start(System::String^ ros_master_ip);
        bool isStarted();
        void publishPose(double x, double y, double z);
        void stop();

    private:
        void clean();

    private:
        PosPublisherImpl^ impl_;
    };
}
