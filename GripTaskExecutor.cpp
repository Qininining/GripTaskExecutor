#include "GripTaskExecutor.h"

GripTaskExecutor::GripTaskExecutor(const char* MotionID, const char* AngleID, const char* deviceID, const QString &portName)
    : manipulator_(new Manipulator(MotionID, AngleID))
    , gripper_(new Gripper(deviceID, portName))
{

}

GripTaskExecutor::~GripTaskExecutor()
{
    delete manipulator_;
    delete gripper_;
}

bool GripTaskExecutor::initialize()
{
    bool result = manipulator_->initialize();
    result &= gripper_->initialize();
    return result;
}


bool GripTaskExecutor::shutdown()
{
    bool result = manipulator_->shutdown();
    result &= gripper_->shutdown();
    return result;
}
