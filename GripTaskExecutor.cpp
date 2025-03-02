#include "GripTaskExecutor.h"

GripTaskExecutor::GripTaskExecutor(const char* MotionID, const char* AngleID, const char* deviceID, const QString &portName)
    : manipulator_(new Manipulator(MotionID, AngleID))
    , gripper_(new Gripper(deviceID, portName))
    , updateTimer_(new QTimer())
{
    updateTimer_->setInterval(20);
    connect(updateTimer_, &QTimer::timeout, this, &GripTaskExecutor::update);
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
    updateTimer_->start();
    return result;
}

bool GripTaskExecutor::shutdown()
{
    updateTimer_->stop();
    bool result = manipulator_->shutdown();
    result &= gripper_->shutdown();
    return result;
}

void GripTaskExecutor::update()
{
    manipulator_->getManipulatorData(gripTaskData_.manipulatorData);
    gripper_->getData(gripTaskData_.gripperData);
}

bool GripTaskExecutor::getGripTaskData(GripTaskData &gripTaskData)
{
    gripTaskData = gripTaskData_;
    return true;
}

void GripTaskExecutor::run()
{

}

bool GripTaskExecutor::grip(int force)
{
    return gripper_->grip(force);
}

bool GripTaskExecutor::release()
{
    return gripper_->release();
}

bool GripTaskExecutor::gotoPositionAbsolute(int positionX, int positionY, int positionZ, int angleX, int angleY, int angleZ)
{
    return manipulator_->gotoPositionAbsolute(positionX, positionY, positionZ, angleX, angleY, angleZ);
}

bool GripTaskExecutor::gotoPositionRelative(int diffX, int diffY, int diffZ, int diffAngleX, int diffAngleY, int diffAngleZ)
{
    return manipulator_->gotoPositionRelative(diffX, diffY, diffZ, diffAngleX, diffAngleY, diffAngleZ);
}

bool GripTaskExecutor::setVelocity(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ)
{
    return manipulator_->setVelocity(velocityX, velocityY, velocityZ, angularVelocityX, angularVelocityY, angularVelocityZ);
}

bool GripTaskExecutor::stopMotion()
{
    return manipulator_->stopMotion();
}

bool GripTaskExecutor::moveJointPosition(const Eigen::VectorXd& jointPosition)
{
    return manipulator_->moveJointPosition(jointPosition);
}

bool GripTaskExecutor::setVelocityMode(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ)
{
    return manipulator_->setVelocityMode(velocityX, velocityY, velocityZ, angularVelocityX, angularVelocityY, angularVelocityZ);
}

bool GripTaskExecutor::closeVelocityMode()
{
    return manipulator_->closeVelocityMode();
}

bool GripTaskExecutor::setVelocityModeX(int velocityX)
{
    return manipulator_->setVelocityModeX(velocityX);
}

bool GripTaskExecutor::setVelocityModeY(int velocityY)
{
    return manipulator_->setVelocityModeY(velocityY);
}

bool GripTaskExecutor::setVelocityModeZ(int velocityZ)
{
    return manipulator_->setVelocityModeZ(velocityZ);
}

bool GripTaskExecutor::setVelocityModeRX(int angularVelocityX)
{
    return manipulator_->setVelocityModeRX(angularVelocityX);
}

bool GripTaskExecutor::setVelocityModeRY(int angularVelocityY)
{
    return manipulator_->setVelocityModeRY(angularVelocityY);
}

bool GripTaskExecutor::setVelocityModeRZ(int angularVelocityZ)
{
    return manipulator_->setVelocityModeRZ(angularVelocityZ);
}

bool GripTaskExecutor::closeVelocityModeX()
{
    return manipulator_->closeVelocityModeX();
}

bool GripTaskExecutor::closeVelocityModeY()
{
    return manipulator_->closeVelocityModeY();
}

bool GripTaskExecutor::closeVelocityModeZ()
{
    return manipulator_->closeVelocityModeZ();
}

bool GripTaskExecutor::closeVelocityModeRX()
{
    return manipulator_->closeVelocityModeRX();
}

bool GripTaskExecutor::closeVelocityModeRY()
{
    return manipulator_->closeVelocityModeRY();
}

bool GripTaskExecutor::closeVelocityModeRZ()
{
    return manipulator_->closeVelocityModeRZ();
}

bool GripTaskExecutor::gotoPositionAbsoluteX(int positionX)
{
    return manipulator_->gotoPositionAbsoluteX(positionX);
}

bool GripTaskExecutor::gotoPositionAbsoluteY(int positionY)
{
    return manipulator_->gotoPositionAbsoluteY(positionY);
}

bool GripTaskExecutor::gotoPositionAbsoluteZ(int positionZ)
{
    return manipulator_->gotoPositionAbsoluteZ(positionZ);
}

bool GripTaskExecutor::gotoPositionAbsoluteRX(int angleX)
{
    return manipulator_->gotoPositionAbsoluteRX(angleX);
}

bool GripTaskExecutor::gotoPositionAbsoluteRY(int angleY)
{
    return manipulator_->gotoPositionAbsoluteRY(angleY);
}

bool GripTaskExecutor::gotoPositionAbsoluteRZ(int angleZ)
{
    return manipulator_->gotoPositionAbsoluteRZ(angleZ);
}

bool GripTaskExecutor::gotoPositionRelativeX(int diffX)
{
    return manipulator_->gotoPositionRelativeX(diffX);
}

bool GripTaskExecutor::gotoPositionRelativeY(int diffY)
{
    return manipulator_->gotoPositionRelativeY(diffY);
}

bool GripTaskExecutor::gotoPositionRelativeZ(int diffZ)
{
    return manipulator_->gotoPositionRelativeZ(diffZ);
}

bool GripTaskExecutor::gotoPositionRelativeRX(int diffAngleX)
{
    return manipulator_->gotoPositionRelativeRX(diffAngleX);
}

bool GripTaskExecutor::gotoPositionRelativeRY(int diffAngleY)
{
    return manipulator_->gotoPositionRelativeRY(diffAngleY);
}

bool GripTaskExecutor::gotoPositionRelativeRZ(int diffAngleZ)
{
    return manipulator_->gotoPositionRelativeRZ(diffAngleZ);
}
