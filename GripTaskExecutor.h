#ifndef GRIPTASKEXECUTOR_H
#define GRIPTASKEXECUTOR_H

#include "Manipulator.h"
#include "Gripper.h"


struct  GripTaskData
{
    ManipulatorData manipulatorData;
    GripperData gripperData;
    GripTaskData()
        : manipulatorData()
        , gripperData() {}
};

class GripTaskExecutor : public QThread
{
    Q_OBJECT
public:
    GripTaskExecutor(const char* MotionID, const char* AngleID, const char* deviceID, const QString &portName);
    ~GripTaskExecutor();
    bool initialize();
    bool shutdown();

    bool grip(int force = 50000); //默认夹持力为50000微牛
    bool release();

    // Motion control functions
    bool gotoPositionAbsolute(int positionX, int positionY, int positionZ, int angleX, int angleY, int angleZ);
    bool gotoPositionRelative(int diffX, int diffY, int diffZ, int diffAngleX, int diffAngleY, int diffAngleZ);
    bool setVelocity(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ);
    bool stopMotion();
    bool moveJointPosition(const Eigen::VectorXd& jointPosition);

    // Speed control functions
    bool setVelocityMode(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ);
    bool closeVelocityMode();

    // Single axis speed control functions
    bool setVelocityModeX(int velocityX);
    bool setVelocityModeY(int velocityY);
    bool setVelocityModeZ(int velocityZ);
    bool setVelocityModeRX(int angularVelocityX);
    bool setVelocityModeRY(int angularVelocityY);
    bool setVelocityModeRZ(int angularVelocityZ);
    bool closeVelocityModeX();
    bool closeVelocityModeY();
    bool closeVelocityModeZ();
    bool closeVelocityModeRX();
    bool closeVelocityModeRY();
    bool closeVelocityModeRZ();

    // Single axis motion control functions
    bool gotoPositionAbsoluteX(int positionX);
    bool gotoPositionAbsoluteY(int positionY);
    bool gotoPositionAbsoluteZ(int positionZ);
    bool gotoPositionAbsoluteRX(int angleX);
    bool gotoPositionAbsoluteRY(int angleY);
    bool gotoPositionAbsoluteRZ(int angleZ);

    // Single axis relative motion control
    bool gotoPositionRelativeX(int diffX);
    bool gotoPositionRelativeY(int diffY);
    bool gotoPositionRelativeZ(int diffZ);
    bool gotoPositionRelativeRX(int diffAngleX);
    bool gotoPositionRelativeRY(int diffAngleY);
    bool gotoPositionRelativeRZ(int diffAngleZ);

    bool findReference();

    bool getData(GripTaskData &gripTaskData);


private:
    void run() override;

private slots:
    void update();



private:
    Manipulator* manipulator_;
    Gripper* gripper_;

    GripTaskData gripTaskData_;

    QTimer* updateTimer_;
};

#endif // GRIPTASKEXECUTOR_H
