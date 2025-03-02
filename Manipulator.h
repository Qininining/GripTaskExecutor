#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "MotionPlatform.h"
#include "AnglePlatform.h"
#include <Eigen/Dense>
#include <QTimer>
#include <QObject>

struct ManipulatorData {
    double positionX; double positionY; double positionZ;
    Eigen::Vector3d position;
    double velocityX; double velocityY; double velocityZ;
    Eigen::Vector3d velocity;int statusX;int statusY;int statusZ;
    Eigen::Vector3d status;

    double angleX;double angleY;double angleZ;
    Eigen::Vector3d angle;
    double angularVelocityX;double angularVelocityY;double angularVelocityZ;
    Eigen::Vector3d angularVelocity;
    int angularStatusX;int angularStatusY;int angularStatusZ;
    Eigen::Vector3d angularStatus;

    // Constructor to initialize all values to zero
    ManipulatorData(Eigen::Vector3d pos = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d vel = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d stat = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d ang = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d angVel = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d angStat = Eigen::Vector3d::Zero())
        : positionX(0), positionY(0), positionZ(0), position(pos)
        , velocityX(0), velocityY(0), velocityZ(0), velocity(vel)
        , statusX(0), statusY(0), statusZ(0), status(stat)
        , angleX(0), angleY(0), angleZ(0), angle(ang)
        , angularVelocityX(0), angularVelocityY(0), angularVelocityZ(0), angularVelocity(angVel)
        , angularStatusX(0), angularStatusY(0), angularStatusZ(0), angularStatus(angStat) {}
};

class Manipulator : public QObject
{
    Q_OBJECT

public:
    Manipulator(const char* MotionID, const char* AngleID);
    ~Manipulator();

    void initialize();
    void shutdown();

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

    // get Data functions
    bool getManipulatorData(ManipulatorData &manipulatorData) const;

    bool findReference();

private slots:
    void update();

private:
    MotionPlatform *p_X_;
    MotionPlatform *p_Y_;
    MotionPlatform *p_Z_;
    AnglePlatform *p_RX_;
    AnglePlatform *p_RY_;
    AnglePlatform *p_RZ_;

    QTimer* updateTimer_;

    ManipulatorData manipulatorData_;

    bool isOpen_;

    QThread *motionThread;
    QThread *angleThread;
};

#endif // MANIPULATOR_H
