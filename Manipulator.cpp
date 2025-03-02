#include "Manipulator.h"

Manipulator::Manipulator(const char* MotionID, const char* AngleID)
    : p_X_(new MotionPlatform(MotionID, CH1))
    , p_Y_(new MotionPlatform(MotionID, CH2))
    , p_Z_(new MotionPlatform(MotionID, CH3))
    , p_RX_(new AnglePlatform(AngleID, CH1))
    , p_RY_(new AnglePlatform(AngleID, CH2))
    , p_RZ_(new AnglePlatform(AngleID, CH3))
    , isOpen_(false)
    , motionThread(new QThread())
    , angleThread(new QThread())
{
    updateTimer_ = new QTimer();
    updateTimer_->setInterval(20);
    connect(updateTimer_, &QTimer::timeout, this, &Manipulator::update);

    p_X_->findSystem();

    p_X_->moveToThread(motionThread);
    p_Y_->moveToThread(motionThread);
    p_Z_->moveToThread(motionThread);
    p_RX_->moveToThread(angleThread);
    p_RY_->moveToThread(angleThread);
    p_RZ_->moveToThread(angleThread);

    motionThread->start();
    angleThread->start();
}

Manipulator::~Manipulator()
{

    motionThread->quit();
    angleThread->quit();
    motionThread->wait();
    angleThread->wait();

    shutdown();

    delete motionThread;
    delete angleThread;
    delete p_X_;
    delete p_Y_;
    delete p_Z_;
    delete p_RX_;
    delete p_RY_;
    delete p_RZ_;
    delete updateTimer_;
}

bool Manipulator::initialize()
{
    bool connectedX = p_X_->connect();
    bool connectedY = p_Y_->connect();
    bool connectedZ = p_Z_->connect();
    bool connectedRX = p_RX_->connect();
    bool connectedRY = p_RY_->connect();
    bool connectedRZ = p_RZ_->connect();

    if (!connectedX && !connectedY && !connectedZ && !connectedRX && !connectedRY && !connectedRZ) {
        updateTimer_->start(20); // Update every 20ms
        isOpen_ = true;
        qDebug() << "Manipulator initialized successfully!";
        return true;
    } else {
        isOpen_ = false;
        qDebug() << "Failed to initialize manipulator!";
        return false;
    }
}

bool Manipulator::shutdown()
{
    updateTimer_->stop();
    p_X_->disconnect();
    p_Y_->disconnect();
    p_Z_->disconnect();
    p_RX_->disconnect();
    p_RY_->disconnect();
    p_RZ_->disconnect();
    isOpen_ = false;
    return true;
}

void Manipulator::update()
{
    if (!isOpen_) return;

    int posX, posY, posZ, angX, angY, angZ;
    signed int velX, velY, velZ, angVelX, angVelY, angVelZ;
    NT_STATUS staX, staY, staZ, angStatX, angStatY, angStatZ;

    // Get position
    p_X_->getPosition(posX);
    p_Y_->getPosition(posY);
    p_Z_->getPosition(posZ);
    p_X_->getVelocity(velX);
    p_Y_->getVelocity(velY);
    p_Z_->getVelocity(velZ);
    p_X_->getsta(staX);
    p_Y_->getsta(staY);
    p_Z_->getsta(staZ);

    // Get angle
    p_RX_->getAngle(angX);
    p_RY_->getAngle(angY);
    p_RZ_->getAngle(angZ);
    p_RX_->getAngularVelocity(angVelX);
    p_RY_->getAngularVelocity(angVelY);
    p_RZ_->getAngularVelocity(angVelZ);
    p_RX_->getsta(angStatX);
    p_RY_->getsta(angStatY);
    p_RZ_->getsta(angStatZ);

    // Update manipulator data
    manipulatorData_.positionX = posX;
    manipulatorData_.positionY = posY;
    manipulatorData_.positionZ = posZ;
    manipulatorData_.velocityX = velX;
    manipulatorData_.velocityY = velY;
    manipulatorData_.velocityZ = velZ;
    manipulatorData_.statusX = staX;
    manipulatorData_.statusY = staY;
    manipulatorData_.statusZ = staZ;

    manipulatorData_.angleX = angX;
    manipulatorData_.angleY = angY;
    manipulatorData_.angleZ = angZ;
    manipulatorData_.angularVelocityX = angVelX;
    manipulatorData_.angularVelocityY = angVelY;
    manipulatorData_.angularVelocityZ = angVelZ;
    manipulatorData_.angularStatusX = angStatX;
    manipulatorData_.angularStatusY = angStatY;
    manipulatorData_.angularStatusZ = angStatZ;

    Eigen::Vector3d jointPosition(posX, posY, posZ);
    Eigen::Vector3d jointVelocity(velX, velY, velZ);
    Eigen::Vector3d jointStatus(staX, staY, staZ);
    Eigen::Vector3d jointAngle(angX, angY, angZ);
    Eigen::Vector3d jointAngularVelocity(angVelX, angVelY, angVelZ);
    Eigen::Vector3d jointAngularStatus(angStatX, angStatY, angStatZ);

    manipulatorData_.position = jointPosition;
    manipulatorData_.velocity = jointVelocity;
    manipulatorData_.status = jointStatus;
    manipulatorData_.angle = jointAngle;
    manipulatorData_.angularVelocity = jointAngularVelocity;
    manipulatorData_.angularStatus = jointAngularStatus;
}

bool Manipulator::getManipulatorData(ManipulatorData &manipulatorData) const
{
    manipulatorData = manipulatorData_;
    return true;
}

// Motion control functions
bool Manipulator::gotoPositionAbsolute(int positionX, int positionY, int positionZ, int angleX, int angleY, int angleZ) {
    if (!isOpen_) return false;
    bool resultX = p_X_->gotoPositionAbsolute(positionX);
    bool resultY = p_Y_->gotoPositionAbsolute(positionY);
    bool resultZ = p_Z_->gotoPositionAbsolute(positionZ);
    bool resultRX = p_RX_->gotoAngleAbsolute(angleX);
    bool resultRY = p_RY_->gotoAngleAbsolute(angleY);
    bool resultRZ = p_RZ_->gotoAngleAbsolute(angleZ);
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

bool Manipulator::gotoPositionRelative(int diffX, int diffY, int diffZ, int diffAngleX, int diffAngleY, int diffAngleZ) {
    if (!isOpen_) return false;
    bool resultX = p_X_->gotoPositionRelative(diffX);
    bool resultY = p_Y_->gotoPositionRelative(diffY);
    bool resultZ = p_Z_->gotoPositionRelative(diffZ);
    bool resultRX = p_RX_->gotoAngleRelative(diffAngleX);
    bool resultRY = p_RY_->gotoAngleRelative(diffAngleY);
    bool resultRZ = p_RZ_->gotoAngleRelative(diffAngleZ);
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

bool Manipulator::setVelocity(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ) {
    if (!isOpen_) return false;
    bool resultX = p_X_->setVelocity(velocityX);
    bool resultY = p_Y_->setVelocity(velocityY);
    bool resultZ = p_Z_->setVelocity(velocityZ);
    bool resultRX = p_RX_->setAngularVelocity(angularVelocityX);
    bool resultRY = p_RY_->setAngularVelocity(angularVelocityY);
    bool resultRZ = p_RZ_->setAngularVelocity(angularVelocityZ);
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

bool Manipulator::stopMotion() {
    if (!isOpen_) return false;
    bool resultX = p_X_->stop();
    bool resultY = p_Y_->stop();
    bool resultZ = p_Z_->stop();
    bool resultRX = p_RX_->stop();
    bool resultRY = p_RY_->stop();
    bool resultRZ = p_RZ_->stop();
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

bool Manipulator::moveJointPosition(const Eigen::VectorXd& jointPosition) {
    if (!isOpen_) return false;
    int positionX = static_cast<int>(jointPosition(0));
    int positionY = static_cast<int>(jointPosition(1));
    int positionZ = static_cast<int>(jointPosition(2));
    int angleX = static_cast<int>(jointPosition(3));
    int angleY = static_cast<int>(jointPosition(4));
    int angleZ = static_cast<int>(jointPosition(5));
    return gotoPositionAbsolute(positionX, positionY, positionZ, angleX, angleY, angleZ);
}

// Speed control functions
bool Manipulator::setVelocityMode(int velocityX, int velocityY, int velocityZ, int angularVelocityX, int angularVelocityY, int angularVelocityZ) {
    if (!isOpen_) return false;
    bool resultX = p_X_->setVelocityMode(velocityX);
    bool resultY = p_Y_->setVelocityMode(velocityY);
    bool resultZ = p_Z_->setVelocityMode(velocityZ);
    bool resultRX = p_RX_->setVelocityMode(angularVelocityX);
    bool resultRY = p_RY_->setVelocityMode(angularVelocityY);
    bool resultRZ = p_RZ_->setVelocityMode(angularVelocityZ);
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

bool Manipulator::closeVelocityMode() {
    if (!isOpen_) return false;
    bool resultX = p_X_->closeVelocityMode();
    bool resultY = p_Y_->closeVelocityMode();
    bool resultZ = p_Z_->closeVelocityMode();
    bool resultRX = p_RX_->closeVelocityMode();
    bool resultRY = p_RY_->closeVelocityMode();
    bool resultRZ = p_RZ_->closeVelocityMode();
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}

// Single axis speed control functions
bool Manipulator::setVelocityModeX(int velocityX) {
    if (!isOpen_) return false;
    return p_X_->setVelocityMode(velocityX);
}

bool Manipulator::setVelocityModeY(int velocityY) {
    if (!isOpen_) return false;
    return p_Y_->setVelocityMode(velocityY);
}

bool Manipulator::setVelocityModeZ(int velocityZ) {
    if (!isOpen_) return false;
    return p_Z_->setVelocityMode(velocityZ);
}

bool Manipulator::setVelocityModeRX(int angularVelocityX) {
    if (!isOpen_) return false;
    return p_RX_->setVelocityMode(angularVelocityX);
}

bool Manipulator::setVelocityModeRY(int angularVelocityY) {
    if (!isOpen_) return false;
    return p_RY_->setVelocityMode(angularVelocityY);
}

bool Manipulator::setVelocityModeRZ(int angularVelocityZ) {
    if (!isOpen_) return false;
    return p_RZ_->setVelocityMode(angularVelocityZ);
}

bool Manipulator::closeVelocityModeX() {
    if (!isOpen_) return false;
    return p_X_->closeVelocityMode();
}

bool Manipulator::closeVelocityModeY() {
    if (!isOpen_) return false;
    return p_Y_->closeVelocityMode();
}

bool Manipulator::closeVelocityModeZ() {
    if (!isOpen_) return false;
    return p_Z_->closeVelocityMode();
}

bool Manipulator::closeVelocityModeRX() {
    if (!isOpen_) return false;
    return p_RX_->closeVelocityMode();
}

bool Manipulator::closeVelocityModeRY() {
    if (!isOpen_) return false;
    return p_RY_->closeVelocityMode();
}

bool Manipulator::closeVelocityModeRZ() {
    if (!isOpen_) return false;
    return p_RZ_->closeVelocityMode();
}

// Single axis motion control functions
bool Manipulator::gotoPositionAbsoluteX(int positionX) {
    if (!isOpen_) return false;
    return p_X_->gotoPositionAbsolute(positionX);
}

bool Manipulator::gotoPositionAbsoluteY(int positionY) {
    if (!isOpen_) return false;
    return p_Y_->gotoPositionAbsolute(positionY);
}

bool Manipulator::gotoPositionAbsoluteZ(int positionZ) {
    if (!isOpen_) return false;
    return p_Z_->gotoPositionAbsolute(positionZ);
}

bool Manipulator::gotoPositionAbsoluteRX(int angleX) {
    if (!isOpen_) return false;
    return p_RX_->gotoAngleAbsolute(angleX);
}

bool Manipulator::gotoPositionAbsoluteRY(int angleY) {
    if (!isOpen_) return false;
    return p_RY_->gotoAngleAbsolute(angleY);
}

bool Manipulator::gotoPositionAbsoluteRZ(int angleZ) {
    if (!isOpen_) return false;
    return p_RZ_->gotoAngleAbsolute(angleZ);
}

bool Manipulator::gotoPositionRelativeX(int diffX) {
    if (!isOpen_) return false;
    return p_X_->gotoPositionRelative(diffX);
}

bool Manipulator::gotoPositionRelativeY(int diffY) {
    if (!isOpen_) return false;
    return p_Y_->gotoPositionRelative(diffY);
}

bool Manipulator::gotoPositionRelativeZ(int diffZ) {
    if (!isOpen_) return false;
    return p_Z_->gotoPositionRelative(diffZ);
}

bool Manipulator::gotoPositionRelativeRX(int diffAngleX) {
    if (!isOpen_) return false;
    return p_RX_->gotoAngleRelative(diffAngleX);
}

bool Manipulator::gotoPositionRelativeRY(int diffAngleY) {
    if (!isOpen_) return false;
    return p_RY_->gotoAngleRelative(diffAngleY);
}

bool Manipulator::gotoPositionRelativeRZ(int diffAngleZ) {
    if (!isOpen_) return false;
    return p_RZ_->gotoAngleRelative(diffAngleZ);
}

bool Manipulator::findReference()
{
    if (!isOpen_) return false;
    bool resultX = p_X_->findReference();
    bool resultY = p_Y_->findReference();
    bool resultZ = p_Z_->findReference();
    bool resultRX = p_RX_->findReference();
    bool resultRY = p_RY_->findReference();
    bool resultRZ = p_RZ_->findReference();
    return resultX && resultY && resultZ && resultRX && resultRY && resultRZ;
}
