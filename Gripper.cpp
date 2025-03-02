#include "Gripper.h"
#include "qdebug.h"

Gripper::Gripper(const char* deviceID, const QString &portName)
    : openState_(false)
    , motionPlatform_(new MotionPlatform(deviceID, CH3))
    , running_(false)
    , forceSensor_(new ForceSensor(portName, 830e-2, 200e-2)) // 初始化 MotionPlatform 和 ForceSensor 对象
    , forceThreshold_(100000) // 最大力值阈值
    , taskState_(Free)
    , safeDistance_(1000000) // 安全距离
    , maxLimit_(14000000) // 最大限位
    , minLimit_(-1000000) // 最小限e
    , gripForce_(50000) // 默认夹持力为50000微牛
{
    motionPlatform_->findSystem();
    updateTimer_ = new QTimer();
    updateTimer_->setInterval(10);
    connect(updateTimer_, &QTimer::timeout, this, &Gripper::update);
    // updateTimer_->start(20); // 设置定时器间隔为20毫秒
}

Gripper::~Gripper() {
    delete motionPlatform_; // 释放 MotionPlatform 对象
    delete forceSensor_; // 释放 ForceSensor 对象
}

void Gripper::open() {
    openState_ = true;
}

void Gripper::close() {
    openState_ = false;
}

bool Gripper::isOpen() const {
    return openState_;
}

bool Gripper::isClosed() const {
    return !openState_;
}


bool Gripper::initialize() {
    bool result = motionPlatform_->connect();
    result &= forceSensor_->connect();
    if (!result) {
        openState_ = true;
        updateTimer_->start();
        this->start();
    }
    return !result;
}

bool Gripper::shutdown() {
    stop();
    openState_ = true;
    bool result = motionPlatform_->disConnect();
    result &= forceSensor_->disConnect();
    if (!result) {
        updateTimer_->stop();
    }
    return !result;
}


void Gripper::startThread()
{
    running_ = false;
    this->start();
}




bool Gripper::grip(int force) {
    if(isOpen() && taskState_ == Free)
    {
        gripForce_ = force;
        taskState_ = Grip;
        return false;
    }
    return false;
}

bool Gripper::release() {
    if(isOpen())
    {
        taskState_ = Release;
        return false;
    }
    return false;
}


void Gripper::run() {
    running_ = true;
    while (running_)
    {
        // qDebug() << "11111111";
        // QThread::msleep(1000); // 休眠100毫秒
        if(taskState_ == Free)
        {
            // grip(0);
        }
        else if(taskState_ == Grip)
        {
            if(gripperData_.forceValue_X >= -20000)
            {
                forceSensor_->setReferenceZeroCH2();
                motionPlatform_->setVelocity(-1000000, 25000000);
                while (gripperData_.forceValue_X >= -5000 && running_ && taskState_ == Grip) {
                    qDebug() << "11111111" << gripperData_.forceValue_X;
                    QThread::msleep(10); // 休眠100毫秒
                }
                motionPlatform_->stop();

                // motionPlatform_->gotoPositionRelative(1)
            }
            qDebug() << "夹爪夹持力不为空";
            taskState_ = Free;
        }
        else if(taskState_ == Release)
        {
            if(gripperData_.position <= maxLimit_)
            {
                motionPlatform_->setVelocity(5000000, safeDistance_);
                // while (gripperData_.position <= maxLimit_) {

                // }
                // motionPlatform_->stop();
            }
            // release(0)t
            taskState_ = Free;
        }
    }
}

void Gripper::stop() {
    running_ = false;
    motionPlatform_->stop();
    taskState_ = Free;
}

void Gripper::update() {

    // static bool forceReferceFlag = 0;
    // 更新 ForceSensor 信息
    forceSensor_->readFirstChannel_R(gripperData_.forceValue_Z);
    forceSensor_->readSecondChannel_R(gripperData_.forceValue_X);


    // if(forceReferceFlag==0 && gripperData_.forceValue_X != 0 && gripperData_.forceValue_Z != 0)
    // {
    //     forceSensor_->setReferenceZeroCH1(gripperData_.forceValue_Z);
    //     forceSensor_->setReferenceZeroCH2(gripperData_.forceValue_X);
    //     forceReferceFlag=1;
    // }

    // qDebug() << "2222222" << gripperData_.forceValue_Z;
    // 更新 MotionPlatform 信息
    motionPlatform_->getPosition(gripperData_.position);
    motionPlatform_->getVelocity(gripperData_.velocity);
    motionPlatform_->getsta(gripperData_.sta);
    qDebug() << "forceValue_X: " << gripperData_.forceValue_X
             << "forceValue_Z: " << gripperData_.forceValue_Z;
    //          << "position: " << gripperData_.position
    //          << "velocity: " << gripperData_.velocity
    //          << "sta: " << gripperData_.sta;
}

void Gripper::getData(GripperData &data) const {
    data = gripperData_;
}

