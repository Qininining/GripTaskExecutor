#include "MotionPlatform.h"
#include <QDebug>

MotionPlatform::MotionPlatform(const char* ID, NT_INDEX channelIndex)
    : motionSta_(0)
    , position_(0)
    , velocity_(0)
    , voltage_(0)
    , bufferIndex_(0)
    , bufferSize_(5)
    , count(0)
    , timerFrequence_(50)
{

    positionBuffer_.resize(bufferSize_);

    deviceID_ = ID;
    isOpen_ = 0;
    channelIndex_ = channelIndex;

    updateTimer_ = new QTimer();
    updateTimer_->setInterval(1000 / timerFrequence_);//Motion信息（位置、电压、状态）更新频率为200Hz，等于力传感器采样频率。

    QObject::connect(updateTimer_, &QTimer::timeout, this, &MotionPlatform::update);
}

MotionPlatform::~MotionPlatform()
{
    if(isOpen_)
        disConnect();
    delete updateTimer_;
}

void MotionPlatform::update()
{
    if (!isOpen_) return;


    error_ = NT_GetPosition_S(ntHandle_, channelIndex_, &position_);
    error_ &= NT_GetStatus_S(ntHandle_, channelIndex_, &motionSta_);


    if (error_ != NT_OK) {
        qDebug() << "MotionPlatform::update error_: " << error_ << "channel:" << channelIndex_;
        // disConnect();
        return;
    }



    // 更新当前位置到缓冲区
    positionBuffer_[bufferIndex_] = position_;
    bufferIndex_ = (bufferIndex_ + 1) % bufferSize_; // 循环更新索引

    if (count < bufferSize_) {
        count++;
    }

    // 计算平均速度，考虑方向
    if (count >= bufferSize_) { // 确保只有在连接状态下才进行计算
        double totalDisplacement = 0;

        // 从最新的位置向前遍历，以保持正确的顺序
        for (int i = 0; i < bufferSize_ - 1; ++i) {
            int currentIndex = (bufferIndex_ + bufferSize_ - 1 - i) % bufferSize_;
            int nextIndex = (currentIndex + bufferSize_ - 1) % bufferSize_;
            totalDisplacement += (positionBuffer_[currentIndex] - positionBuffer_[nextIndex]);
        }
        // 平均速度 = 总位移 / 时间间隔总和
        // 注意这里的时间间隔是基于updateTimer_的周期，即20ms
        velocity_ = int(totalDisplacement / (bufferSize_ * 1.00 / timerFrequence_)); // 20ms转换为秒

        // qDebug() << "NT velocity_: " << velocity_ << "\t\tchannel:" << channelIndex_;
    }
}

bool MotionPlatform::connect()
{
    error_ = NT_OpenSystem(&ntHandle_, deviceID_, "sync");
    if (error_)
    {
        qDebug() << "Open NT Motion system: error_: " << error_ << "ntHandle_:" << ntHandle_;
    }
    else{
        isOpen_ = 1;
        qDebug() << "Open NT Motion system successfully!" << "ntHandle:" << ntHandle_ << "channel:" <<channelIndex_;

#ifdef TIMERMODE_MOTION
        updateTimer_->start();
#endif
    }
    // NT_SetClosedLoopMaxFrequency_S(ntHandle_, channelIndex_, 8000);
    NT_LimitEnable_S(ntHandle_, channelIndex_, NT_LIMIT_DISABLED);//限位会影响中速运动
    NT_SetAccumulateRelativePositions_S(ntHandle_, channelIndex_, 0);// 不累积相对位置
    return error_;
}

bool MotionPlatform::disConnect()
{
    if(isOpen_)
    {
        error_ = NT_CloseSystem(ntHandle_);

#ifdef TIMERMODE_MOTION
        // updateTimer_->stop();
#endif
        if (error_)
        {
           // qDebug() << "Close NT system: error_: " << error_ << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
        else{
           qDebug() << "Close NT system  successfully!" << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
    }
    // else
        // qDebug() << "Close NT system: error_: " << "Platform not Connected";
    return error_;
}

bool MotionPlatform::gotoPositionAbsolute( signed int position)
{
    if (!isOpen_) return false;
    error_ = NT_GotoPositionAbsolute_S(ntHandle_, channelIndex_, position);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::GotoPositionAbsolute error_:" << error_;
    return error_;
}

bool MotionPlatform::gotoPositionRelative(signed int diff)
{
    if (!isOpen_) return false;
    error_ = NT_GotoPositionRelative_S(ntHandle_, channelIndex_, diff);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::GotoPositionRelative error_:" << error_;
    return error_;
}

bool MotionPlatform::getPosition(int &position)
{

#ifdef TIMERMODE_MOTION
    position = position_;
    return true;
#else
    error_ = NT_GetPosition_S(ntHandle_, channelIndex_, &position);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GetPosition error_:" << error_;
    return error_;
#endif
}

bool MotionPlatform::stepMove(signed int steps, unsigned int amplitude, unsigned int frequency)
{
    if (!isOpen_) return false;
    error_ = NT_StepMove_S(ntHandle_, channelIndex_, steps, amplitude, frequency);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::StepMove error_:" << error_;
    return error_;
}

bool MotionPlatform::continueMove(bool dir, unsigned int amplitude, unsigned int frequency)
{
    if (!isOpen_) return false;
    signed int steps = 30000;
    steps = steps * dir;
    error_ = NT_StepMove_S(ntHandle_, channelIndex_, steps, amplitude, frequency);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::StepMove error_:" << error_;
    return error_;
}


//● target (unsigned 32bit)，输入 – 绝对目标位置。有效输入范围为 0...4,095，0 对应
//0V，4,095 对于 100V。
//● scanSpeed (unsigned 32bit), 输入 - 扫描速度。有效输入范围是 1 ... 4,095,000。
//将 0-100V 等分成 4096 份，扫描速度表示每秒扫描电压变化的份数。比如，值为 1
//时，在从 0 扫描到 4,095 需要 4,095 秒，而全速扫描（值为 4,095,000）只需一毫秒。
bool MotionPlatform::scanMoveAbsolute(unsigned int target, unsigned int time)
{
    if (!isOpen_) return false;
    error_ = NT_ScanMoveAbsolute_S(ntHandle_, channelIndex_, target, 4095000 / time);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::ScanMoveAbsolute error_:" << error_;
    return error_;
}

bool MotionPlatform::scanMoveRelative(signed int target, unsigned int time)
{
    if (!isOpen_) return false;
    error_ = NT_ScanMoveRelative_S(ntHandle_, channelIndex_, target, 4095000 / time);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::ScanMoveRelative error_:" << error_;
    return error_;
}


//● speed (unsigned 32bit)，输入 - 设置定位台匀速运动的速度值,线性定位台的单位是
//nm/s,旋转定位台的单位是 u°/s。有效值范围是 0...5,000,000。0 默认禁用速度控制。
bool MotionPlatform::setVelocity(signed int velocity, signed int target)
{
    if (!isOpen_) return false;
    int dir = 0;
    unsigned int speed = 0;
    if (velocity > 0)
    {
        dir = 1;
        speed = velocity;
    }
    else if (velocity < 0)
    {
        dir = -1;
        speed = -1 * velocity;
    }
    else
    {
        stop();
        return 1;
    }

    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_ENABLED, speed);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocity error_(1):" << error_;
    error_ = NT_GotoPositionRelative_S(ntHandle_, channelIndex_, target * dir);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocity error_(2):" << error_;
    return error_;
}


bool MotionPlatform::setVelocityMode(unsigned int velocity)
{
    if (!isOpen_) return false;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_ENABLED, velocity);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocityMode error_:" << error_;
    return error_;
}

bool MotionPlatform::closeVelocityMode()
{
    if (!isOpen_) return false;
    error_ = NT_Stop_S(ntHandle_, channelIndex_);
    error_ &= NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_DISABLED, 0);
    error_ &= NT_SetClosedLoopHoldEnabled_S(ntHandle_,channelIndex_,NT_CLOSELOOP_ENABLED);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::closeVelocityMode error_:" << error_;
    return error_;
}

bool MotionPlatform::getVelocity(signed int &velocity)
{
    velocity = velocity_;
    return true;
}


bool MotionPlatform::setPosition()
{
    error_ = NT_SetPosition_S(ntHandle_, channelIndex_, 0);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setPosition error_:" << error_;
    return error_;
}

bool MotionPlatform::stop()
{
    if (!isOpen_) return false;
    error_ = NT_Stop_S(ntHandle_, channelIndex_);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::closeVelocityMode error_(1):" << error_;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_DISABLED, 0);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::closeVelocityMode error_(2):" << error_;
    NT_SetClosedLoopHoldEnabled_S(ntHandle_,channelIndex_,NT_CLOSELOOP_ENABLED);
    return error_;
}

bool MotionPlatform::getVoltage(unsigned int &voltage)
{
// #ifdef TIMERMODE_MOTION
//     voltage = voltage_;
//     return true;
// #else
    error_ = NT_GetVoltageLevel_S(ntHandle_,channelIndex_,&voltage);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GetVoltage error_:" << error_;
    return error_;
// #endif
}

bool MotionPlatform::findReference()
{
    error_ = NT_FindReferenceMark_S(ntHandle_, channelIndex_, NT_FIND_FORWARD, NT_AUTO_ZERO_ENABLED);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::findReference error_:" << error_;
    return error_;
}


bool MotionPlatform::getMotionInfo()
{
    unsigned int numOfChannels;
    char info[200];
    unsigned int size = sizeof(info);
    unsigned int known;
    unsigned int speed;

    if(isOpen_)
    {
        error_ = NT_GetNumberOfChannels(ntHandle_, &numOfChannels);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(1):" << error_;
        else
            qDebug() << "Number of Channels:" << numOfChannels;

        error_ = NT_GetVersionInfo(ntHandle_, info, &size);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(2):" << error_;
        else
            qDebug() << "VersionInfo:" << info;

        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,0,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH1 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,1,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH2 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,2,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH3 PhysicalPositionKnown:" << known;

        NT_GetClosedLoopMoveSpeed_S(ntHandle_,0,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH1 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_,1,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH2 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_,2,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH3 ClosedLoopMoveSpeed:" << speed;
    }
    else
    {
        qDebug() << "Platform not Connected";
    }
    return error_;
}
//    // channel status codes
//    #define NT_STOPPED_STATUS                           0
//    #define NT_STEPPING_STATUS                          1
//    #define NT_SCANNING_STATUS                          2
//    #define NT_HOLDING_STATUS                           3
//    #define NT_TARGET_STATUS                            4
//    #define NT_SENSOR_CLOSED_STATUS                     5
//    #define NT_NO_HOLDING_STATUS                        6
//    #define NT_REFERENCING_STATUS                       7
//    #define NT_CALIBRATING_STATUS                       8
//    #define NT_PHY_LIMIT_STATUS                         10
//    #define NT_SOFT_LIMIT_STATUS                        11
//    #define NT_SHORT_CIRCUIT_STATUS                     13
bool MotionPlatform::getsta( NT_STATUS &sta)
{
#ifdef TIMERMODE_MOTION
    sta = motionSta_;
    return true;
#else
    error_ = NT_GetStatus_S(ntHandle_,channelIndex_,&sta);
    return error_;
#endif
}


bool MotionPlatform::findSystem()
{
    char outBuffer[4096];
    unsigned int bufferSize = sizeof(outBuffer);
    NT_STATUS result = NT_FindSystems("", outBuffer, &bufferSize);
    if(result == NT_OK){
        // outBuffer holds the locator strings, separated by '\n'
        // bufferSize holds the number of bytes written to outBuffer
        qDebug() << "findSystem:\r\n" << outBuffer;
    }
    else{
        qDebug() << "find 0 System";
    }
    return true;
}
