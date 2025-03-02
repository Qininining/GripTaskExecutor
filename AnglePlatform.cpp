#include "AnglePlatform.h"

AnglePlatform::AnglePlatform(const char* ID, NT_INDEX channelIndex)
    : motionSta_(0)
    , angle_(0)
    , angularVelocity_(0)
    , isOpen_(false)
    , bufferIndex_(0)
    , bufferSize_(5)
    , count(0)
    , timerFrequence_(50)
{
    positionBuffer_.resize(bufferSize_);

    ID_ = ID;
    motionSta_ = 0;
    channelIndex_ = channelIndex;
    updateTimer_ = new QTimer();
    updateTimer_->setInterval(timerFrequence_);//Motion信息（位置、电压、状态）更新频率为50Hz，等于力传感器采样频率。

    QObject::connect(updateTimer_, &QTimer::timeout, this, &AnglePlatform::update);
}

AnglePlatform::~AnglePlatform()
{
    if (isOpen_)
        disconnect();
    delete updateTimer_;
}

void AnglePlatform::update()
{
    if (!isOpen_) return;

    unsigned int temple_angle = 0;
    int temple_revolution = 0;
    error_ = NT_GetAngle_S(ntHandle_, channelIndex_, &temple_angle, &temple_revolution);
    // qDebug() << "AnglePlatform::angle_:" << angle_;
    if (temple_angle > 180000000)
    {
        angle_ = int(temple_angle) - 360000000;
    }
    else
    {
        angle_ = int(temple_angle);
    }
    error_ &=NT_GetStatus_S(ntHandle_, channelIndex_, &motionSta_);


    if (error_ != NT_OK) {
        qDebug() << "MotionPlatform::update error_: " << error_ << "channel:" << channelIndex_;
        // disConnect();
        return;
    }





    // 更新当前位置到缓冲区
    positionBuffer_[bufferIndex_] = angle_;
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
        angularVelocity_ = int(totalDisplacement / (bufferSize_ * 1.00 / timerFrequence_)); // 20ms转换为秒

        // qDebug() << "NT velocity_: " << velocity_ << "\t\tchannel:" << channelIndex_;
    }
}

bool AnglePlatform::connect()
{
    error_ = NT_OpenSystem(&ntHandle_, ID_, "sync");
    if (error_ != NT_OK)
    {
        qDebug() << "Open NT Angles system: error_: " << error_ << "ntHandle:" << ntHandle_ << "channel:" << channelIndex_;
    }
    else
    {
        qDebug() << "Open NT Angles system successfully!" << "ntHandle:" << ntHandle_ << "channel:" << channelIndex_;
        updateTimer_->start();
        isOpen_ = true; // 设置 isOpen_ 标志位为 true
    }
    NT_LimitEnable_S(ntHandle_, channelIndex_, NT_LIMIT_DISABLED);
    NT_SetAccumulateRelativePositions_S(ntHandle_, channelIndex_, 0);// 不累积相对位置
    return error_;
}

bool AnglePlatform::disconnect()
{
    if (isOpen_)
    {
        // updateTimer_->stop();
        error_ = NT_CloseSystem(ntHandle_);
        if (error_)
        {
            // qDebug() << "Close NT system: error_: " << error_ << "ntHandle_:" << ntHandle_;
            isOpen_ = false; // 设置 isOpen_ 标志位为 false
        }
        else
        {
            // qDebug() << "Close NT system successfully!" << "ntHandle_:" << ntHandle_;
            // motionSta_ = 0;
            isOpen_ = false; // 设置 isOpen_ 标志位为 false
        }
    }
    else
    {
        //qDebug() << "Close NT system: error_: " << "Platform not Connected";
    }
    return error_;
}

bool AnglePlatform::gotoAngleAbsolute(signed int angle)
{
    /* CHR：该函数，正方向正常，负方向不能用 2025/03/01----------------------------------------------*/
    // 不能输入负数，接下来通过算法修改
    // angle (unsigned 32bit)，输入 – 绝对运动角度，单位 u°。有效范围：0~359,999,999
    // NT_GotoAngleAbsolute_S(ntHandle_,0,    90000000,         0);
    // 系统索引:ntHandle_ 通道索引:channelIndex 绝对转动角度:单位 u° 绝对运动圈数:0
    // 绝对运动圈数 0或-1; 默认该平台不存在转动超过1圈的情况，由于限位
    if(angle >= 0)
    {          error_ = NT_GotoAngleAbsolute_S(ntHandle_, channelIndex_, angle, 0);
        if(error_ != NT_OK)
            qDebug() << "AnglePlatform::GotoAngleAbsolute error_:" << error_;
        return error_;
    }
    else
    {
        unsigned int temple_angle = angle - angle_;
        // error_ = NT_GotoAngleAbsolute_S(ntHandle_, channelIndex_, temple_angle, 0);
        error_ = NT_GotoAngleRelative_S(ntHandle_, channelIndex_, temple_angle, 0);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::GotoAngleAbsolute error_:" << error_;
        return error_;
    }
}

bool AnglePlatform::gotoAngleRelative(signed int angle)
{
    if (!isOpen_) return false;

    error_ = NT_GotoAngleRelative_S(ntHandle_, channelIndex_, angle, 0);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::GotoAngleRelative error_:" << error_;
    return error_;
}

bool AnglePlatform::getAngle(signed int &angle)
{
    if (!isOpen_) return false;

    angle = angle_;
    return true;
}

bool AnglePlatform::setAngularVelocity(signed int angularVelocity, long long target)
{
    if (!isOpen_) return false;

    int dir = 0;
    unsigned int speed = 0;
    if (angularVelocity > 0)
    {
        dir = 1;
        speed = angularVelocity;
    }
    else if (angularVelocity < 0)
    {
        dir = -1;
        speed = -1 * angularVelocity;
    }
    else
    {
        stop();
        return error_; // return代表函数结束，
    }
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_ENABLED, speed);

    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::setAngularVelocity error_(1):" << error_;
    error_ = NT_GotoAngleRelative_S(ntHandle_, channelIndex_, target * dir, 0);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::setAngularVelocity error_(2):" << error_; // 按照这个速度转动到指定位置
    return error_; // return代表函数结束，
}

bool AnglePlatform::getAngularVelocity(signed int &angularVelocity)
{
    if (!isOpen_) return false;

    angularVelocity = angularVelocity_;
    return true;
}

bool AnglePlatform::setVelocityMode(unsigned int velocity)
{
    if (!isOpen_) return false;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_ENABLED, velocity);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocityMode error_:" << error_;
    return error_;
}


bool AnglePlatform::closeVelocityMode()
{
    if (!isOpen_) return false;

    error_ = NT_Stop_S(ntHandle_, channelIndex_);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::closeVelocityMode error_(1):" << error_;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_DISABLED, 0);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::closeVelocityMode error_(2):" << error_;
    NT_SetClosedLoopHoldEnabled_S(ntHandle_, channelIndex_, NT_CLOSELOOP_ENABLED);
    return error_;
}

//● target (unsigned 32bit)，输入 – 绝对目标位置。有效输入范围为 0...4,095，0 对应
//0V，4,095 对于 100V。
//● scanSpeed (unsigned 32bit), 输入 - 扫描速度。有效输入范围是 1 ... 4,095,000。
//将 0-100V 等分成 4096 份，扫描速度表示每秒扫描电压变化的份数。比如，值为 1
//时，在从 0 扫描到 4,095 需要 4,095 秒，而全速扫描（值为 4,095,000）只需一毫秒。
bool AnglePlatform::ScanMoveAbsolute(unsigned int target, unsigned int time)
{
    if (!isOpen_) return false;

    error_ = NT_ScanMoveAbsolute_S(ntHandle_, channelIndex_, target, 4095000 / time);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::ScanMoveAbsolute error_:" << error_;
    return error_;
}

bool AnglePlatform::ScanMoveRelative(signed int target, unsigned int time)
{
    if (!isOpen_) return false;

    error_ = NT_ScanMoveRelative_S(ntHandle_, channelIndex_, target, 4095000 / time);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::ScanMoveRelative error_:" << error_;
    return error_;
}

bool AnglePlatform::getVoltage(unsigned int &voltage)
{
    if (!isOpen_) return false;

    error_ = NT_GetVoltageLevel_S(ntHandle_, channelIndex_, &voltage);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::GetVoltage error__:" << error_;
    return error_;
}

bool AnglePlatform::setRelativeReference(signed int Angle)
{
    if (!isOpen_) return false;

    if (Angle < 0)
        Angle = Angle + 360000000;
    error_ = NT_SetPosition_S(ntHandle_, channelIndex_, Angle);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::setRelativeReference error_:" << error_;
    return error_;
}

bool AnglePlatform::setRelativeReference()
{
    if (!isOpen_) return false;

    error_ = NT_SetPosition_S(ntHandle_, channelIndex_, 0);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::setRelativeReference error_:" << error_;
    return error_;
}

bool AnglePlatform::getChannelState(unsigned int &status)
{
    if (!isOpen_) return false;

    error_ = NT_GetStatus_S(ntHandle_, channelIndex_, &status);
    return error_;
}

bool AnglePlatform::stop()
{
    if (!isOpen_) return false;

    angularVelocity_ = 0;
    error_ = NT_Stop_S(ntHandle_, channelIndex_); // 此命令也会关闭闭环命令的位置保持功能
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::closeVelocityMode error_(1):" << error_;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_DISABLED, 0);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::closeVelocityMode error_(2):" << error_;
    NT_SetClosedLoopHoldEnabled_S(ntHandle_, channelIndex_, NT_CLOSELOOP_ENABLED); // 打开闭环保持功能
    return error_;
}

bool AnglePlatform::findReference()
{
    if (!isOpen_) return false;

    // 找到机械原点
    error_ = NT_FindReferenceMark_S(ntHandle_, channelIndex_, NT_FIND_FORWARD, NT_AUTO_ZERO_ENABLED);
    if (error_ != NT_OK)
        qDebug() << "AnglePlatform::findReference error__:" << error_;
    return error_;
}

bool AnglePlatform::getPlatformInfo()
{
    if (!isOpen_) return false;

    unsigned int numOfChannels;
    char info[200];
    unsigned int size = sizeof(info);
    unsigned int known;
    unsigned int speed;

    if (isOpen_)
    {
        error_ = NT_GetNumberOfChannels(ntHandle_, &numOfChannels);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(1):" << error_;
        else
            qDebug() << "Number of Channels:" << numOfChannels;

        error_ = NT_GetVersionInfo(ntHandle_, info, &size);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(2):" << error_;
        else
            qDebug() << "VersionInfo:" << info;

        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_, 0, &known);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(3):" << error_;
        else
            qDebug() << "CH1 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_, 1, &known);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(3):" << error_;
        else
            qDebug() << "CH2 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_, 2, &known);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(3):" << error_;
        else
            qDebug() << "CH3 PhysicalPositionKnown:" << known;

        NT_GetClosedLoopMoveSpeed_S(ntHandle_, 0, &speed);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(4):" << error_;
        else
            qDebug() << "CH1 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_, 1, &speed);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(4):" << error_;
        else
            qDebug() << "CH2 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_, 2, &speed);
        if (error_ != NT_OK)
            qDebug() << "AnglePlatform::getPlatformInfo() error_(4):" << error_;
        else
            qDebug() << "CH3 ClosedLoopMoveSpeed:" << speed;
    }
    else
    {
        qDebug() << "Platform not Connected";
    }
    return error_;
}

bool AnglePlatform::getsta(NT_STATUS &sta)
{
    if (!isOpen_) return false;

    sta = motionSta_;
    return true;
}

bool AnglePlatform::findSystem()
{
    char outBuffer[4096];
    unsigned int bufferSize = sizeof(outBuffer);
    NT_STATUS result = NT_FindSystems("", outBuffer, &bufferSize);
    if (result == NT_OK)
    {
        // outBuffer holds the locator strings, separated by '\n'
        // bufferSize holds the number of bytes written to outBuffer
        qDebug() << "findSystem:\r\n" << outBuffer;
    }
    else
    {
        qDebug() << "find 0 System";
    }
    return true;
}
