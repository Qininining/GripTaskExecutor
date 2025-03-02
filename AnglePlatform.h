#ifndef ANGLEPLATFORM_H
#define ANGLEPLATFORM_H

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QDebug>
#include "NTControl.h"

using namespace std;

/**
 * @file AnglePlatform.h
 * @brief 提供转动平台控制功能。
 *
 * 本文件定义了AnglePlatform类，该类封装了NTControl库接口，
 * 主要用于配置、打开和关闭连接到运动平台的串口，并执行各种运动控制命令，
 * 包括移动到绝对或相对位置、步进转动、连续转动、扫描转动等操作，以及获取电压和当前位置信息。
 *
 * @author 汪博闻 <2401599729@qq.com>
 * @version 1.0
 * @date 2024-01-06
 * @copyright Copyright (c) 2024 Dalian University of Technology. All rights reserved.
 */


// 定义通道索引常量
#define CH1 0 ///< 第一通道索引  RZ
#define CH2 1 ///< 第二通道索引  RY
#define CH3 2 ///< 第三通道索引  RX

/**
 * @class AnglePlatform
 * @brief 提供转动平台控制功能。
 */
class AnglePlatform : public QObject {
    Q_OBJECT

public:
    AnglePlatform(const char* ID, NT_INDEX channelIndex);
    virtual ~AnglePlatform();


public:
    bool connect();
    bool disconnect();
    bool gotoAngleAbsolute(signed int angle);
    bool gotoAngleRelative(signed int angle);
    bool getAngle(signed int &angle);
    bool setAngularVelocity(signed int angularVelocity, long long target = 100000000);
    bool getAngularVelocity(signed int &angularVelocity);
    bool setVelocityMode(unsigned int velocity);
    bool closeVelocityMode();
    bool ScanMoveAbsolute(unsigned int target, unsigned int time);
    bool ScanMoveRelative(signed int target, unsigned int time);
    bool getVoltage(unsigned int &voltage);
    bool setRelativeReference(signed int Angle);
    bool setRelativeReference();
    bool findReference();
    bool getChannelState(NT_STATUS &status);
    bool getsta(NT_STATUS &sta);
    bool stop();
    bool getPlatformInfo();
    bool findSystem();

private slots:
    void update();

private:
    NT_STATUS error_ = NT_OK;
    const char *ID_;
    NT_INDEX ntHandle_;
    NT_INDEX channelIndex_;

    unsigned int motionSta_;

    signed int angle_;
    signed int angularVelocity_;
    bool isOpen_;

    QVector<double> positionBuffer_; ///< 用于存储位置的循环缓冲区。
    int bufferIndex_; ///< 当前写入缓冲区的位置索引。
    const int bufferSize_; ///< 缓冲区大小，可以调整为适当值。
    int count;

    QTimer *updateTimer_;
    int timerFrequence_;
};

#endif // ANGLEPLATFORM_H
