#ifndef GRIPTASKEXECUTOR_H
#define GRIPTASKEXECUTOR_H

#include "Manipulator.h"
#include "Gripper.h"

class GripTaskExecutor : public QThread
{
    Q_OBJECT
public:
    GripTaskExecutor(const char* MotionID, const char* AngleID, const char* deviceID, const QString &portName);
    ~GripTaskExecutor();
    bool initialize();
    bool shutdown();




public:
    Manipulator* manipulator_;
    Gripper* gripper_;
};

#endif // GRIPTASKEXECUTOR_H
