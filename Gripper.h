#ifndef GRIPPER_H
#define GRIPPER_H

#include "MotionPlatform.h"
#include "ForceSensor.h"
#include <QThread>
#include <QTimer>

struct GripperData {
    int forceValue_X;
    int forceValue_Z;
    int position;
    int velocity;
    unsigned int sta;
    GripperData()
        : forceValue_X(0)
        , forceValue_Z(0)
        , position(0)
        , velocity(0)
        , sta(0) {}
};

class Gripper : public QThread {
    Q_OBJECT
public:
    Gripper(const char* deviceID, const QString &portName);
    ~Gripper();

    void open();
    void close();
    bool isOpen() const;
    bool isClosed() const;
    bool grip(int force = 50000); //默认夹持力为50000微牛
    bool release();
    bool initialize();
    bool shutdown();


    void run() override;
    void stop();
    void startThread();


    void getData(GripperData &data) const;

    enum TaskState {
        Free = 0,
        Grip,
        Release,
        Error
    };



private slots:
    void update();

private:
    bool openState_;
    MotionPlatform* motionPlatform_;
    bool running_;
    QTimer* updateTimer_;
    ForceSensor* forceSensor_;

    GripperData gripperData_;

    const int forceThreshold_;


    TaskState taskState_;

    int safeDistance_;

    int maxLimit_;
    int minLimit_;

    int gripForce_;



};

#endif // GRIPPER_H
