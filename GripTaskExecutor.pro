QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    AnglePlatform.cpp \
    ForceSensor.cpp \
    GripTaskExecutor.cpp \
    Gripper.cpp \
    Manipulator.cpp \
    MotionPlatform.cpp \
    SerialCommon.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    AnglePlatform.h \
    ForceSensor.h \
    GripTaskExecutor.h \
    Gripper.h \
    Manipulator.h \
    MotionPlatform.h \
    SerialCommon.h \
    mainwindow.h

FORMS += \
    mainwindow.ui



INCLUDEPATH += \
    $$PWD/NanoDrive2.8.12/04_SDK/include \
    $$PWD/eigen-3.4.0 \


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32: LIBS += -L$$PWD/NanoDrive2.8.12/04_SDK/lib64/ -lNTControl

INCLUDEPATH += $$PWD/NanoDrive2.8.12/04_SDK/lib64
DEPENDPATH += $$PWD/NanoDrive2.8.12/04_SDK/lib64

win32: LIBS += -L$$PWD/modern_robotics_lib/ -lmodern_robotics

INCLUDEPATH += $$PWD/modern_robotics_lib
DEPENDPATH += $$PWD/modern_robotics_lib
