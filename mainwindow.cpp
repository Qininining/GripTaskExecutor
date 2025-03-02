#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , gripTaskExecutor_(new GripTaskExecutor("usb:id:0497537675", "usb:id:4997391945", "usb:id:7027461260", "COM10"))
    , updatetimer_(new QTimer())
    , speedModeEnabled(false)
{
    ui->setupUi(this);

    ui->lineEdit__X_A->setText("0");
    ui->lineEdit__X_R->setText("1000000");
    ui->lineEdit__X_V->setText("1000000");
    ui->lineEdit__RX_A->setText("0");
    ui->lineEdit__RX_R->setText("1000000");
    ui->lineEdit__RX_V->setText("1000000");

    updatetimer_->setInterval(20);
    connect(updatetimer_, SIGNAL(timeout()), this, SLOT(updateGUI()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::updateGUI()
{
    gripTaskExecutor_->getData(data_);

    ui->lineEdit_Pos_X->setText(QString::number(data_.manipulatorData.positionX, 'f', 0));
    ui->lineEdit_Pos_Y->setText(QString::number(data_.manipulatorData.positionY, 'f', 0));
    ui->lineEdit_Pos_Z->setText(QString::number(data_.manipulatorData.positionZ, 'f', 0));

    ui->lineEdit_Vel_X->setText(QString::number(data_.manipulatorData.velocityX, 'f', 0));
    ui->lineEdit_Vel_Y->setText(QString::number(data_.manipulatorData.velocityY, 'f', 0));
    ui->lineEdit_Vel_Z->setText(QString::number(data_.manipulatorData.velocityZ, 'f', 0));

    ui->lineEdit_Sta_X->setText(QString::number(data_.manipulatorData.statusX));
    ui->lineEdit_Sta_Y->setText(QString::number(data_.manipulatorData.statusY));
    ui->lineEdit_Sta_Z->setText(QString::number(data_.manipulatorData.statusZ));

    ui->lineEdit_Pos_X_2->setText(QString::number(data_.manipulatorData.angleX, 'f', 0));
    ui->lineEdit_Pos_Y_2->setText(QString::number(data_.manipulatorData.angleY, 'f', 0));
    ui->lineEdit_Pos_Z_2->setText(QString::number(data_.manipulatorData.angleZ, 'f', 0));

    ui->lineEdit_Vel_X_2->setText(QString::number(data_.manipulatorData.angularVelocityX, 'f', 0));
    ui->lineEdit_Vel_Y_2->setText(QString::number(data_.manipulatorData.angularVelocityY, 'f', 0));
    ui->lineEdit_Vel_Z_2->setText(QString::number(data_.manipulatorData.angularVelocityZ, 'f', 0));

    ui->lineEdit_Sta_X_2->setText(QString::number(data_.manipulatorData.angularStatusX));
    ui->lineEdit_Sta_Y_2->setText(QString::number(data_.manipulatorData.angularStatusY));
    ui->lineEdit_Sta_Z_2->setText(QString::number(data_.manipulatorData.angularStatusZ));

    ui->lineEdit_Pos_G->setText(QString::number(data_.gripperData.position, 'f', 0));
    ui->lineEdit_Vel_G->setText(QString::number(data_.gripperData.velocity, 'f', 0));
    ui->lineEdit_Sta_G->setText(QString::number(data_.gripperData.sta));

    ui->lineEdit_Force_X->setText(QString::number(data_.gripperData.forceValue_X, 'f', 0));
    ui->lineEdit_Force_Z->setText(QString::number(data_.gripperData.forceValue_Z, 'f', 0));
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat())
    {
        qDebug() << "Key released:" << event->key();
        QMainWindow::keyReleaseEvent(event); // 调用基类实现

        // 关闭速度模式
        gripTaskExecutor_->closeVelocityMode(); // 假设存在这样一个方法来禁用速度模式
        speedModeEnabled = false;
    }

}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat())
    {
        int distance = 10000000;
        int velocity = 150000;
        qDebug() << "Key pressed:" << event->key();
        QMainWindow::keyPressEvent(event); // Call base class implementation

        bool ctrlPressed = event->modifiers() & Qt::ControlModifier;

        gripTaskExecutor_->setVelocityMode(velocity, velocity, velocity, velocity, velocity, velocity);

        switch (event->key())
        {
        case Qt::Key_Shift:
            gripTaskExecutor_->stopMotion();
            break;

        case Qt::Key_W:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRY(-distance); // Rotate Y axis clockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeY(-distance); // Move Y axis left
            }
            break;
        case Qt::Key_S:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRY(distance); // Rotate Y axis counterclockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeY(distance); // Move Y axis right
            }
            break;
        case Qt::Key_A:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRX(distance); // Rotate Y axis clockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeX(distance); // Move Y axis forward
            }
            break;
        case Qt::Key_D:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRX(-distance); // Rotate Y axis counterclockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeX(-distance); // Move Y axis backward
            }
            break;
        case Qt::Key_Q:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRZ(distance); // Rotate Z axis clockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeZ(distance); // Move Z axis up
            }
            break;
        case Qt::Key_E:
            if (ctrlPressed) {
                gripTaskExecutor_->gotoPositionRelativeRZ(-distance); // Rotate Z axis counterclockwise
            } else {
                gripTaskExecutor_->gotoPositionRelativeZ(-distance); // Move Z axis down
            }
            break;
        case Qt::Key_G:
            gripTaskExecutor_->grip(); // Grip
            break;
        case Qt::Key_R:
            gripTaskExecutor_->release(); // Release
            break;
        default:
            break;
        }
    }
}





void MainWindow::on_Connect_clicked(bool checked)
{
    if (checked) {
        gripTaskExecutor_->initialize();
        updatetimer_->start();
        ui->Connect->setText("Disconnect");
    } else {
        gripTaskExecutor_->shutdown();
        updatetimer_->stop();
        ui->Connect->setText("Connect");
    }
}


void MainWindow::on_Btu_X_A_clicked()
{
    bool ok;
    signed int X = ui->lineEdit__X_A->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionAbsoluteX(X);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}


void MainWindow::on_Btu_X_R_1_clicked()
{
    bool ok;
    signed int X = ui->lineEdit__X_R->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionRelativeX(X);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }

}


void MainWindow::on_Btu_X_R_2_clicked()
{
    bool ok;
    signed int X = -1 * ui->lineEdit__X_R->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionRelativeX(X);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}


void MainWindow::on_Btu_X_V_clicked(bool checked)
{
    bool ok;
    signed int X = ui->lineEdit__X_V->text().toInt(&ok);

    if (ok) {
        if (checked) {
            gripTaskExecutor_->setVelocityModeX(X);
            ui->Btu_X_V->setText("Stop");
        } else {
            gripTaskExecutor_->closeVelocityModeX();
            ui->Btu_X_V->setText("Open");
        }
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}

void MainWindow::on_Btu_RX_A_clicked()
{
    bool ok;
    signed int RX = ui->lineEdit__RX_A->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionAbsoluteRX(RX);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}


void MainWindow::on_Btu_RX_R_clicked()
{
    bool ok;
    signed int RX = ui->lineEdit__RX_R->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionRelativeRX(RX);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}


void MainWindow::on_Btu_RX_R_2_clicked()
{
    bool ok;
    signed int RX = -1 * ui->lineEdit__RX_R->text().toInt(&ok);

    if (ok) {
        gripTaskExecutor_->gotoPositionRelativeRX(RX);
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}


void MainWindow::on_Btu_RX_V_clicked(bool checked)
{
    bool ok;
    signed int RX = ui->lineEdit__RX_V->text().toInt(&ok);

    if (ok) {
        if (checked) {
            gripTaskExecutor_->setVelocityModeRX(RX);
            ui->Btu_RX_V->setText("Stop");
        } else {
            gripTaskExecutor_->closeVelocityModeRX();
            ui->Btu_RX_V->setText("Open");
        }
    } else {
        QMessageBox::warning(this, "Error", "Please enter a valid signed integer.");
    }
}





void MainWindow::on_Reference_clicked()
{
    gripTaskExecutor_->findReference();
}

