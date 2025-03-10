#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QKeyEvent>
#include "GripTaskExecutor.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_Connect_clicked(bool checked);

    void on_Btu_X_A_clicked();

    void on_Btu_X_R_1_clicked();

    void on_Btu_X_R_2_clicked();

    void on_Btu_X_V_clicked(bool checked);

    void on_Btu_RX_A_clicked();

    void on_Btu_RX_R_clicked();

    void on_Btu_RX_R_2_clicked();

    void on_Btu_RX_V_clicked(bool checked);

    void on_Reference_clicked();

private slots:
    void updateGUI();

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;




private:
    Ui::MainWindow *ui;
    GripTaskExecutor* gripTaskExecutor_;
    QTimer* updatetimer_;

    GripTaskData data_;

    bool speedModeEnabled;
};
#endif // MAINWINDOW_H
