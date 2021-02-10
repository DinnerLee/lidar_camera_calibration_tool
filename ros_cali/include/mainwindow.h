#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QString>
#include <QMainWindow>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void addFloat_roll(const std_msgs::Float32::ConstPtr& msg){
        newString_roll(QString(std::to_string(msg->data).c_str()));
    }
    void addFloat_pitch(const std_msgs::Float32::ConstPtr& msg){
        newString_pitch(QString(std::to_string(msg->data).c_str()));
    }
    void addFloat_yaw(const std_msgs::Float32::ConstPtr& msg){
        newString_yaw(QString(std::to_string(msg->data).c_str()));
    }
    void addFloat_x(const std_msgs::Float32::ConstPtr& msg){
        newString_x(QString(std::to_string(msg->data).c_str()));
    }
    void addFloat_y(const std_msgs::Float32::ConstPtr& msg){
        newString_y(QString(std::to_string(msg->data).c_str()));
    }
    void addFloat_z(const std_msgs::Float32::ConstPtr& msg){
        newString_z(QString(std::to_string(msg->data).c_str()));
    }
    void addImage(const sensor_msgs::Image::ConstPtr& msg){
        QImage img(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
        img = std::move(img).rgbSwapped();
        img_cali = QPixmap::fromImage(img);   //이미지를 버퍼에 옮긴다.

        img_cali = img_cali.scaled(img.width(),img.height()); //이미지 사이즈 조절
    }

    void addString(const QString& s)
    {
        //newString(s);
    }

    //Emit this as a signal to be caught locally in order to prevent race conditions
    void addString(const std_msgs::String::ConstPtr& msg)
    {
       // newString("I heard: [" + QString::fromStdString(str) + "]");
    }
private slots:
    void on_listWidget_roll_currentTextChanged(const QString &currentText);

    void on_listWidget_pitch_currentTextChanged(const QString &currentText);

    void on_listWidget_yaw_currentTextChanged(const QString &currentText);

    void on_listWidget_x_currentTextChanged(const QString &currentText);

    void on_listWidget_y_currentTextChanged(const QString &currentText);

    void on_listWidget_z_currentTextChanged(const QString &currentText);

private:
    Ui::MainWindow *ui;
    QPixmap img_cali;
    QPixmap img_logo;

signals:
    void stringsChanged();
    void newString_roll(QString s);
    void newString_pitch(QString s);
    void newString_yaw(QString s);
    void newString_x(QString s);
    void newString_y(QString s);
    void newString_z(QString s);
};

#endif // MAINWINDOW_H
