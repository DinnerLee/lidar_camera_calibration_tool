#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <functional>

#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QApplication>
#include <QUrl>
#include <QString>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

#include <signal.h>

#include "mainwindow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_cali");
    ros::NodeHandle node;

    QApplication a(argc, argv);
    MainWindow w;

    ros::Subscriber sub_roll = node.subscribe("/cali/roll", 1000, &MainWindow::addFloat_roll, &w);
    ros::Subscriber sub_pitch = node.subscribe("/cali/pitch", 1000, &MainWindow::addFloat_pitch, &w);
    ros::Subscriber sub_yaw = node.subscribe("/cali/yaw", 1000, &MainWindow::addFloat_yaw, &w);
    ros::Subscriber sub_x = node.subscribe("/cali/x", 1000, &MainWindow::addFloat_x, &w);
    ros::Subscriber sub_y = node.subscribe("/cali/y", 1000, &MainWindow::addFloat_y, &w);
    ros::Subscriber sub_z = node.subscribe("/cali/z", 1000, &MainWindow::addFloat_z, &w);
    ros::Subscriber sub_img = node.subscribe("/cali/image", 1000, &MainWindow::addImage, &w);
    w.show();
    //Start ros in separate thread, and trigger Qt shutdown when it exits
    //If Qt exits before ros, be sure to shutdown ros
    QFutureWatcher<void> rosThread;
    rosThread.setFuture(QtConcurrent::run(&ros::spin));
    QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &a, &QCoreApplication::quit);
    QObject::connect(&a, &QCoreApplication::aboutToQuit, [](){ros::shutdown();});


    return a.exec();
}
