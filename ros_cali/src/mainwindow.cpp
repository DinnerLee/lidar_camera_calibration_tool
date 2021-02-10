#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this, &MainWindow::newString_roll, this, &MainWindow::on_listWidget_roll_currentTextChanged);
    connect(this, &MainWindow::newString_pitch, this, &MainWindow::on_listWidget_pitch_currentTextChanged);
    connect(this, &MainWindow::newString_yaw, this, &MainWindow::on_listWidget_yaw_currentTextChanged);
    connect(this, &MainWindow::newString_x, this, &MainWindow::on_listWidget_x_currentTextChanged);
    connect(this, &MainWindow::newString_y, this, &MainWindow::on_listWidget_y_currentTextChanged);
    connect(this, &MainWindow::newString_z, this, &MainWindow::on_listWidget_z_currentTextChanged);

    img_logo.load("/home/a/catkin_ws/src/ros_cali/logo.png");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_listWidget_roll_currentTextChanged(const QString &currentText)
{
    ui->listWidget_roll->clear();
    ui->listWidget_roll->addItem(currentText);
    ui->label->clear();
    int w = ui->label->width();
    int h = ui->label->height();
    ui->label->setPixmap(img_cali.scaled(w, h, Qt::KeepAspectRatioByExpanding));

    int w2 = ui->label_2->width();
    int h2 = ui->label_2->height();
    ui->label_2->setPixmap(img_logo.scaled(w2, h2, Qt::KeepAspectRatio));
}

void MainWindow::on_listWidget_pitch_currentTextChanged(const QString &currentText)
{
    ui->listWidget_pitch->clear();
    ui->listWidget_pitch->addItem(currentText);
}

void MainWindow::on_listWidget_yaw_currentTextChanged(const QString &currentText)
{
    ui->listWidget_yaw->clear();
    ui->listWidget_yaw->addItem(currentText);
}

void MainWindow::on_listWidget_x_currentTextChanged(const QString &currentText)
{
    ui->listWidget_x->clear();
    ui->listWidget_x->addItem(currentText);
}

void MainWindow::on_listWidget_y_currentTextChanged(const QString &currentText)
{
    ui->listWidget_y->clear();
    ui->listWidget_y->addItem(currentText);
}

void MainWindow::on_listWidget_z_currentTextChanged(const QString &currentText)
{
    ui->listWidget_z->clear();
    ui->listWidget_z->addItem(currentText);
}
