#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtGui>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //(90,100,100),cvScalar(115,290,260),imgThreshed);

    //35,90,0),cvScalar(50,175,255),imgThreshed);

    blue_h_l=90; blue_h_h=115;
    blue_s_l=100; blue_s_h=290;
    blue_v_l=110; blue_v_h=260;

    red_h_l=40; red_h_h=50;
    //red_h_l=105; red_h_h=135;
    red_s_l=70; red_s_h=250;
    red_v_l=30; red_v_h=255;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    fin=0;
    MainWindow::beginProjection();

}

void MainWindow::on_pushButton_2_clicked()
{
    fin=1;
    delete ui;
}

void MainWindow::on_pushButton_4_clicked()
{
    MainWindow::fin=0;
    MainWindow::CalibBlue();

}

void MainWindow::on_pushButton_3_clicked()
{
    MainWindow::fin=0;
    MainWindow::CalibRed();
}

void MainWindow::on_pushButton_6_clicked()
{
    MainWindow::fin=0;
    MainWindow::beginHand();
}

void MainWindow::on_pushButton_5_clicked()
{
    MainWindow::fin=0;
    MainWindow::beginThimble();
}

void MainWindow::on_pushButton_7_clicked()
{
    int r=MainWindow::match(ui->comboBox->currentIndex(),ui->comboBox_2->currentIndex());

    if(r==1) {
        ui->resultLabel->setText("FOUND");
    }
    else ui->resultLabel->setText("NOT FOUND");
}



void MainWindow::on_pushButton_8_clicked()
{

    MainWindow::fin=0;
    MainWindow::beginGreen(100+ ui->horizontalScrollBar->value());
}

void MainWindow::on_horizontalScrollBar_valueChanged(int value)
{
    char str1[10];
    sprintf(str1,"%d",value+100);
    QString qstr=qstr.fromStdString(str1);
    ui->lineEdit->setText(qstr);
}

void MainWindow::on_pushButton_9_clicked()
{
    MainWindow::fin=0;
    MainWindow::beginDrawing();
}
