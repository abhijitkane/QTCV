#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    int blue_h_h, blue_h_l, blue_s_h, blue_s_l, blue_v_h, blue_v_l;
    int red_h_h, red_h_l, red_s_h, red_s_l, red_v_h, red_v_l;
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();


    void on_pushButton_4_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_7_clicked();

    void on_comboBox_activated(const QString &arg1);

    void on_pushButton_8_clicked();

    void on_horizontalScrollBar_valueChanged(int value);

    void on_pushButton_9_clicked();

private:
    Ui::MainWindow *ui;
    int begin();
    int beginHand();
    int beginThimble();
    int beginProjection();
    int match(int,int);
    int beginGreen(int);

    void beginDrawing();

    void CalibBlue();
    void CalibRed();
    int fin;
};

#endif // MAINWINDOW_H
