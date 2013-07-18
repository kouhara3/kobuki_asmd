/**
 * @file /kobuki_driver/src/test/MapShow.cpp
 *
 * 
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QPainter>
//#include "Map.cpp"
/*****************************************************************************
** Classes
*****************************************************************************/

class MapShowWidget : public QWidget {
  protected:
    void paintEvent(QPaintEvent*);
};

void MapShowWidget::paintEvent(QPaintEvent *event) {

    Map m;
    m.setMap(200, 200);
    m.getMapInfo();
    m.init();

    QPen pen(Qt::red);
    QPainter painter(this);
    painter.setPen(pen);
    painter.begin(m.getPixmapaPointer());
    painter.drawPixmap(20, 20, (QPixmap)*m.getPixmapaPointer());
    painter.drawLine(30,30,40,56);
    painter.drawLine(40,56,180,20);
    painter.drawLine(180,20,130,100);
    painter.end();

}

/*****************************************************************************
** Test Main
*****************************************************************************/
void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char** argv) {

    QApplication app(argc,argv);
    
    MapShowWidget pWidget;
    pWidget.setWindowTitle("QPixmap of field!");
    pWidget.resize(300, 300);

    QLabel label("Hello, this is a map!~", &pWidget);
    
    QPushButton btnExit("Push me to quit!", &pWidget);
    btnExit.move(150, 260);
    QObject::connect(&btnExit, SIGNAL(clicked()), &app, SLOT(quit()));

    pWidget.show();
    app.exec();

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("mapChatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}


