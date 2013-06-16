/**
 * @file /kobuki_driver/src/test/Coordinate.cpp
 *
 * @Map in coordinate.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>

#include <iostream>
#include "Coordinate.cpp"
#include "KobukiManager.cpp"
/*****************************************************************************
** Classes
*****************************************************************************/

class Map {
public:
  Map(){
    this->max.setCoordinate(0,0);
    //this->KobukiManager[0] = KobukiManager();
    //this->KobukiManager[1] = KobukiManager();
  }
  Map(int coord_x, int coord_y){
    this->max.setCoordinate(coord_x, coord_y);
  }
  void setMap(int coord_x, int coord_y){
    this->max.setCoordinate(coord_x, coord_y);
  }

  void init(){

    if(this->max.getCoordinateX()>0 && this->max.getCoordinateY()>0) {

      this->pixmap = new QPixmap(this->max.getCoordinateX(), this->max.getCoordinateY());
      pixmap->fill(Qt::white);

    } else {
      std::cout<< "This map has a wrong size!" << std::endl;
    }

    return;
  }

  QPixmap* getPixmapaPointer(){
    return this->pixmap;
  }
  
  void getMapInfo(){
    std::cout << "This map have a max coordinate of [" << this->max.getCoordinateX() << ", " << this->max.getCoordinateY() << "]" << std::endl;
  }
  
  
private:
  Coordinate max;
  QPixmap* pixmap;
  //KobukiManager[2] Manager;
};
/*****************************************************************************
** PainterWidget
*****************************************************************************/

class PainterWidget : public QWidget {
  protected:
    void paintEvent(QPaintEvent*);
};

void PainterWidget::paintEvent(QPaintEvent *event) {

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

int main(int argc, char** argv) {
    QApplication app(argc,argv);
      
    PainterWidget pWidget;
    pWidget.setWindowTitle("QPixmap & QBitmap");
    pWidget.resize(300, 300);

    QLabel label("Hello, this is a map!~", &pWidget);
    
    QPushButton button1("Push me to quit!", &pWidget);
    button1.move(150, 260);
    QObject::connect(&button1, SIGNAL(clicked()), &app, SLOT(quit()));

    pWidget.show();
    app.exec();
    return 0;
}
