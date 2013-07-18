/**
 * @file /kobuki_driver/src/test/ShowMap.cpp
 *
 * 
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QPainter>
#include "Map.cpp"
/*****************************************************************************
** Methods
*****************************************************************************/
void Map::paintEvent(QPaintEvent *event) {
    int max_x = this->max.getCoordinateX()*100;
    int max_y = this->max.getCoordinateY()*100;
    QPen pen(Qt::red);
    QPainter painter(this);
    painter.setPen(pen);
    painter.begin(this->pix);
    painter.drawPixmap(20, 20, this->pix);
    painter.drawRect(max_x-0, 0, 50, 50);
    painter.end();
}

