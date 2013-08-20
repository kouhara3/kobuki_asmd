#include <stdio.h>
#include <stdlib.h>
#include <QApplication>
#include <QLabel>
#include <QPixmap>
#include <QImage>

///////////////////////////////////////

int main( int argc, char *argv[]);//本来はProto.h内で定義し、includeする

/*********************************************************************
    main fanction
*********************************************************************/
int main( int argc, char *argv[] ){
    QApplication application( argc, argv );
    
    return application.exec();//Pass control to Qt
}
