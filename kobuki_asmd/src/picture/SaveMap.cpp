#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <GL/glut.h>
#include "Coordinate.cpp"
#include "b.cpp"

#define BMP_Header_Length 54  
  

  std::vector< std::vector<Block> > Global_Block_List;
  std::vector< Block > Global_Wall_List;
  Coordinate Global_Max;
  static int Save_Flag = 0;

  void bmp_file(GLint WindowWidth, GLint WindowHeight)
  {

	glReadBuffer(GL_FRONT);

	FILE* pDummyFile;
	FILE* pWritingFile;
	GLubyte* pPixelData;
	GLubyte BMP_Header[BMP_Header_Length]; 
	GLint i, j; 
	GLint PixelDataLength; 
	
	// 计算像素数据的实际长度
	i = WindowWidth * 3;		// 得到每一行的像素数据长度 
	while( i%4 != 0 )		// 补充数据，直到i是的倍数 
		++i; 
	PixelDataLength = i * WindowHeight; 
	
	// 分配内存和打开文件 
	pPixelData = (GLubyte*)malloc(PixelDataLength); 
	if( pPixelData == 0 ) 
		exit(0); 
	pDummyFile = fopen("/home/winter/dummy.bmp", "rb+");
  if( pDummyFile == 0 ) 
	{  
           printf("Can't open dummy.bmp\n");
   	   exit(0); 
	}
	pWritingFile = fopen("/home/winter/Map.bmp", "wb+");
  if( pWritingFile == 0 )
	{  
           printf("Can't open Map.bmp\n");
   	   exit(0); 
	}
	
	// 读取像素
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glReadPixels(0, 0, WindowWidth, WindowHeight, GL_RGB, GL_UNSIGNED_BYTE, pPixelData); 
	// 把dummy.bmp的文件头复制为新文件的文件头
	fread(BMP_Header, sizeof(BMP_Header), 1, pDummyFile);
	fwrite(BMP_Header, sizeof(BMP_Header), 1, pWritingFile);
	fseek(pWritingFile, 0x0012, SEEK_SET);
	
	i = WindowWidth; 
	j = WindowHeight;
	fwrite(&i, sizeof(i), 1, pWritingFile);
	fwrite(&j, sizeof(j), 1, pWritingFile);
	
	// 写入像素数据 
	fseek(pWritingFile, 0, SEEK_END); 
	fwrite(pPixelData, PixelDataLength, 1, pWritingFile); 
	
	// 释放内存和关闭文件 
	fclose(pDummyFile);
	fclose(pWritingFile);
	free(pPixelData);
	
	printf("Picture was saved as map.bmp\n");
  }

  void save()
  {

    float width = Global_Max.getCoordinateX()*100;
    float height = Global_Max.getCoordinateY()*100;
    bmp_file((GLint)width, (GLint)height);
    
    return;
  }

  void key_callback(unsigned char key, int x, int y)
  {
        switch(key) {
		case 'q':
			exit(0);
			break;
		case 's':
                        save();	
			break;
		default:
			break;
	}
	glutPostRedisplay(); 
  }

  void makeMap()
  {
    printf("Make map start!\n");

    float width = Global_Max.getCoordinateX()*100/2;
    //printf("[width: %f]", width);
    float height = Global_Max.getCoordinateY()*100/2;
    //printf("[height: %f]", width);

    float block_width =  DEFAULT_BLOCK_LENGTH*100/width;
    float block_height =  DEFAULT_BLOCK_LENGTH*100/height;

    int idx_x = Global_Block_List.size();
    int idx_y = Global_Block_List[0].size();
    
    glClearColor(1.0f, 1.0f ,1.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);


    for(int i = 0; i<idx_x; i++){

      float pointX = -1.0f + i * block_width;

      for(int j = 0; j<idx_y ; j++){ 
          float pointY = -1.0f + j*block_height;
          glEnable(GL_LINE_STIPPLE);
          glLineStipple(2, 0x0F0F);
          glLineWidth(1.0f);
          glColor3f(0.6f, 0.6f, 0.6f);

	  glBegin(GL_LINES); 
	    glVertex2f(pointX, pointY);                                 glVertex2f(pointX + block_width, pointY); 
            glVertex2f(pointX + block_width, pointY);                   glVertex2f(pointX + block_width, pointY + block_height); 
            glVertex2f(pointX + block_width, pointY + block_height);    glVertex2f(pointX, pointY + block_height); 
            glVertex2f(pointX, pointY + block_height);                  glVertex2f(pointX, pointY);
          glEnd();
          
          glDisable(GL_LINE_STIPPLE);
          
          if(Global_Block_List[i][j].getMark() == OBSTACLE )
          {
            Borders* borders = Global_Block_List[i][j].borders;

            glColor3f(0.0f, 0.0f, 1.0f);
            
            if(borders->left != 0.0f)
            {
              glRectf(pointX + borders->left*100/width, pointY, pointX + block_width, pointY + block_height);
            }
            else if(borders->right != 0.0f) 
            {
              glRectf(pointX , pointY, pointX + (DEFAULT_BLOCK_LENGTH-borders->right)*100/width, pointY + block_height);
            }
            else if(borders->up != 0.0f) 
            {
              glRectf(pointX, pointY, pointX + block_width, pointY + (DEFAULT_BLOCK_LENGTH-borders->up)*100/height);
            }
            else if(borders->down != 0.0f) 
            {
              glRectf(pointX , pointY + borders->down*100/height , pointX + block_width, pointY + block_width);
            }
          }
/*
          if(Global_Block_List[i][j].getMark() == WALL)
          {
            Borders* borders = Global_Block_List[i][j].borders;


            glLineWidth(3.0f);
            glColor3f(1.0f, 0.0f, 0.0f);

            glBegin(GL_LINES); 
            if(borders->left != 0.0f)
            {              
	        glVertex2f(pointX + borders->left/width, pointY);  glVertex2f(pointX + borders->left/width, pointY + block_height); 
            }
            else if(borders->right != 0.0f) 
            {
	        glVertex2f(pointX + (BLOCK_SIZE-borders->right)/width, pointY);  glVertex2f(pointX + (BLOCK_SIZE-borders->right)/width, pointY + block_height); 
            }
            if(borders->up != 0.0f)
            {
	        glVertex2f(pointX, pointY + (BLOCK_SIZE-borders->up)/height);  glVertex2f(pointX + block_width, pointY+ (BLOCK_SIZE-borders->up)/height); 
            }
            else if(borders->down != 0.0f) 
            {
	        glVertex2f(pointX, pointY + borders->down/height);  glVertex2f(pointX + block_width, pointY + borders->down/height); 
            }
            glEnd();
          }
*/
      }
    }



//////////make wall start////////////

    printf("Make wall start!\n");

    float wallPointX = -1.0f;
    float wallPointY = -1.0f;

          for(int i = 0; i < Global_Wall_List.size(); i++)
          {
            Borders* borders = Global_Wall_List[i].borders;

            Coordinate coord = Global_Wall_List[i].getCenterPoint();
            float blockX = -1.0f + (coord.getCoordinateX() + DEFAULT_BLOCK_LENGTH) *100/width ;
            float blockY = -1.0f + (coord.getCoordinateY() + DEFAULT_BLOCK_LENGTH) *100/height ;

            glLineWidth(3.0f);
            glColor3f(1.0f, 0.0f, 0.0f);

            glBegin(GL_LINES); 
            if(borders->left != 0.0f)
            {
                glVertex2f(wallPointX, wallPointY);  glVertex2f(blockX + borders->left*100/width, blockY + block_height/2); 
                wallPointX = blockX + borders->left*100/width;
                wallPointY = blockY + block_height/2;
            }
            else if(borders->right != 0.0f) 
            {
                glVertex2f(wallPointX, wallPointY);  glVertex2f(blockX + (DEFAULT_BLOCK_LENGTH-borders->right)*100/width, blockY + block_height/2); 
                wallPointX = blockX + (DEFAULT_BLOCK_LENGTH-borders->right)*100/width;
                wallPointY = blockY + block_height/2;
            }
            else if(borders->up != 0.0f)
            {
                glVertex2f(wallPointX, wallPointY);  glVertex2f(blockX + block_width/2, blockY+ (DEFAULT_BLOCK_LENGTH-borders->up)*100/height); 
                wallPointX = wallPointX + block_width/2;
                wallPointY = wallPointY+ (DEFAULT_BLOCK_LENGTH-borders->up)*100/height;
            }
            else if(borders->down != 0.0f) 
            {
                glVertex2f(wallPointX, wallPointY);  glVertex2f(blockX + block_width/2, blockY + borders->down*100/height); 
                wallPointX = blockX + block_width/2;
                wallPointY = blockY + borders->down*100/height;
            }
            glEnd();
          }
    printf("wallPointX,Y:[%f , %f]\n",wallPointX, wallPointY);
    printf("Make wall end!\n");
///////make wall over////////

    printf("Make map end!\n");

    glFlush();
    glutSwapBuffers();

    if(Save_Flag != 1)
    {
      save();
      Save_Flag = 1;
    }

  }

  int showMap(int argc, char * argv[])
  {
    int width = Global_Max.getCoordinateX()*100;
    int height = Global_Max.getCoordinateY()*100;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Mapping!");
    glutDisplayFunc(&makeMap);
    glutKeyboardFunc(&key_callback);

    glutMainLoop();
    return 0;
  }
/*
  void outputBlockList( Map& map ){
    Global_Block_List = map.getBlockList();
    return;
  }

void outputWallList( Map& map ){
    Global_Wall_List = map.getWallList();
    return;
  }

  void outputMax( Map& map ){
    Global_Max = map.getMax();
    return;
  }
*/

int main(int argc, char * argv[])
{
  for (int i = 0; i < 5; i++)
  {
    std::vector<Block> list;
      
        for(int j = 0; j<5; j++) {
          Coordinate coord;
          coord.setCoordinate(i*DEFAULT_BLOCK_LENGTH-0.4f, j*DEFAULT_BLOCK_LENGTH-0.4f);
          Block block(coord, DEFAULT_BLOCK_LENGTH, i, j);
          if(i == 1 && j==2) 
          {
            block.borders = new Borders;
            block.borders->left = 0.1f;
            block.setMark(WALL);
            Global_Wall_List.push_back(block);
          }
          if(i == 3 && j == 2) 
          {
            block.borders = new Borders;
            block.borders->left = 0.35f;
            block.setMark(WALL);
            Global_Wall_List.push_back(block);
          }

          /*if(i == 2 && j == 4) 
          {
            Borders b;
            b.left = 0.01f;
            block.borders = &b;
            block.setMark(WALL);
            Global_Wall_List.push_back(block);
          }*/
  	  list.push_back(block);
        }

        Global_Block_List.push_back(list);   
        list.clear(); 
  }

  Global_Max.setCoordinate(2.0f, 2.0f);

  return showMap(argc, argv);
}

