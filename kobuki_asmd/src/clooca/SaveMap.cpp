#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <GL/glut.h>
#include "Map.cpp"

#define BLOCK_SIZE 40
#define BMP_Header_Length 54  

  std::vector< std::vector<Block> > Global_Block_List;
  Coordinate Global_Max;

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
	pDummyFile = fopen("/home/ishida/dummy.bmp", "rb+");
  if( pDummyFile == 0 ) 
	{  
           printf("Can't open dummy.bmp\n");
   	   exit(0); 
	}
	pWritingFile = fopen("/home/ishida/Map.bmp", "wb+");
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
	
	printf("Picture was saved as grap.bmp\n");
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

    float width = Global_Max.getCoordinateX()*100/2;
    //printf("[width: %f]", width);
    float height = Global_Max.getCoordinateY()*100/2;
    //printf("[height: %f]", width);
    float block_width =  BLOCK_SIZE/width;
    float block_height =  BLOCK_SIZE/height;
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
          glLineWidth(2.0f);
          glColor3f(0.0f, 0.0f, 0.0f);

	  glBegin(GL_LINES); 
	    glVertex2f(pointX, pointY);                                 glVertex2f(pointX + block_width, pointY); 
            glVertex2f(pointX + block_width, pointY);                   glVertex2f(pointX + block_width, pointY + block_height); 
            glVertex2f(pointX + block_width, pointY + block_height);    glVertex2f(pointX, pointY + block_height); 
            glVertex2f(pointX, pointY + block_height);                  glVertex2f(pointX, pointY);
          glEnd();
          
          glDisable(GL_LINE_STIPPLE);

          if(Global_Block_List[i][j].getMark() == OBSTACLE || Global_Block_List[i][j].getMark() == WALL)
          {
            Borders* borders = Global_Block_List[i][j].borders;
            glColor3f(0.5f, 0.0f, 0.0f);
            //glRectf(pointX + borders->left/width, pointY + borders->down/height, pointX + (BLOCK_SIZE-borders->right)/width, pointY + (BLOCK_SIZE-borders->up)/height);
            if(borders->left != 0.0f)
            {
              glRectf(pointX + borders->left/width, pointY, pointX + block_width, pointY + block_height);
            }
            else if(borders->right != 0.0f) 
            {
              glRectf(pointX , pointY, pointX + (BLOCK_SIZE-borders->right)/width, pointY + block_height);
            }
            else if(borders->up != 0.0f) 
            {
              glRectf(pointX, pointY, pointX + block_width, pointY + (BLOCK_SIZE-borders->up)/height);
            }
            else if(borders->down != 0.0f) 
            {
              glRectf(pointX , pointY + borders->down/height , pointX + block_width, pointY + block_width);
            }
          }

          if(Global_Block_List[i][j].getMark() == WALL)
          {
            Borders* borders = Global_Block_List[i][j].borders;
            glColor3f(0.0f, 0.0f, 0.0f);

            if(borders->left != 0.0f)
            {
              glBegin(GL_LINES); 
	        glVertex2f(pointX + borders->left/width, pointY);  glVertex2f(pointX + borders->left/width, pointY + block_height); 
              glEnd();
            }
            else if(borders->right != 0.0f) 
            {
              glBegin(GL_LINES); 
	        glVertex2f(pointX + (BLOCK_SIZE-borders->right)/width, pointY);  glVertex2f(pointX + (BLOCK_SIZE-borders->right)/width, pointY + block_height); 
              glEnd();
            }
            if(borders->up != 0.0f)
            {
              glBegin(GL_LINES); 
	        glVertex2f(pointX, pointY + (BLOCK_SIZE-borders->up)/height);  glVertex2f(pointX + block_width, pointY+ (BLOCK_SIZE-borders->up)/height); 
              glEnd();
            }
            else if(borders->down != 0.0f) 
            {
              glBegin(GL_LINES); 
	        glVertex2f(pointX, pointY + borders->down/height);                                 glVertex2f(pointX + block_width, pointY + borders->down/height); 
              glEnd();
            }
          }

      }
    }

    glFlush();
    glutSwapBuffers();
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

  void outputBlockList( Map& map ){
    Global_Block_List = map.getBlockList();
    return;
  }

  void outputMax( Map& map ){
    Global_Max = map.getMax();
    return;
  }

/*
int main(int argc, char * argv[])
{
  for (int i = 0; i < 5; i++)
  {
    std::vector<Block> list;
      
        for(int j = 0; j<5; j++) {
          Coordinate coord;
          coord.setCoordinate(i*DEFAULT_BLOCK_LENGTH, j*(DEFAULT_BLOCK_LENGTH));
          Block block(coord, DEFAULT_BLOCK_LENGTH, i, j);
  	  list.push_back(block);
        }

        Global_Block_List.push_back(list);   
        list.clear(); 
  }

  Global_Max.setCoordinate(2.0f, 2.0f);

  return showMap(argc, argv);
}
*/
