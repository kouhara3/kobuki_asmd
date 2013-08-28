#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>

#define WIDTH 600 
#define HEIGHT 600 
#define BMP_Header_Length 54


void bmp_file(GLint WindowWidth, GLint WindowHeight) {

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
        pWritingFile = fopen("/home/winter/grab.bmp", "wb+"); 
	if( pWritingFile == 0 )
	{  
           printf("Can't open grab.bmp\n");
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
	
	/*
	GLint width, height;		// 使用OpenGL的GLint类型，它是32位的。 
								// 而C语言本身的int则不一定是32位的。 
	FILE* pFile;				// 在这里进行“打开文件”的操作 
	fseek(pFile, 0x0012, SEEK_SET);				// 移动到0x0012位置 
	fread(&width, sizeof(width), 1, pFile);		// 读取宽度 
	fseek(pFile, 0x0016, SEEK_SET);				// 移动到0x0016位置 
												// 由于上一句执行后本就应该在0x0016位置 
												// 所以这一句可省略 
	fread(&height, sizeof(height), 1, pFile);	// 读取高度
	*/
	//glReadPixels：读取一些像素
	/*
		该函数总共有七个参数。前四个参数可以得到一个矩形，该矩形所包括的像素都会被读取出来。
		（第一、二个参数表示了矩形的左下角横、纵坐标，坐标以窗口最左下角为零，最右上角为最大值；
		第三、四个参数表示了矩形的宽度和高度） 第五个参数表示读取的内容，
		例如：GL_RGB就会依次读取像素的红、绿、蓝三种数据，GL_RGBA则会依次读取像素的红、绿、蓝、alpha四种数据，
		GL_RED则只读取像素的红色数据（类似的还有GL_GREEN，GL_BLUE，以及GL_ALPHA）。
		如果采用的不是RGBA颜色模式，而是采用颜色索引模式，则也可以使用GL_COLOR_INDEX来读取像素的颜色索引。
		目前仅需要知道这些，但实际上还可以读取其它内容，例如深度缓冲区的深度数据等。 
		第六个参数表示读取的内容保存到内存时所使用的格式，例如：GL_UNSIGNED_BYTE会把各种数据保存为GLubyte，GL_FLOAT会把各种数据保存为GLfloat等。 
		第七个参数表示一个指针，像素数据被读取后，将被保存到这个指针所表示的地址。
		注意，需要保证该地址有足够的可以使用的空间，以容纳读取的像素数据。
		例如一幅大小为256*256的图象，如果读取其RGB数据，且每一数据被保存为GLubyte，总大小就是：256*256*3 = 196608字节，即192千字节。
		如果是读取RGBA数据，则总大小就是256*256*4 = 262144字节，即256千字节。 
		
		注意：glReadPixels实际上是从缓冲区中读取数据，如果使用了双缓冲区，则默认是从正在显示的缓冲（即前缓冲）中读取，
		而绘制工作是默认绘制到后缓冲区的。因此，如果需要读取已经绘制好的像素，往往需要先交换前后缓冲。
	*/


	//glDrawPixels：绘制一些像素
	//glCopyPixels：复制一些像素,比前两者组合效率高
	/*
		glCopyPixels函数有五个参数，第一、二个参数表示复制像素来源的矩形的左下角坐标，
		第三、四个参数表示复制像素来源的举行的宽度和高度，第五个参数通常使用GL_COLOR，
		表示复制像素的颜色，但也可以是GL_DEPTH或GL_STENCIL，分别表示复制深度缓冲数据或模板缓冲数据。
	*/




	/*
		3.2 解决OpenGL常用的RGB像素数据与BMP文件的BGR像素数据顺序不一致问题
		可以使用一些代码交换每个像素的第一字节和第三字节，
		使得RGB的数据变成BGR的数据。当然也可以使用另外的方式解决问题：
		新版本的OpenGL除了可以使用GL_RGB读取像素的红、绿、蓝数据外，也可以使用GL_BGR按照相反的顺序依次读取像素的蓝、绿、红数据，
		这样就与BMP文件格式相吻合了。
		即使你的gl/gl.h头文件中没有定义这个GL_BGR，也没有关系，可以尝试使用GL_BGR_EXT。
		虽然有的OpenGL实现（尤其是旧版本的实现）并不能使用GL_BGR_EXT，但我所知道的Windows环境下各种OpenGL实现都对GL_BGR提供了支持，
		毕竟Windows中各种表示颜色的数据几乎都是使用BGR的顺序，而非RGB的顺序。这可能与IBM-PC的硬件设计有关。
	*/

	/*
		3.3 消除BMP文件中“对齐”带来的影响 实际上OpenGL也支持使用了这种“对齐”方式的像素数据。
		只要通过glPixelStore修改“像素保存时对齐的方式”就可以了。
		像这样： 
		int alignment = 4; 
		glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

		第一个参数表示“设置像素的对齐值”，第二个参数表示实际设置为多少。
		这里像素可以单字节对齐（实际上就是不使用对齐）、双字节对齐（如果长度为奇数，则再补一个字节）、四字节对齐（如果长度不是四的倍数，则补为四的倍数）、八字节对齐。
		分别对应alignment的值为1, 2, 4, 8。实际上，默认的值是4，正好与BMP文件的对齐方式相吻合。
	*/

}

static void key_callback(unsigned char key, int x, int y)
{
	switch(key) {
		case 'q':										//按下q键退出程序
			exit(0);
			break;
		case 's':
			bmp_file(WIDTH, HEIGHT);					
			break;
		default:
			break;
	}
	glutPostRedisplay(); 
	/* this redraws the scene without 
		waiting for the display callback so that any changes appear 
	instantly */
}


void bmp_copy(void){
	glClear(GL_COLOR_BUFFER_BIT);

	glBegin(GL_TRIANGLES);
	glColor3f(1.0f,0.0f,0.0f);	glVertex2f(0.0f,0.0f);
	glColor3f(0.0f,1.0f,0.0f);	glVertex2f(1.0f,0.0f);
	glColor3f(0.0f,0.0f,1.0f);	glVertex2f(0.5f,1.0f);
	glEnd();

	glPixelZoom(-0.5f, -0.5f);
	glRasterPos2i(1,1);

	glCopyPixels(WIDTH/2, HEIGHT/2, WIDTH/2, HEIGHT/2, GL_COLOR);

	glutSwapBuffers();
}


int bmp_copy_test(int argc, char* argv[]) { 
	glutInit(&argc, argv); 
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowPosition(200, 200);
	glutInitWindowSize(WIDTH, HEIGHT); 
	glutCreateWindow("BMP Pixel Copy Testing"); 

	glutDisplayFunc(bmp_copy);
	glutKeyboardFunc(key_callback);
	glutMainLoop(); 

	return 0; 
}
