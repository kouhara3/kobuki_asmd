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
	
	// �����������ݵ�ʵ�ʳ���
	i = WindowWidth * 3;		// �õ�ÿһ�е��������ݳ��� 
	while( i%4 != 0 )		// �������ݣ�ֱ��i�ǵı��� 
		++i; 
	PixelDataLength = i * WindowHeight; 
	
	// �����ڴ�ʹ��ļ� 
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
	
	// ��ȡ���� 
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glReadPixels(0, 0, WindowWidth, WindowHeight, GL_RGB, GL_UNSIGNED_BYTE, pPixelData); 
	// ��dummy.bmp���ļ�ͷ����Ϊ���ļ����ļ�ͷ
	fread(BMP_Header, sizeof(BMP_Header), 1, pDummyFile);
	fwrite(BMP_Header, sizeof(BMP_Header), 1, pWritingFile);
	fseek(pWritingFile, 0x0012, SEEK_SET);
	
	i = WindowWidth; 
	j = WindowHeight;
	fwrite(&i, sizeof(i), 1, pWritingFile);
	fwrite(&j, sizeof(j), 1, pWritingFile);
	
	// д���������� 
	fseek(pWritingFile, 0, SEEK_END); 
	fwrite(pPixelData, PixelDataLength, 1, pWritingFile); 
	
	// �ͷ��ڴ�͹ر��ļ� 
	fclose(pDummyFile);
	fclose(pWritingFile);
	free(pPixelData);
	
	printf("Picture was saved as grap.bmp\n");
	
	/*
	GLint width, height;		// ʹ��OpenGL��GLint���ͣ�����32λ�ġ� 
								// ��C���Ա����int��һ����32λ�ġ� 
	FILE* pFile;				// ��������С����ļ����Ĳ��� 
	fseek(pFile, 0x0012, SEEK_SET);				// �ƶ���0x0012λ�� 
	fread(&width, sizeof(width), 1, pFile);		// ��ȡ��� 
	fseek(pFile, 0x0016, SEEK_SET);				// �ƶ���0x0016λ�� 
												// ������һ��ִ�к󱾾�Ӧ����0x0016λ�� 
												// ������һ���ʡ�� 
	fread(&height, sizeof(height), 1, pFile);	// ��ȡ�߶�
	*/
	//glReadPixels����ȡһЩ����
	/*
		�ú����ܹ����߸�������ǰ�ĸ��������Եõ�һ�����Σ��þ��������������ض��ᱻ��ȡ������
		����һ������������ʾ�˾��ε����½Ǻᡢ�����꣬�����Դ��������½�Ϊ�㣬�����Ͻ�Ϊ���ֵ��
		�������ĸ�������ʾ�˾��εĿ�Ⱥ͸߶ȣ� �����������ʾ��ȡ�����ݣ�
		���磺GL_RGB�ͻ����ζ�ȡ���صĺ졢�̡����������ݣ�GL_RGBA������ζ�ȡ���صĺ졢�̡�����alpha�������ݣ�
		GL_RED��ֻ��ȡ���صĺ�ɫ���ݣ����ƵĻ���GL_GREEN��GL_BLUE���Լ�GL_ALPHA����
		������õĲ���RGBA��ɫģʽ�����ǲ�����ɫ����ģʽ����Ҳ����ʹ��GL_COLOR_INDEX����ȡ���ص���ɫ������
		Ŀǰ����Ҫ֪����Щ����ʵ���ϻ����Զ�ȡ�������ݣ�������Ȼ�������������ݵȡ� 
		������������ʾ��ȡ�����ݱ��浽�ڴ�ʱ��ʹ�õĸ�ʽ�����磺GL_UNSIGNED_BYTE��Ѹ������ݱ���ΪGLubyte��GL_FLOAT��Ѹ������ݱ���ΪGLfloat�ȡ� 
		���߸�������ʾһ��ָ�룬�������ݱ���ȡ�󣬽������浽���ָ������ʾ�ĵ�ַ��
		ע�⣬��Ҫ��֤�õ�ַ���㹻�Ŀ���ʹ�õĿռ䣬�����ɶ�ȡ���������ݡ�
		����һ����СΪ256*256��ͼ�������ȡ��RGB���ݣ���ÿһ���ݱ�����ΪGLubyte���ܴ�С���ǣ�256*256*3 = 196608�ֽڣ���192ǧ�ֽڡ�
		����Ƕ�ȡRGBA���ݣ����ܴ�С����256*256*4 = 262144�ֽڣ���256ǧ�ֽڡ� 
		
		ע�⣺glReadPixelsʵ�����Ǵӻ������ж�ȡ���ݣ����ʹ����˫����������Ĭ���Ǵ�������ʾ�Ļ��壨��ǰ���壩�ж�ȡ��
		�����ƹ�����Ĭ�ϻ��Ƶ��󻺳����ġ���ˣ������Ҫ��ȡ�Ѿ����ƺõ����أ�������Ҫ�Ƚ���ǰ�󻺳塣
	*/


	//glDrawPixels������һЩ����
	//glCopyPixels������һЩ����,��ǰ�������Ч�ʸ�
	/*
		glCopyPixels�����������������һ������������ʾ����������Դ�ľ��ε����½����꣬
		�������ĸ�������ʾ����������Դ�ľ��еĿ�Ⱥ͸߶ȣ����������ͨ��ʹ��GL_COLOR��
		��ʾ�������ص���ɫ����Ҳ������GL_DEPTH��GL_STENCIL���ֱ��ʾ������Ȼ������ݻ�ģ�建�����ݡ�
	*/




	/*
		3.2 ���OpenGL���õ�RGB����������BMP�ļ���BGR��������˳��һ������
		����ʹ��һЩ���뽻��ÿ�����صĵ�һ�ֽں͵����ֽڣ�
		ʹ��RGB�����ݱ��BGR�����ݡ���ȻҲ����ʹ������ķ�ʽ������⣺
		�°汾��OpenGL���˿���ʹ��GL_RGB��ȡ���صĺ졢�̡��������⣬Ҳ����ʹ��GL_BGR�����෴��˳�����ζ�ȡ���ص������̡������ݣ�
		��������BMP�ļ���ʽ���Ǻ��ˡ�
		��ʹ���gl/gl.hͷ�ļ���û�ж������GL_BGR��Ҳû�й�ϵ�����Գ���ʹ��GL_BGR_EXT��
		��Ȼ�е�OpenGLʵ�֣������Ǿɰ汾��ʵ�֣�������ʹ��GL_BGR_EXT��������֪����Windows�����¸���OpenGLʵ�ֶ���GL_BGR�ṩ��֧�֣�
		�Ͼ�Windows�и��ֱ�ʾ��ɫ�����ݼ�������ʹ��BGR��˳�򣬶���RGB��˳���������IBM-PC��Ӳ������йء�
	*/

	/*
		3.3 ����BMP�ļ��С����롱������Ӱ�� ʵ����OpenGLҲ֧��ʹ�������֡����롱��ʽ���������ݡ�
		ֻҪͨ��glPixelStore�޸ġ����ر���ʱ����ķ�ʽ���Ϳ����ˡ�
		�������� 
		int alignment = 4; 
		glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

		��һ��������ʾ���������صĶ���ֵ�����ڶ���������ʾʵ������Ϊ���١�
		�������ؿ��Ե��ֽڶ��루ʵ���Ͼ��ǲ�ʹ�ö��룩��˫�ֽڶ��루�������Ϊ���������ٲ�һ���ֽڣ������ֽڶ��루������Ȳ����ĵı�������Ϊ�ĵı����������ֽڶ��롣
		�ֱ��Ӧalignment��ֵΪ1, 2, 4, 8��ʵ���ϣ�Ĭ�ϵ�ֵ��4��������BMP�ļ��Ķ��뷽ʽ���Ǻϡ�
	*/

}

static void key_callback(unsigned char key, int x, int y)
{
	switch(key) {
		case 'q':										//����q���˳�����
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
