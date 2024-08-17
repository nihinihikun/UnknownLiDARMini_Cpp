// main.cpp
#include "types.h"
#include "lidar.h"

#include <cmath>
#include <GL/freeglut.h>//graphic関係
#include <unistd.h>//close(),serialport関係で利用
#include <termios.h>//serialport関係で利用

#define DISPLAYBUFFSIZE 1024
#define DEBUG

LIDARPAYLOAD payload;
RAWDATA rawdata;
POLAR_DATA polardata[16];
int serial_port;

POLAR_DATA display_polardata_buff[DISPLAYBUFFSIZE];
int que_en=0;

//グラフィック関係
double zoomScale = 1.0; // ズームレベルの初期値は1.0（変更なし）
void keyboard(unsigned char key,int x,int y);
void mouse(int button, int state, int x, int y);
void updateData();
void initOpenGL();
void reshape(int width, int height);
static void display(void);
void queue(POLAR_DATA* comingdata,POLAR_DATA* display_polardata_buff);

int main(int argc,char **argv) {
    serial_port = serialBegin("/dev/ttyUSB0", B230400);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    int screenWidth = glutGet(GLUT_SCREEN_WIDTH);
    int screenHeight = glutGet(GLUT_SCREEN_HEIGHT);
    int windowWidth = screenWidth * 0.4;
    int windowHeight = screenHeight * 0.4;
    int windowPosX = (screenWidth - windowWidth) / 2;
    int windowPosY = (screenHeight - windowHeight) / 2;

    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(windowPosX, windowPosY);
    glutCreateWindow("LiDAR Polar Plot");
    initOpenGL();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
    close(serial_port);
    return 0;
}

void initOpenGL() {
    glClearColor(0.0, 0.0, 0.0, 0.0); // Black background
    gluOrtho2D(-2000.0, 2000.0, -2000.0, 2000.0);
}

void reshape(int width, int height) {
    // 新しいウィンドウサイズにビューポートを設定
    glViewport(0, 0, width, height);

    // 投影行列の設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // ウィンドウサイズに基づいて拡大するための設定
    double maxDistance = 10000.0*zoomScale; // 最大表示距離;
    double aspectRatio = (double)width / (double)height;

    if (width >= height) {
        gluOrtho2D(-maxDistance * aspectRatio, maxDistance * aspectRatio, -maxDistance, maxDistance);
    } else {
        gluOrtho2D(-maxDistance, maxDistance, -maxDistance / aspectRatio, maxDistance / aspectRatio);
    }
    // モデルビューマトリックスモードに戻る
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1.0, 1.0, 1.0); // White color
    glPointSize(5.0);
    updateData();

    glBegin(GL_POINTS);
    for (int i = 0; i < DISPLAYBUFFSIZE; i++) {
        double angle_radians = display_polardata_buff[i].angle * PI / 180.0;
        double x = display_polardata_buff[i].distance * cos(angle_radians)*zoomScale;
        double y = display_polardata_buff[i].distance * sin(angle_radians)*zoomScale;
        glVertex2d(x, y);
    }
    glEnd();
    glColor3f(1.0, 0.0, 0.0); // 赤色を設定
    glBegin(GL_POINTS);
    glVertex2d(0.0, 0.0); // 0,0 の位置に点を描画
    glEnd();
    glutSwapBuffers();
    // 画面を再描画
    glutPostRedisplay();
}

void keyboard(unsigned char key,int x,int y){
    switch(key){
        case '\033': /* '\033' は ESC の ASCII コード*/
            exit(0);
        case 'a':
            zoomScale*=1.2;
            break;
        case 'd':
            zoomScale/=1.2;
            break;
        default:
            break;
    }
}

/**
 * @brief ディスプレイ表示する点のバッファをFIFOするための関数
 * @param comingdata SerialPortから読み取った最新の値
 * @param display_polardata_buff ディスプレイに表示するためのバッファ
 */
void queue(POLAR_DATA* comingdata,POLAR_DATA* display_polardata_buff){
    if(que_en>=DISPLAYBUFFSIZE){
        for (int i=0;i<DISPLAYBUFFSIZE-16;i++){
            display_polardata_buff[i]=display_polardata_buff[i+16];
        }
        for (int i=0;i<16;i++) display_polardata_buff[DISPLAYBUFFSIZE-16+i]=comingdata[i];
    }else{
        for (int i=0;i<16;i++){            
            display_polardata_buff[que_en]=comingdata[i];
            que_en++;      
        }
    }
}

/**
 * @brief ディスプレイ表示する点の情報を更新，ディスプレイ用のバッファを更新
 */
void updateData() {
    int num_bytes = getPayload(serial_port, &payload, 100);

    if (num_bytes == 56) {
        extractData(&payload, &rawdata);
        convertToPolarMap(&rawdata, polardata);
    }
    queue(polardata,display_polardata_buff);
}

