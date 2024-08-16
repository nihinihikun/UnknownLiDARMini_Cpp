// main.cpp
#include "types.h"
#include "lidar.h"

#include <cmath>
#include <GL/glut.h>//graphic関係
#include <unistd.h>//close(),serialport関係で利用
#include <termios.h>//serialport関係で利用

#define DISPLAYBUFFSIZE 1024

LIDARPAYLOAD payload;
RAWDATA rawdata;
POLAR_DATA polardata[16];
int serial_port;

POLAR_DATA display_polardata_buff[DISPLAYBUFFSIZE];
int que_en=0;

//グラフィック関係
void keyboard(unsigned char key,int x,int y);
void updateData();
void initOpenGL();
void reshape(int width, int height);
static void display(void);
void queue(POLAR_DATA* comingdata,POLAR_DATA* display_polardata_buff);

int main(int argc,char **argv) {
    serial_port = serialBegin("/dev/ttyUSB0", B230400);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(100, 100);
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
    gluOrtho2D(-2000.0, 2000.0, -2000.0, 2000.0); // Set the coordinate system
}

void reshape(int width, int height) {
    // 新しいウィンドウサイズにビューポートを設定
    glViewport(0, 0, width, height);

    // 投影行列の設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // ウィンドウサイズに基づいて拡大するための設定
    double maxDistance = 10000.0; // 最大距離
    double aspectRatio = (double)width / (double)height;

    if (width >= height) {
        // 横長の場合
        gluOrtho2D(-maxDistance * aspectRatio, maxDistance * aspectRatio, -maxDistance, maxDistance);
    } else {
        // 縦長の場合
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
        double x = display_polardata_buff[i].distance * cos(angle_radians);
        double y = display_polardata_buff[i].distance * sin(angle_radians);
        glVertex2d(x, y);
    }
    glEnd();
    glutSwapBuffers();
    // 画面を再描画
    glutPostRedisplay();
}

void keyboard(unsigned char key,int x,int y){
    switch(key){
        case '\033': /* '\033' は ESC の ASCII コード*/
            exit(0);
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

