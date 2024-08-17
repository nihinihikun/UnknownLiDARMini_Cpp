// lidar.cpp
#include "lidar.h"
#include <string.h>
#include <fcntl.h>//serialport関連で利用
#include <errno.h>
#include <termios.h>//serialport関係で利用
#include <unistd.h>//close(),serialport関係で利用

/**
 * @brief シリアルポートの設定
 * @param path シリアルポートのパス
 * @param baudrate baudrateの設定
 * @return 0,-1 0:Sucess,-1:Error
 */
int serialBegin(const std::string &path, int baudrate) {
    int serial_port = open(path.c_str(), O_RDWR); 
    
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;  // エラーコードを正しく -1 に設定
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag &= ~PARENB; // クリアパリティビット
    tty.c_cflag &= ~CSTOPB; // 1ストップビット
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8ビット
    tty.c_cflag &= ~CRTSCTS; // ハードウェアフロー制御無効
    tty.c_cflag |= CREAD | CLOCAL; // レシーバーを有効、ローカルモード

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // エコー無効
    tty.c_lflag &= ~ECHOE; // エラー時のエコー無効
    tty.c_lflag &= ~ISIG; // INTR, QUIT, SUSP, DSUSP シグナル無効

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // ソフトウェアフロー制御無効
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 特殊入力処理無効

    tty.c_oflag &= ~OPOST; // 特殊出力処理無効
    tty.c_oflag &= ~ONLCR; // NL to CR/NL 無効

    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    return serial_port;
}

/**
 * @brief シリアルポートからデータを読み取ってpayloadを抽出する
 * @param serial_port シリアルポート
 * @param payload packetからheadderを取り除いたデータ群
 * @param timeout_cnt タイムアウト用
 * @return 読み取ったデータサイズ(Byte)
 */
int getPayload(int serial_port, LIDARPAYLOAD* payload, int timeout_cnt) {
    //header(4) payload(56) header(4)
    //ヘッダ検出用バッファ
    unsigned char header[4] = {0x55, 0xAA, 0x23, 0x10};
    unsigned char tmp_buff;
    int tmp_buff_length;

    static uint8_t payload_buff[56];

    // ヘッダの検出・ペイロードの読み取り
    for (int attempt = 0; attempt < timeout_cnt; ++attempt) {
        for (int j = 0; j < 4; ++j) {
            tmp_buff_length = read(serial_port, &tmp_buff, 1);
            if (tmp_buff_length != 1 || tmp_buff != header[j]) {
                break;  // ヘッダが一致しなければループを出る
            }
            if (j == 3) {
                // ペイロードの読み取り
                int read_len = read(serial_port, payload_buff, 56);

                #ifdef DEBUG
                printf("payload_buff:");
                for (int i=0;i<56;i++)printf("%02X ",payload_buff[i]);
                printf("\n");
                #endif

                if(read_len==0)return 0;    
                memcpy(payload, payload_buff, sizeof(LIDARPAYLOAD));          
                
                // 末尾のヘッダを捨てる
                char discard_header[4]={0};
                read(serial_port, discard_header, sizeof(discard_header));
                return read_len;
            }
        }
    }
    return 0; // タイムアウトによりヘッダが見つからなかった場合
}

/**
 * @brief payloadからデータをまとめる
 * @param payload packetからheadderを取り除いたデータ群
 * @param polardata payloadからまとめたデータ
 * @return 0
 */
int extractData(LIDARPAYLOAD* payload, RAWDATA* rawdata) {
    rawdata->rotation_speed = payload->rotation_speed / 64;
    rawdata->angle_begin = (payload->angle_begin - 40960) / 64;
    rawdata->angle_end = (payload->angle_end - 40960) / 64;
    
    rawdata->distance[0]=payload->distance_0& 0x3FFF;
    rawdata->distance[1]=payload->distance_1& 0x3FFF;
    rawdata->distance[2] = payload->distance_2& 0x3FFF;
    rawdata->distance[3] = payload->distance_3& 0x3FFF;
    rawdata->distance[4] = payload->distance_4& 0x3FFF;
    rawdata->distance[5] = payload->distance_5& 0x3FFF;
    rawdata->distance[6] = payload->distance_6& 0x3FFF;
    rawdata->distance[7] = payload->distance_7& 0x3FFF;
    rawdata->distance[8] = payload->distance_8& 0x3FFF;
    rawdata->distance[9] = payload->distance_9& 0x3FFF;
    rawdata->distance[10] = payload->distance_10& 0x3FFF;
    rawdata->distance[11] = payload->distance_11& 0x3FFF;
    rawdata->distance[12] = payload->distance_12& 0x3FFF;
    rawdata->distance[13] = payload->distance_13& 0x3FFF;
    rawdata->distance[14] = payload->distance_14& 0x3FFF;
    rawdata->distance[15] = payload->distance_15& 0x3FFF;

    return 0;
}

/**
 * @brief payloadからまとめたデータを極座標に変換
 * @param rawdata payloadからまとめたデータ
 * @param polardata 極座標
 * @return 0
 */
int convertToPolarMap(RAWDATA* rawdata,POLAR_DATA* polardata){
    double deltaangle=rawdata->angle_end-rawdata->angle_begin;
    if(deltaangle<0)deltaangle+=360;
    deltaangle/=15;
    for (int i=0;i<16;i++){
        polardata[i].angle=static_cast<double>(rawdata->angle_begin+deltaangle*i);
        polardata[i].distance=static_cast<double>(rawdata->distance[i]);
    }
    #ifdef DEBUG
    printPolarData(polardata,16);
    #endif
    return 0;
}

/**
 * @brief デバック用
 * @param payload
 */
int printPayload(LIDARPAYLOAD* payload) {
    printf("Payload Data:\n");
    printf("Rotation Speed: %u\n", payload->rotation_speed);
    printf("Angle Begin: %u\n", payload->angle_begin);
    printf("Distances:\n");
    printf(" Distance 0: %u\n", payload->distance_0 & 0x3FFF);
    printf(" Distance 1: %u\n", payload->distance_1 & 0x3FFF);
    printf(" Distance 2: %u\n", payload->distance_2 & 0x3FFF);
    printf(" Distance 3: %u\n", payload->distance_3 & 0x3FFF);
    printf(" Distance 4: %u\n", payload->distance_4 & 0x3FFF);
    printf(" Distance 5: %u\n", payload->distance_5 & 0x3FFF);
    printf(" Distance 6: %u\n", payload->distance_6 & 0x3FFF);
    printf(" Distance 7: %u\n", payload->distance_7 & 0x3FFF);
    printf(" Distance 8: %u\n", payload->distance_8 & 0x3FFF);
    printf(" Distance 9: %u\n", payload->distance_9 & 0x3FFF);
    printf(" Distance 10: %u\n", payload->distance_10 & 0x3FFF);
    printf(" Distance 11: %u\n", payload->distance_11 & 0x3FFF);
    printf(" Distance 12: %u\n", payload->distance_12 & 0x3FFF);
    printf(" Distance 13: %u\n", payload->distance_13 & 0x3FFF);
    printf(" Distance 14: %u\n", payload->distance_14 & 0x3FFF);
    printf(" Distance 15: %u\n", payload->distance_15 & 0x3FFF);
    printf("Angle End: %u\n", payload->angle_end);
    printf("CRC: %u\n", payload->crc);

    return 0;
}

/**
 * @brief デバック用
 * @param polardata
 * @param size
 */
int printPolarData(POLAR_DATA* polardata, int size) {
    // printf("Polar Data:\n");
    // for (int i = 0; i < size; i++) {
    //     printf("  Angle: %5.2f degrees, Distance: %5.2f units\n", polardata[i].angle, polardata[i].distance);
    // }
    // return 0;
    for (int i = 0; i < size; i++) {
        printf("[%5.2f,%5.2f]", polardata[i].angle, polardata[i].distance);
    }
    printf("\n");
    return 0;
}
