// lidar.h
#pragma once
#include "types.h"
#include <string>

int serialBegin(const std::string &path, int baudrate);
int getPayload(int serial_port, LIDARPAYLOAD* payload, int timeout_cnt);
int extractData(LIDARPAYLOAD* payload, RAWDATA* rawdata);
int convertToPolarMap(RAWDATA* rawdata, POLAR_DATA* polardata);
