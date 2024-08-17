// types.h
#pragma once
#include <cstdint>

constexpr double PI = 3.14159265358979323846;

typedef struct {
  uint16_t rotation_speed;
  uint16_t angle_begin;
  uint16_t distance_0;
  uint8_t reserved_0;
  uint16_t distance_1;
  uint8_t reserved_1;
  uint16_t distance_2;
  uint8_t reserved_2;
  uint16_t distance_3;
  uint8_t reserved_3;
  uint16_t distance_4;
  uint8_t reserved_4;
  uint16_t distance_5;
  uint8_t reserved_5;
  uint16_t distance_6;
  uint8_t reserved_6;
  uint16_t distance_7;
  uint8_t reserved_7;
  uint16_t distance_8;
  uint8_t reserved_8;
  uint16_t distance_9;
  uint8_t reserved_9;
  uint16_t distance_10;
  uint8_t reserved_10;
  uint16_t distance_11;
  uint8_t reserved_11;
  uint16_t distance_12;
  uint8_t reserved_12;
  uint16_t distance_13;
  uint8_t reserved_13;
  uint16_t distance_14;
  uint8_t reserved_14;
  uint16_t distance_15;
  uint8_t reserved_15;
  uint16_t angle_end;
  uint16_t crc;
} __attribute__((packed)) LIDARPAYLOAD;

typedef struct {
    double angle_begin;
    double angle_end;
    double rotation_speed;
    uint16_t distance[16];
} RAWDATA;

typedef struct {
    double angle;
    double distance;
} POLAR_DATA;

typedef struct {
    double x;
    double y;
} XY_DATA;
