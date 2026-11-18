/**
 * @file attitude.cpp
 * @brief Complementary filter for attitude estimation using IMU data
 */
#include "attitude.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cassert>