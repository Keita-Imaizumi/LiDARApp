/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIDAR_H
#define LIDAR_H

/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <fstream>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
using namespace std;


/* Exported functions --------------------------------------------------------*/
int InitLivoxSDK(const std::string& path);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/