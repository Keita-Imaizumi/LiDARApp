/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIDAR_H
#define LIDAR_H

/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <fstream>


#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
using namespace std;


/* Exported functions --------------------------------------------------------*/
int InitLivoxSDK(const std::string& path);

int countFilteredPoints(LivoxLidarCartesianHighRawPoint* p_point_data, uint32_t dot_num, 
                        int32_t x_min, int32_t x_max, 
                        int32_t y_min, int32_t y_max, 
                        int32_t z_min, int32_t z_max);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/