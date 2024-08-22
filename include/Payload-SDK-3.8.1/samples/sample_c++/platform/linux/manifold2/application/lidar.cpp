
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif
#include "lidar.hpp"
#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include "data_transmission/test_data_transmission.h"

extern bool stopSignal;
extern int data_num;
void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
		LivoxLidarEthernetPacket *data, void *client_data) {
	if (data == nullptr) {
		return;
	}
	// printf(
	// 		"point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
	// 		handle, data->dot_num, data->data_type, data->length,
	// 		data->frame_cnt);
	if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
		LivoxLidarCartesianHighRawPoint *p_point_data =
				(LivoxLidarCartesianHighRawPoint*) data->data;
	   	std::vector<std::vector<std::vector<int>>> coordinateArray;
		//点群フィルタリング(mm)
		data_num = countFilteredPoints(p_point_data, data->dot_num, 
			0, 1000, 
			-40, 40,
			-40, 40
			);
		// for (uint32_t i = 0; i < data->dot_num; i++) {
		// 	printf("x:%d,y:%d, z:%d\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
		// 	//distance:[mm]
		// 	float distance = std::sqrt(p_point_data[i].x * p_point_data[i].x +
		// 			p_point_data[i].y* p_point_data[i].y +
		// 			p_point_data[i].z * p_point_data[i].z);
		// 	if (distance < 500 && distance >100){
		// 		coordinateArray.push_back({{p_point_data[i].x, p_point_data[i].y, p_point_data[i].z}});
        //         // 条件を満たした場合、stopSignalをtrueに設定
		// 		stopSignal = true;
		// 	}
        //     else {
        //         stopSignal = false;
        //     }
		// }
		//printf("datanum: %zu\n", coordinateArray.size());
	} else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
		LivoxLidarCartesianLowRawPoint *p_point_data =
				(LivoxLidarCartesianLowRawPoint*) data->data;
	} else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
		LivoxLidarSpherPoint *p_point_data = (LivoxLidarSpherPoint*) data->data;
	}
}
int countFilteredPoints(LivoxLidarCartesianHighRawPoint* p_point_data, uint32_t dot_num, 
                        int32_t x_min, int32_t x_max, 
                        int32_t y_min, int32_t y_max, 
                        int32_t z_min, int32_t z_max) {
    int count = 0;

    for (uint32_t i = 0; i < dot_num; i++) {
        // x, y, z の条件をチェック
        if (p_point_data[i].x >= x_min && p_point_data[i].x <= x_max &&
            p_point_data[i].y >= y_min && p_point_data[i].y <= y_max &&
            p_point_data[i].z >= z_min && p_point_data[i].z <= z_max) {
            count++;
        }
    }

    return count;
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,
		LivoxLidarEthernetPacket *data, void *client_data) {
	if (data == nullptr) {
		return;
	}
	// printf(
	// 		"Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
	// 		handle, data->dot_num, data->data_type, data->length,
	// 		data->frame_cnt);
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void WorkModeCallback(livox_status status, uint32_t handle,
		LivoxLidarAsyncControlResponse *response, void *client_data) {
	if (response == nullptr) {
		return;
	}
	printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
			status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle,
		LivoxLidarRebootResponse *response, void *client_data) {
	if (response == nullptr) {
		return;
	}
	printf("RebootCallback, status:%u, handle:%u, ret_code:%u", status, handle,
			response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle,
		LivoxLidarAsyncControlResponse *response, void *client_data) {
	if (response == nullptr) {
		return;
	}
	printf(
			"LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
			status, handle, response->ret_code, response->error_key);

	if (response->ret_code == 0 && response->error_key == 0) {
		LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
	}
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle,
		LivoxLidarDiagInternalInfoResponse *response, void *client_data) {
	if (status != kLivoxLidarStatusSuccess) {
		printf("Query lidar internal info failed.\n");
		QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
		return;
	}

	if (response == nullptr) {
		return;
	}

	uint8_t host_point_ipaddr[4] { 0 };
	uint16_t host_point_port = 0;
	uint16_t lidar_point_port = 0;

	uint8_t host_imu_ipaddr[4] { 0 };
	uint16_t host_imu_data_port = 0;
	uint16_t lidar_imu_data_port = 0;

	uint16_t off = 0;
	for (uint8_t i = 0; i < response->param_num; ++i) {
		LivoxLidarKeyValueParam *kv =
				(LivoxLidarKeyValueParam*) &response->data[off];
		if (kv->key == kKeyLidarPointDataHostIpCfg) {
			memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
			memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
			memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
		} else if (kv->key == kKeyLidarImuHostIpCfg) {
			memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
			memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
			memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
		}
		off += sizeof(uint16_t) * 2;
		off += kv->length;
	}

	printf(
			"Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
			host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2],
			host_point_ipaddr[3], host_point_port, lidar_point_port);

	printf(
			"Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
			host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2],
			host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info,
		void *client_data) {
	if (info == nullptr) {
		printf("lidar info change callback failed, the info is nullptr.\n");
		return;
	}
	printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle,
			info->sn);

	// set the work mode to kLivoxLidarNormal, namely start the lidar
	SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

	QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

	// LivoxLidarIpInfo lidar_ip_info;
	// strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
	// strcpy(lidar_ip_info.net_mask, "255.255.255.0");
	// strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
	// SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type,
		const char *info, void *client_data) {
	struct in_addr tmp_addr;
	tmp_addr.s_addr = handle;
	std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr)
			<< ", push msg info: " << std::endl;
	std::cout << info << std::endl;
	return;
}

int InitLivoxSDK(const std::string& path) {
	if (!LivoxLidarSdkInit(path.c_str())) {
		printf("Livox Init Failed\n");
		printf(path.c_str());
		LivoxLidarSdkUninit();
		return -1;
	}
	// REQUIRED, to get point cloud data via 'PointCloudCallback'
	SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

	// OPTIONAL, to get imu data via 'ImuDataCallback'
	// some lidar types DO NOT contain an imu component
	SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);

	SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);

	// REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
	SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

// #ifdef WIN32
//   Sleep(300000);
// #else
// 	sleep(300);
// #endif
// 	LivoxLidarSdkUninit();
 	printf("set callback!\n");
	return 0;
}
