
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>


#include "dji_low_speed_data_channel.h"
#include "dji_high_speed_data_channel.h"
#include <dji_platform.h>
#include <dji_logger.h>
#include <dji_core.h>
#include <dji_aircraft_info.h>
#include <csignal>
#include "dji_low_speed_data_channel.h"
#include "dji_error.h"
#include "dji_config_manager.h"

#include "osal.h"
#include "osal_fs.h"
#include "osal_socket.h"
#include "hal_usb_bulk.h"
#include "hal_uart.h"
#include "hal_network.h"

#define USER_APP_NAME               "PtoMApp"
#define USER_APP_ID                 "148071"
#define USER_APP_KEY                "a0a2faa45fb2a58f04697df9c504ca8"
#define USER_APP_LICENSE            "UXfD35GOhQMia6IHYQ0kVZaQA0B2qQtbNIx/GAockId92kaL/8rG4rK0DlSJBHuji0VoGMOdI5U8oXLy6+MX0HanEYbHwRyXdHisF3q5KpQd51tQVCjC9CHbkmvVwrgY0511yXgpuUDU3G9RXf6ssoM/3d4svst/kk25ZkjM+zYrqp8/ZkXvk2Q2NPgIxfC3SvDIcn5oVFi3ZfRZpuITq8X3nKw1b9YNe6IszDY8SVFNZyN9QEjexqUq0j6idTse2QyrGfPhDgXWNW2ZGe/z32i6vqAlPjvn9E0eBGmGYpal7fnM3Zp2ZtgX4WcC5Ml5C+YNbrUoxz1AfOq3PxrtDg=="
#define USER_DEVELOPER_ACCOUNT      "kit.imaizumi.ibm@gmail.com"
#define USER_BAUD_RATE              "115200"

#define USER_UTIL_UNUSED(x)                                 ((x) = (x))
#define USER_UTIL_MIN(a, b)                                 (((a) < (b)) ? (a) : (b))
#define USER_UTIL_MAX(a, b)                                 (((a) > (b)) ? (a) : (b))   

#define DATA_TRANSMISSION_TASK_FREQ         (1)
#define DATA_TRANSMISSION_TASK_STACK_SIZE   (2048)


static void *UserDataTransmission_Task(void *arg);

static T_DjiTaskHandle s_userDataTransmissionThread;
static T_DjiAircraftInfoBaseInfo s_aircraftInfoBaseInfo;

bool stopSignal;


void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
		LivoxLidarEthernetPacket *data, void *client_data) {
	E_DjiChannelAddress channelAddress;
	T_DjiReturnCode djiStat;
	const uint8_t stopMessage[] = "Stop";
	if (data == nullptr) {
		return;
	}
	printf(
			"point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
			handle, data->dot_num, data->data_type, data->length,
			data->frame_cnt);
	if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
		LivoxLidarCartesianHighRawPoint *p_point_data =
				(LivoxLidarCartesianHighRawPoint*) data->data;
	   std::vector<std::vector<std::vector<int>>> coordinateArray;
		for (uint32_t i = 0; i < data->dot_num; i++) {
//			printf("x:%d,y:%d, z:%d\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
			//distance:[mm]
			float distance = std::sqrt(p_point_data[i].x * p_point_data[i].x +
					p_point_data[i].y* p_point_data[i].y +
					p_point_data[i].z * p_point_data[i].z);
			if (distance < 500 && distance >100){
				channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
				coordinateArray.push_back({{p_point_data[i].x, p_point_data[i].y, p_point_data[i].z}});
				printf("x:%d,y:%d, z:%d\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
				djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, stopMessage, sizeof(stopMessage));
				if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("Failed to send 'Stop' message.");
				}

			}
		}
		printf("datanum: %zu\n", coordinateArray.size());
	} else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
		LivoxLidarCartesianLowRawPoint *p_point_data =
				(LivoxLidarCartesianLowRawPoint*) data->data;
	} else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
		LivoxLidarSpherPoint *p_point_data = (LivoxLidarSpherPoint*) data->data;
	}
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,
		LivoxLidarEthernetPacket *data, void *client_data) {
	if (data == nullptr) {
		return;
	}
	printf(
			"Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
			handle, data->dot_num, data->data_type, data->length,
			data->frame_cnt);
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

void DjiUser_SetupEnvironment(int argc, char **argv)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler osalHandler = {0};
    T_DjiHalUartHandler uartHandler = {0};
    T_DjiHalUsbBulkHandler usbBulkHandler = {0};
    T_DjiLoggerConsole printConsole;
    T_DjiLoggerConsole localRecordConsole;
    T_DjiFileSystemHandler fileSystemHandler = {0};
    T_DjiSocketHandler socketHandler{0};
    T_DjiHalNetworkHandler networkHandler = {0};
    T_DjiUserLinkConfig linkConfig;

    networkHandler.NetworkInit = HalNetWork_Init;
    networkHandler.NetworkDeInit = HalNetWork_DeInit;
    networkHandler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

    socketHandler.Socket = Osal_Socket;
    socketHandler.Bind = Osal_Bind;
    socketHandler.Close = Osal_Close;
    socketHandler.UdpSendData = Osal_UdpSendData;
    socketHandler.UdpRecvData = Osal_UdpRecvData;
    socketHandler.TcpListen = Osal_TcpListen;
    socketHandler.TcpAccept = Osal_TcpAccept;
    socketHandler.TcpConnect = Osal_TcpConnect;
    socketHandler.TcpSendData = Osal_TcpSendData;
    socketHandler.TcpRecvData = Osal_TcpRecvData;

    osalHandler.TaskCreate = Osal_TaskCreate;
    osalHandler.TaskDestroy = Osal_TaskDestroy;
    osalHandler.TaskSleepMs = Osal_TaskSleepMs;
    osalHandler.MutexCreate = Osal_MutexCreate;
    osalHandler.MutexDestroy = Osal_MutexDestroy;
    osalHandler.MutexLock = Osal_MutexLock;
    osalHandler.MutexUnlock = Osal_MutexUnlock;
    osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
    osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
    osalHandler.SemaphoreWait = Osal_SemaphoreWait;
    osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
    osalHandler.SemaphorePost = Osal_SemaphorePost;
    osalHandler.Malloc = Osal_Malloc;
    osalHandler.Free = Osal_Free;
    osalHandler.GetTimeMs = Osal_GetTimeMs;
    osalHandler.GetTimeUs = Osal_GetTimeUs;
    osalHandler.GetRandomNum = Osal_GetRandomNum;

    // printConsole.func = DjiUser_PrintConsole;
    printConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
    printConsole.isSupportColor = true;

    localRecordConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG;
    // localRecordConsole.func = DjiUser_LocalWrite;
    localRecordConsole.isSupportColor = false;

    uartHandler.UartInit = HalUart_Init;
    uartHandler.UartDeInit = HalUart_DeInit;
    uartHandler.UartWriteData = HalUart_WriteData;
    uartHandler.UartReadData = HalUart_ReadData;
    uartHandler.UartGetStatus = HalUart_GetStatus;

    usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
    usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
    usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
    usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
    usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

    fileSystemHandler.FileOpen = Osal_FileOpen,
    fileSystemHandler.FileClose = Osal_FileClose,
    fileSystemHandler.FileWrite = Osal_FileWrite,
    fileSystemHandler.FileRead = Osal_FileRead,
    fileSystemHandler.FileSync = Osal_FileSync,
    fileSystemHandler.FileSeek = Osal_FileSeek,
    fileSystemHandler.DirOpen = Osal_DirOpen,
    fileSystemHandler.DirClose = Osal_DirClose,
    fileSystemHandler.DirRead = Osal_DirRead,
    fileSystemHandler.Mkdir = Osal_Mkdir,
    fileSystemHandler.Unlink = Osal_Unlink,
    fileSystemHandler.Rename = Osal_Rename,
    fileSystemHandler.Stat = Osal_Stat,

    returnCode = DjiPlatform_RegOsalHandler(&osalHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register osal handler error.");
    }

    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal uart handler error.");
    }

#if DJI_USE_SDK_CONFIG_BY_JSON
    if (argc > 1) {
        DjiUserConfigManager_LoadConfiguration(argv[1]);
    } else {
        DjiUserConfigManager_LoadConfiguration(nullptr);
    }

    DjiUserConfigManager_GetLinkConfig(&linkConfig);
    if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_NETWORK_DEVICE) {
        returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            throw std::runtime_error("Register hal network handler error");
        }
    } else if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_USB_BULK_DEVICE) {
        returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            throw std::runtime_error("Register hal usb bulk handler error.");
        }
    } else {
        /*!< Attention: Only use uart hardware connection. */
    }
#else
#if (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
    returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal usb bulk handler error.");
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
    returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal network handler error");
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
    /*!< Attention: Only use uart hardware connection.
     */
#endif
#endif
}

T_DjiReturnCode DjiUser_FillInUserInfo(T_DjiUserInfo *userInfo)
{
    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->appLicense, 0, sizeof(userInfo->appLicense));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));
    memset(userInfo->baudRate, 0, sizeof(userInfo->baudRate));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) > sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount) ||
        strlen(USER_BAUD_RATE) > sizeof(userInfo->baudRate)) {
        USER_LOG_ERROR("Length of user information string is beyond limit. Please check.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    if (!strcmp(USER_APP_NAME, "your_app_name") ||
        !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") ||
        !strcmp(USER_BAUD_RATE, "your_app_license") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account") ||
        !strcmp(USER_BAUD_RATE, "your_baud_rate")) {
        USER_LOG_ERROR(
            "Please fill in correct user information to 'samples/sample_c++/platform/linux/manifold2/application/dji_sdk_app_info.h' file.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, USER_UTIL_MIN(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, USER_UTIL_MIN(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    memcpy(userInfo->appLicense, USER_APP_LICENSE,
           USER_UTIL_MIN(sizeof(userInfo->appLicense), strlen(USER_APP_LICENSE)));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, USER_UTIL_MIN(sizeof(userInfo->baudRate), strlen(USER_BAUD_RATE)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void DjiUser_NormalExitHandler(int signalNum)
{
    USER_UTIL_UNUSED(signalNum);
    exit(0);
}

void DjiUser_InitApplicationStart(){
	T_DjiUserInfo userInfo;
    T_DjiReturnCode returnCode;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    T_DjiFirmwareVersion firmwareVersion = {
        .majorVersion = 1,
        .minorVersion = 0,
        .modifyVersion = 0,
        .debugVersion = 0,
    };

    // attention: when the program is hand up ctrl-c will generate the coredump file
    signal(SIGTERM, DjiUser_NormalExitHandler);

#if DJI_USE_SDK_CONFIG_BY_JSON
    DjiUserConfigManager_GetAppInfo(&userInfo);
#else
    returnCode = DjiUser_FillInUserInfo(&userInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Fill user info error, please check user info config.");
    }
#endif

    returnCode = DjiCore_Init(&userInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        sleep(1);
        throw std::runtime_error("Core init error.");
    }

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Get aircraft base info error.");
    }

    if (aircraftInfoBaseInfo.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT
        && DJI_MOUNT_POSITION_EXTENSION_LITE_PORT != aircraftInfoBaseInfo.mountPosition) {
        throw std::runtime_error("Please run this sample on extension port.");
    }

    returnCode = DjiCore_SetAlias("PSDK_APPALIAS");
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set alias error.");
    }

    returnCode = DjiCore_SetFirmwareVersion(firmwareVersion);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set firmware version error.");
    }

    returnCode = DjiCore_SetSerialNumber("PSDK12345678XX");
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set serial number error");
    }

}

T_DjiReturnCode startDataTransmission(){
	static T_DjiAircraftInfoBaseInfo s_aircraftInfoBaseInfo;
	T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiChannelAddress channelAddress;
    const T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidthProportionOfHighspeedChannel =
        {10, 60, 30};
    char ipAddr[DJI_IP_ADDR_STR_SIZE_MAX];
    uint16_t port;
    

    djiStat = DjiLowSpeedDataChannel_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data transmission module error. Error is %llu", djiStat);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiAircraftInfo_GetBaseInfo(&s_aircraftInfoBaseInfo);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
	 if (osalHandler->TaskCreate("user_transmission_task", UserDataTransmission_Task,
                                DATA_TRANSMISSION_TASK_STACK_SIZE, NULL, &s_userDataTransmissionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data transmission task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}

static void *UserDataTransmission_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    const uint8_t dataToBeSent[] = "DJI Data Transmission Test Data.";
    T_DjiDataChannelState state = {0};
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiChannelAddress channelAddress;

    USER_UTIL_UNUSED(arg);

    while (1) {
        osalHandler->TaskSleepMs(1000 / DATA_TRANSMISSION_TASK_FREQ);

        channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
        if (stopSignal) {
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            printf("sending");
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to mobile error.");

        }
        printf("stopSignal: %d\n", stopSignal);  
        
        djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_DEBUG(
                "send to mobile state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                state.busyState);
        } else {
            USER_LOG_ERROR("get send to mobile channel state error.");
        }

        if (s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T) {
            channelAddress = DJI_CHANNEL_ADDRESS_CLOUD_API;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to cloud error.");

            djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
            if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_DEBUG(
                    "send to cloud state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                    state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                    state.busyState);
            } else {
                USER_LOG_ERROR("get send to cloud channel state error.");
            }
        }

        if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 ||
            s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 ||
            s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3) {
            channelAddress = DJI_CHANNEL_ADDRESS_EXTENSION_PORT;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to extension port error.");

            djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
            if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_DEBUG(
                    "send to extension port state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                    state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                    state.busyState);
            } else {
                USER_LOG_ERROR("get send to extension port channel state error.");
            }

            if (DjiPlatform_GetSocketHandler() != NULL) {
#ifdef SYSTEM_ARCH_LINUX
                djiStat = DjiHighSpeedDataChannel_SendDataStreamData(dataToBeSent, sizeof(dataToBeSent));
                if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                    USER_LOG_ERROR("send data to data stream error.");

                djiStat = DjiHighSpeedDataChannel_GetDataStreamState(&state);
                if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_DEBUG(
                        "data stream state: realtimeBandwidthLimit: %d, realtimeBandwidthBeforeFlowController: %d, busyState: %d.",
                        state.realtimeBandwidthLimit, state.realtimeBandwidthBeforeFlowController, state.busyState);
                } else {
                    USER_LOG_ERROR("get data stream state error.");
                }
#endif
            }
        } else if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_EXTENSION_PORT
                    || DJI_MOUNT_POSITION_EXTENSION_LITE_PORT == s_aircraftInfoBaseInfo.mountPosition) {
            channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to extension port error.");

            djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
            if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_DEBUG(
                    "send to extension port state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                    state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                    state.busyState);
            } else {
                USER_LOG_ERROR("get send to extension port channel state error.");
            }
        }
    }
}


int main(int argc, char **argv) {
	if (argc != 2) {
		printf("Params Invalid, must input config path.\n");
		return -1;
	}
	const std::string path = argv[1];

	// REQUIRED, to init Livox SDK2
	if (!LivoxLidarSdkInit(path.c_str())) {
		printf("Livox Init Failed\n");
		LivoxLidarSdkUninit();
		return -1;
	}
	//set up enviroment and init PSDK
	DjiUser_SetupEnvironment(argc, argv);
	DjiUser_InitApplicationStart();

	startDataTransmission();
	


	
	// REQUIRED, to get point cloud data via 'PointCloudCallback'
	SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

	// OPTIONAL, to get imu data via 'ImuDataCallback'
	// some lidar types DO NOT contain an imu component
	SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);

	SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);

	// REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
	SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

#ifdef WIN32
  Sleep(300000);
#else
	sleep(300);
#endif
	LivoxLidarSdkUninit();
	printf("Livox Quick Start Demo End!\n");
	return 0;
}
