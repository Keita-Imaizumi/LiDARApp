/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <unistd.h> // for sleep function
#include <pthread.h> // for pthread_create and pthread_join
#include <dji_low_speed_data_channel.h>
#include "dji_high_speed_data_channel.h"
#include "test_mobile_entry.h"
#include "dji_core.h"
#include <dji_logger.h>
/* Private functions declaration ---------------------------------------------*/
// command state
// Function to be executed in the thread
T_DjiReturnCode DjiUser_MobileSample(void) {
    bool keepLoopRunning = true;
    const char *data = "Hello DJI";
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiChannelAddress channelAddress;
    const T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidthProportionOfHighspeedChannel =
        {10, 60, 30};
    char ipAddr[DJI_IP_ADDR_STR_SIZE_MAX];
    uint16_t port;

    djiStat = DjiLowSpeedDataChannel_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data transmission module error. Error code: %d", djiStat);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
    // Setup a thread to send data to MSDK every second
    
    
    while (keepLoopRunning)
    {   
        djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, (const uint8_t *)data, strlen(data));
        //djiStat = DjiHighSpeedDataChannel_SendDataStreamData((const uint8_t *)data, strlen(data));
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
            USER_LOG_ERROR("send data to mobile error. Error code: %d", djiStat);
        }
        else {
            USER_LOG_ERROR("sending data");
        }
        
        sleep(1); // Wait for 1 second
    }
    
    // User input
    std::cout << "Listening to mobile commands. Press any key to exit.\n";
    char a;
    std::cin >> a;

    keepLoopRunning = false;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
