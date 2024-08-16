/**
 ********************************************************************
 * @file    osdk_policy.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   Define the osdk policy binary array
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */


#ifndef _OSDK_POLICY_H_
#define _OSDK_POLICY_H_

#include <stdint.h>

#define OSDK_IDENTITY_VERIFY_CALMD5_MAX_NUM          (33 + 17)
#define OSDK_POLICY_FILE_VERSION_OFFSET              (256)
/*!
 * OSDK Policy
 * Creator  : DJI
 * Date     : 2020-05-07
 * OSDK Ver : OSDK 4.0
 * */

static const uint8_t s_osdkPolicyFileBinaryArray[] =
    {0x90, 0xA0, 0x50, 0x4B, 0x51, 0xB2, 0x2D, 0xBA, 0x30, 0xE2, 0x92, 0x96,
     0x9E, 0xEB, 0x27, 0x90, 0x15, 0x41, 0x97, 0x81, 0x99, 0xA2, 0x53, 0x61,
     0xC6, 0xEE, 0x3C, 0xEC, 0xAB, 0xF8, 0x70, 0x76, 0x09, 0xB0, 0x39, 0xCC,
     0x4D, 0x4B, 0x50, 0xA5, 0x4E, 0x34, 0xDE, 0xE8, 0xB6, 0xBC, 0x6C, 0x3D,
     0xD3, 0x67, 0xA7, 0xA3, 0x42, 0x77, 0xE1, 0xCC, 0x4B, 0x06, 0x93, 0x76,
     0xE0, 0xC8, 0x34, 0xF4, 0x8E, 0x8B, 0x57, 0x47, 0x1C, 0x19, 0xE0, 0xE0,
     0x0E, 0xD9, 0x42, 0xA8, 0x5A, 0xDB, 0x12, 0x89, 0xF1, 0xEF, 0xC3, 0x60,
     0xD5, 0xCD, 0x71, 0x1C, 0x82, 0x90, 0xB4, 0xAC, 0xFA, 0x93, 0x3F, 0xC9,
     0x44, 0x86, 0x2C, 0xB3, 0xB7, 0xCD, 0x9A, 0x9D, 0x78, 0xE6, 0x81, 0xE0,
     0xC1, 0x3B, 0xFE, 0xD9, 0xA9, 0x43, 0xAE, 0x3D, 0xF4, 0x49, 0xE8, 0xDC,
     0x22, 0x99, 0x49, 0x55, 0xC6, 0xE6, 0x8C, 0x67, 0x29, 0x6D, 0xF7, 0x55,
     0x41, 0xA3, 0x4F, 0xC4, 0x3E, 0xE8, 0xA5, 0x6B, 0x9A, 0xAD, 0x81, 0x99,
     0xAD, 0x14, 0x32, 0x56, 0xC6, 0x09, 0x66, 0x19, 0x85, 0x18, 0xCF, 0x5B,
     0x3E, 0xB1, 0xD8, 0x52, 0xBC, 0xF7, 0xA3, 0x0F, 0x82, 0x37, 0xA8, 0xC1,
     0x27, 0x9C, 0x69, 0x8E, 0x47, 0xCF, 0x54, 0x53, 0xDA, 0xFB, 0x0C, 0x24,
     0x3E, 0x15, 0xE2, 0xCC, 0xB8, 0xAD, 0x01, 0x12, 0x4F, 0x8A, 0xDF, 0x4A,
     0xC0, 0x7B, 0xF5, 0xD8, 0xD0, 0x49, 0x8D, 0x99, 0xF9, 0x51, 0x6E, 0x3E,
     0x62, 0x63, 0x72, 0xB0, 0x13, 0xF9, 0xC1, 0xF6, 0x59, 0x79, 0x5C, 0x93,
     0x3D, 0x84, 0x5A, 0xD3, 0x3B, 0xD0, 0x46, 0x9F, 0x89, 0x97, 0x46, 0x38,
     0x05, 0xD5, 0x01, 0x02, 0x04, 0x0F, 0x72, 0x79, 0xDD, 0x98, 0x62, 0x62,
     0xD0, 0x97, 0xAC, 0xE2, 0x68, 0x55, 0x94, 0x72, 0x04, 0x36, 0x3E, 0xEA,
     0xA6, 0x05, 0xD9, 0x38, 0x00, 0x00, 0x73, 0x00, 0x01, 0x00, 0x6F, 0x00,
     0x01, 0x00, 0x02, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x04, 0x00, 0x00, 0x05,
     0x26, 0x27, 0x02, 0x07, 0x12, 0x00, 0x01, 0x02, 0x10, 0x11, 0x1E, 0x1F,
     0x22, 0x2B, 0x2E, 0x2F, 0x30, 0x31, 0x34, 0x35, 0x6A, 0x6B, 0xC4, 0xC6,
     0x09, 0xB8, 0x80, 0x4A, 0xE1, 0xEB, 0xEC, 0x03, 0x01, 0x00, 0x00, 0x1B,
     0x04, 0x02, 0x00, 0x00, 0x0A, 0x4C, 0x05, 0x01, 0x00, 0x00, 0x0B, 0x08,
     0x01, 0x02, 0x00, 0x67, 0x68, 0x65, 0x0B, 0x01, 0x00, 0x00, 0x04, 0x0D,
     0x00, 0x02, 0x00, 0x01, 0x03, 0x21, 0x00, 0x02, 0x00, 0x05, 0x06, 0x22,
     0x01, 0x04, 0x00, 0x01, 0x0E, 0x17, 0x1E, 0x27, 0x24, 0x01, 0x02, 0x00,
     0x11, 0x12, 0x32, 0x3C, 0x02, 0x00, 0x00, 0x37, 0x40, 0x49, 0x02, 0x00,
     0x00, 0x20, 0x30};

#endif    /* _OSDK_POLICY_H_ */
