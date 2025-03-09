//
// Created by Administrator on 2024/1/19.
//

#ifndef RTTHREAD_REFEREE_TASK_H
#define RTTHREAD_REFEREE_TASK_H

#include "drv_common.h"
#include "string.h"
#include "stdint.h"
#include "rm_task.h"

#define HEADER_SOF 0xA5
#define Agreement_RX_BUF_NUM 512        //DMA要传输的数据项数目NDTR寄存器填充值 200，即收200字节后自动填充并转换缓冲数组

#define FIFO_BUF_LENGTH     1024
#ifndef SETINGS_REFEREE_SYSTEM_H
#define SETINGS_REFEREE_SYSTEM_H
#endif
#define REF_PROTOCOL_HEADER_SIZE            sizeof(referee_data_header_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

void referee_thread_entry(void *argument);

/**
 * @brief 裁判系统接收初始化
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
/**
 * @brief 裁判系统接收数据帧解包
 */
void Referee_Data_Unpack();

/**
 * @brief 裁判系统数据更新并保存
 */
void Referee_Data_Solve(uint8_t* referee_data_frame);
void referee_UI_task_init(void);
void referee_control_task(void);
void Referee_Data_Solve(uint8_t* frame);

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;
static struct referee_msg referee_data;




#endif

