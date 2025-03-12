/*
* Change Logs:
* Date            Author               Notes
* 2024-01-19     ChenSihan             1.0
* 2025-03-03     SuperChen             2.0
*/
#include "referee_task.h"
#include "drv_gpio.h"
#include "drv_msg.h"
#include "crc.h"
#include "ui_protocol.h"
#include "priority_ui.h"
#include "ui.h"
#include "fifo.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO



UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;


static fifo_s_t RX_AgreementData_FIFO;           //实例化的裁判系统接收数据FIFO容器
static uint8_t RX_FIFO_Space[FIFO_BUF_LENGTH];              //FIFO的实际存储区
static uint8_t RX_AgreementData_Buffer0[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区0,该缓冲区设置的相当富裕
static uint8_t RX_AgreementData_Buffer1[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区1，该缓冲区设置的相当富裕
//uint8_t RX_Agreement_Data[Agreement_RX_BUF_NUM];          //用来单独存放接收数据的Data段
static unpack_data_t referee_unpack_obj;
static referee_data_header_t Referee_Data_header;         //实例化一个帧头结构体
static referee_data_t     Referee_Data;                //实例化一个数据帧结构体

/*!结构体实例化*/
static game_status_t                           game_status;
static game_result_t                           game_result;
static game_robot_HP_t                         game_robot_HP;
static event_data_t                            event_data;
static referee_warning_t                       referee_warning;
static dart_info_t                             dart_info;
static robot_status_t                          robot_status;
static power_heat_data_t                       power_heat_data;
static robot_pos_t                             robot_pos;
static buff_t                                  buff;
static hurt_data_t                             hurt_data;
static shoot_data_t                            shoot_data;
static projectile_allowance_t                  projectile_allowance;
static rfid_status_t                           rfid_status;
static dart_client_cmd_t                       dart_client_cmd;
static ground_robot_position_t                 ground_robot_position;
static radar_mark_data_t                       radar_mark_data;
static sentry_info_t                           sentry_info;
static radar_info_t                            radar_info;
static robot_interaction_data_t                robot_interaction_data;
static map_command_t                           map_command;
static map_robot_data_t                        map_robot_data;
static map_data_t                              map_data;
static custom_info_t                           custom_info;
static custom_robot_data_t                     custom_robot_data;
static robot_custom_data_t                     robot_custom_data;
static remote_control_t                        remote_control;
static custom_client_data_t                    custom_client_data;


/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static publisher_t *pub_refree;
static subscriber_t *sub_cmd,*sub_ins,*sub_gim;
static void trans_sub_pull(void);
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_pub_init(void);







// 线程同步信号量
static struct rt_semaphore rx_sem;

/**
 * @brief trans 线程中所有订阅者初始化（如有其它数据需求可在其中添加）
 */
static void refree_sub_init(void)
{

    sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));

}

/**
 * @brief trans 线程中所有订阅者获取更新话题（如有其它数据需求可在其中添加）
 */
static void refree_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_gim, &gim_fdb);
}

/**
 * @brief cmd 线程中所有发布者初始化
 */
static void refree_pub_init(void)
{
    pub_refree = pub_register("refree_fdb",sizeof(struct referee_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void refree_pub_push(void)
{
    pub_push_msg(pub_refree,&referee_data);
}

uint8_t *  rx1_buf;
uint8_t *rx2_buf;
uint16_t dma_buf_num;

/*裁判系统线程入口*/
void referee_thread_entry(void *argument){

    /*用户3pin串口初始化*/
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    /* HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);*/
    huart6.Instance=USART6;
    huart6.Init.BaudRate=115200;
    huart6.Init.WordLength=UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity=UART_PARITY_NONE;
    huart6.Init.Mode=UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart6.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);


    /*订阅数据初始化*/
    refree_sub_init();
    /*发布数据初始化*/
    refree_pub_init();

    My_Ui_Init();


    /* USER CODE BEGIN DataAnalysisTask */
    Referee_system_Init(RX_AgreementData_Buffer0,RX_AgreementData_Buffer1,Agreement_RX_BUF_NUM);


    /* 主循环 */
    while (1) {


        Ui_Send();        // 发送更新UI
        Ui_Info_Update();

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
        } else {
            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
        }

        memcpy(&(referee_data.robot_status),&robot_status, sizeof(robot_status_t));
        memcpy(&(referee_data.power_heat_data),&power_heat_data, sizeof(power_heat_data_t));
        memcpy(&(referee_data.game_status),&game_status, sizeof(game_status_t));
        memcpy(&(referee_data.game_robot_HP),&game_robot_HP, sizeof(game_robot_HP_t));
        memcpy(&(referee_data.event_data),&game_robot_HP, sizeof(event_data_t));

        client_info_update(referee_data);//识别自生红蓝,需要先提前设置兵种,判断相同兵种的红蓝状态

        refree_pub_push(); // 发布数据更新
        rt_thread_mdelay(1);
    }
}

/**
 *@brief 裁判系统初始化串口通信以及实例化结构体
 * @param rx1_buf       设定的接收裁判系统返回数据的接收缓冲区1
 * @param rx2_buf       设定的接收裁判系统返回数据的接收缓冲区2
 * @param dma_buf_num   DMA转送数据的空间大小
 *
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    memset(&Referee_Data_header, 0, sizeof(referee_data_header_t));
    memset(&Referee_Data, 0, sizeof(referee_data_t));
    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&map_robot_data, 0, sizeof(map_robot_data_t ));
    memset(&map_data, 0, sizeof(map_data_t));
    memset(&custom_info, 0, sizeof(custom_info_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
    memset(&custom_client_data, 0, sizeof(custom_client_data_t));
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //失效DMA，并等待直至SxCR_EN寄存器置0，以保证后续的配置数据可以写入
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //fifo初始化
    fifo_s_init(&RX_AgreementData_FIFO,RX_FIFO_Space,FIFO_BUF_LENGTH);    //创建FIFO存储区域

}

/**
 * @brief 裁判系统数据解包函数
 */

void Referee_Data_Unpack()
{
    unpack_data_t *p_obj = &referee_unpack_obj;
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    while(fifo_s_used(&RX_AgreementData_FIFO))
    {
        byte = fifo_s_get(&RX_AgreementData_FIFO);
        switch (p_obj->unpack_step)  //状态转换机
        {
            case STEP_HEADER_SOF:      //如果是读取帧头SOF的状态
            {
                if(byte == sof)       //判断是否为SOF
                {
                    p_obj->unpack_step = STEP_LENGTH_LOW;       //改变状态，下次拿出来的byte，去试图照应数据长度的低八位
                    p_obj->protocol_packet[p_obj->index++] = byte;  //将数据码好，并将索引长度加1
                }
                else
                {
                    p_obj->index = 0;   //如果不是，就再从fifo中拿出来一个byte，继续读，直到读出来一个sof
                }
            }break;
            case STEP_LENGTH_LOW:       //如果目前的状态是读的数据长度的低八位
            {
                p_obj->data_len = byte;           //低八位直接放入
                p_obj->protocol_packet[p_obj->index++] = byte;   //码好数据
                p_obj->unpack_step = STEP_LENGTH_HIGH;          //转变状态
            }break;

            case STEP_LENGTH_HIGH:  //如果目前的状态时读数据长度的高八位
            {
                p_obj->data_len |= (byte << 8);     //放入data_len的高八位
                p_obj->protocol_packet[p_obj->index++] = byte;  //码好数据
                //整个交互数据的包总共长最大为 128 个字节，减去 frame_header,cmd_id 和 frame_tail 共 9 个字节以及数据段头结构的 6 个字节
                if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                {
                    p_obj->unpack_step = STEP_FRAME_SEQ;        //转变状态，下一个该读包序号
                }
                else
                {
                    //如果数据长度不合法，就重头开始读取，并且之前码好的数据作废
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:
            {
                p_obj->protocol_packet[p_obj->index++] = byte;  //码好数据
                p_obj->unpack_step = STEP_HEADER_CRC8;          //转换状态，下一个byte读的是CRC8
            }break;

            case STEP_HEADER_CRC8:
            {
                //先将这一byte数据放入，使帧头结构完整，以便后面可以进行CRC校验
                p_obj->protocol_packet[p_obj->index++] = byte;
                //如果这一byte放入之后，数据长度是一个帧头的长度，那么就进行CRC校验
                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
                {
                    if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
                    {
                        p_obj->unpack_step = STEP_DATA_CRC16;   //如果校验通过，则状态转换成去读取帧尾
                    }
                    else
                    {
                        //如果校验不通过，则从头开始，之前码好的数据作废
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }break;

            case STEP_DATA_CRC16:
            {
                //从帧头到帧尾的过程中的数据一律码好
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                //如果数据读取到data末尾，则转换状态，准备开始新一帧的读取
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                    //整包数据校验
                    if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
                    {
                        //校验通过，则将码好的数据memcp到指定结构体中
                        Referee_Data_Solve(p_obj->protocol_packet);
                    }
                }
            }break;

        }
    }
}



/*!串口接收*/
void USART6_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //清除空闲中断
        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前的缓冲区是缓冲区0
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            //将这1帧数据放入fifo0
            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer0,this_time_rx_len);
        }
        else //如果当前的缓冲区是缓冲区1
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            //osSemaphoreRelease(RefereeRxOKHandle);  //释放信号量
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区0
            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer1,this_time_rx_len);
        }
    }
    HAL_UART_IRQHandler(&huart6);
}

/**
 * @brief 裁判系统命令数据解包函数
 * @param referee_data_frame: 接收到的整帧数据
 */
void Referee_Data_Solve(uint8_t* frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;
    memcpy(&Referee_Data_header, frame, sizeof(referee_data_header_t));
    index += sizeof(referee_data_header_t);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID:
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&event_data, frame + index, sizeof(event_data_t));
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            break;
        case DART_FIRE_CMD_ID :
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
            break;
        case ROBOT_STATUS_CMD_ID:
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&buff, frame + index, sizeof(buff_t));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            break;
        case ROBOT_RFID_CMD_ID :
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
            break;
        case DART_DIRECTIONS_CMD_ID  :
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
            break;
        case ROBOT_LOCATION_CMD_ID :
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
            break;
        case RADAR_PROGRESS_CMD_ID :
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
            break;
        case SENTRY_AUTONOMY__CMD_ID :
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
            break;
        case RADAR_AUTONOMY_CMD_ID :
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_2 :
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
            break;
        case PLAYER_MINIMAP_CMD_ID :
            memcpy(&map_command, frame + index, sizeof(map_command_t));
            break;
        case KEYBOARD_MOUSE_CMD_ID :
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
            break;
        case RADAR_MINIMAP_CMD_ID :
            memcpy(&map_robot_data, frame + index, sizeof(map_robot_data_t));
            break;
        case CUSTOMER_CONTROLLER_PLAYER_CMD_ID :
            memcpy(&custom_client_data, frame + index, sizeof(custom_client_data_t));
            break;
        case PLAYER_MINIMAP_SENTRY_CMD_ID :
            memcpy(&map_data, frame + index, sizeof(map_data_t));
            break;
        case PLAYER_MINIMAP_ROBOT_CMD_ID :
            memcpy(&custom_info, frame + index, sizeof(custom_info_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_9 :
            memcpy(&robot_custom_data, frame + index, sizeof(robot_custom_data_t));
            break;
        default:
            break;
    }
}