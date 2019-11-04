/******************************************/
/*           广州旭东阪田电子有限公司     */
/*Project:     cx743多媒体开关 V2版       */
/*Guest:                                  */
/*Name:             main.c                */
/*Mcu chip:         Atmega64              */
/*Main clock:       外部晶体11.0592MHz    */
/*Rtc clock:                              */
/*Author:           Loyer                 */
/*Create date:      2019.03.05            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/
#include <iom64v.h>
#include <stdio.h>
#include <macros.h>
#include <port.h>
#include <default.h>
#include <delay.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <commands.h>

//坐标值记录数组
INT32 posi_buffer[16];


//==按键变量定义=============================
uchar key_now = 0;
uchar key_old = 0;
uchar key_code = 0;
uchar key_cnt = 0;
#define KEY_COUNTER 5
//电机运行位置
typedef struct coordinate
{
    INT32 X;
    INT32 Y;
    INT32 Z;
} Position;
Position current_posi = {0, 0, 0}; //当前位置
Position last_posi = {0, 0, 0}; //记录位置
Position de_posi = {0, 0, 0}; //调试位置
//==是否开始脉冲计数并且返回变量定义
INT32 X_paulse = 0; //调试时脉冲个数
INT32 X_runPaulse = 0; //运行时脉冲个数
INT32 X_paulseTemp = 0; //脉冲个数记录(行程)
INT32 Y_paulse = 0; //调试时脉冲个数
INT32 Y_runPaulse = 0; //运行时脉冲个数
INT32 Y_paulseTemp = 0; //脉冲个数记录(行程)
INT32 Z_paulse = 0; //调试时脉冲个数
INT32 Z_runPaulse = 0; //运行时脉冲个数
INT32 Z_paulseTemp = 0; //脉冲个数记录(行程)
char *X_allowRun = "false"; //允许X轴电机运行标志
char *Y_allowRun = "false"; //允许Y轴电机运行标志
char *Z_allowRun = "false"; //允许Z轴电机运行标志
char *IO_status = "init"; //上位机指令类型
char *motor_reseting = "false"; //电机复位状态
char *isStarted = "false"; //程序启动标志
char *sendPos = "false"; //发送行程(脉冲当量)

volatile uchar Z_isConductive = FALSE; //导通标志位
volatile uchar X_isConductive = FALSE; //导通标志位
volatile uchar Y_isConductive = FALSE; //导通标志位
volatile uchar isDebug = FALSE; //调试标志
volatile uchar isDe_posi = FALSE; //调试走固定位置标志
volatile uchar isDe_reset = FALSE; //调试回原点标志
volatile uchar MOTOR_SPEED = 1; //电机运行速度(值越大，速度越慢)
uchar T_count = 0; //生效电机运行速度更改
uchar step_counter = 0; //测试步数计数器
#define RECORD_STEPS 1200
INT32 NG_STEP = RECORD_STEPS; //NG时电机后退脉冲数
INT32 PASS_STEP = RECORD_STEPS; //PASS时电机后退脉冲数
uchar limit_count = 0; //电机限位计数器(防误判)
uchar stop_count = 0; //急停按下计数器(防误判)
uchar p2_count = 0; //下压拉力计第二个点设定触发计数器

#define X_REC 1700
#define Y_REC 1400
#define Z_REC 5300
INT32 x_re = 0; //测四向摆动力电机运行补偿
INT32 y_re = 0; //测四向摆动力电机运行补偿
INT32 z_re = 0; //测四向摆动力电机运行补偿
INT32 z_comp = 0; //测四向摆动力电机运行补偿
INT32 z_backStep = 0; //Z轴回退脉冲数
INT32 stroke_limit = 0; //按压被测开关总行程限制

/***********USART0接收中断服务函数 start**********************/
//USART接收缓冲区
#define RX_BUFFER_SIZE 16                  //接收缓冲区大小，可根据需要修改。
unsigned char rx_buffer[RX_BUFFER_SIZE];   //定义接收缓冲区
unsigned char rx_counter = 0;              //定义rx_counter为存放在队列中的已接收到字符个数。

//定义一个标志位Usart0_RECVFlag1:=1表示串口0接收到了一个完整的数据包
//在port.h中定义

#pragma interrupt_handler usart0_rxc_isr:19  //接收中断服务程序
void usart0_rxc_isr(void)
{
    uchar status, data;
    status = UCSR0A;
    data = UDR0;
    if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //判断是否允许接收一个新的数据包
    {
        if ((status & (USART0_FRAMING_ERROR | USART0_PARITY_ERROR | USART0_DATA_OVERRUN)) == 0)
        {
            rx_buffer[rx_counter] = data;
            rx_counter++;
            switch (rx_counter)
            {
            case 1:       // 检验起始字符
            {
                if (data != FIRST_TEXT) rx_counter = 0;
            }
            break;
            case 2:
            {
                if (data != SECOND_TEXT) rx_counter = 0;
            }
            break;
            case 15:
            {
                if(data == 0x10) IO_status = "motor_reset";
                else if(data == 0x20) IO_status = "debug";
                else if(data == 0x22) IO_status = "de_posi";
                //else if(data == 0x44) IO_status = "direction";
                else if(data == 0x46) IO_status = "finished";
                //else if(data == 0x50) IO_status = "position";
                else if(data == 0x4E) IO_status = "NG";
                else if(data == 0x72) IO_status = "RELAY_CRTL";
                else if(data == 0x78) IO_status = "X_conductive";
                else if(data == 0x79) IO_status = "Y_conductive";
                else if(data == 0x7a) IO_status = "Z_conductive";
                else if(data > 0 && data < 10) IO_status = "common";
            }
            break;
            case 16:      // 检验结束字符
            {
                rx_counter = 0;
                if (data == END_TEXT) set_bit(flag1, Usart0_RECVFlag1); // Usart0_RecvFlag=1，表示正确接收到一个数据包
            }
            break;
            default:
                break;
            }
        }
    }
}
/***************USART0接收中断服务函数 end**********************/
/*============================================================*/
/*============================================================*/
/***************USART0发送中断服务函数 start********************/
#define TX_BUFFER_SIZE 16
unsigned char tx_buffer[TX_BUFFER_SIZE];
unsigned char tx_wr_index = 0, tx_rd_index = 0, tx_counter = 0;

#pragma interrupt_handler usart0_txc_isr:21  //发送中断服务程序
void usart0_txc_isr(void)
{
    if (tx_counter)//队列不为空
    {
        --tx_counter;//出队列
        UDR0 = tx_buffer[tx_rd_index];
        if (++tx_rd_index == TX_BUFFER_SIZE) tx_rd_index = 0;
    }
}
/***********USART0发送中断服务函数 end**********************/

/*============================================================*/
/***********USART0发送一个字符函数 start**********************/
void USART0_putchar(unsigned char c)
{
    while (tx_counter == TX_BUFFER_SIZE);
    CLI();//#asm("cli")关闭全局中断允许
    if (tx_counter || ((UCSR0A & USART0_DATA_REGISTER_EMPTY) == 0)) //发送缓冲器不为空
    {
        tx_buffer[tx_wr_index] = c; //数据进入队列
        if (++tx_wr_index == TX_BUFFER_SIZE) tx_wr_index = 0; //队列已满
        ++tx_counter;
    }
    else
        UDR0 = c;
    SEI(); //#asm("sei")打开全局中断允许
}
/***********USART0发送服务函数 end**********************/
//清零所有坐标值
void init_posi_buffer(void)
{
    uchar cnt = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        posi_buffer[cnt] = 0;
    }
}
//将上位机发回的坐标值写入
void fill_posi_buffer(void)
{
    uchar cnt = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        posi_buffer[cnt] = rx_buffer[cnt];
    }
}
//定义行程值(脉冲数)填充函数
void fill_stroke(void)
{
    *(strokes + 2) = (X_paulseTemp >> 24) & 0xff; //取24-31位值
    *(strokes + 3) = (X_paulseTemp >> 16) & 0xff; //取16-23位值
    *(strokes + 4) = (X_paulseTemp >> 8) & 0xff; //取8-15位值
    *(strokes + 5) = X_paulseTemp & 0xff;       //取0-7位值

    *(strokes + 6) = (Y_paulseTemp >> 24) & 0xff; //取24-31位值
    *(strokes + 7) = (Y_paulseTemp >> 16) & 0xff; //取16-23位值
    *(strokes + 8) = (Y_paulseTemp >> 8) & 0xff; //取8-15位值
    *(strokes + 9) = Y_paulseTemp & 0xff;       //取0-7位值

    *(strokes + 10) = (Z_paulseTemp >> 24) & 0xff; //取24-31位值
    *(strokes + 11) = (Z_paulseTemp >> 16) & 0xff; //取16-23位值
    *(strokes + 12) = (Z_paulseTemp >> 8) & 0xff; //取8-15位值
    *(strokes + 13) = Z_paulseTemp & 0xff;       //取0-7位值
}
//调试时反馈电机位置(脉冲数)
void fill_debugStroke(void)
{
    *(debug_strokes + 2) = (X_paulse >> 24) & 0xff; //取24-31位值
    *(debug_strokes + 3) = (X_paulse >> 16) & 0xff; //取16-23位值
    *(debug_strokes + 4) = (X_paulse >> 8) & 0xff; //取8-15位值
    *(debug_strokes + 5) = X_paulse & 0xff;       //取0-7位值

    *(debug_strokes + 6) = (Y_paulse >> 24) & 0xff; //取24-31位值
    *(debug_strokes + 7) = (Y_paulse >> 16) & 0xff; //取16-23位值
    *(debug_strokes + 8) = (Y_paulse >> 8) & 0xff; //取8-15位值
    *(debug_strokes + 9) = Y_paulse & 0xff;       //取0-7位值

    *(debug_strokes + 10) = (Z_paulse >> 24) & 0xff; //取24-31位值
    *(debug_strokes + 11) = (Z_paulse >> 16) & 0xff; //取16-23位值
    *(debug_strokes + 12) = (Z_paulse >> 8) & 0xff; //取8-15位值
    *(debug_strokes + 13) = Z_paulse & 0xff;       //取0-7位值
}
/**
 * 串口发送函数
 * @buff[] 待发送的字节数组
 */
void send(unsigned char buff[])
{
    uchar i = 0;
	for(i = 0; i < TX_BUFFER_SIZE; i++)
    {
        USART0_putchar(buff[i]);
    }
}
//发送行程值到主机
void sendStrokeToHost(void)
{
    fill_stroke();
	send(strokes);
}
//发送调试好的坐标值给上位机
void sendDebugPosition(void)
{
    fill_debugStroke();
	send(debug_strokes);
}
//到达指定位置
void sendArrivals(void)
{
    send(spe_location);
}
//更新所有坐标值(以及方向)
void update_XYZ(void)
{
    if(current_posi.X > last_posi.X)
    {
        X_forward();
        X_runPaulse = current_posi.X - last_posi.X;
        last_posi.X = current_posi.X;
    }
    else
    {
        X_backward();
        X_runPaulse = last_posi.X - current_posi.X;
        last_posi.X = current_posi.X;
    }
    ///////
    if(current_posi.Y > last_posi.Y)
    {
        Y_forward();
        Y_runPaulse = current_posi.Y - last_posi.Y;
        last_posi.Y = current_posi.Y;
    }
    else
    {
        Y_backward();
        Y_runPaulse = last_posi.Y - current_posi.Y;
        last_posi.Y = current_posi.Y;
    }
    //////
    if(current_posi.Z > last_posi.Z)
    {
        Z_forward();
        Z_runPaulse = current_posi.Z - last_posi.Z;
        last_posi.Z = current_posi.Z;
    }
    else
    {
        Z_backward();
        Z_runPaulse = last_posi.Z - current_posi.Z;
        last_posi.Z = current_posi.Z;
    }
}
//写入当前坐标值
void fill_currentPosi(void)
{
    fill_posi_buffer();
    //current_posi.X = (rx_buffer[2] << 24) | (rx_buffer[3] << 16) | (rx_buffer[4] << 8) | rx_buffer[5];
    //current_posi.Y = (rx_buffer[6] << 24) | (rx_buffer[7] << 16) | (rx_buffer[8] << 8) | rx_buffer[9];
    //current_posi.Z = (rx_buffer[10] << 24) | (rx_buffer[11] << 16) | (rx_buffer[12] << 8) | rx_buffer[13];
    current_posi.X = (posi_buffer[2] << 24) | (posi_buffer[3] << 16) | (posi_buffer[4] << 8) | posi_buffer[5];
    current_posi.Y = (posi_buffer[6] << 24) | (posi_buffer[7] << 16) | (posi_buffer[8] << 8) | posi_buffer[9];
    current_posi.Z = (posi_buffer[10] << 24) | (posi_buffer[11] << 16) | (posi_buffer[12] << 8) | posi_buffer[13];
}
//向上位机发送复位指令
void send_reset(void)
{
    send(reset);
}
//电机复位函数
void motor_reset(void)
{
    if(!Z_cylinderArrivalsUP())
    {
        Z_cylinderUP();
    }
    product_powerOFF();
    Z_backward();
    Z_enable();
    X_backward();
    X_enable();
    Y_backward();
    Y_enable();
    last_posi.X = 0;
    last_posi.Y = 0;
    last_posi.Z = 0;
    limit_count = 0;
    stop_count = 0;
	p2_count = 0;
    motor_reseting = "true";
    MOTOR_SPEED = 1;
}
//系统复位函数
void mcu_reset(void)
{
    send_reset();
    motor_reset();
}
//封装指令发送函数
void function(void (*fuc)())
{
    uchar cnt = 0;
    do
    {
        if(cnt++ >= 3)
        {
            cnt = 0;
            break; //如果重试次数大于RETYR_TIMES,结束循环
        }
        (*fuc)();    //发送测试指令
        delay_nms(300); //等待被测板数据返回
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
void standBy(void)
{
    clr_bit(flag1, Usart0_RECVFlag1);
}
/////////////////////////////////////////////////////////////////////////////////////
void trans_init(void)
{
    send(init);
}
void trans_start(void)
{
    send(start);
}

void send_stop(void)
{
    send(stop);
}

void send_timeout(void)
{
    send(timeout);
}
//停留在NG状态
void stayAtNG(void)
{
    Timer0_disable();
    clr_bit(flag2, Alarm_LIMITFlag2);
    clr_bit(flag2, motor_ARRIVALSFlag2);
    if(step_counter > 5)
    {
        //NG();
        NG_STEP = RECORD_STEPS;
        set_bit(flag2, is_NGFlag2);
        //Timer0_disable();
    }
    else
    {
        NG();
        //Timer0_disable();
        Timer1_disable();
    }
}

void send_start(void)
{
    do
    {
        trans_start();
        delay_nms(200); //等待被测板数据返回
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
//定时中断函数(自动检测用)
#pragma interrupt_handler timer1_count_isr:15
void timer1_count_isr(void)
{
    TCNT1 = 65536 - CRYSTAL / 8 / 2 * 0.0002; //重装0.0002s定时

    //====急停或至少有一个电机到达限位处=====
    if(stopKey_clicked())
    {
        if(stop_count > KEY_COUNTER)
        {
            stop_count = 0;
            send_stop();
            isStarted = "false";
            product_powerOFF();
            //product_cylinderUP();
            //Z_cylinderUP();
            Timer1_disable();
            Timer0_disable();
            //SREG = 0x00; //全局禁止中断
        }
        stop_count++;
    }
    if(X_Y_Z_ArrivalsLimit() && (motor_reseting == "false"))
    {
        if(limit_count > KEY_COUNTER)
        {
            limit_count = 0;
            //PORTG &= ~(1 << PG3);
            send_stop();
            NG();
            isStarted = "false";
            product_powerOFF();
            //product_cylinderUP();
            //Z_cylinderUP();
            Timer1_disable();
            Timer0_disable();
        }
        limit_count++;
    }
    //=================================
    if(++T_count != MOTOR_SPEED) return;
    T_count = 0;
    ///////////////////////////////////////

    //////////////////////////////////////
    if((X_arrivalsLimit() || Y_arrivalsLimit() || Z_arrivalsLimit()) && ((flag2 & (1 << Alarm_LIMITFlag2)) == 0)
            && (motor_reseting == "false") && ((flag2 & (1 << is_NGFlag2)) == 0) && ((flag2 & (1 << is_PASSFlag2)) == 0))  //拉力报警，减速电机
    {
        set_bit(flag2, Alarm_LIMITFlag2);
        MOTOR_SPEED = 50; //减速电机
    }
    //========生效更改=====================================
    if(isStarted == "true" && ((flag2 & (1 << allowMotorFlag2)) != 0)) //程序运行中，允许电机走位
    {
        clr_bit(flag2, allowMotorFlag2);  //清除标志位
        X_allowRun = "true";
        X_enable();
        Y_allowRun = "true";
        Y_enable();
        Z_allowRun = "true";
        Z_enable();
        set_bit(flag2, motor_GOFlag2);
    }
    //=========执行更改======================
    if((flag2 & (1 << motor_GOFlag2)) != 0)
    {
        //====走指定坐标位置===
        if(X_runPaulse > 0 && X_allowRun == "true")
        {
            X_RUN();
            X_runPaulse--;
        }
        if(Y_runPaulse > 0 && Y_allowRun == "true")
        {
            Y_RUN();
            Y_runPaulse--;
        }
        if(X_runPaulse <= 0 && Y_runPaulse <= 0 && Z_runPaulse > 0 && Z_allowRun == "true")
        {
            Z_RUN();
            Z_runPaulse--;
        }
        //====到达指定坐标位置===
        if((X_runPaulse <= 0) && (Y_runPaulse <= 0) && (Z_runPaulse <= 0) && ((flag2 & (1 << motor_ARRIVALSFlag2)) == 0)
                && (motor_reseting == "false") && ((flag2 & (1 << is_NGFlag2)) == 0) && ((flag2 & (1 << is_PASSFlag2)) == 0))
        {
            X_allowRun = "false";
            Y_allowRun = "false";
            Z_allowRun = "false";
            //PORTG&=~(1<<PG3);
            if(step_counter == 8)
            {
                x_re = X_REC;
                y_re = Y_REC;
                z_re = Z_REC;
                X_enable();
                X_forward();
                Y_enable();
                Y_forward();
                Z_enable();
                Z_backward();
                set_bit(flag3, isRECFlag3);
            }
			Timer1_disable(); //不允许操作
            sendArrivals();
			Timer1_enable();
            set_bit(flag2, motor_ARRIVALSFlag2);
            //clr_bit(flag2, motor_GOFlag2);
            MOTOR_SPEED = 30;
        }//*/
    }
    //=====执行按键按压操作=======
    if((flag2 & (1 << motor_ARRIVALSFlag2)) != 0)
    {
        clr_bit(flag2, motor_GOFlag2);
		//==拉力计设定点2有效
		if(Rally_P2_effictive())
		{
	        if(p2_count > KEY_COUNTER)
			{
			    p2_count = 0;
				send_timeout();
				stayAtNG();
			}
			p2_count++;
		}
		//===========================
		switch (step_counter)
        {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        {
            if((flag2 & (1 << Alarm_LIMITFlag2)) != 0)
            {
                Z_paulseTemp++;
                Z_allowRun = "false";
                Z_runPaulse = 0;
            }
            if(Z_isConductive == TRUE)
            {
                Z_isConductive = FALSE;
				strokes[14] = 0x7a;
                sendStrokeToHost();
                Z_paulseTemp = 0;
				stroke_limit = 0;
                clr_bit(flag2, Alarm_LIMITFlag2);
                clr_bit(flag2, motor_ARRIVALSFlag2);
                break;
            }
            //Z_cylinderDOWN();
            if(!Z_cylinderArrivalsDOWN())
            {
                Z_cylinderDOWN();
            }//*/
            if(Z_DirIsBackward())
            {
                Z_forward();
            }
            if(Z_isDisabled())
            {
                Z_enable();
            }
            Z_RUN();
			if(stroke_limit++ > 1800)
			{
			    send_timeout();
				stayAtNG();
			}
            Z_DirIsForward() ? (last_posi.Z++) : (last_posi.Z--);
        }
        break;
        case 6:
        {
            if((flag2 & (1 << Alarm_LIMITFlag2)) != 0)
            {
                Y_paulseTemp++;
                Y_allowRun = "false";
                Y_runPaulse = 0;
            }
            if(Y_isConductive == TRUE)
            {
			    Y_isConductive = FALSE;
				strokes[14] = 0x79;
                sendStrokeToHost();
                Y_paulseTemp = 0;
				stroke_limit = 0;
                clr_bit(flag2, Alarm_LIMITFlag2);
                clr_bit(flag2, motor_ARRIVALSFlag2);
                break;
            }
            if(!X_Y_cylinderArrivalsOrigin())
            {
                trunY_X();
            }
            if(!Z_cylinderArrivalsUP())
            {
                Z_cylinderUP();
            }
            if(Y_DirIsForward())
            {
                Y_backward();
            }
            if(Y_isDisabled())
            {
                Y_enable();
            }
            Y_RUN();
			if(stroke_limit++ > 2688)
			{
			    send_timeout();
				stayAtNG();
			}
            Y_DirIsForward() ? (last_posi.Y++) : (last_posi.Y--);
        }
        break;
        case 7:
        {
            if((flag2 & (1 << Alarm_LIMITFlag2)) != 0)
            {
                Y_paulseTemp++;
                Y_allowRun = "false";
                Y_runPaulse = 0;
            }
            if(Y_isConductive == TRUE)
            {
                Y_isConductive = FALSE;
				strokes[14] = 0x79;
                sendStrokeToHost();
                Y_paulseTemp = 0;
				stroke_limit = 0;
                clr_bit(flag2, Alarm_LIMITFlag2);
                clr_bit(flag2, motor_ARRIVALSFlag2);
                break;
            }
            if(!X_Y_cylinderArrivalsOrigin())
            {
                trunY_X();
            }
            if(!Z_cylinderArrivalsUP())
            {
                Z_cylinderUP();
            }
            if(Y_DirIsBackward())
            {
                Y_forward();
            }
            if(Y_isDisabled())
            {
                Y_enable();
            }
            Y_RUN();
			if(stroke_limit++ > 2688)
			{
			    send_timeout();
				stayAtNG();
			}
            Y_DirIsForward() ? (last_posi.Y++) : (last_posi.Y--);
        }
        break;
        case 8:
        {
            if((flag3 & (1 << isRECFlag3)) != 0)
            {
                MOTOR_SPEED = 1;
                if(z_re > 0)
                {
                    Z_RUN();
                    z_re--;
                }
                else
                {
                    if(x_re > 0)
                    {
                        X_RUN();
                        x_re--;
                        X_DirIsForward() ? (last_posi.X++) : (last_posi.X--);
                    }
                    if(y_re > 0)
                    {
                        Y_RUN();
                        y_re--;
                        Y_DirIsForward() ? (last_posi.Y++) : (last_posi.Y--);
                    }
                }
                if(x_re <= 0 && y_re <= 0 && z_re <= 0)
                {
                    clr_bit(flag3, isRECFlag3);
                    if(X_Y_cylinderArrivalsOrigin())
                    {
                        trunX_Y();
                    }
                    trunX_Y();
					delay_nms(10);					
                    z_comp = Z_REC + 50;
                    Z_forward();
					set_bit(flag3, isCOMPFlag3);
					return;
                }
            }
            else if((flag3 & (1 << isCOMPFlag3)) != 0)
            {
                if(z_comp > 0)
                {
                    Z_RUN();
                    z_comp--;
                }
                else
                {
                    clr_bit(flag3, isCOMPFlag3);
                    MOTOR_SPEED = 15;
                }
            }
            else
            {
                if((flag2 & (1 << Alarm_LIMITFlag2)) != 0)
                {
                    X_paulseTemp++;
                    X_allowRun = "false";
                    X_runPaulse = 0;
                }
                if(X_isConductive == TRUE)
                {
                    X_isConductive = FALSE;
					strokes[14] = 0x78;
                    sendStrokeToHost();
                    X_paulseTemp = 0;
					stroke_limit = 0;
                    clr_bit(flag2, Alarm_LIMITFlag2);
                    clr_bit(flag2, motor_ARRIVALSFlag2);
                    break;
                }
                if(!Z_cylinderArrivalsUP())
                {
                    Z_cylinderUP();
                }
                if(X_DirIsForward())
                {
                    X_backward();
                }
                if(X_isDisabled())
                {
                    X_enable();
                }
                X_RUN();
				if(stroke_limit++ > 2688)
				{
			        send_timeout();
					stayAtNG();
				}
                X_DirIsForward() ? (last_posi.X++) : (last_posi.X--);
            }
        }
        break;
        case 9:
        {
            if((flag2 & (1 << Alarm_LIMITFlag2)) != 0)
            {
                X_paulseTemp++;
                X_allowRun = "false";
                X_runPaulse = 0;
            }
            if(X_isConductive == TRUE)
            {
                X_isConductive = FALSE;
				strokes[14] = 0x78;
                sendStrokeToHost();
                X_paulseTemp = 0;
				stroke_limit = 0;
                clr_bit(flag2, Alarm_LIMITFlag2);
                clr_bit(flag2, motor_ARRIVALSFlag2);
                break;
            }
            if(X_Y_cylinderArrivalsOrigin())
            {
                trunX_Y();
            }
            if(!Z_cylinderArrivalsUP())
            {
                Z_cylinderUP();
            }
            if(X_DirIsBackward())
            {
                X_forward();
            }
            if(X_isDisabled())
            {
                X_enable();
            }
            X_RUN();
			if(stroke_limit++ > 2688)
			{
			    send_timeout();
				stayAtNG();
			}
            X_DirIsForward() ? (last_posi.X++) : (last_posi.X--);
        }
        break;
        default:
            break;
        }
    }
    //NG时先回归到中间点
    if((flag2 & (1 << is_NGFlag2)) != 0)
    {
        clr_bit(flag2, Alarm_LIMITFlag2);
        clr_bit(flag2, motor_ARRIVALSFlag2);
        MOTOR_SPEED = 3;
        if(step_counter == 6)
        {
            Y_forward();
            Y_enable();
            if(NG_STEP > 0)
            {
                Y_RUN();
                NG_STEP--;
            }
            else
            {
                NG();
                clr_bit(flag2, is_NGFlag2);
                Timer1_disable();
            }
        }
        else if(step_counter == 7)
        {
            Y_backward();
            Y_enable();
            if(NG_STEP > 0)
            {
                Y_RUN();
                NG_STEP--;
            }
            else
            {
                NG();
                clr_bit(flag2, is_NGFlag2);
                Timer1_disable();
            }
        }
        else if(step_counter == 8)
        {
            X_forward();
            X_enable();
            if(NG_STEP > 0)
            {
                X_RUN();
                NG_STEP--;
            }
            else
            {
                //NG();
                clr_bit(flag2, is_NGFlag2);
                clr_bit(flag2, is_PASSFlag2);
                clr_bit(flag2, motor_GOFlag2);
                clr_bit(flag2, motor_ARRIVALSFlag2);
                motor_reseting = "false";
                //X_disable();
                //Y_disable();
                //Z_backward();
                z_backStep = Z_REC;
                set_bit(flag3, Z_BACKFlag3);
                //Timer1_disable();
                //motor_reset();
            }
        }
        
        else if(step_counter == 9)
        {
            X_backward();
            X_enable();
            if(NG_STEP > 0)
            {
                X_RUN();
                NG_STEP--;
            }
            else
            {
                //NG();
                clr_bit(flag2, is_NGFlag2);
        		clr_bit(flag2, is_PASSFlag2);
        		clr_bit(flag2, motor_GOFlag2);
        		clr_bit(flag2, motor_ARRIVALSFlag2);
        		motor_reseting = "false";
        		//X_disable();
        		//Y_disable();
        		//Z_backward();
        		z_backStep = Z_REC;
        		set_bit(flag3, Z_BACKFlag3);
                //Timer1_disable();
        		//motor_reset();
            }
        }//*/
    }
    //八九步NG--Z轴回退一点

    if((flag3 & (1 << Z_BACKFlag3)) != 0)
    {
        MOTOR_SPEED = 1;
        X_disable();
        Y_disable();
        Z_backward();
        Z_enable();
        if(z_backStep > 0)
        {
            Z_RUN();
            z_backStep--;
        }
        else
        {
            NG();
            clr_bit(flag3, Z_BACKFlag3);
            Timer1_disable();
        }
    }//*/

    //===测完后回归原位
    if((flag2 & (1 << is_PASSFlag2)) != 0)
    {
        MOTOR_SPEED = 1;
        X_backward();
        X_enable();
        if(PASS_STEP > 0)
        {
            X_RUN();
            PASS_STEP--;
        }
        else
        {
            clr_bit(flag2, is_PASSFlag2);
            motor_reset();
        }
    }
    //======复位====
    if((motor_reseting == "true"))
    {
        if(Z_arrivalsOrigin() && X_arrivalsOrigin() && Y_arrivalsOrigin())
        {
            if(!X_Y_cylinderArrivalsOrigin())
            {
                trunY_X();
            }
            if(product_cylinderArrivals())
            {
                product_cylinderUP();
            }
            motor_reseting = "false";
            X_paulse = 0;
            Y_paulse = 0;
            Z_paulse = 0;
        }
        //==========
        if(!Z_arrivalsOrigin()) //先回归Z轴
        {
            Z_RUN();
        }
        else
        {
            if(!X_arrivalsOrigin())
            {
                X_RUN();
            }
            if(!Y_arrivalsOrigin())
            {
                Y_RUN();
            }
        }
    }
    //=====复位完成====


}
#pragma interrupt_handler timer0_isr:17
void timer0_isr(void)
{
    TCNT0 = 256 - CRYSTAL / 64 / 2 * 0.0003; //重装0.0002s定时(用与电机运行最快的速度扫描，可避免电机跑飞)

    //========以下为调试参数设置==========
    if((flag1 & (1 << Usart0_RECVFlag1)) != 0 && isStarted == "false") //收到PC发来的数据,且主运行没有开始 //&& isStarted == "false"
    {
        clr_bit(flag1, Usart0_RECVFlag1);
        if(IO_status == "debug")
        {
            isDebug = TRUE;
            isDe_posi = FALSE;
            isDe_reset = FALSE;
            rx_buffer[2] == 0x01 ? X_forward() : X_backward();
            rx_buffer[3] == 0x01 ? Y_forward() : Y_backward();
            rx_buffer[4] == 0x01 ? Z_forward() : Z_backward();
            if(rx_buffer[5] == 0x01)
            {
                X_allowRun = "true";
                X_enable();
            }
            else
            {
                X_allowRun = "false";
                X_disable();
            }
            if(rx_buffer[6] == 0x01)
            {
                Y_allowRun = "true";
                Y_enable();
            }
            else
            {
                Y_allowRun = "false";
                Y_disable();
            }
            if(rx_buffer[7] == 0x01)
            {
                Z_allowRun = "true";
                Z_enable();
            }
            else
            {
                Z_allowRun = "false";
                Z_disable();
            }
            rx_buffer[8] == 0x01 ? (sendPos = "true") : (sendPos = "false");
        }
        else if(IO_status == "RELAY_CRTL")
        {
            rx_buffer[2] == 0x01 ? product_powerON() : product_powerOFF();
            rx_buffer[3] == 0x01 ? Z_cylinderDOWN() : Z_cylinderUP();
            rx_buffer[4] == 0x01 ? product_cylinderDOWN() : product_cylinderUP();
            rx_buffer[5] == 0x01 ? trunX_Y() : trunY_X();
            rx_buffer[6] == 0x01 ? NG() : NG_CANCEL();
        }
        else if(IO_status == "de_posi") //走固定步数
        {
            if(!X_arrivalsOrigin() || !Y_arrivalsOrigin() || !Z_arrivalsOrigin())
            {
                motor_reset();
                return;
            }
            isDebug = FALSE;
            isDe_reset = FALSE;
            isDe_posi = TRUE;
            fill_posi_buffer();
            de_posi.X = (posi_buffer[2] << 24) | (posi_buffer[3] << 16) | (posi_buffer[4] << 8) | posi_buffer[5];
            de_posi.Y = (posi_buffer[6] << 24) | (posi_buffer[7] << 16) | (posi_buffer[8] << 8) | posi_buffer[9];
            de_posi.Z = (posi_buffer[10] << 24) | (posi_buffer[11] << 16) | (posi_buffer[12] << 8) | posi_buffer[13];
            X_forward();
            X_enable();
            Y_forward();
            Y_enable();
            Z_forward();
            Z_enable();
        }
        else if(IO_status == "motor_reset")
        {
            isDebug = FALSE;
            isDe_posi = FALSE;
            isDe_reset = TRUE;
        }
        else
        {
            isDe_reset = FALSE;
            isDebug = FALSE;
            isDe_posi = FALSE;
        }
    }
    //=============以下为测试中配置============
    else if((flag1 & (1 << Usart0_RECVFlag1)) != 0 && isStarted == "true" && (flag2 & (1 << init_COMPFlag2)) != 0) //已成功启动测试，且下位机初始化完成
    {
        clr_bit(flag1, Usart0_RECVFlag1);
        isDe_reset = FALSE;
        isDebug = FALSE;
        isDe_posi = FALSE;
        //====首先判断按键是否导通====
        (IO_status == "X_conductive") ? (X_isConductive = TRUE) : (X_isConductive = FALSE);
        (IO_status == "Y_conductive") ? (Y_isConductive = TRUE) : (Y_isConductive = FALSE);
        (IO_status == "Z_conductive") ? (Z_isConductive = TRUE) : (Z_isConductive = FALSE);
        if(IO_status == "finished") //测试完成，产品断电，各气缸归位，电机归位
        {
            isStarted = "false";
            clr_bit(flag2, allowMotorFlag2);
            clr_bit(flag2, init_COMPFlag2);
            //product_powerOFF();
            //product_cylinderUP();
            PASS_STEP = RECORD_STEPS;
            set_bit(flag2, is_PASSFlag2);
            //Z_cylinderUP();
            //motor_reset();
        }
        else if(IO_status == "common")
        {
            step_counter = rx_buffer[14];
			stroke_limit = 0;
			p2_count = 0;
            MOTOR_SPEED = 1;
            Z_cylinderUP();
            fill_currentPosi();
            update_XYZ();
            clr_bit(flag2, motor_ARRIVALSFlag2);
            set_bit(flag2, allowMotorFlag2);
        }
        else if(IO_status == "NG")
        {
            stayAtNG();
        }
    }
    //========生效调试==============================================
    if(isDebug == TRUE)
    {
        if(X_allowRun == "true")
        {
            X_RUN();
            if(X_DirIsForward()) X_paulse++;
            else
            {
                if(X_paulse > 0) X_paulse--;
            }
        }
        else
        {
            clr_bit(PORTC, PC0);
        }
        if(Y_allowRun == "true")
        {
            Y_RUN();
            if(Y_DirIsForward()) Y_paulse++;
            else
            {
                if(Y_paulse > 0) Y_paulse--;
            }
        }
        else
        {
            clr_bit(PORTC, PC5);
        }
        if(Z_allowRun == "true")
        {
            Z_RUN();
            LED_BLINK();
            if(Z_DirIsForward()) Z_paulse++;
            else
            {
                if(Z_paulse > 0) Z_paulse--;
            }
        }
        else
        {
            set_bit(PORTD, PD6); //防止指示灯一直闪
            clr_bit(PORTF, PF2);
        }
        //===========================================================
        if(sendPos == "true")
        {
            sendPos = "false";
            //Timer0_disable();
            sendDebugPosition();
            //Timer0_enable();
        }
    }
    else
    {
        X_paulse = 0;
        Y_paulse = 0;
        Z_paulse = 0;
    }
    //=====调试走固定步数===================
    if(isDe_posi == TRUE)
    {
        if(de_posi.X > 0)
        {
            X_RUN();
            de_posi.X--;
        }
        if(de_posi.Y > 0)
        {
            Y_RUN();
            de_posi.Y--;
        }
        if(de_posi.Z > 0 && de_posi.Y <= 0 && de_posi.X <= 0)
        {
            Z_RUN();
            de_posi.Z--;
        }
        if(de_posi.X <= 0 && de_posi.Y <= 0 && de_posi.Z <= 0)
        {
            isDe_posi = FALSE;
            de_posi.X = 0;
            de_posi.Y = 0;
            de_posi.Z = 0;
        }
    }
    else
    {
        de_posi.X = 0;
        de_posi.Y = 0;
        de_posi.Z = 0;
    }
    //===========调试回原点=======
    if(isDe_reset == TRUE)
    {
        isDe_reset = FALSE;
        motor_reset();
    }

}
//定时器0初始化
void init_TIMER0_OVF(void)
{
    TCCR0 = 0x04; //64分频
    TCNT0 = 256 - CRYSTAL / 64 / 2 * 0.05; //0.05s定时
    TIMSK |= (1 << TOIE0); //定时器0中断使能
    SREG = 0x80;
}
//定时器1初始化
void init_TIMER1_OVF(void)
{
    TCCR1B = 0x02; //8分频
    TCNT1 = 65536 - CRYSTAL / 8 / 2 * 0.05; //0.05s定时
    TIMSK |= (1 << TOIE1); //定时器使能
    SREG = 0x80;    //全局使能中断
}
/***************USART01初始化函数 start*************************/
void init_usart0(void)
{
    UCSR0B = 0x00;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //异步，8位数据，无奇偶校验，一个停止位，无倍速
    UBRR0L = BAUD_L;                        //设定波特率
    UBRR0H = BAUD_H;
    UCSR0A = 0x00;
    UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0); //0XD8  接收、发送使能， 开中断使能
}
/***************USART0初始化函数 end***************************/
/***************系统初始化函数 start ***********/
void init_cpu(void)
{
    EIMSK = 0x00; //屏蔽INT0~INT1的所有外部中断
    clr_bit(SFIOR, PUD); //设置输入上拉电阻有效

    DDRA = 0x1f; //1是输出，0是输入 PA0/PA1/PA2/PA3/PA4继电器输出(不带上拉)
    PORTA = 0xe0; //PA5/PA6/PA7 输入-低电平有效

    DDRB = 0xfe; //1是输出，0是输入
    PORTB = 0x01; //PINB0输入带上拉

    DDRC = 0xe7; //PC3/PC4为限位输入 上拉有效
    PORTC = 0xff;

    DDRD  = 0x40; //PIND6为输出，其余输入
    PORTD = 0xff;  //外接上拉，按键带上拉

    DDRE = 0xc2; //RXD0输入，且上拉有效,PE2/3/4/5输入,PE6/7输出
    PORTE = 0xfd; //TXD0输出,不带上拉

    DDRF = 0x1c; //PF0/1/5/6为限位输入 PF7为产品感应 上拉有效
    PORTF = 0xff;

    DDRG = 0xf8; //PG0/PG1气缸到位感应 上拉有效
    PORTG = 0x0f; //PG2输入

    init_usart0();
    init_TIMER0_OVF();
    init_TIMER1_OVF();
    init_posi_buffer();
    SEI();

    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    flagerr = 0;
}
/***************系统初始化函数 end ***********/
void cx743_auto(void)
{
    uchar init_cnt = 0;
    if(HasProduct() && isStarted == "false" && motor_reseting == "false")
    {
        if(!X_arrivalsOrigin() || !Y_arrivalsOrigin() || !Z_arrivalsOrigin())
        {
            motor_reset();
            return;
        }
        isStarted = "true";
        send_start(); //发送启动测试指令
        if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //未与上位机通讯成功
        {
            isStarted = "false";
            clr_bit(flag2, allowMotorFlag2);
            clr_bit(flag2, init_COMPFlag2);
            return;
        }
        else
        {
            standBy(); //等待清除成功接收标志位
            if(rx_buffer[14] == 0x52) //R->0x52 ReStart
            {
                isStarted = "false";
                clr_bit(flag2, allowMotorFlag2);
                clr_bit(flag2, init_COMPFlag2);
                return;
            }
            else if(rx_buffer[14] == 0x53) //S->0x53 Start
            {
                product_cylinderDOWN();
                while(!product_cylinderArrivals())
                {
                    if(init_cnt >= 3) break;
                    delay_nms(100);
                    init_cnt++;
                }
                //delay_nms(100);
                if(product_cylinderArrivals()) //产品气缸下压到位
                {
                    product_powerON();
                    delay_nms(500);
                    product_powerOFF();
                    delay_nms(1000);
                    product_powerON();
                    set_bit(flag2, init_COMPFlag2); //初始化完成
                    trans_init();
                }
                else
                {
                    isStarted = "false";
                    send_stop();
                    clr_bit(flag2, allowMotorFlag2);
                    clr_bit(flag2, init_COMPFlag2);
                    return;
                }
            }
        }
    }
}

//按键处理函数===============================================
void key_scan(void)
{
    if((flag1 && (1 << keyprq_flag1)) == 0)  //如果没有按键按下
    {
        if((PIND & (1 << key1)) == 0) //启动测试按键
        {
            key_now = 1;
        }
        /*
        else if((PIND & (1<<key2)) == 0)  //急停按键
        {
            key_now = 2;
        }//*/
        else
        {
            key_now = 0;
            key_old = 0;
            key_code = 0;
        }
        if(key_now != 0)
        {
            if(key_now != key_code)
            {
                key_code = key_now;
                key_cnt = 0;
            }
            else
            {
                key_cnt++;
                if(key_cnt >= KEY_COUNTER)
                {
                    set_bit(flag1, keyprq_flag1);
                }
            }
        }
    }
}
//按键处理函数===============================================
void key_process(void)
{
    if((flag1 & (1 << keyprq_flag1)) != 0)
    {
        clr_bit(flag1, keyprq_flag1);
        if(key_code == key_old)
        {
            ; //do nothing~
        }
        else
        {
            key_old = key_code;
            set_bit(flag1, keyeff_flag1);  //按键有效
        }
        if((flag1 & (1 << keyeff_flag1)) != 0)
        {
            clr_bit(flag1, keyeff_flag1);
            switch(key_old)
            {
            case 1:  //启动测试按键按下
            {
                clr_bit(PORTD, PD6);
                cx743_auto();
            }
            break;
            default:
                break;
            }
        }
    }
}
/***************主函数 start *******************/
void main(void)
{
    //init_TIMER0_OVF();
    //init_TIMER1_OVF();

    init_cpu();    //初始化CPU
	delay_nms(100);
    mcu_reset();
	delay_nms(100);
    while(1)
    {
        key_scan();
        key_process();
        delay_nms(10);
    }
}