/******************************************/
/*           ����������������޹�˾     */
/*Project:     cx743��ý�忪�� V2��       */
/*Guest:                                  */
/*Name:             main.c                */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ⲿ����11.0592MHz    */
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

//����ֵ��¼����
INT32 posi_buffer[16];


//==������������=============================
uchar key_now = 0;
uchar key_old = 0;
uchar key_code = 0;
uchar key_cnt = 0;
#define KEY_COUNTER 5
//�������λ��
typedef struct coordinate
{
    INT32 X;
    INT32 Y;
    INT32 Z;
} Position;
Position current_posi = {0, 0, 0}; //��ǰλ��
Position last_posi = {0, 0, 0}; //��¼λ��
Position de_posi = {0, 0, 0}; //����λ��
//==�Ƿ�ʼ����������ҷ��ر�������
INT32 X_paulse = 0; //����ʱ�������
INT32 X_runPaulse = 0; //����ʱ�������
INT32 X_paulseTemp = 0; //���������¼(�г�)
INT32 Y_paulse = 0; //����ʱ�������
INT32 Y_runPaulse = 0; //����ʱ�������
INT32 Y_paulseTemp = 0; //���������¼(�г�)
INT32 Z_paulse = 0; //����ʱ�������
INT32 Z_runPaulse = 0; //����ʱ�������
INT32 Z_paulseTemp = 0; //���������¼(�г�)
char *X_allowRun = "false"; //����X�������б�־
char *Y_allowRun = "false"; //����Y�������б�־
char *Z_allowRun = "false"; //����Z�������б�־
char *IO_status = "init"; //��λ��ָ������
char *motor_reseting = "false"; //�����λ״̬
char *isStarted = "false"; //����������־
char *sendPos = "false"; //�����г�(���嵱��)

volatile uchar Z_isConductive = FALSE; //��ͨ��־λ
volatile uchar X_isConductive = FALSE; //��ͨ��־λ
volatile uchar Y_isConductive = FALSE; //��ͨ��־λ
volatile uchar isDebug = FALSE; //���Ա�־
volatile uchar isDe_posi = FALSE; //�����߹̶�λ�ñ�־
volatile uchar isDe_reset = FALSE; //���Ի�ԭ���־
volatile uchar MOTOR_SPEED = 1; //��������ٶ�(ֵԽ���ٶ�Խ��)
uchar T_count = 0; //��Ч��������ٶȸ���
uchar step_counter = 0; //���Բ���������
#define RECORD_STEPS 1200
INT32 NG_STEP = RECORD_STEPS; //NGʱ�������������
INT32 PASS_STEP = RECORD_STEPS; //PASSʱ�������������
uchar limit_count = 0; //�����λ������(������)
uchar stop_count = 0; //��ͣ���¼�����(������)
uchar p2_count = 0; //��ѹ�����Ƶڶ������趨����������

#define X_REC 1700
#define Y_REC 1400
#define Z_REC 5300
INT32 x_re = 0; //������ڶ���������в���
INT32 y_re = 0; //������ڶ���������в���
INT32 z_re = 0; //������ڶ���������в���
INT32 z_comp = 0; //������ڶ���������в���
INT32 z_backStep = 0; //Z�����������
INT32 stroke_limit = 0; //��ѹ���⿪�����г�����

/***********USART0�����жϷ����� start**********************/
//USART���ջ�����
#define RX_BUFFER_SIZE 16                  //���ջ�������С���ɸ�����Ҫ�޸ġ�
unsigned char rx_buffer[RX_BUFFER_SIZE];   //������ջ�����
unsigned char rx_counter = 0;              //����rx_counterΪ����ڶ����е��ѽ��յ��ַ�������

//����һ����־λUsart0_RECVFlag1:=1��ʾ����0���յ���һ�����������ݰ�
//��port.h�ж���

#pragma interrupt_handler usart0_rxc_isr:19  //�����жϷ������
void usart0_rxc_isr(void)
{
    uchar status, data;
    status = UCSR0A;
    data = UDR0;
    if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //�ж��Ƿ��������һ���µ����ݰ�
    {
        if ((status & (USART0_FRAMING_ERROR | USART0_PARITY_ERROR | USART0_DATA_OVERRUN)) == 0)
        {
            rx_buffer[rx_counter] = data;
            rx_counter++;
            switch (rx_counter)
            {
            case 1:       // ������ʼ�ַ�
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
            case 16:      // ��������ַ�
            {
                rx_counter = 0;
                if (data == END_TEXT) set_bit(flag1, Usart0_RECVFlag1); // Usart0_RecvFlag=1����ʾ��ȷ���յ�һ�����ݰ�
            }
            break;
            default:
                break;
            }
        }
    }
}
/***************USART0�����жϷ����� end**********************/
/*============================================================*/
/*============================================================*/
/***************USART0�����жϷ����� start********************/
#define TX_BUFFER_SIZE 16
unsigned char tx_buffer[TX_BUFFER_SIZE];
unsigned char tx_wr_index = 0, tx_rd_index = 0, tx_counter = 0;

#pragma interrupt_handler usart0_txc_isr:21  //�����жϷ������
void usart0_txc_isr(void)
{
    if (tx_counter)//���в�Ϊ��
    {
        --tx_counter;//������
        UDR0 = tx_buffer[tx_rd_index];
        if (++tx_rd_index == TX_BUFFER_SIZE) tx_rd_index = 0;
    }
}
/***********USART0�����жϷ����� end**********************/

/*============================================================*/
/***********USART0����һ���ַ����� start**********************/
void USART0_putchar(unsigned char c)
{
    while (tx_counter == TX_BUFFER_SIZE);
    CLI();//#asm("cli")�ر�ȫ���ж�����
    if (tx_counter || ((UCSR0A & USART0_DATA_REGISTER_EMPTY) == 0)) //���ͻ�������Ϊ��
    {
        tx_buffer[tx_wr_index] = c; //���ݽ������
        if (++tx_wr_index == TX_BUFFER_SIZE) tx_wr_index = 0; //��������
        ++tx_counter;
    }
    else
        UDR0 = c;
    SEI(); //#asm("sei")��ȫ���ж�����
}
/***********USART0���ͷ����� end**********************/
//������������ֵ
void init_posi_buffer(void)
{
    uchar cnt = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        posi_buffer[cnt] = 0;
    }
}
//����λ�����ص�����ֵд��
void fill_posi_buffer(void)
{
    uchar cnt = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        posi_buffer[cnt] = rx_buffer[cnt];
    }
}
//�����г�ֵ(������)��亯��
void fill_stroke(void)
{
    *(strokes + 2) = (X_paulseTemp >> 24) & 0xff; //ȡ24-31λֵ
    *(strokes + 3) = (X_paulseTemp >> 16) & 0xff; //ȡ16-23λֵ
    *(strokes + 4) = (X_paulseTemp >> 8) & 0xff; //ȡ8-15λֵ
    *(strokes + 5) = X_paulseTemp & 0xff;       //ȡ0-7λֵ

    *(strokes + 6) = (Y_paulseTemp >> 24) & 0xff; //ȡ24-31λֵ
    *(strokes + 7) = (Y_paulseTemp >> 16) & 0xff; //ȡ16-23λֵ
    *(strokes + 8) = (Y_paulseTemp >> 8) & 0xff; //ȡ8-15λֵ
    *(strokes + 9) = Y_paulseTemp & 0xff;       //ȡ0-7λֵ

    *(strokes + 10) = (Z_paulseTemp >> 24) & 0xff; //ȡ24-31λֵ
    *(strokes + 11) = (Z_paulseTemp >> 16) & 0xff; //ȡ16-23λֵ
    *(strokes + 12) = (Z_paulseTemp >> 8) & 0xff; //ȡ8-15λֵ
    *(strokes + 13) = Z_paulseTemp & 0xff;       //ȡ0-7λֵ
}
//����ʱ�������λ��(������)
void fill_debugStroke(void)
{
    *(debug_strokes + 2) = (X_paulse >> 24) & 0xff; //ȡ24-31λֵ
    *(debug_strokes + 3) = (X_paulse >> 16) & 0xff; //ȡ16-23λֵ
    *(debug_strokes + 4) = (X_paulse >> 8) & 0xff; //ȡ8-15λֵ
    *(debug_strokes + 5) = X_paulse & 0xff;       //ȡ0-7λֵ

    *(debug_strokes + 6) = (Y_paulse >> 24) & 0xff; //ȡ24-31λֵ
    *(debug_strokes + 7) = (Y_paulse >> 16) & 0xff; //ȡ16-23λֵ
    *(debug_strokes + 8) = (Y_paulse >> 8) & 0xff; //ȡ8-15λֵ
    *(debug_strokes + 9) = Y_paulse & 0xff;       //ȡ0-7λֵ

    *(debug_strokes + 10) = (Z_paulse >> 24) & 0xff; //ȡ24-31λֵ
    *(debug_strokes + 11) = (Z_paulse >> 16) & 0xff; //ȡ16-23λֵ
    *(debug_strokes + 12) = (Z_paulse >> 8) & 0xff; //ȡ8-15λֵ
    *(debug_strokes + 13) = Z_paulse & 0xff;       //ȡ0-7λֵ
}
/**
 * ���ڷ��ͺ���
 * @buff[] �����͵��ֽ�����
 */
void send(unsigned char buff[])
{
    uchar i = 0;
	for(i = 0; i < TX_BUFFER_SIZE; i++)
    {
        USART0_putchar(buff[i]);
    }
}
//�����г�ֵ������
void sendStrokeToHost(void)
{
    fill_stroke();
	send(strokes);
}
//���͵��Ժõ�����ֵ����λ��
void sendDebugPosition(void)
{
    fill_debugStroke();
	send(debug_strokes);
}
//����ָ��λ��
void sendArrivals(void)
{
    send(spe_location);
}
//������������ֵ(�Լ�����)
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
//д�뵱ǰ����ֵ
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
//����λ�����͸�λָ��
void send_reset(void)
{
    send(reset);
}
//�����λ����
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
//ϵͳ��λ����
void mcu_reset(void)
{
    send_reset();
    motor_reset();
}
//��װָ��ͺ���
void function(void (*fuc)())
{
    uchar cnt = 0;
    do
    {
        if(cnt++ >= 3)
        {
            cnt = 0;
            break; //������Դ�������RETYR_TIMES,����ѭ��
        }
        (*fuc)();    //���Ͳ���ָ��
        delay_nms(300); //�ȴ���������ݷ���
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
//ͣ����NG״̬
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
        delay_nms(200); //�ȴ���������ݷ���
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
//��ʱ�жϺ���(�Զ������)
#pragma interrupt_handler timer1_count_isr:15
void timer1_count_isr(void)
{
    TCNT1 = 65536 - CRYSTAL / 8 / 2 * 0.0002; //��װ0.0002s��ʱ

    //====��ͣ��������һ�����������λ��=====
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
            //SREG = 0x00; //ȫ�ֽ�ֹ�ж�
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
            && (motor_reseting == "false") && ((flag2 & (1 << is_NGFlag2)) == 0) && ((flag2 & (1 << is_PASSFlag2)) == 0))  //�������������ٵ��
    {
        set_bit(flag2, Alarm_LIMITFlag2);
        MOTOR_SPEED = 50; //���ٵ��
    }
    //========��Ч����=====================================
    if(isStarted == "true" && ((flag2 & (1 << allowMotorFlag2)) != 0)) //���������У���������λ
    {
        clr_bit(flag2, allowMotorFlag2);  //�����־λ
        X_allowRun = "true";
        X_enable();
        Y_allowRun = "true";
        Y_enable();
        Z_allowRun = "true";
        Z_enable();
        set_bit(flag2, motor_GOFlag2);
    }
    //=========ִ�и���======================
    if((flag2 & (1 << motor_GOFlag2)) != 0)
    {
        //====��ָ������λ��===
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
        //====����ָ������λ��===
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
			Timer1_disable(); //���������
            sendArrivals();
			Timer1_enable();
            set_bit(flag2, motor_ARRIVALSFlag2);
            //clr_bit(flag2, motor_GOFlag2);
            MOTOR_SPEED = 30;
        }//*/
    }
    //=====ִ�а�����ѹ����=======
    if((flag2 & (1 << motor_ARRIVALSFlag2)) != 0)
    {
        clr_bit(flag2, motor_GOFlag2);
		//==�������趨��2��Ч
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
    //NGʱ�Ȼع鵽�м��
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
    //�˾Ų�NG--Z�����һ��

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

    //===�����ع�ԭλ
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
    //======��λ====
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
        if(!Z_arrivalsOrigin()) //�Ȼع�Z��
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
    //=====��λ���====


}
#pragma interrupt_handler timer0_isr:17
void timer0_isr(void)
{
    TCNT0 = 256 - CRYSTAL / 64 / 2 * 0.0003; //��װ0.0002s��ʱ(���������������ٶ�ɨ�裬�ɱ������ܷ�)

    //========����Ϊ���Բ�������==========
    if((flag1 & (1 << Usart0_RECVFlag1)) != 0 && isStarted == "false") //�յ�PC����������,��������û�п�ʼ //&& isStarted == "false"
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
        else if(IO_status == "de_posi") //�߹̶�����
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
    //=============����Ϊ����������============
    else if((flag1 & (1 << Usart0_RECVFlag1)) != 0 && isStarted == "true" && (flag2 & (1 << init_COMPFlag2)) != 0) //�ѳɹ��������ԣ�����λ����ʼ�����
    {
        clr_bit(flag1, Usart0_RECVFlag1);
        isDe_reset = FALSE;
        isDebug = FALSE;
        isDe_posi = FALSE;
        //====�����жϰ����Ƿ�ͨ====
        (IO_status == "X_conductive") ? (X_isConductive = TRUE) : (X_isConductive = FALSE);
        (IO_status == "Y_conductive") ? (Y_isConductive = TRUE) : (Y_isConductive = FALSE);
        (IO_status == "Z_conductive") ? (Z_isConductive = TRUE) : (Z_isConductive = FALSE);
        if(IO_status == "finished") //������ɣ���Ʒ�ϵ磬�����׹�λ�������λ
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
    //========��Ч����==============================================
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
            set_bit(PORTD, PD6); //��ָֹʾ��һֱ��
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
    //=====�����߹̶�����===================
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
    //===========���Ի�ԭ��=======
    if(isDe_reset == TRUE)
    {
        isDe_reset = FALSE;
        motor_reset();
    }

}
//��ʱ��0��ʼ��
void init_TIMER0_OVF(void)
{
    TCCR0 = 0x04; //64��Ƶ
    TCNT0 = 256 - CRYSTAL / 64 / 2 * 0.05; //0.05s��ʱ
    TIMSK |= (1 << TOIE0); //��ʱ��0�ж�ʹ��
    SREG = 0x80;
}
//��ʱ��1��ʼ��
void init_TIMER1_OVF(void)
{
    TCCR1B = 0x02; //8��Ƶ
    TCNT1 = 65536 - CRYSTAL / 8 / 2 * 0.05; //0.05s��ʱ
    TIMSK |= (1 << TOIE1); //��ʱ��ʹ��
    SREG = 0x80;    //ȫ��ʹ���ж�
}
/***************USART01��ʼ������ start*************************/
void init_usart0(void)
{
    UCSR0B = 0x00;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //�첽��8λ���ݣ�����żУ�飬һ��ֹͣλ���ޱ���
    UBRR0L = BAUD_L;                        //�趨������
    UBRR0H = BAUD_H;
    UCSR0A = 0x00;
    UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0); //0XD8  ���ա�����ʹ�ܣ� ���ж�ʹ��
}
/***************USART0��ʼ������ end***************************/
/***************ϵͳ��ʼ������ start ***********/
void init_cpu(void)
{
    EIMSK = 0x00; //����INT0~INT1�������ⲿ�ж�
    clr_bit(SFIOR, PUD); //������������������Ч

    DDRA = 0x1f; //1�������0������ PA0/PA1/PA2/PA3/PA4�̵������(��������)
    PORTA = 0xe0; //PA5/PA6/PA7 ����-�͵�ƽ��Ч

    DDRB = 0xfe; //1�������0������
    PORTB = 0x01; //PINB0���������

    DDRC = 0xe7; //PC3/PC4Ϊ��λ���� ������Ч
    PORTC = 0xff;

    DDRD  = 0x40; //PIND6Ϊ�������������
    PORTD = 0xff;  //�������������������

    DDRE = 0xc2; //RXD0���룬��������Ч,PE2/3/4/5����,PE6/7���
    PORTE = 0xfd; //TXD0���,��������

    DDRF = 0x1c; //PF0/1/5/6Ϊ��λ���� PF7Ϊ��Ʒ��Ӧ ������Ч
    PORTF = 0xff;

    DDRG = 0xf8; //PG0/PG1���׵�λ��Ӧ ������Ч
    PORTG = 0x0f; //PG2����

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
/***************ϵͳ��ʼ������ end ***********/
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
        send_start(); //������������ָ��
        if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //δ����λ��ͨѶ�ɹ�
        {
            isStarted = "false";
            clr_bit(flag2, allowMotorFlag2);
            clr_bit(flag2, init_COMPFlag2);
            return;
        }
        else
        {
            standBy(); //�ȴ�����ɹ����ձ�־λ
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
                if(product_cylinderArrivals()) //��Ʒ������ѹ��λ
                {
                    product_powerON();
                    delay_nms(500);
                    product_powerOFF();
                    delay_nms(1000);
                    product_powerON();
                    set_bit(flag2, init_COMPFlag2); //��ʼ�����
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

//����������===============================================
void key_scan(void)
{
    if((flag1 && (1 << keyprq_flag1)) == 0)  //���û�а�������
    {
        if((PIND & (1 << key1)) == 0) //�������԰���
        {
            key_now = 1;
        }
        /*
        else if((PIND & (1<<key2)) == 0)  //��ͣ����
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
//����������===============================================
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
            set_bit(flag1, keyeff_flag1);  //������Ч
        }
        if((flag1 & (1 << keyeff_flag1)) != 0)
        {
            clr_bit(flag1, keyeff_flag1);
            switch(key_old)
            {
            case 1:  //�������԰�������
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
/***************������ start *******************/
void main(void)
{
    //init_TIMER0_OVF();
    //init_TIMER1_OVF();

    init_cpu();    //��ʼ��CPU
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