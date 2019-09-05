
/******************************************/
/*           ����������������޹�˾     */
/*Project:      FCT��������               */
/*Guest:                                  */
/*Name:             default.h             */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ⲿ����11.0592MHz    */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

#ifndef _DEFAULT_H
#define _DEFAULT_H

#define uchar unsigned char
#define uint unsigned int
#define INT32 unsigned long

#define FIRST_TEXT   0xf3
#define SECOND_TEXT  0xf4
#define END_TEXT    0x0a  //�����ֽ�

#define BAUD 9600           //�����ʲ���9600b/s
#define CRYSTAL 11059200UL    //ϵͳʱ��11.0592M
 
//����Ͷ��岨�������ò���
#define BAUD_SETTING (unsigned int) ((unsigned long)CRYSTAL/(16*(unsigned long)BAUD)-1)
#define BAUD_H (unsigned char)(BAUD_SETTING>>8)
#define BAUD_L (unsigned char)(BAUD_SETTING)

//USART0
#define USART0_FRAMING_ERROR (1<<FE0)
#define USART0_PARITY_ERROR (1<<PE0)  //PE
#define USART0_DATA_OVERRUN (1<<DOR0)
#define USART0_DATA_REGISTER_EMPTY (1<<UDRE0)

//USART1
#define USART1_FRAMING_ERROR (1<<FE1)
#define USART1_PARITY_ERROR (1<<PE1)  //PE
#define USART1_DATA_OVERRUN (1<<DOR1)
#define USART1_DATA_REGISTER_EMPTY (1<<UDRE1)

#define set_bit(x,y) (x|=(1<<y)) //��1����
#define clr_bit(x,y) (x&=~(1<<y)) //��0����
#define xor_bit(x,y) (x^=(1<<y)) //ȡ������
#define bit(x) (1<<x)            //��ĳλ����
#define get_bit(x,y) (x&=(1<<y)) //ȡ��ĳλ 

#define TRUE 1
#define FALSE 0

#define LED_BLINK() (PORTD^=(1<<PD6))

#define X_RUN() (PORTC^=(1<<0))
#define X_enable() (PORTC|=(1<<1))
#define X_disable() (PORTC&=~(1<<1))
#define X_isDisabled() ((PINC&(1<<1))==0)
#define X_backward() (PORTC|=(1<<2))
#define X_forward() (PORTC&=~(1<<2))
#define X_DirIsBackward() ((PINC&(1<<2))!=0)
#define X_DirIsForward() ((PINC&(1<<2))==0)

#define X_arrivalsOrigin() ((PINC&(1<<3))==0)
#define X_arrivalsLimit() ((PINC&(1<<4))==0)

#define Y_RUN() (PORTC^=(1<<5))
#define Y_enable() (PORTC|=(1<<6))
#define Y_disable() (PORTC&=~(1<<6))
#define Y_isDisabled() ((PINC&(1<<6))==0)
#define Y_backward() (PORTC|=(1<<7))
#define Y_forward() (PORTC&=~(1<<7))
#define Y_DirIsBackward() ((PINC&(1<<7))!=0)
#define Y_DirIsForward() ((PINC&(1<<7))==0)

#define Y_arrivalsOrigin() ((PINF&(1<<0))==0)
#define Y_arrivalsLimit() ((PINF&(1<<1))==0)

#define Z_RUN() (PORTF^=(1<<2))
#define Z_enable() (PORTF|=(1<<3))
#define Z_disable() (PORTF&=~(1<<3))
#define Z_isDisabled() ((PINF&(1<<3))==0)
#define Z_forward() (PORTF|=(1<<4))
#define Z_backward() (PORTF&=~(1<<4))
#define Z_DirIsForward() ((PINF&(1<<4))!=0)
#define Z_DirIsBackward() ((PINF&(1<<4))==0)

#define Z_arrivalsOrigin() ((PINF&(1<<5))==0)
#define Z_arrivalsLimit() ((PINF&(1<<6))==0)

#define Timer1_enable() (TIMSK|=(1<<TOIE1))
#define Timer1_disable() (TIMSK&=~(1<<TOIE1))
#define Timer0_enable() (TIMSK|=(1<<TOIE0))
#define Timer0_disable() (TIMSK&=~(1<<TOIE0))

#define HasProduct() ((PINF&(1<<7))==0)
#define product_powerON() (PORTA|=(1<<0))
#define product_powerOFF() (PORTA&=~(1<<0))
#define Z_cylinderDOWN() (PORTA|=(1<<1))
#define Z_cylinderUP() (PORTA&=~(1<<1))
#define Z_cylinderArrivalsUP() ((PING&(1<<0))==0)
#define Z_cylinderArrivalsDOWN() ((PING&(1<<1))==0)
#define product_cylinderDOWN() (PORTA|=(1<<2))
#define product_cylinderUP() (PORTA&=~(1<<2))
#define product_cylinderArrivals() ((PINA&(1<<5))==0)
#define stopKey_clicked() ((PIND&(1<<5))==0)
#define NG() (PORTA|=(1<<3))
#define NG_CANCEL() (PORTA&=~(1<<3))
#define trunX_Y() (PORTA|=(1<<4))
#define trunY_X() (PORTA&=~(1<<4))
#define X_Y_Z_ArrivalsLimit() ((PING&(1<<2))==0)
#define X_Y_cylinderArrivalsOrigin() ((PINA&(1<<6))==0)
//�������趨�ڶ�������Ч(��ѹ������)
#define Rally_P2_effictive() ((PINB&(1<<0))==0)


#endif
