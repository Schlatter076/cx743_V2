#ifndef _COMMANDS_H
#define _COMMANDS_H


/*
 * 其中第14字节为命令类型字节 此时为0x02
 * stroke[2]-[5]为X行程值 stroke[6]-[9]为Y行程值 stroke[10]-[13]为Z行程值
 * 高字节在前
 */
uchar strokes[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0a};
//其中第14字节为命令类型字节 此时为0x20
uchar debug_strokes[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x020, 0x0a};
uchar start[] = {0xf3, 0xf4, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a};
uchar stop[] = {0xf3, 0xf4, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x0a};
uchar init[] = {0xf3, 0xf4, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x0a};
uchar reset[] = {0xf3, 0xf4, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0a};
//到达指定位置反馈数组
uchar spe_location[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x0a};

//行程超最大值报警
uchar timeout[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x0a};

#endif