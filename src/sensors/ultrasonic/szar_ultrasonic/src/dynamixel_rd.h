#ifndef __DYNAMIXEL_RD_H
#define	__DYNAMIXEL_RD_H

#define Dynamixel_Reserved 0x00// reserved����������

#define Dynamixel_ID_Motor 0x01        //Instruction error��
#define Dynamixel_ID_Ul 0x02

/*********Instruction************/
#define Dynamixel_Instruction_Read 0x02         //Instruction read
#define Dynamixel_Instruction_Write 0x03       //Instruction Write
#define Dynamixel_Instruction_Return 0x55       //Instruction return����
#define Dynamixel_Instruction_Error 0x66        //Instruction error��


#define Dynamixel_State_Success      0
#define Dynamixel_State_Incomplete   0x01     //error
#define Dynamixel_State_overrange    0x02     //error
#define Dynamixel_State_Error        0x03

#define Dynamixel_Return_Position_Value_Max 150000
#define Dynamixel_Return_Success 150001
#define Dynamixel_Return_Error   150002


long Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start, unsigned char data_blk_size_end);
void  Dynamixel_Value_Dackward(long Value, unsigned char data_blk_size, unsigned char *Return_data);
unsigned short Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

unsigned char  CheckSum(unsigned char uData[], unsigned char leng);

unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data);
unsigned int Dynamixel_Slave_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data);
//�ӻ�����
unsigned char Dynamixel_Slave_Send( unsigned char *DataBuffer, unsigned char Address, unsigned char data_length, unsigned char *Return_Data);

//��������
unsigned char Dynamixel_Master_Send(unsigned char Dynamixel_ID,unsigned char Instruction, unsigned short Address, long *Value,
                                    unsigned char Value_Length, unsigned char *Return_Data);

#endif//
