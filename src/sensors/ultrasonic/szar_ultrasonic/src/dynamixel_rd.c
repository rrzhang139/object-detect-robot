#include "dynamixel_rd.h"
#include "dynamixel_protocol.h"
#include "stdio.h"


#define Uart_Blk_Length 40

unsigned char Blk_Num = 0;
unsigned char Data_Blk[Uart_Blk_Length];

//��������
//0  1     2       3         4           5              ....
//aa bb  reserved length Instruction  Payload
unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
{
  unsigned char  i;
  unsigned char Data2[1];
  unsigned int Return_State = Dynamixel_State_Error;
  for (i = 0; i < Data_Length; i++)
  {
    if (Blk_Num < 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 2)
    {
//      if ((unsigned char)Data[i] == Dynamixel_Reserved)
//      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
//      }
//      else Blk_Num = 0;
    }

    else if (Blk_Num == 3) //Packet Length
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
    }

    else if (Blk_Num == 4)//Instruction
    {
      if (((unsigned char)Data[i] == Dynamixel_Instruction_Return) || ((unsigned char)Data[i] == Dynamixel_Instruction_Error) ||
          ((unsigned char)Data[i] == Dynamixel_Instruction_Read) || ((unsigned char)Data[i] == Dynamixel_Instruction_Write))
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }

      else Blk_Num = 0;
    }

    else if (Blk_Num == 5) //Payload
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;

      if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];

        Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);

        if (Data2[0] == Data[Data_Blk[3] + 3])
        {
          Return_State = Dynamixel_State_Success;
          Return_Data[0] = (unsigned char)Data_Blk[5];//Payload

          Blk_Num = 0;
        }
      }

    }

    else if (Blk_Num <= Data_Blk[3] + 4)
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;

      if (Blk_Num == Data_Blk[3] + 4)
      {
        Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk, Data_Blk[3] + 3), 1, Data2);
        // Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);
//             Serial.print(Data2[0]);
//       Serial.print(",");
//       Serial.print(Data_Blk[Data_Blk[3] + 3]);       Serial.print(",");
//
//       Serial.print("\r\n");
        if (Data2[0] == Data_Blk[Data_Blk[3] + 3])
        {
          Return_State = Dynamixel_State_Success;

          if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)//
          {
            Return_Data[0] = (unsigned char)Data_Blk[5];//Payload
          }
          else
          {
            Return_Data[0] = (unsigned char)Data_Blk[2];//id
            Return_Data[1] = (unsigned char)Data_Blk[5];//Address
            Return_Data[2] = Data_Blk[3] - 3;//
            for (char k = 0; k < Return_Data[2]; k++)
            {
              Return_Data[3 + k] = (unsigned char)Data_Blk[6 + k];
            }

          }
        }
        Blk_Num = 0;
      }
    }
    else if (Blk_Num >= Uart_Blk_Length)
      Blk_Num = 0;
  }

  return Return_State;
}
//从机接收
//0  1     2       3         4           5              ....
//aa bb  reserved length Instruction  Payload
unsigned int Dynamixel_Slave_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
{
  //地址，数据长度，数据
  unsigned char  i;
  unsigned char Data2[1];
  unsigned int Return_State = Dynamixel_State_Error;
  for (i = 0; i < Data_Length; i++)
  {
    if (Blk_Num < 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 2)
    {
//      if ((unsigned char)Data[i] == Dynamixel_Reserved)
//      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
//      }
//      else Blk_Num = 0;
    }

    else if (Blk_Num == 3) //Packet Length
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
    }

    else if (Blk_Num == 4)//Instruction
    {
      if (((unsigned char)Data[i] == Dynamixel_Instruction_Read) || ((unsigned char)Data[i] == Dynamixel_Instruction_Write))
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }

      else Blk_Num = 0;
    }

    else if (Blk_Num == 5)//Address
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;

    }

    else if (Blk_Num < Data_Blk[3] + 3)
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
      if (Blk_Num == Data_Blk[3] + 3)
      {
        Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk, Data_Blk[3] + 3), 1, Data2);
        //   Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);

        if (Data2[0] == Data[Data_Blk[3] + 3])
        {
          Return_State = Dynamixel_State_Success;
          if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Read)
          {
//            printf("Read data:instruction_Address \r\n");

            Return_Data[0] = (unsigned char)Data_Blk[4];//instruction
            Return_Data[1] = (unsigned char)Data_Blk[5];//Address
            Return_Data[2] = (unsigned char)Data_Blk[6];;//待读取长度
          }
          else//写指令
          {
            //            printf("write data:instruction_Address \r\n");
            Return_Data[0] = (unsigned char)Data_Blk[4];//instruction
            Return_Data[1] = (unsigned char)Data_Blk[5];//Address
            Return_Data[2] = Data_Blk[3] - 3;//写入长度
            for (char k = 0; k < Return_Data[2]; k++)
            {
              Return_Data[3 + k] = (unsigned char)Data_Blk[6 + k];
              //              printf("0x%X ", Return_Data[3 + k]);
            }

          }

        }
        Blk_Num = 0;
      }
    }
    else if (Blk_Num >= Uart_Blk_Length)
      Blk_Num = 0;
  }

  return Return_State;
}
//�ӻ�����
unsigned char Dynamixel_Slave_Send( unsigned char *DataBuffer, unsigned char Address, unsigned char data_length, unsigned char *Return_Data)
{
  unsigned char i = 0;
  unsigned char Length = 0;
  unsigned char Data[1];
  //Header
  for (i = 0; i < 2; i++)
  {
    Return_Data[Length] = Dynamixel_Header[i];
    Length++;
  }
  //Reserved
  Return_Data[Length] = Dynamixel_Reserved;
  Length++;

  //Packet Length
  Return_Data[Length] = data_length + 3;
  Length++;
  //Instruction
  Return_Data[Length] = Dynamixel_Instruction_Return;
  Length++;

  //Data
  Return_Data[Length] = Address;
  Length++;
  for (i = Address; i < data_length + Address; i++)
  {
    Return_Data[Length] = DataBuffer[i];
    Length++;
  }

  // Dynamixel_Value_Dackward(CheckSum(Return_Data, Length), 1, Data);
  Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Return_Data, Length), 1, Data);
  Return_Data[Length] = Data[0];
  Length++;

  return Length;
}
//��������
unsigned char Dynamixel_Master_Send(unsigned char Dynamixel_ID,unsigned char Instruction, unsigned short Address, long *Value,
                                    unsigned char Value_Length, unsigned char *Return_Data)
{
  unsigned char i = 0;
  unsigned char Data4[4];
  unsigned char Data[1];
  unsigned char Length = 0;

  //	Send_Data = (unsigned char*)malloc(sizeof(unsigned char) *(Value_Length+5+7));

  //Header
  for (i = 0; i < 2; i++)
  {
    Return_Data[Length] = Dynamixel_Header[i];
    Length++;
  }

  //Reserved
  Return_Data[Length] = Dynamixel_ID;
  Length++;

  //Packet Length
  if (Instruction == 0x02)
  {
    Return_Data[Length] = 0x04;
    Length++;
  }
  else//0x03
  {
    if (Address == 0x09)
    {
      Return_Data[Length] = 12 + 3;
      Length++;
    }
    else
    {
      Return_Data[Length] = Value_Length + 3;
      Length++;
    }

  }

  //Instruction
  Return_Data[Length] = Instruction;
  Length++;

  //Start Address
  Return_Data[Length] = Address;
  Length++;

  //Data
  if (Instruction == 0x02)
  {
    Return_Data[Length] = Value_Length;
    Length++;
  }
  else//0x03
  {
    if (Address == 0x09)
    {
      for (int j = 0; j < 6; j++)
      {
        Dynamixel_Value_Dackward(Value[j], 2, Data4);
        for (i = 0; i < 2; i++)
        {
          Return_Data[Length] = Data4[i];
          Length++;
        }
      }

    } else//
    {
      Dynamixel_Value_Dackward(Value[0], Value_Length, Data4);
      for (i = 0; i < Value_Length; i++)
      {
        Return_Data[Length] = Data4[i];
        Length++;
      }
    }
  }

  //CRC
  Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Return_Data, Length), 1, Data);
  // Dynamixel_Value_Dackward(CheckSum(Return_Data, Length), 1, Data);
  Return_Data[Length] = Data[0];
  Length++;

  // printf("send data is:\r\n");
  for (int i = 0; i < Length; i++)
  {
    //  printf("%02x \n", Return_Data[i]);
  }

  return Length;

}

long Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start, unsigned char data_blk_size_end)
{
  long Value = 0x00000000;
  unsigned char i;
  //if(data_blk_size_start == data_blk_size_end)
  //return Dynamixel_Return_Success;
  for (i = data_blk_size_end; i >= data_blk_size_start; i--)
  {
    Value = Value << 8;
    Value = Value | data_blk_ptr[i];
  }
  if(data_blk_size_end-data_blk_size_start<2)
  {
    if (Value > (0xffff/2-100) )
      Value -= (0xffff + 1);
  }
  else  if (Value > (0xffffffff/2-1000))
      Value -= (0xffffffff + 1);
  return Value;
}

void  Dynamixel_Value_Dackward(long Value , unsigned char data_blk_size , unsigned char *Return_data)
{
  unsigned char i;
  for (i = 0; i < data_blk_size; i++)
  {
    Return_data[i] = Value & 0xff;
    Value = Value >> 8;
  }
}

unsigned short Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ Dynamixel_crc_table[i];
  }

  return crc_accum;
}

unsigned char  CheckSum(unsigned char uData[], unsigned char leng)
{
  char checksum = uData[0];
  for (int i = 0; i < leng; i++)
  {
    checksum ^= uData[i];
  }

  return checksum;
}
