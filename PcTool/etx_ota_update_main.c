
/**************************************************

file: etx_ota_update_main.c
purpose: 

compile with the command: gcc etx_ota_update_main.c RS232\rs232.c -IRS232 -Wall -Wextra -o2 -o etx_ota_app

**************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

#define ETX_OTA_MAX_BLOCK_SIZE ( 1024 )
#define ETX_OTA_MAX_FW_SIZE    ( ETX_OTA_MAX_BLOCK_SIZE * 48 )

uint8_t APP_BIN[ETX_OTA_MAX_FW_SIZE];

void delay(uint32_t us)
{
#ifdef _WIN32
    //Sleep(ms);
    __int64 time1 = 0, time2 = 0, freq = 0;

    QueryPerformanceCounter((LARGE_INTEGER *) &time1);
    QueryPerformanceFrequency((LARGE_INTEGER *)&freq);

    do {
        QueryPerformanceCounter((LARGE_INTEGER *) &time2);
    } while((time2-time1) < us);
#else
    usleep(us);
#endif
}

int main(int argc, char *argv[])
{
  int comport;
  int bdrate   = 115200;       /* 115200 baud */
  char mode[]={'8','N','1',0}; /* *-bits, No parity, 1 stop bit */
  char bin_name[1024];
  int ex = 0;
  FILE *Fptr = NULL;

  do
  {
    if( argc <= 2 )
    {
      printf("Please feed the COM PORT number and the Application Image....!!!\n");
      printf("Example: .\\etx_ota_app.exe 8 ..\\..\\Application\\Debug\\Blinky.bin");
      ex = -1;
      break;
    }

    //get the COM port Number
    comport = atoi(argv[1]) -1;
    strcpy(bin_name, argv[2]);

    printf("Opening COM%d...\n", comport+1 );

    if( RS232_OpenComport(comport, bdrate, mode, 0) )
    {
      printf("Can not open comport\n");
      ex = -1;
      break;
    }

    printf("Opening Binary file : %s\n", bin_name);

    Fptr = fopen(bin_name,"rb");

    if( Fptr == NULL )
    {
      printf("Can not open %s\n", bin_name);
      ex = -1;
      break;
    }

    fseek(Fptr, 0L, SEEK_END);
    uint32_t app_size = ftell(Fptr);
    fseek(Fptr, 0L, SEEK_SET);

    printf("File size = %d\n", app_size);

    if( app_size > ETX_OTA_MAX_FW_SIZE )
    {
      printf("Application Size is more than the Maximum Size (%dKb)\n", ETX_OTA_MAX_FW_SIZE/ETX_OTA_MAX_BLOCK_SIZE);
      ex = -1;
      break;
    }

    //read the full image
    if( fread( APP_BIN, 1, app_size, Fptr ) != app_size )
    {
      printf("App/FW read Error\n");
      ex = -1;
      break;
    }

    unsigned char rec;

    printf("Waiting for OTA Start\n");
    do
    {
      RS232_PollComport( comport, &rec, 1);
    } while( rec != 'g' );

    printf("Sending r...\r\n");
    RS232_SendByte(comport, 'r');

    printf("Sending FW Size...\n");
    do
    {
      RS232_PollComport( comport, &rec, 1);
    } while( rec != 'y' );
    
    if( RS232_SendByte(comport, (uint8_t)app_size))
    {
      //some data missed.
      printf("OTA DATA : Send Err\n");
      ex = -1;
      break;
    }

    do
    {
      RS232_PollComport( comport, &rec, 1);
    } while( rec != 'x' );
    
    if( RS232_SendByte(comport, (uint8_t)(app_size >> 8)) )
    {
      //some data missed.
      printf("OTA DATA : Send Err\n");
      ex = -1;
      break;
    }

    printf("Sending Data...\n");

    for( uint32_t i = 0; i < app_size;)
    {
      do
      {
        RS232_PollComport( comport, &rec, 1);
      } while( rec != 'y' );
      
      if( RS232_SendByte(comport, APP_BIN[i++]) )
      {
        //some data missed.
        printf("OTA DATA : Send Err\n");
        ex = -1;
        break;
      }

      do
      {
        RS232_PollComport( comport, &rec, 1);
      } while( rec != 'x' );
      
      if( RS232_SendByte(comport, APP_BIN[i++]) )
      {
        //some data missed.
        printf("OTA DATA : Send Err\n");
        ex = -1;
        break;
      }
      if( ( i % ETX_OTA_MAX_BLOCK_SIZE ) == 0 )
      {
        printf("Block %d Transmitted...\r\n", i/ETX_OTA_MAX_BLOCK_SIZE);
      }
      
    }

    if( ex < 0 )
    {
      break;
    }

  } while (false);

  if(Fptr)
  {
    fclose(Fptr);
  }

  if( ex < 0 )
  {
    printf("OTA ERROR\n");
  }
  return(ex);
}

