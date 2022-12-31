/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/

/** INCLUDES *******************************************************/

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "mcc_generated_files/device_config.h"
#include "mcc_generated_files/mcc.h"
#include "app.h"
#include "usb.h"
#include "usb_config.h"

enum RXSTATE { ST_NONE, ST_LED, ST_SCANCODE };
static uint8_t ps2powsts = 0;
static int g_WaitCnt100us = 0;

// PS/2 command.
#define PS2CMD_TEST		0xFF
#define PS2CMD_ECHO		0xEE	
#define PS2CMD_LED		0xED
#define PS2CMD_IDREAD	0xF2
#define PS2CMD_ACK		0xFA
#define PS2CMD_TESTDONE	0xAA
#define PS2CMD_RESEND	0xfe


inline uint8_t PS2POW_IN()
{
	return PORTCbits.RC4;
}

inline uint8_t CLK_IN()
{
	return PORTCbits.RC1;
}

inline uint8_t DAT_IN()
{
	return PORTCbits.RC0; //PORTCbits.RC0;
}

inline void CLK_OUT(int t)
{
	LATCbits.LC3 = (uint8_t)t;
	return;
}

inline void DAT_OUT(int t)
{
	LATCbits.LC2 = (uint8_t)t;
	return;
}
static bool outputClock(void)
{
  if( CLK_IN() == 0 ){
	CLK_OUT(0);
	return false;
  }
  CLK_OUT(1);		// L
  __delay_us(40);
  CLK_OUT(0);		// H
  __delay_us(40);
  return true;
}

static bool outputBit(const int dt)
{
  if( CLK_IN() == 0 ){
	CLK_OUT(0);
	return false;
  }
  DAT_OUT(dt);
   __delay_us(20);
  CLK_OUT(1);		// L
  __delay_us(40);
  CLK_OUT(0);		// H
  __delay_us(20);
  return true;
}

static bool recvDatOS2(uint8_t *pData)
{
	/* 開始ビットを読む */
	if( DAT_IN() != 0 )
		return false;
	if( !outputClock() )
		return false;

	uint8_t cnt = 0;
	uint8_t data = 0;
	for(int t = 0; t < 8; ++t){
		/* データビットを読む */
		const uint8_t inDt = DAT_IN();
		cnt += inDt;
		data |= inDt << t;
		if( !outputClock() )
			return false;
	}
	/* パリティビットを読む */
	cnt += DAT_IN();
	if( !outputClock() )
		return false;
	if( (cnt & 0x01) == 0 )
		return false;	// 奇数パリティ・エラー
	/* 終了ビットを読む */
	if(DAT_IN() == 0 )
		return false;
	/* 応答ビットを書き込む */
	DAT_OUT(1);
	if( !outputClock() )
		return false;
	DAT_OUT(0);
	
	*pData = data;
	return true;
}

static bool sendDatOS2(uint8_t dt)
{
	/* 開始ビットを書き込む */
	if( !outputBit(1) )
		return false;

	uint8_t cnt = 0;
	for(int t = 0; t < 8; ++t){
		/* データビットを書き込む */
		uint8_t b = (dt >> t) & 0x01;
		cnt += b;
		b = (!b) & 0x01;
		if( !outputBit(b) )
			return false;
	}
	/* パリティビットを書き込む */
	if( !outputBit(cnt & 0x01) )
		return false;
	/* 終了ビットを書き込む */
	if( !outputBit(0) )
		return false;
	__delay_us(20);
  return true;
}

/*********************************************************************
*/
struct RINGBUFF
{
	int top;
	int btm;
	int len;
	uint8_t buff[CDC_DATA_OUT_EP_SIZE*2];
};
struct RINGBUFF g_Buff;

void t_InitBuff(struct RINGBUFF *p)
{
	p->top = 0;
	p->btm = 0;
	p->len = 0;
	return;
}

void t_PushBuff(struct RINGBUFF *p, const uint8_t dt)
{
	if( 20 <= p->len )
		return;
	p->buff[p->top++] = dt;
	p->len++;
	if(p->top == sizeof(p->buff))
		p->top = 0;
	return;
}
bool t_PopBuff(struct RINGBUFF *p, uint8_t *pDt)
{
	if( p->len == 0 )
		return false;
	*pDt = p->buff[p->btm];
	return true;
}
bool t_DelBtmBuff(struct RINGBUFF *p)
{
	if( p->len == 0 )
		return false;
	p->btm++;
	p->len--;
	if(p->btm == sizeof(p->buff))
		p->btm = 0;
	return true;
}

void MyTMR0Handler(void)
{
	if(g_WaitCnt100us < 20)	// 2ms
		++g_WaitCnt100us;
	return;
}

/*********************************************************************
* Function: void APP_Initialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_Initialize()
{
	ps2powsts = PS2POW_IN();
	t_InitBuff(&g_Buff);
	TMR0_StopTimer();
	TMR0_SetInterruptHandler(MyTMR0Handler);
	TMR0_StartTimer();
	return;
}

/*********************************************************************
* Function: void APP_Tasks(void);
*
* Overview: Keeps the demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_Initialize() and APP_DeviceCDCBasicDemoStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_Tasks()
{
	// PS/2への送信
	uint8_t dt;
	if( t_PopBuff(&g_Buff, &dt) ) {
		if( 20<=g_WaitCnt100us && CLK_IN() == 1 && DAT_IN() == 1 ){
			if( sendDatOS2(dt) ){
				t_DelBtmBuff(&g_Buff);
				g_WaitCnt100us = 0;
			}
		}
	}

	// USBの準備ができるまではここで処理終了
    if( USBGetDeviceState() < CONFIGURED_STATE )
        return;
    if( USBIsDeviceSuspended()== true )
        return;
	
	static uint8_t lastData = PS2CMD_TESTDONE;
	static enum RXSTATE st = ST_NONE;
	if( CLK_IN() == 1 && DAT_IN() == 0 ){
		uint8_t data;
		if( recvDatOS2(&data) ){
			if(st == ST_NONE){
				//----------------------------
				switch(data){
					case PS2CMD_LED: {
						t_PushBuff(&g_Buff, lastData=PS2CMD_ACK);
						st = ST_LED;
						break;
					}
					case PS2CMD_TEST:{
						t_PushBuff(&g_Buff, lastData=PS2CMD_TESTDONE);
						uint8_t mess[] = { 1, data};
						putUSBUSART(mess, sizeof(mess));
						break;
					}
					case PS2CMD_ECHO:{
						t_PushBuff(&g_Buff, lastData=PS2CMD_ECHO);
						break;
					}
					case PS2CMD_IDREAD:{
						t_PushBuff(&g_Buff, lastData=PS2CMD_ACK);
						uint8_t mess[] = { 1, data};
						putUSBUSART(mess, sizeof(mess));
						// TODO: 返信
						break;
					}
					case PS2CMD_RESEND:{
						t_PushBuff(&g_Buff, lastData);
						uint8_t mess[] = { 1, data};
						putUSBUSART(mess, sizeof(mess));
						break;
					}
				}

			}
			else if(st == ST_LED){
				t_PushBuff(&g_Buff, lastData = PS2CMD_ACK);
				uint8_t mess[] = { 2, PS2CMD_LED, data};
				putUSBUSART(mess, sizeof(mess));
				st = ST_NONE;
			}
		}
		else{
			t_PushBuff(&g_Buff, PS2CMD_RESEND);
		}
	}

	// USBからの受信
	bool bReqInfo = false;
	static uint8_t usbReadBuff[CDC_DATA_OUT_EP_SIZE];
	const int numBytes = getsUSBUSART(usbReadBuff, sizeof(usbReadBuff));
	if( 0 < numBytes)
	{
		switch( usbReadBuff[0] ){
			case 'I':
			{
				bReqInfo = true;
				break;
			}
			case 'S':
			{
				for(int t = 1; t < numBytes; ++t) {
					t_PushBuff(&g_Buff, usbReadBuff[t]); 
				}
				break;
			}
		}
	}
	
	if( ps2powsts != PS2POW_IN() ){
		ps2powsts = PS2POW_IN();
		bReqInfo = true;
	}

	if (bReqInfo){
		uint8_t mess[] = "\x9PS2USB:00";
		mess[9] = '0' + ps2powsts;
		putUSBUSART(mess, sizeof(mess)-1);
	}
	
	return;
}

void APP_SYSTEM_Initialize( APP_SYSTEM_STATE state )
{
    switch(state)
    {
        case APP_SYSTEM_STATE_USB_START:
            break;
            
        case APP_SYSTEM_STATE_USB_SUSPEND: 
            break;
            
        case APP_SYSTEM_STATE_USB_RESUME:
            break;
    }
}

#if(__XC8_VERSION < 2000)
    #define INTERRUPT interrupt
#else
    #define INTERRUPT __interrupt()
#endif

void INTERRUPT SYS_InterruptHigh(void)
{
    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif

	// TIMER0 Handler
    if(INTCONbits.TMR0IE == 1 && INTCONbits.TMR0IF == 1) {
        TMR0_ISR();
    }

}
