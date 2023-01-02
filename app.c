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

static const int OUT_H = 0;
static const int OUT_L = 1;
static const int IN_H = 1;
static const int IN_L = 0;


static uint8_t ps2powsts = 0;
static int g_WaitCnt100us = 0;
static int g_WaitCnt100usTarget = 0;
static int g_Wait100us = 0;


// 各ピンの入出力設定は、PIN_MANAGER_Initialize()に記述されています。
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
	return PORTCbits.RC0;
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
  CLK_OUT(OUT_L);
  __delay_us(35);
  CLK_OUT(OUT_H);
  __delay_us(35);
  return true;
}

// @return false 送信禁止だった
static bool outputBit(const int dt)
{
	DAT_OUT(dt);
	__delay_us(5);
	CLK_OUT(OUT_L);
	__delay_us(35);
	CLK_OUT(OUT_H);
	__delay_us(35);
	return true;
}

static bool recvDataFromPS2(uint8_t *pData)
{
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
	if( (cnt & 0x01) == 0 )
		return false;	// 奇数パリティ・エラー

	if( !outputClock() )
		return false;

	// ストップビットを読む
	if(DAT_IN() == IN_L )
		return false;

	/* 応答ビットを書き込む */
	DAT_OUT(OUT_L);

	CLK_OUT(OUT_L);
  	__delay_us(20);
  	CLK_OUT(OUT_H);

  	__delay_us(20);
	DAT_OUT(OUT_H);
	
	*pData = data;
	return true;
}

static bool sendDataToPS2(uint8_t dt)
{
	// スタートビットを出力する
	if( !outputBit(OUT_L) )
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
	if( !outputBit(OUT_H) )
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

static void taskUSB()
{
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
				g_WaitCnt100us = 0;
				g_WaitCnt100usTarget = 10;
				break;
			}
		}
	}
	
	// SX-2側の電源状態が変化したらそれをPC側に通知する
	if( ps2powsts != PS2POW_IN() ){
		ps2powsts = PS2POW_IN();
		bReqInfo = true;
	}

	if (bReqInfo){
		static uint8_t mess[] = "\x9PS2USB:00";
		mess[9] = '0' + ps2powsts;
		putUSBUSART(mess, sizeof(mess)-1);		// -1 は文字列終端の分
	}
	return;
}

// PS/2 command.
enum PS2CMD
{
	PS2CMD_TEST		= 0xFF,
	PS2CMD_ECHO		= 0xEE,
	PS2CMD_LED		= 0xED,
	PS2CMD_IDREAD	= 0xF2,
	PS2CMD_ACK		= 0xFA,
	PS2CMD_TESTDONE	= 0xAA,
	PS2CMD_RESEND	= 0xfe,
};

void tasksub_ReceiveData(const uint8_t data, bool *pbWaitLed, enum PS2CMD *pLastData)
{
	switch(data)
	{
		case PS2CMD_LED:
		{
			t_PushBuff(&g_Buff, *pLastData=PS2CMD_ACK);
			g_WaitCnt100us = 0;
			g_WaitCnt100usTarget = 4;
			*pbWaitLed = true;
			break;
		}
		case PS2CMD_TEST:
		{
			t_PushBuff(&g_Buff, *pLastData=PS2CMD_TESTDONE);
			static uint8_t mess[2];
			mess[0] = 1;
			mess[1] = data;
			putUSBUSART(mess, sizeof(mess));	// putUSBUSARTに渡すポインタはstatic領域であること。
			g_WaitCnt100us = 0;
			g_WaitCnt100usTarget = 10;

// リファクタリングと、
// CLH=H、DAT=Lでも受信を開始する処理を追加する。

			break;
		}
		case PS2CMD_ECHO:
		{
			t_PushBuff(&g_Buff, *pLastData=PS2CMD_ECHO);
			break;
		}
		case PS2CMD_IDREAD:
		{
			t_PushBuff(&g_Buff, *pLastData=PS2CMD_ACK);
			g_WaitCnt100us = 0;
			g_WaitCnt100usTarget = 4;
			static uint8_t mess[2];
			mess[0] = 1;
			mess[1] = data;
			putUSBUSART(mess, sizeof(mess));
			// TODO: 返信
			break;
		}
		case PS2CMD_RESEND:
		{
			t_PushBuff(&g_Buff, *pLastData);
			static uint8_t mess[2];
			mess[0] = 1;
			mess[1] = data;
			putUSBUSART(mess, sizeof(mess));
			break;
		}
	}
	return;
}

// ホスト側はデータを送信したい場合、
//		CLKをHのままCLKをLにする（OCM version 3.8.2）
//		もしくは、 CLKをLしてCLKをLにする（OCM version 3.9.0、3.9.1）
// 予備知識：
//		PS/2 インターフェースは、ホスト(SX-2)とデバイス側(PS2VKBD)とは、CLK、DATの２本のラインで
//		双方向通信を行う。同じラインに対して両社が出力を行うため、出力を行いつつ、同じラインがどの
//		ような状態になっているかのチェックを行う、これの前提を理解しておくこと。
//		オープンコレクタによって出力を行っているので、両者の出力の論理積がラインの本当の状態になる。
//		例えば、両者がHを出力するとラインはHの状態になるが、片方がLを出力にするとラインはLになる。
//		デバイス側がHを出力してもホスト側がKを出力すれば、ラインはLの状態になる（デバイスがラインの
//		状態をチェックするとLが入力される）。逆も同じ。
// デバイス側は
//		はじめはCLK=H、DAT=Hとし状態遷移もアイドル状態（RXST_IDOL）とする
//		RXST_IDOLのとき、
//			DAT=Lを検出したら、RXST_STANBYRX状態に遷移する
//				これはSX-2側が何かを送信したいというサインなので、その準備を行うということ。
//				OCM-PLDのverによって送信したいサインは異なるが、DAT=Lは同じ
//					SX-2 OCM-PLD ver 3.8.2 では、CLK=H、DAT=L
//					SX-2 OCM-PLD ver 3.9.0、3.9.1 では、CLK=L、DAT=L
//			DAT=HかつCLK=Hで、SX-2へ送信したいデータがあれば（CLK=Hはホスト側がデバイス側に対して送信を許可しているということ）
//				送信処理を開始する。
//		ST=STANBY_RXのとき、
//			CLK=Lの場合、CLK=Hになるまで待機する（15ms以上経過してもCLK=Hの場合、いったんIDOL状態へ戻す）
//	 		CLK=Hだったら、RXST_RX(受信状態)に遷移する
//		RXST_RXのとき、
//			１バイト分の受信処理を行って、IDOL状態へ戻す
//	
// 現バージョン、対応できていない動作：
//	（１）デバイスから１バイトの送信している最中でもホストからの送信禁止の支持をチェックしなければならないのだろうが、
//			１バイト分を送信しきるまでチェックをしない。PS2VKBDの回路の設計が悪く、デバイスの出力がラインに範囲されるまで
//			少し時間がかかるため、CLK=Hを出力した直後にCLK=Hかどうかをチェックしても正確に確認できないため。
//	（２）複数バイト（ブレークコード等）をホストへ送信している途中に送信禁止になった場合、
//			最初のバイトから再送しなければならないが、それがまだできていない。
//
static void taskReceivePS2()
{
	enum PS2_RXST { RXST_IDOL, RXST_STANBYRX, RXST_RX};
	static enum PS2_RXST sts = RXST_IDOL;
	static bool bWaitLed = false;
	switch(sts)
	{
		case RXST_IDOL:
		{
			const uint8_t clk = CLK_IN();
			const uint8_t dat = DAT_IN();
			if (dat == IN_L) {
				sts = RXST_STANBYRX;
				g_Wait100us = 0;
			}
			else if(clk == IN_H) {
				// 送信許可で送信データがある場合は、PS/2への送信を行う
				uint8_t dt;
				if (!t_PopBuff(&g_Buff, &dt))
					break;
				if (g_WaitCnt100us < g_WaitCnt100usTarget )
					break;
				if( sendDataToPS2(dt) ){
					t_DelBtmBuff(&g_Buff);
					g_WaitCnt100us = 0;
				}
			}
			break;
		}
		case RXST_STANBYRX:
		{
			if (CLK_IN() == IN_H) {
				sts = RXST_RX;
				g_Wait100us = 0;
			}
			else {
				if (150 < ++g_Wait100us){	// 15ms Timeout
					sts = RXST_IDOL;
				}
			}
			break;
		}
		case RXST_RX:
		{
  			static enum PS2CMD lastData = PS2CMD_TESTDONE;
			uint8_t data;
			if (2000 < ++g_Wait100us){
				sts = RXST_IDOL;
			}
			// スタートビットの終わりまで待つ
			if( DAT_IN() == IN_H )
				break;
			// データビット、パリティの受信を行う。
			//	成功したら、受信データに応じた処理を行う。
			if (recvDataFromPS2(&data)) {
				if (bWaitLed) {
					bWaitLed = false;
					t_PushBuff(&g_Buff, lastData = PS2CMD_ACK);
					g_WaitCnt100us = 0;
					g_WaitCnt100usTarget = 4;
					static uint8_t mess[3];
					mess[0] = 2;
					mess[1] = PS2CMD_LED;
					mess[2] = data;
					putUSBUSART(mess, sizeof(mess));
				} 
				else {
					tasksub_ReceiveData(data, &bWaitLed, &lastData);
				}
			}
			sts = RXST_IDOL;
			break;
		}
	}
	return;
}

// タイマー割込みを使用すると 40us などの待ちを待ちを使用するのに__delay_us()を使用したときに、
// __delay_us()の精度が悪くなるので、タイマー割込みを使用しないようにしている。
static void taskTimeCount()
{
	static uint8_t timebase = 0;
	// メインループ1周は約25usかかるという実測に基づく。よって4下位カウントして100usを作り出す。
	// ソフト修正したら、周期を測定しなおす必要あり
	if (++timebase == 4) {
		timebase = 0;
		if(g_WaitCnt100us < 10)	// 1ms
			++g_WaitCnt100us;
		++g_Wait100us;
	}
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
	CLK_OUT(OUT_H);
	DAT_OUT(OUT_H);
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
	if (USBGetDeviceState() < CONFIGURED_STATE || USBIsDeviceSuspended() == true)
		return;
	taskReceivePS2();
	taskUSB();
	taskTimeCount();
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
