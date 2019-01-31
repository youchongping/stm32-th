/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.32                          *
*        Compiled Oct  8 2015, 11:59:02                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "DIALOG.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"
#include "bsp_ds18b20.h"
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0     (GUI_ID_USER + 0x00)
#define ID_SLIDER_0     (GUI_ID_USER + 0x01)
#define ID_TEXT_0     (GUI_ID_USER + 0x03)
#define ID_TEXT_1     (GUI_ID_USER + 0x04)
#define ID_TEXT_2     (GUI_ID_USER + 0x05)
#define ID_TEXT_3     (GUI_ID_USER + 0x06)
#define ID_TEXT_4     (GUI_ID_USER + 0x07)
#define ID_TEXT_5     (GUI_ID_USER + 0x08)
#define ID_TEXT_6     (GUI_ID_USER + 0x08)

#define WM_USER_SET_TO_CHANGED_BY_APP (WM_USER + 0X01)
#define WM_USER_WIFI_STATUS_CHANGED   (WM_USER + 0X02)
#define WM_USER_CURRENT_TEMP_CHANGED   (WM_USER + 0X03)

// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, -2, 480, 320, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "Slider", ID_SLIDER_0, 30, 241, 420, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "00", ID_TEXT_0, 138, 91, 141, 89, 0, 0x0, 0 },
  { TEXT_CreateIndirect, ".0", ID_TEXT_1, 265, 119, 55, 57, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "00.0", ID_TEXT_2, 247, 208, 51, 28, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "set to:", ID_TEXT_3, 191, 211, 55, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "wifi disconnected", ID_TEXT_4, 380, 0, 100, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "\xb0\x43", ID_TEXT_5, 330, 91, 55, 57, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "\xb0\x43", ID_TEXT_6, 290, 212, 51, 28, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};
 static int temp=0;
/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  char	  set_to[10];
  char	  current_temp[10];
  int	  set_temprature;
 
  // USER START (Optionally insert additional variables)
  // USER END
  
  if(temp>500)temp=0;
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Window'
    //
    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(GUI_DARKRED));
    //
    // Initialization of '00'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, GUI_FONT_D80);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00000000));
    //
    // Initialization of '.0'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetFont(hItem, GUI_FONT_D48);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of '00.0'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, GUI_FONT_20B_1);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'set to:'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_20B_1);
    //
    // Initialization of 'wifi disconnect'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    // USER START (Optionally insert additional code for further widget initialization)
	 hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
	 SLIDER_SetRange(hItem,0,500);
	 hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
     TEXT_SetFont(hItem, GUI_FONT_32B_1);
     TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
	 hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
     TEXT_SetFont(hItem, GUI_FONT_20B_1);
     TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_SLIDER_0: // Notifications sent by 'Slider'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
		  set_temprature=SLIDER_GetValue(hItem);
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
		  memset(set_to,0,sizeof(set_to));
		  sprintf(set_to,"%d.%d",set_temprature/10,set_temprature%10);
		  TEXT_SetText(hItem,set_to);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
	case WM_USER_SET_TO_CHANGED_BY_APP:
		 hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
		 SLIDER_SetValue(hItem,(temp)%500);
		break;
	case WM_USER_WIFI_STATUS_CHANGED:
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
		  if(temp%2==0)
		  {
			  
			  TEXT_SetText(hItem,"wifi linked");
		  }
		  else
		  {
			  TEXT_SetText(hItem,"wifi disconnected");
		  }
		break;
	case WM_USER_CURRENT_TEMP_CHANGED:
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		  memset(current_temp,0,sizeof(current_temp));
		  sprintf(current_temp,"%d",tempratrue_now/16);
		  TEXT_SetText(hItem,current_temp);
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
		  memset(current_temp,0,sizeof(current_temp));
		  sprintf(current_temp,".%d",(int)((tempratrue_now%16)*10/16));
		  TEXT_SetText(hItem,current_temp);
		break;
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateWindow
*/
WM_HWIN CreateWindow(void);
WM_HWIN CreateWindow(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/

void MainTask(void)
{
  char buf[20]={0};
	WM_HWIN hDlg;
	WM_HWIN ClientWindow;
	WM_SetCreateFlags(WM_CF_MEMDEV);
	GUI_Init();
	hDlg = CreateWindow();
	ClientWindow = WM_GetClientWindow(hDlg);
	while(1)
	{
		//WM_SendMessageNoPara(ClientWindow, WM_USER_SET_TO_CHANGED_BY_APP);
		//WM_SendMessageNoPara(ClientWindow, WM_USER_WIFI_STATUS_CHANGED);
		
		while (xQueueReceive(public_queque, (void *) buf, 0) == pdTRUE)
		{
			if(strncmp(buf,"temp_changed",strlen("temp_changed")) == 0)
                {
									WM_SendMessageNoPara(ClientWindow, WM_USER_CURRENT_TEMP_CHANGED);
								}
		}
		
		GUI_Delay(100);
		temp++;
	}

}



