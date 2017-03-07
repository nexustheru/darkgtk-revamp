/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DARKGDK HEADER ///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Include
#include "DarkSDK.h"

// DarkGDK2 Additional Access
bool DarkGDKStart ( void );
void DarkGDKInit ( DWORD* pdwDisplayType, DWORD* pdwWidth, DWORD* pdwHeight, DWORD* pdwDepth, HINSTANCE* phInstance, LPSTR* ppApplicationName, HWND* ppParentHWND, DWORD* pdwInExStyle, DWORD* pdwInStyle);
bool CreateSDKApplication ( int w, int h,HWND parentwin,HINSTANCE hinstance);
bool CreateSDKApplication ( void );
int  DarkGDKEnd ( void );
bool LoadSDKDLLs ( void );
bool LoopGDK ( void );
bool StartSDK ( void );

bool EndSDK ( void );
