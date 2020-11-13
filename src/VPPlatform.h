/////////////////////////////////////////////////////////////////////
// This open-source software sample is offered free of charge and without guarantee.
// Polhemus accepts no responsibility or liability for any use of this code.
/////////////////////////////////////////////////////////////////////
/* VSS $Header: /Viper/VPlibusb.root/VPlibusb/VPlibusb/VPPlatform.h 3     8/31/18 4:01p Suzanne $
*/

#pragma once


#ifndef _WINDOWS_

//#define LARGE_INTEGER void
//
//bool QueryPerformanceFrequency(LARGE_INTEGER *x){return false;}
//bool QueryPerformanceCounter(LARGE_INTEGER *x){return false;}

#ifndef PVOID
typedef void *PVOID;
#endif
typedef void *HANDLE;
typedef char * LPSTR;
typedef char const * LPCSTR;
typedef uint32_t DWORD;
typedef uint8_t BYTE;
typedef unsigned short WORD;
typedef struct _DCB {
	DWORD DCBlength;      /* sizeof(DCB)                     */
	DWORD BaudRate;       /* Baudrate at which running       */
	DWORD fBinary : 1;     /* Binary Mode (skip EOF check)    */
	DWORD fParity : 1;     /* Enable parity checking          */
	DWORD fOutxCtsFlow : 1; /* CTS handshaking on output       */
	DWORD fOutxDsrFlow : 1; /* DSR handshaking on output       */
	DWORD fDtrControl : 2;  /* DTR Flow control                */
	DWORD fDsrSensitivity : 1; /* DSR Sensitivity              */
	DWORD fTXContinueOnXoff : 1; /* Continue TX when Xoff sent */
	DWORD fOutX : 1;       /* Enable output X-ON/X-OFF        */
	DWORD fInX : 1;        /* Enable input X-ON/X-OFF         */
	DWORD fErrorChar : 1;  /* Enable Err Replacement          */
	DWORD fNull : 1;       /* Enable Null stripping           */
	DWORD fRtsControl : 2;  /* Rts Flow control                */
	DWORD fAbortOnError : 1; /* Abort all reads and writes on Error */
	DWORD fDummy2 : 17;     /* Reserved                        */
	WORD wReserved;       /* Not currently used              */
	WORD XonLim;          /* Transmit X-ON threshold         */
	WORD XoffLim;         /* Transmit X-OFF threshold        */
	BYTE ByteSize;        /* Number of bits/byte, 4-8        */
	BYTE Parity;          /* 0-4=None,Odd,Even,Mark,Space    */
	BYTE StopBits;        /* 0,1,2 = 1, 1.5, 2               */
	char XonChar;         /* Tx and Rx X-ON character        */
	char XoffChar;        /* Tx and Rx X-OFF character       */
	char ErrorChar;       /* Error replacement char          */
	char EofChar;         /* End of Input character          */
	char EvtChar;         /* Received Event character        */
	WORD wReserved1;      /* Fill for now.                   */
} DCB, *LPDCB;


#else

//#include <WinSock2.h>
//#include <Iphlpapi.h>
//#include <WS2tcpip.h>

#ifdef _DEBUG
#ifndef DEBUG_NEW
#define DEBUG_NEW   new( _CLIENT_BLOCK, __FILE__, __LINE__)
#endif
#endif

#endif

#ifndef __TCHAR_DEFINED
#ifdef _UNICODE
#include <wchar.h>
#define tstringbuf      std::wstringbuf
#define tstring         std::wstring
#define _stprintf_s     swprintf_s
#define _sntprintf_s    snwprintf_s
#define _ftprintf       fwprintf
#define _tcslen         wcslen
#define _tcscpy_s       wcscpy_s
#define _tcsncpy        wcsncpy
#define _tfsopen        _wfsopen
#define _vsntprintf_s   _vsnwprintf_s
#define _tcsftime       wcsftime
#define _tcsftime_l     _wcsftime_l
#define _stscanf_s      swscanf_s
#define _stscanf_s_l    _swscanf_s_l
#define _fputts         fputws
typedef wchar_t         WCHAR;
typedef wchar_t const*  LPCTSTR;
typedef wchar_t *       LPTSTR;
typedef wchar_t         TCHAR;
#define _T(x)	L ## x
#else
#define tstringbuf      std::stringbuf
#define tstring         std::string
#define _stprintf_s     sprintf_s
#define _ftprintf       fprintf
#define _sntprintf_s    _snprintf_s
#define _tcslen         strlen
#define _tcscpy_s       strcpy_s
#define _tcsncpy        strncpy
#define _tfsopen        _fsopen
#define _vsntprintf_s   _vsnprintf_s
#define _tcsftime       strftime
#define _tcsftime_l     _strftime_l
#define _stscanf_s      sscanf_s
#define _stscanf_s_l    _sscanf_s_l
#define _fputts         fputs
typedef LPCSTR          LPCTSTR;
typedef LPSTR           LPTSTR;
typedef char            TCHAR;
#define _T(x)  x
#endif
#endif



