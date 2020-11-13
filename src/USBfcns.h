/*-------------------------------------------------------------------------------------------------*/
/*																								   */
/* This file is originally created by Polhemus. 												   */
/* It has been modified by Gabriel Gosden,														   */
/* Email: s174865@student.dtu.dk as a part of the bachelor project:								   */
/* "Recreating Operator Paths Using an Electromagnetic Motion Tracker and a Robot Manipulator.	   */	
/* This work has been done for Teknologisk Institut, DMRI.										   */
/*																								   */
/*-------------------------------------------------------------------------------------------------*/


#pragma once
#include <libusb-1.0/libusb.h>
#include <cstring>

#define VPUSB_MAX_DISCOVERABLE 16 
#define VPUSB_WRITE_TIMEOUT_MS 50
#define VPUSB_READ_TIMEOUT_MS 100

typedef struct _vp_usbdevinfo {
	int     usbDevIndex;
	uint8_t usbBusNum;
	uint8_t usbDevNum;
	uint16_t usbVID;
	uint16_t usbPID;
	uint8_t usbNumInterfaces;
	uint8_t ep_in;
	uint8_t ep_out;
	uint16_t epout_maxPktsize;

	_vp_usbdevinfo() : usbDevIndex(-1), usbBusNum(0), usbDevNum(0), usbVID(0), usbPID(0), usbNumInterfaces(0), ep_in(0), ep_out(0), epout_maxPktsize(64) {};
	_vp_usbdevinfo(_vp_usbdevinfo & rv) { memcpy(&usbDevIndex, &rv, sizeof(_vp_usbdevinfo)); }
	_vp_usbdevinfo & operator = (const _vp_usbdevinfo & rv) { memcpy(&usbDevIndex, &rv, sizeof(_vp_usbdevinfo)); return *this; }

}vp_usbdevinfo;

int DiscoverVidPid(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinf, uint16_t VID, uint16_t PID);
void ReleaseUSB(libusb_device_handle **, vp_usbdevinfo &);
//int CreateVidPidList(libusb_context* pctx, libusb_device ** & devlist, uint16_t vid, uint16_t pid, vp_usbdevinfo arrDevInfo[], size_t & arrcount);
//void find_endpoints(libusb_config_descriptor *conf_desc, int iface, uint8_t & ep_in, uint8_t & ep_out, uint16_t & out_pktsize);
int WriteUSB(libusb_device_handle *usbhnd, vp_usbdevinfo &usbinfo, uint8_t *pbuf, int & count, uint32_t timeout = VPUSB_WRITE_TIMEOUT_MS);
// pbuf	 : in  :pointer to buffer to place input data
// count : in  : size of pbuf buffer. 
//       : out : number of bytes read into buffer
int ReadUSB(libusb_device_handle *usbhnd, vp_usbdevinfo &usbinfo, uint8_t *pbuf, int & count, uint32_t timeout = VPUSB_READ_TIMEOUT_MS, bool bTOisErr = false);

