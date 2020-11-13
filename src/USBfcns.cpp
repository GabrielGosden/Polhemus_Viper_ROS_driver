/*-------------------------------------------------------------------------------------------------*/
/*																								   */
/* This file is originally created by Polhemus. 												   */
/* It has been modified by Gabriel Gosden,														   */
/* Email: s174865@student.dtu.dk as a part of the bachelor project:								   */
/* "Recreating Operator Paths Using an Electromagnetic Motion Tracker and a Robot Manipulator.	   */	
/* This work has been done for Teknologisk Institut, DMRI.										   */
/*																								   */
/*-------------------------------------------------------------------------------------------------*/
#include "USBfcns.h"
#include <iostream>
using namespace std;
int CreateVidPidList(libusb_context* pctx, libusb_device ** & devlist, uint16_t vid, uint16_t pid, vp_usbdevinfo arrDevInfo[], size_t & arrcount);
void find_endpoints(libusb_config_descriptor *conf_desc, int iface, uint8_t & ep_in, uint8_t & ep_out, uint16_t & out_pktsize);

int DiscoverVidPid(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinfo, uint16_t vid, uint16_t pid)
{

	int r = 0;

	/*Libusb init*/
	if (libusb_init(NULL)!=0){
		//cout << "Error initialsing libusb!" << endl;
	}else
	{
		//cout << "Libusb initialized succesfully!" << endl;
	}
	
	
	if (*usbhnd){
		//cout << "No USB found!" << endl;	
		ReleaseUSB(usbhnd, usbinfo);
	}
		

	vp_usbdevinfo arrDevInfo[VPUSB_MAX_DISCOVERABLE];
	size_t arrcount = VPUSB_MAX_DISCOVERABLE;
	libusb_device **devlist;

	if ((r = CreateVidPidList(NULL, devlist, vid, pid, arrDevInfo, arrcount)) == 0){
		return -1;
	}

	for (int d = 0; d < (int)arrcount; d++)
	{
		
		libusb_device * dev = devlist[arrDevInfo[d].usbDevIndex];
		libusb_device_handle * handle = 0;
		libusb_config_descriptor *conf_desc = 0;
		uint8_t nb_ifaces = 0, claimed_ifaces = 0;
		uint8_t ep_in = 0, ep_out = 0;
		uint16_t out_pktsize = 0;
		

		if ((r = libusb_open(dev, &handle)))
		{
		}
		else if ((r = libusb_get_config_descriptor(dev, 0, &conf_desc)))
		{
			libusb_close(handle);
		}
		else
		{
			nb_ifaces = conf_desc->bNumInterfaces;
			claimed_ifaces = 0;
			for (uint8_t i = 0; (i < nb_ifaces) && (r == 0); i++)
			{
				if ((r = libusb_claim_interface(handle, (int)i)))
				{
					//cout << "Error claiming Polhemus Viper!" << endl;
				}
				else
				{
					//cout << "Success claiming Polhemus Viper!" << endl;
					claimed_ifaces++;

					find_endpoints(conf_desc, (int)i, ep_in, ep_out, out_pktsize);
				}
			}
			
			if (claimed_ifaces == nb_ifaces)
			{
				*usbhnd = handle;
				usbinfo = arrDevInfo[d];
				usbinfo.usbNumInterfaces = nb_ifaces;
				usbinfo.ep_in = ep_in;
				usbinfo.ep_out = ep_out;
				usbinfo.epout_maxPktsize = out_pktsize;

				break;
			}
		}

		// clean up failed attempt before trying the next one.
		if (claimed_ifaces != nb_ifaces)
		{
			

			for (int k = 0; k < nb_ifaces; k++)
				libusb_release_interface(handle, k);
		}
		if (handle)
			libusb_close(handle);

		libusb_free_config_descriptor(conf_desc);
	}

	libusb_free_device_list(devlist, 1);
	if (!usbhnd)
		r = -99;
	return r;
}

int CreateVidPidList(libusb_context* pctx, libusb_device ** & devlist, uint16_t vid, uint16_t pid, vp_usbdevinfo arrDevInfo[], size_t & arrcount)
{
	int r = 0;

	ssize_t devcount = libusb_get_device_list(pctx, &devlist);
	//cout << "Found " << devcount << " devices!"<< endl;
	if (devcount < 0)
		return (int)devcount; // returns error code < 0

							  //struct libusb_device *found = NULL;
							  //struct libusb_device_handle *found_dev_handle = NULL;
	libusb_device *dev;
	int iFoundCount = 0;

	int iFoundArrIndex = -1;
	int i = 0;
	while ((dev = devlist[i]) != NULL)
	{
		struct libusb_device_descriptor desc;
		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0){
			break;
		}
			

		if (desc.idVendor == vid && desc.idProduct == pid)
		{
			
			iFoundCount++; iFoundArrIndex++;
			// populate the array if size permits
			if (iFoundCount <= (int)arrcount)
			{
				//cout << "Found Polhemus Viper!" << endl;
				vp_usbdevinfo * p = &arrDevInfo[iFoundArrIndex];
				p->usbDevIndex = i;
				p->usbBusNum = libusb_get_bus_number(dev);
				p->usbDevNum = libusb_get_device_address(dev);
				p->usbVID = vid; p->usbPID = pid;
			}
		}

		i++;
	}
	
	arrcount = iFoundCount;
	return arrcount;
	//return r;
}

void find_endpoints(libusb_config_descriptor *conf_desc, int iface, uint8_t & ep_in, uint8_t & ep_out, uint16_t & out_pktsize)
{
	for (int j = 0; j < conf_desc->interface[iface].num_altsetting; j++)
	{
		for (int k = 0; k < conf_desc->interface[iface].altsetting[j].bNumEndpoints; k++)
		{
			const struct libusb_endpoint_descriptor *p_ep;
			p_ep = &conf_desc->interface[iface].altsetting[j].endpoint[k];

			if ((p_ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT))
			{
				if (p_ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
				{
					if (!ep_in)
						ep_in = p_ep->bEndpointAddress;
				}
				else
				{
					if (!ep_out)
					{
						ep_out = p_ep->bEndpointAddress;
						out_pktsize = p_ep->wMaxPacketSize;
					}
				}
			}

		}
	}
}

int WriteUSB(libusb_device_handle *hnd, vp_usbdevinfo &usbinfo, uint8_t *pbuf, int & count, uint32_t timeout)
{
	int nActual = 0;
	int r = 0;

	if ((r = libusb_bulk_transfer(hnd, usbinfo.ep_out, pbuf, count, &nActual, timeout)))
	{
		//r = (r << 16) | E_VPERR_LIBUSB
	}
	else if (nActual != count)
	{
		count = nActual;
		r = -1;// E_VPUSB_ERR_WRITE_COUNT_WRONG;

	}
	else if ((nActual % usbinfo.epout_maxPktsize) == 0)
	{
		r = libusb_bulk_transfer(hnd, usbinfo.ep_out, nullptr, 0, &nActual, timeout);
	}

	return r;

}

int ReadUSB(libusb_device_handle *hnd, vp_usbdevinfo &usbinfo, uint8_t *pbuf, int & count, uint32_t timeout, bool bTOisErr/*=false*/)
{

	int r = 0;
	int nActual = 0;

	r = libusb_bulk_transfer(hnd, usbinfo.ep_in, pbuf, count, &nActual, timeout);
	// timeout is not an error
	//if ((r == LIBUSB_ERROR_TIMEOUT) && !ReadCmdTOIsError())
	if ((r == LIBUSB_ERROR_TIMEOUT) && !bTOisErr)
	{
		r = 0;
		count = 0;
	}
	else
		count = nActual;

	return r;
}

void ReleaseUSB(libusb_device_handle **hnd, vp_usbdevinfo &usbinfo)
{
	if ((hnd == 0) || (*hnd == 0))
		return;

	int r = 0;
	for (int i = 0; i < usbinfo.usbNumInterfaces; i++)
		r = libusb_release_interface(*hnd, i);

	libusb_close(*hnd);
	*hnd = 0;
	usbinfo = vp_usbdevinfo();
}
