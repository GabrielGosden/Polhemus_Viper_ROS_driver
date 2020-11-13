#include <iostream>
#include <string>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <bitset>

#include "ViperInterface.h"
#include "Viper_setup_cmd.h"
#include "VPif3.h"
#include "USBfcns.h"
#include "VPPlatform.h"




int CmdStationMap (CStationMap &stamap)
{
	int r = 0;

	CVPcmd cmd;
	
	
	cmd.Fill(0, CMD_STATION_MAP, CMD_ACTION_GET);
	
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;

	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		cout << "Write CMD_ACTION_GET failed with error"  << endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		if (r == 0)
		{
			CFrameInfo fi(g_rxbuf, g_nrxcount);
		
			if ((fi.cmd() == CMD_STATION_MAP) && (fi.IsAck()))
			{
				CStationMap cmap((STATION_MAP*)fi.PCmdPayload());
				stamap = cmap;

				std::string s;
				cmap.Report(s);
				//cout << s << endl;
			}
		}
	}
	
	return r;

}

int CmdSingle()
{
	
	int r = 0;

	CVPcmd cmd;
	cmd.Fill(0, CMD_SINGLE_PNO, CMD_ACTION_GET);
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;

	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		//DisplayError(r, "Write CMD_ACTION_GET failed with error ");
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		//DisplayError(r, "CmdStationMap() Read : ");
		
		if (r == 0)
		{
			
			CFrameInfo fi(g_rxbuf, g_nrxcount);
			//cout << "cmd single pno"<< fi.cmd() << endl;
			if ((fi.cmd() == CMD_SINGLE_PNO) && (fi.IsAck()))
			{
				CVPSeuPno pno;
				int bytesextracted = pno.Extractseupno(fi.PCmdPayload());

				if (bytesextracted)
				{
					std::string s;
					pno.Report(s,true);
 					//chatter_pub.publish(s);
					//std_msgs::String msg;
					//cout << "\n" << endl;
					//cout << "Src | frame no. |       Bitfields     |      x        y      z     |    Az       El       Ro    |" << endl;
					//cout << s << endl;+

				}
				else
				r = -1;
			}else
			{
				r = -1;
			}
		}
	}
	return r;

}

int CmdHemisphere(CHemisphereCfg & hemcfg)
{
	int r = 0;

	//NOTE Arg1=-1, setting hemisphere for all sensors.  Can be set individually if needed.
	CVPcmd cmd;
	cmd.Fill(0, CMD_HEMISPHERE, CMD_ACTION_SET, -1, 0, hemcfg, hemcfg.Size());
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;
	

	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		//DisplayError(r, "Write CMD_ACTION_SET failed with error ");
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);

		if (r == 0)
		{
			//cout << "In CmdHemisphere" << endl;
			//out << g_nrxcount << endl;
			//cout << g_rxbuf << endl;
			CFrameInfo fi(g_rxbuf, g_nrxcount);
			//cout << "fi.cmd (CmdHemisphere = 0) = " << fi.cmd() << endl;
			if ((fi.cmd() != CMD_HEMISPHERE) || !(fi.IsAck()))
			{
				r = -1;
			}
		}
	}
	return r;

}



int CmdContPno(eCmdActions eAct)
{
	int r = 0;

	CVPcmd cmd;
	cmd.Fill(0, CMD_CONTINUOUS_PNO, eAct);
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;

	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		//DisplayError(r, "Write CMD_ACTION_SET failed with error ");
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);

		if (r == 0)
		{

			CFrameInfo fi(g_rxbuf, g_nrxcount);

			if ((fi.cmd() != CMD_CONTINUOUS_PNO) || !(fi.IsAck()))
			{
				r = -1;
			}
		}
	}
	return r;

}

int CmdStartCont()
{
	return CmdContPno(CMD_ACTION_SET);
}
int CmdStopCont()
{
	return CmdContPno(CMD_ACTION_RESET);
}


int PrintPNO()
{

	CmdSingle();
	/*
	int r = 0;
	r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		//DisplayError(r, "CmdStationMap() Read : ");
		
		if (r == 0)
		{
			
			CFrameInfo fi(g_rxbuf, g_nrxcount);
			//cout << "cmd single pno"<< fi.cmd() << endl;
			if ((fi.cmd() == CMD_CONTINUOUS_PNO) && (fi.IsAck()))
			{
				CVPSeuPno pno;
				int bytesextracted = pno.Extractseupno(fi.PCmdPayload());

				if (bytesextracted)
				{
					std::string s;
					pno.Report(s,true);
					cout << "\n" << endl;
					cout << "Src | frame no. |       Bitfields     |      x        y      z     |    Az       El       Ro    |" << endl;
					cout << s << endl;
				}
				else
				r = -1;
			}else
			{
				r = -1;
			}
		}
*/


}


int CmdSetUnits(CUnitsCfg & pos_units){
	int r = 0;

	CVPcmd cmd;
	CFrameInfo fi(g_rxbuf, g_nrxcount);

	cmd.Fill(0, CMD_UNITS, CMD_ACTION_SET, -1, 0, pos_units,sizeof(pos_units));
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;
	
	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		cout << "Write CMD_ACTION_SET failied with error" << endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
	
		if (r == 0)
		{	
			//cout << "fi.cmd (CmdSetUnits = 7) = " << fi.cmd() << endl;
			if ((fi.cmd() != CMD_UNITS) && (fi.IsAck()))
			{	
				
				r=-1;
			}
		}
	}
	return r;
}


int CmdSetFramerate (CEnumCfg & val){
	
	int r = 0;
	CFrameInfo fi(g_rxbuf, g_nrxcount);
	CVPcmd cmd;
	//cmd.Fill(0,CMD_FRAMERATE,CMD_ACTION_GET);
	cmd.Fill(0, CMD_FRAMERATE, CMD_ACTION_SET, -1, 0, val, sizeof(val));
	cmd.Prepare(g_txbuf, g_ntxcount);

	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;

	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		cout << "Write CMD_ACTION_SET failied with error" << endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;

		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		//cout << g_rxbuf << endl;
		if (r == 0)
		{
		
			
			if ((fi.cmd() != CMD_FRAMERATE) && (fi.IsAck()))
			{	
				
				std::string s;
				val.Report(s,true);
				//cout << s << endl;	
				//cout << "fi.cmd (CmdSetFramerate = 6) = " << fi.cmd() << endl;
				r=-1;
			}
		}
	}
	return r;
	
}


int CmdSetFFT (CEnumCfg & val){
	int r = 0;

	CVPcmd cmd;

	cmd.Fill(0, CMD_FTT_MODE, CMD_ACTION_SET, -1, 0, val, sizeof(val));
	cmd.Prepare(g_txbuf, g_ntxcount);

	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;
	
	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		cout << "Write CMD_ACTION_SET failied with error" << endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		if (r == 0)
		{
			
			CFrameInfo fi(g_rxbuf, g_nrxcount);
		
			if ((fi.cmd() != CMD_FTT_MODE) && (fi.IsAck()))
			{	
				cout << "fi.cmd = " << fi.cmd() << endl;
				cout << "fi.IsAck = " << fi.IsAck() << endl;
				r=-1;
			}
			if((fi.cmd() == CMD_FTT_MODE) && (fi.IsAck()))
			{

				std::string s;
				val.Report(s,true);
				//cout << s << endl;	
			}
		}
	}
	return r;
}


int CmdSetFilter(CFilterCfg & el){
	int r = 0;

	CVPcmd cmd;

	cmd.Fill(0, CMD_FILTER, CMD_ACTION_SET, -1, 0, el, sizeof(el));
	cmd.Prepare(g_txbuf, g_ntxcount);

	int nBytes = g_ntxcount;
	uint8_t *pbuf = g_txbuf;
	
	if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
	{
		cout << "Write CMD_ACTION_SET failied with error" << endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
		if (r == 0)
		{
			
			CFrameInfo fi(g_rxbuf, g_nrxcount);
			//cout << "fi.cmd (CmdSetFilter = 1) = " << fi.cmd() << endl;
			if ((fi.cmd() != CMD_FILTER) && (fi.IsAck()))
			{	
				r=-1;
			}
		}
	}
	return r;	
}


eViperFilterPresets int_to_eViperFilterPresets(int filter_level){
	if(filter_level==0)
	return FILTER_LVL_NONE;
	if(filter_level==1)
	return FILTER_LVL_LIGHT;
	if(filter_level==2)
	return FILTER_LVL_MEDIUM;
	if(filter_level==3)
	return FILTER_LVL_HEAVY;
	if(filter_level==4)
	return FILTER_LVL_CUSTOM;
	if(filter_level==5)
	return FILTER_LVL_E_LIGHT;
	if(filter_level==6)
	return FILTER_LVL_E_MEDIUM;
	if(filter_level==7)
	return FILTER_LVL_E_HEAVY;
	if(filter_level==8)
	return FILTER_LVL_RESERVED;



}