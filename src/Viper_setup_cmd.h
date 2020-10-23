

#include "VPif3.h"
#include "USBfcns.h"

#define TX_BUF_SIZE  0x400
#define RX_BUF_SIZE  0x400


uint8_t g_txbuf[TX_BUF_SIZE];
int g_ntxcount;

uint8_t g_rxbuf[RX_BUF_SIZE];
int g_nrxcount;

libusb_device_handle *g_usbhnd = 0;
vp_usbdevinfo g_usbinfo;

int CmdStationMap(CStationMap & stamap);
int CmdSingle();
int CmdHemisphere(CHemisphereCfg & hemcfg);
int CmdSetUnits(CUnitsCfg & pos_units); 
int CmdSetFramerate (CEnumCfg & val); //Not working
int CmdSetFFT (CEnumCfg & val);
int CmdSetFilter(CFilterCfg & el);
int CmdStartCont();
int CmdStopCont();
int PrintPNO();
eViperFilterPresets int_to_eViperFilterPresets(int filter_level);