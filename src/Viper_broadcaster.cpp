
/*-------------------------------------------------------------------------------------------------*/
/*																								   */
/* This file is created by Gabriel Gosden. 													   	   */
/* Email: s174865@student.dtu.dk as a part of the bachelor project:								   */
/* "Recreating Operator Paths Using an Electromagnetic Motion Tracker and a Robot Manipulator.	   */
/* This work has been done for Teknologisk Institut, DMRI.										   */
/*																								   */
/*-------------------------------------------------------------------------------------------------*/

#include "std_msgs/String.h"
#include "VPif3.h"
#include "USBfcns.h"
#include "Viper_setup_cmd.h"
#include "ros/ros.h"

#include <sstream>

// Include the custom messages for the Viper
#include "viper_tf2_broadcaster/viper_msg_dist.h"
#include "viper_tf2_broadcaster/viper_msg_n.h"
//#include "viper_tf2_broadcaster/viper_msg_ori.h"
#include "viper_tf2_broadcaster/viper_msg_pose_ori.h"




int main(int argc, char **argv)
{
	/*-------------------------------------*/
	/* Variables for viper_settings.launch */
	float x_hs, y_hs, z_hs;
	int pos_unit, ori_unit;
	int filter_level;
	int FTT_mode;
	/*-------------------------------------*/

	ros::init(argc, argv, "viper_broadcaster");

	/*Create nodehandle */
	ros::NodeHandle n;

	/* Create ROS publishers*/
	ros::Publisher viper_broadcaster_pose_ori_pub= n.advertise<viper_tf2_broadcaster::viper_msg_pose_ori>("viper_broadcaster_pose_ori", 1000);
	ros::Publisher viper_broadcaster_n_pub= n.advertise<viper_tf2_broadcaster::viper_msg_n>("viper_broadcaster_n", 1000);
	ros::Publisher viper_broadcaster_dist_pub= n.advertise<viper_tf2_broadcaster::viper_msg_dist>("viper_broadcaster_dist", 1000);
 
	ros::Rate loop_rate(240);
		
	/*Discover open and claim Polemus Viper*/
	if (DiscoverVidPid(&g_usbhnd, g_usbinfo, POLHEMUS_USB_VID, VIPER_USB_PID)!=0)
	{	
		ROS_ERROR("Could not connect to viper!");
		return 1;
	}else
	{
		ROS_INFO("Connected to Polhemus Viper!");
	}

	/*Get the station map (connected sensors and sources) */
  	CStationMap cstamap;
	if (CmdStationMap(cstamap))
	{
		ROS_ERROR("CmdStationMap error");
		return 1;
	}
	else
	{
		if(cstamap.SnsDetectedCount() && cstamap.src_detected_count){
			ROS_INFO("Found %d sensor and %d Source",cstamap.SnsDetectedCount(),cstamap.src_detected_count);
		}else
		{
			ROS_ERROR("Found the incorrect number of Sensor(s) and/or Source(s). Please check connections.");
			return 1;
		}
	}

	/*Set the hemisphere according to viper_settings.launch*/
	CHemisphereCfg hemcfg;
	n.getParam("/x_hs", x_hs);
	n.getParam("/y_hs", y_hs);
	n.getParam("/z_hs", z_hs);
	float hemarr[3] = { x_hs, y_hs, z_hs};
	hemcfg.Fill(hemarr, false);
	
		if (CmdHemisphere(hemcfg))
		{
			ROS_ERROR("CmdHemisphere error!");
			return 1;
		}
		else
		{
			ROS_INFO("+X Hemisphere set on all sensors!");
		}

	/*Set units to cm and orientation to degrees*/
	CUnitsCfg pos_units;
	n.getParam("/pos_unit",pos_unit);
	n.getParam("/ori_unit",ori_unit);
	pos_units.Fill(pos_unit,ori_unit);
	if (CmdSetUnits(pos_units))
		{
			ROS_ERROR("Position and orientation units not set!");
			return 1;
		}
		else
		{
			ROS_INFO("Position and orientation units set!");
		}

	/* Set filter mode*/
	CFilterCfg level;
	n.getParam("/filter_level",filter_level);
	level.Fill(int_to_eViperFilterPresets(filter_level));
		if (CmdSetFilter(level))
		{
			ROS_ERROR("Filter level not set!");
			return 1;
		}
		else
		{
			ROS_INFO("Filter level set!");
		}

	/*Set FTT mode*/
	CEnumCfg val;
	
	n.getParam("/ftt_mode",FTT_mode);
	val.Fill(FTT_mode);
	if (CmdSetFFT(val))
		{
			ROS_ERROR("FTT not set!");
			return 1;
		}
		else
		{
			ROS_INFO("FTT set!");
		}

	ROS_INFO("Polhemus Viper Setup successfully performed!");	
	

  
	int r = 0;
	ROS_INFO("Starting broadcasting of Viper frames!");
	while (ros::ok())
	{
		
		viper_tf2_broadcaster :: viper_msg_pose_ori msg_pose_ori;
		viper_tf2_broadcaster :: viper_msg_n msg_n;
		viper_tf2_broadcaster :: viper_msg_dist msg_dist;
		CVPcmd cmd;
		cmd.Fill(0, CMD_SINGLE_PNO, CMD_ACTION_GET);
		cmd.Prepare(g_txbuf, g_ntxcount);


		int nBytes = g_ntxcount;
		uint8_t *pbuf = g_txbuf;

		if (r = WriteUSB(g_usbhnd, g_usbinfo, pbuf, nBytes))
		{
			ROS_ERROR("Write CMD_ACTION_GET failed with error ");
		}
		else
		{
			g_nrxcount = RX_BUF_SIZE;
			r = ReadUSB(g_usbhnd, g_usbinfo, g_rxbuf, g_nrxcount, true);
			
			if (r == 0)
			{
				CFrameInfo fi(g_rxbuf, g_nrxcount);
				if ((fi.cmd() == CMD_SINGLE_PNO) && (fi.IsAck()))
				{
					CVPSeuPno pno;
					int bytesextracted = pno.Extractseupno(fi.PCmdPayload());
					if (bytesextracted)
					{
						std::string s;
						pno.Report(s,true);
						
						msg_pose_ori.x = x_out;
						msg_pose_ori.y = y_out;
						msg_pose_ori.z = z_out;
						msg_pose_ori.az = az_out;
						msg_pose_ori.el = el_out;
						msg_pose_ori.ro = ro_out;
						msg_dist.dist = dist_out;
						msg_n.n = n_out;

						viper_broadcaster_pose_ori_pub.publish(msg_pose_ori);
						viper_broadcaster_dist_pub.publish(msg_dist);
						viper_broadcaster_n_pub.publish(msg_n);
					}
					else
					r = -1;
				}else
				{
					r = -1;
				}
			}else
			{
				ROS_ERROR("Read CMD_ACTION_GET failed with error");
			}
		}

 
 	ros::spinOnce();

	loop_rate.sleep();
}
return 0;
}