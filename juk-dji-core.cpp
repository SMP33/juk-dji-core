#include <dji_status.hpp> 
#include <dji_vehicle.hpp>
#include "common/dji_linux_helpers.hpp"

#include "msg/juk_dji_gps_msg.h"
#include "msg/juk_dji_device_status_msg.h"
#include "msg/juk_dji_control_msg.h"

#include <iostream>
#include <LinuxChrono.h>
#include <ros/ros.h>

#define NO_DJI_HARDWARE

using namespace std;

Vehicle*   v;
const int F_MODE = 1684;
auto default_ctrlFlag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
     Control::YAW_RATE | Control::STABLE_ENABLE;

DJI::OSDK::Control::CtrlData            default_ctrlData(default_ctrlFlag,0,0,0,0);
DJI::OSDK::Control::CtrlData            current_ctrlData(default_ctrlData);

DJI::OSDK::Telemetry::RCFullRawData  data_RC;
DJI::OSDK::Telemetry::Battery        data_Bat;
DJI::OSDK::Telemetry::GlobalPosition data_GPS;
DJI::OSDK::Telemetry::Velocity       data_Velocity;
DJI::OSDK::Telemetry::SDKInfo        data_Status;
double                               data_Course;

custom_msg::juk_dji_gps_msg msg_GPS;
custom_msg::juk_dji_device_status_msg msg_device_status;

ros::Time last_ctrl_update_time;


void ctrl_callback(const custom_msg::juk_dji_control_msg::ConstPtr& msg)
{
	last_ctrl_update_time = ros::Time::now();
	current_ctrlData.x = msg->vx;
	current_ctrlData.y = msg->vy;
	current_ctrlData.z = msg->vz;
	
	current_ctrlData.yaw = msg->course;
}

void update_data()
{
#ifndef NO_DJI_HARDWARE
	data_RC =
   v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>();
	data_Bat =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
	data_Velocity =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
	data_GPS = v->broadcast->getGlobalPosition();

	data_Status =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
	Telemetry::Quaternion quat = v->broadcast->getQuaternion();
	cout << data_GPS.altitude << endl;
	double   q2sqr = quat.q2 * quat.q2;
	double   t0    = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
	double   t1    = + 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
	double   t2    = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
	double   t3    = + 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
	double   t4    = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;

	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;

	  data_Course = atan2(t1, t0);
#endif
	msg_GPS.lat = data_GPS.latitude;
	msg_GPS.lng = data_GPS.longitude;
	msg_GPS.alt = data_GPS.altitude;
	msg_GPS.quality = data_GPS.health;
	msg_GPS.satellites = 0;
	msg_GPS.vx = data_Velocity.data.x;
	msg_GPS.vy = data_Velocity.data.y;
	msg_GPS.vz = data_Velocity.data.z;
	msg_GPS.course = data_Course;
	const long max_mute_duration = 100000;
	switch (data_RC.lb2.mode)
	{
	case F_MODE:
		{
			#ifndef NO_DJI_HARDWARE
			if (data_Status.deviceStatus != 2)
			{
				v->obtainCtrlAuthority();
				msg_device_status.changeTime = ros::Time::now(); 
			}
			
			v->control->flightCtrl(current_ctrlData);
			#endif
			msg_device_status.authority = custom_msg::juk_dji_device_status_msg::CONTROL_BY_SDK;
			
			
		}
		break;
      
	default:
		{	
			#ifndef NO_DJI_HARDWARE
			if (data_Status.deviceStatus == 2)
			{
				v->releaseCtrlAuthority();
				msg_device_status.changeTime = ros::Time::now(); 
			}
			#endif
			
			msg_device_status.authority = custom_msg::juk_dji_device_status_msg::CONTROL_BY_RC;
			msg_device_status.changeTime = ros::Time::now();
		
			break;
		}
	}
	
	msg_device_status.voltage = data_Bat.voltage;
}
int main(int argc, char *argv[])
{
	cout << argv[0] << endl;
	ros::init(argc, argv, "JUK_DJI_CORE_NODE");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	last_ctrl_update_time = ros::Time::now();
	ros::Publisher pub_GPS = nh.advertise<custom_msg::juk_dji_gps_msg>("JUK/DJI/GPS", 1);
	ros::Publisher pub_device_status = nh.advertise<custom_msg::juk_dji_device_status_msg>("JUK/DJI/DEVICE_STATUS", 1);
	
	ros::Subscriber sub = nh.subscribe("JUK/DJI/CONTROL", 1, ctrl_callback);
	
#ifndef NO_DJI_HARDWARE
	LinuxSetup ls(argc, argv); 
	v = ls.getVehicle();
	
	auto st=v->broadcast->getStatus();

	//===============Подписка на указанные темы==========//
	ACK::ErrorCode subscribeStatus;
	subscribeStatus = v->subscribe->verify(5000);

	int                             pkgIndex        = 0;
	int                             freq            = 50;
	DJI::OSDK::Telemetry::TopicName topicList50Hz[] = {
		DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA,
		DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO,
		DJI::OSDK::Telemetry::TOPIC_VELOCITY,
		DJI::OSDK::Telemetry::TOPIC_GPS_FUSED,
		DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE
	};

	int  numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
	bool enableTimestamp = false;

	bool pkgStatus = v->subscribe->initPackageFromTopicList(
	  pkgIndex,
		numTopic,
		topicList50Hz,
		enableTimestamp,
		freq);

	subscribeStatus = v->subscribe->startPackage(pkgIndex, 50000);
#endif 
	//==========Основной цикл==========//
	ros::Rate r(50);
	
	while (ros::ok())
	{
		update_data();
		//create_msg_GPS();
		pub_GPS.publish(msg_GPS);
		pub_device_status.publish(msg_device_status);
		
		
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}