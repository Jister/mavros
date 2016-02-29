/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <viconros/viconmocap.h>



namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class ViconPlugin : public MavRosPlugin
{
public:
	ViconPlugin() :
		mp_nh("~vicon"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		vicon_sub = mp_nh.subscribe("/vicon", 500, &ViconPlugin::vicon_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	UAS *uas;

	ros::Subscriber vicon_sub;
	/* -*- low-level send -*- */
	void vicon_send
		(uint64_t usec,
			float x, float y, float z,
			float vx, float vy, float vz)
	{
		mavlink_message_t msg;
		mavlink_msg_vicon_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				x,
				y,
				z,
				vx,
				vy,
				vz);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */
	void vicon_cb(const viconros::viconmocap &vicon)
	{
		vicon_send(vicon.header.stamp.toNSec() / 1000,
				vicon.position.x,
				vicon.position.y,
				vicon.position.z,
				vicon.position.vx,
				vicon.position.vy,
				vicon.position.vz);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ViconPlugin, mavplugin::MavRosPlugin)
