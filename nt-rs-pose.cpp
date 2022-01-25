// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "example-utils.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <jsoncpp/json/json.h>
#include <fstream>


int main(int argc, char * argv[]) try
{
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
        return EXIT_SUCCESS;

    int team;
    //read team number
    try {
        std::ifstream frc_json("/boot/frc.json", std::ifstream::binary);
        Json::Value frc;
        frc_json >> frc;
        std::cout << frc["team"] << std::endl;
        team = frc["team"].asInt();
        frc_json.close();
    } catch (...)
    {
        team = 4143;
    }



    // Create network table
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("/RealSensePose");
    nt::NetworkTableEntry xEntry = table->GetEntry("x");
    nt::NetworkTableEntry yEntry = table->GetEntry("y");
    nt::NetworkTableEntry zEntry = table->GetEntry("z");
    nt::NetworkTableEntry vxEntry = table->GetEntry("vx");
    nt::NetworkTableEntry vyEntry = table->GetEntry("vy");
    nt::NetworkTableEntry vzEntry = table->GetEntry("vz");
    nt::NetworkTableEntry rxEntry = table->GetEntry("rx");
    nt::NetworkTableEntry ryEntry = table->GetEntry("ry");
    nt::NetworkTableEntry rzEntry = table->GetEntry("rz");
    nt::NetworkTableEntry rwEntry = table->GetEntry("rw");
    nt::NetworkTableEntry resetposeEntry = table->GetEntry("Reset Pose");
    inst.StartClientTeam(team);


    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

	// send pose to network tables
	xEntry.SetDouble(pose_data.translation.x);
	yEntry.SetDouble(pose_data.translation.y);
	zEntry.SetDouble(pose_data.translation.z);
	vxEntry.SetDouble(pose_data.velocity.x);
	vyEntry.SetDouble(pose_data.velocity.y);
	vzEntry.SetDouble(pose_data.velocity.z);
	rxEntry.SetDouble(pose_data.rotation.x);
	ryEntry.SetDouble(pose_data.rotation.y);
	rzEntry.SetDouble(pose_data.rotation.z);
	rwEntry.SetDouble(pose_data.rotation.w);
	bool reset = resetposeEntry.GetBoolean(false);
	if(reset) {
		std::cout << "got reset value" << std::endl;
		// run reset pose code - not sure how yet
		resetposeEntry.SetBoolean(false);
	}

        // Print the x, y, z values of the translation, relative to initial position
        //std::cout << "\rposition: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << " velocity: " << pose_data.velocity.x << " " << pose_data.velocity.y << " rotation: " << pose_data.rotation.x << " " << pose_data.rotation.y << " " << pose_data.rotation.z << " " << pose_data.rotation.w;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
