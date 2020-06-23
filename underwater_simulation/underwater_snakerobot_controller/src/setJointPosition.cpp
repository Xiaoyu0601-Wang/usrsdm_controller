/*
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	if (argc != 7) {
                std::cerr << "Usage: " << argv[0] << "<topic> <q1> <q2> <q3> <q4> <q5>" << std::endl;
		std::cerr << "Units are radians" << std::endl;
                exit(0);
        }
	std::string topic(argv[1]);

	ros::init(argc, argv, "setJointPosition");
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<sensor_msgs::JointState>(topic,1);
	ros::Rate rate(30);

	double q[5];
	for (int i=0; i<5; i++) q[i]=atof(argv[i+2]);

	while (ros::ok()) {

		sensor_msgs::JointState js;
					js.name.push_back(std::string("base_link_to_screw_1_left_joint"));
					js.velocity.push_back(q[0]);
					js.name.push_back(std::string("base_link_to_screw_1_right_joint"));
					js.velocity.push_back(q[0]);
					js.name.push_back(std::string("base_link_to_link_2"));
					js.velocity.push_back(q[2]);
					js.name.push_back(std::string("link_2_to_screw_2_left_joint"));
					js.velocity.push_back(q[0]);
					js.name.push_back(std::string("link_2_to_screw_2_right_joint"));
					js.velocity.push_back(q[0]);

        	position_pub.publish(js);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
