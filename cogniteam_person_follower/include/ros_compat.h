/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __ROS_DEEP_LEARNING_COMPATABILITY_H_
#define __ROS_DEEP_LEARNING_COMPATABILITY_H_

#define ROS_KINETIC  0
#define ROS_MELODIC  1
#define ROS_NOETIC   2
#define ROS_DASHING  3
#define ROS_ELOQUENT 4
#define ROS_FOXY     5
#define ROS_GALACTIC 6


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

// #define ROS_CREATE_NODE(name)				\
//  		ros::NodeHandle nh; 			\
// 		ros::NodeHandle private_nh("~");

template<class MessageType>
using Publisher = ros::Publisher*;

// #define ROS_CREATE_PUBLISHER(msg, topic, queue, ptr)					ros::Publisher __publisher_##ptr = private_nh.advertise<msg>(topic, queue); ptr = &__publisher_##ptr
// #define ROS_CREATE_PUBLISHER_STATUS(msg, topic, queue, callback, ptr)	ros::Publisher __publisher_##ptr = private_nh.advertise<msg>(topic, queue, [&](const ros::SingleSubscriberPublisher& connect_msg){callback();}); ptr = &__publisher_##ptr

// // #define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)				private_nh.subscribe(topic, queue, callback)
// #define ROS_SUBSCRIBER_TOPIC(subscriber)								subscriber.getTopic()

// #define ROS_NUM_SUBSCRIBERS(publisher)									publisher->getNumSubscribers()
// #define ROS_GET_NAMESPACE()												private_nh.getNamespace()
// #define ROS_GET_PARAMETER(name, val)									private_nh.getParam(name, val)
// #define ROS_GET_PARAMETER_OR(name, val, alt)							private_nh.param(name, val, alt)
// #define ROS_SET_PARAMETER(name, val)									private_nh.setParam(name, val)

// template<typename T> static void __ros_declare_parameter( ros::NodeHandle& nh, const std::string& name, const T& default_value )
// {
// 	if( !nh.hasParam(name) )
// 		nh.setParam(name, default_value);
// }

// #define ROS_DECLARE_PARAMETER(name, default_value)				__ros_declare_parameter(private_nh, name, default_value)

#define ROS_TIME_NOW()										ros::Time::now()
#define ROS_SPIN()											ros::spin()
#define ROS_SPIN_ONCE()										ros::spinOnce()
#define ROS_OK()											ros::ok()
#define ROS_SHUTDOWN() 										ros::shutdown()


#endif

