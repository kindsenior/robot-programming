///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021, Kei Okada and Kunio Kojima
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of Kei Okada nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#define BAUDRATE 115200

// ROS
#include <ros.h>
ros::NodeHandle  nh;
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// publisher
std_msgs::Float32MultiArray multi_array_msg;
ros::Publisher multi_array_pub("test/multi_array_arduino2pc", &multi_array_msg);

// subscriber
void multi_array_cb(const std_msgs::Float32MultiArray& msg)
{
  int data_length = multi_array_msg.data_length < msg.data_length ? multi_array_msg.data_length : msg.data_length;
  for(int i=0; i < data_length; ++i){
    multi_array_msg.data[i] = msg.data[i];
  }
}
ros::Subscriber<std_msgs::Float32MultiArray> multi_array_sub("test/multi_array_pc2arduino", multi_array_cb);

void setup()
{
  Serial.begin(BAUDRATE); // serial port settings

  // setup ros functions
  nh.getHardware()->setBaud(57600);
  // nh.getHardware()->setBaud(9600);
  nh.initNode();

  multi_array_msg.data = (float*)malloc(sizeof(float)*5);
  multi_array_msg.data_length = 5;
  multi_array_msg.data[0] = 1.0;
  multi_array_msg.data[1] = 2.0;
  multi_array_msg.data[2] = 3.0;
  multi_array_msg.data[3] = 4.0;
  multi_array_msg.data[4] = 5.0;

  nh.advertise(multi_array_pub);
  nh.subscribe(multi_array_sub);
}


void loop()
{
    multi_array_pub.publish(&multi_array_msg);
    nh.spinOnce();
}
