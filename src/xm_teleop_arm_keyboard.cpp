/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Control the arm by using keyboard.

// Create Date: 2015.11.1

// Authors: myyerrol


#include <termio.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <xm_msgs/xm_JointPos.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>


#define KEYCODE_A 0X61
#define KEYCODE_D 0X64
#define KEYCODE_S 0X73
#define KEYCODE_W 0X77
#define KEYCODE_Q 0X71
#define KEYCODE_E 0X65
#define KEYCODE_X 0x78

#define KEYCODE_A_CAP 0X41
#define KEYCODE_D_CAP 0X44
#define KEYCODE_S_CAP 0X53
#define KEYCODE_W_CAP 0X57
#define KEYCODE_Q_CAP 0X51
#define KEYCODE_E_CAP 0X45
#define KEYCODE_X_CAP 0x58


class XMTeleopKeyboard
{
	private:
        double joint_angular;
        double joint_angular_step;
        double gripper_angular;
        double gripper_angular_step;
        std::vector<std::string> joint_name;

        ros::NodeHandle xm_nh;

        ros::Publisher joint_pos_pub;
        ros::Publisher lift_pos_pub;
        ros::Publisher waist_pos_pub;
        ros::Publisher big_arm_pos_pub;
        ros::Publisher forearm_pos_pub;
        ros::Publisher wrist_pitching_pos_pub;
        ros::Publisher wrist_rotation_pos_pub;
        ros::Publisher gripper_pos_pub;

        xm_msgs::xm_JointPos joint_pos;
        std_msgs::Float64 gripper_pos;

    public:
        XMTeleopKeyboard()
        {
            joint_name.push_back("joint_lift") ;
            joint_name.push_back("joint_waist") ;
            joint_name.push_back("joint_big_arm") ;
            joint_name.push_back("joint_forearm") ;
            joint_name.push_back("joint_wrist_pitching");
            joint_name.push_back("joint_wrist_rotation");

            joint_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>("joint_pos_cmd", 1000);
            gripper_pos_pub = xm_nh.advertise<std_msgs::Float64>("gripper_joint/command", 1000);
/*
            std::stringstream ss0;
            ss0 << "joint_pos_cmd/" << joint_name[0];
            lift_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss0.str(), 1000);

            std::stringstream ss1;
            ss1 << "joint_pos_cmd/" << joint_name[1];
            waist_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss1.str(), 1000);

            std::stringstream ss2;
            ss2 << "joint_pos_cmd/" << joint_name[2];
            big_arm_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss2.str(), 1000);

            std::stringstream ss3;
            ss3 << "joint_pos_cmd/" << joint_name[3];
            forearm_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss3.str(), 1000);

            std::stringstream ss4;
            ss4 << "joint_pos_cmd/" << joint_name[4];
            wrist_pitching_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss4.str(), 1000);

            std::stringstream ss5;
            ss5 << "joint_pos_cmd/" << joint_name[5];
            wrist_rotation_pos_pub = xm_nh.advertise<xm_msgs::xm_JointPos>(ss5.str(), 1000);
*/
            ros::NodeHandle n_private("~");
            n_private.param("joint_angular", joint_angular, 0.0);
            n_private.param("joint_angular_step", joint_angular_step, 0.0174);
            n_private.param("gripper_angular", gripper_angular, 0.0);
            n_private.param("gripper_angular_step", gripper_angular_step, 0.1);
        }
        ~XMTeleopKeyboard() { }

        void KeyboardLoop();

        void StopArm()
        {
/*
            ROS_INFO("Stop arm move!");
            joint_pos.command  = 0x02;
            joint_pos.joint    = 0.0;
            joint_pos.position = 0.0;
            joint_pos_pub.publish(joint_pos);
*/
        }
};


XMTeleopKeyboard* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;


int main(int argc, char** argv)
{
    ros::init(argc,argv,"xm_teleop_keyboard", ros::init_options::NoSigintHandler);
    XMTeleopKeyboard tbk;

    boost::thread t = boost::thread(boost::bind(&XMTeleopKeyboard::KeyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.StopArm();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}


void XMTeleopKeyboard::KeyboardLoop()
{
    char keyboard_cmd;
    int joint_number;
    double joint_pos_array[6];
    memset(joint_pos_array, 0, sizeof(joint_pos_array));

    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

	puts("-----------------------------------");
    puts("      Move The Arm By KeyBoard     ");
    puts("-----------------------------------");
	puts("Q                W                E");
    puts("A                                 D");
    puts("                 S                 ");
    puts("                 X                 ");
    puts("-----------------------------------");
    puts("W:Lift-UP          S:Waist-L       ");
    puts("A:Big_Arm-UP       D:Forearm-UP    ");
    puts("Q:Wrist_Pitching-UP                ");
    puts("E:Wrist_Rotation-CLOCK++           ");
    puts("X:Gripper-Open                     ");
    puts("-----------------------------------");
    puts("Shift+W:Lift-DOWN  Shift+S:Waist-R ");
    puts("Shift+A:Big_Arm-DOWN               ");
    puts("Shift+D:Forearm-DOWN               ");
    puts("Shift+Q:Wrist_Pitching-DOWN        ");
    puts("Shift+E:Wrist_Rotation-CLOCK--     ");
    puts("Shift+X:Gripper-Close              ");
	puts("-----------------------------------");
    puts("QUIT/CTRL-C TO QUIT                ");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &keyboard_cmd, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if(dirty == true)
            {
                //StopArm();
                //dirty = false;
            }
            continue;
        }

        switch(keyboard_cmd)
        {
            case KEYCODE_W:
                joint_number = 0;
                joint_angular_step = 0.01;
                joint_pos_array[0] += joint_angular_step;
                joint_angular_step = 0.0174;
                if(joint_pos_array[0] >= 0.20)
                    joint_pos_array[0] = 0.20;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[0];
                joint_pos_pub.publish(joint_pos);
                //lift_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_S:
                joint_number = 1;
                joint_pos_array[1] += joint_angular_step;
                if(joint_pos_array[1] >= 1.047)
                    joint_pos_array[1] = 1.047;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[1];
                joint_pos_pub.publish(joint_pos);
                //waist_pos_pub.publish(joint_pos);*/
                break;
            case KEYCODE_A:
                joint_number = 2;
                joint_pos_array[2] -= joint_angular_step;
                //if(joint_pos_array[2] >= 1.396)
                    //joint_pos_array[2] = 1.396;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[2];
                joint_pos_pub.publish(joint_pos);
                //big_arm_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_D:
                joint_number = 3;
                joint_pos_array[3] += joint_angular_step;
                //if(joint_pos_array[3] >= 2.234)
                    //joint_pos_array[3] = 2.234;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[3];
                joint_pos_pub.publish(joint_pos);
                //forearm_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_Q:
                joint_number = 4;
                joint_pos_array[4] += joint_angular_step;
                //if(joint_pos_array[4] >= 2.182)
                    //joint_pos_array[4] = 2.182;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[4];
                joint_pos_pub.publish(joint_pos);
                //wrist_pitching_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_E:
                joint_number = 5;
                joint_pos_array[5] += joint_angular_step;
                //if(joint_pos_array[5] >= 3.14)
                    //joint_pos_array[5] = 3.14;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[5];
                joint_pos_pub.publish(joint_pos);
                //wrist_rotation_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_W_CAP:
                joint_number = 0;
                joint_angular_step = 0.01;
                joint_pos_array[0] -= joint_angular_step;
                joint_angular_step = 0.0174;
                if(joint_pos_array[0] <= -0.20)
                    joint_pos_array[0] = -0.20;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[0];
                joint_pos_pub.publish(joint_pos);
                //lift_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_S_CAP:
                joint_number = 1;
                joint_pos_array[1] -= joint_angular_step;
                if(joint_pos_array[1] <= -1.047)
                    joint_pos_array[1]  = -1.047;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[1];
                joint_pos_pub.publish(joint_pos);
                //waist_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_A_CAP:
                joint_number = 2;
                joint_pos_array[2] += joint_angular_step;
                //if(joint_pos_array[2] <= -1.920)
                    //joint_pos_array[2]  = -1.920;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[2];
                joint_pos_pub.publish(joint_pos);
                //big_arm_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_D_CAP:
                joint_number = 3;
                joint_pos_array[3] -= joint_angular_step;
                //if(joint_pos_array[3] <= -2.234)
                    //joint_pos_array[3]  = -2.234;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[3];
                joint_pos_pub.publish(joint_pos);
                //forearm_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_Q_CAP:
                joint_number = 4;
                joint_pos_array[4] -= joint_angular_step;
                //if(joint_pos_array[4] <= -2.182)
                    //joint_pos_array[4]  = -2.182;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[4];
                joint_pos_pub.publish(joint_pos);
                //wrist_pitching_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_E_CAP:
                joint_number = 5;
                joint_pos_array[5] -= joint_angular_step;
                //if(joint_pos_array[5] <= 0.0)
                    //joint_pos_array[5] = 0;
                dirty = true;
                joint_pos.command  = 0x01;
                joint_pos.joint    = joint_number;
                joint_pos.position = joint_pos_array[5];
                joint_pos_pub.publish(joint_pos);
                //wrist_rotation_pos_pub.publish(joint_pos);
                break;
            case KEYCODE_X:
                gripper_angular += gripper_angular_step;
                if(gripper_angular >= 1.0)
                    gripper_angular = 1.0;
                dirty = true;
                gripper_pos.data = gripper_angular;
                gripper_pos_pub.publish(gripper_pos);
                break;
            case KEYCODE_X_CAP:
                gripper_angular -= gripper_angular_step;
                if(gripper_angular <= -1.0)
                    gripper_angular = -1.0;
                dirty = true;
                gripper_pos.data = gripper_angular;
                gripper_pos_pub.publish(gripper_pos);
                break;
            default:
                dirty = false;
        }
    }
}
