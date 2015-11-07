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

// Control the gripper by using keyboard.

// Create Date: 2015.11.1

// Authors: myyerrol


#include <termio.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>


#define KEYCODE_W 0X77
#define KEYCODE_S 0X73


class TeleopGripper
{
	private:
        double gripper_angular;
        double gripper_angular_step;

        ros::NodeHandle xm_nh;
        ros::Publisher gripper_pos_pub;

        std_msgs::Float64 gripper_pos;

    public:
        TeleopGripper()
        {
            gripper_pos_pub = xm_nh.advertise<std_msgs::Float64>("gripper_joint/command", 1000);

            ros::NodeHandle n_private("~");
            n_private.param("gripper_angular", gripper_angular, 0.0);
            n_private.param("gripper_angular_step", gripper_angular_step, 0.1);
        }

        ~TeleopGripper() { }

        void GripperLoop();

        void StopGripper() { }
};


TeleopGripper* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;


int main(int argc, char** argv)
{
    ros::init(argc,argv,"xm_teleop_gripper", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    TeleopGripper tbk;

    boost::thread t = boost::thread(boost::bind(&TeleopGripper::GripperLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.StopGripper();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}


void TeleopGripper::GripperLoop()
{
    char gripper_cmd;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("---------------------------------------");
    puts("      Move The Gripper By KeyBoard     ");
    puts("---------------------------------------");
    puts("            W           S              ");
    puts("---------------------------------------");
    puts("       W:Gripper++  S:Gripper--        ");
    puts("---------------------------------------");
    puts("QUIT/CTRL-C TO QUIT                    ");

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
            if(read(kfd, &gripper_cmd, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if(dirty == true)
            {
                //StopGripper();
                //dirty = false;
            }
            continue;
        }

        switch(gripper_cmd)
        {
            case KEYCODE_W:
                gripper_angular += gripper_angular_step;
                if(gripper_angular >= 1.5)
                    gripper_angular = 1.5;
                dirty = true;
                gripper_pos.data = gripper_angular;
                gripper_pos_pub.publish(gripper_pos);
                break;
            case KEYCODE_S:
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
