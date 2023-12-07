/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
typedef struct
{
    double x;
    double y;
    double z;
}Linear_Acceleration;

typedef struct
{
    double x;
    double y;
    double z;
}Angular_Velocity;

typedef struct
{
    double x;
    double y;
    double z;
}Mag_Msg;

typedef struct
{
    double x;
    double y;
    double z;
    double w;
}Orientation;

typedef struct
{
    Linear_Acceleration linear_acceleration;
    Angular_Velocity angular_velocity;
    Orientation orientation;
    double orientation_covariance[9];
    float time;
}Imu_Msg;

#include <string>
#include "filtter.h"
class ComplementaryFilterROS
{
    public:
        ComplementaryFilterROS();
        ~ComplementaryFilterROS() ;
        void initializeParams();

    void imuMagCallback(Imu_Msg imu_msg_raw,
                        Mag_Msg mav_msg);
    void imuCallback(Imu_Msg imu_msg_raw);

    float quat[4];
private:

        // Parameters:
        bool use_mag_{};
        bool publish_tf_{};
        bool reverse_tf_{};
        double constant_dt_{};
        bool publish_debug_topics_{};
        std::string fixed_frame_;
        double orientation_variance_{};

        // State variables:

        ComplementaryFilter filter_;
        float time_prev_;
        bool initialized_filter_;

        void getOrientation(Imu_Msg imu_msg_raw);
};  // namespace imu_tools


void get_angle(float q[4], float *yaw, float *pitch, float *roll);
#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
