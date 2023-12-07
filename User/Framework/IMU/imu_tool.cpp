//
// Created by hp on 2023/11/1.
//
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

#include <cmath>
#include "imu_tool.h"

Linear_Acceleration a;
Angular_Velocity w;
Mag_Msg m;
    void ComplementaryFilterROS::initializeParams()
    {
        double gain_acc;
        double gain_mag;
        bool do_bias_estimation;
        double bias_alpha;
        bool do_adaptive_gain;
        double orientation_stddev;

        use_mag_ = false;
        reverse_tf_ = false;
        constant_dt_ = 0.0;
        gain_acc = 0.01;
        gain_mag = 0.01;

        bias_alpha = 0.01;

        do_bias_estimation = true;
        do_adaptive_gain = true;
        orientation_stddev = 0.0;
        orientation_variance_ = orientation_stddev * orientation_stddev;
        initialized_filter_ = false;

        filter_.setDoBiasEstimation(true);//ture
        filter_.setDoAdaptiveGain(true);//true

        // check for illegal constant_dt values
        if (constant_dt_ < 0.0)
        {
            constant_dt_ = 0.0;
        }
    }

    void ComplementaryFilterROS::
    imuCallback(Imu_Msg imu_msg_raw_raw)
    {
        a = imu_msg_raw_raw.linear_acceleration;
        w = imu_msg_raw_raw.angular_velocity;
        const float time = imu_msg_raw_raw.time;

        // Initialize.
        if (!initialized_filter_)
        {
            time_prev_ = time;
            initialized_filter_ = true;
            return;
        }

        // determine dt: either constant, or from IMU timestamp
        double dt = (time - time_prev_);
        time_prev_ = time;

        time_prev_ = time;

        // Update the filter.
        filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

        // Publish state.
        getOrientation(imu_msg_raw_raw);
    }

    void ComplementaryFilterROS::imuMagCallback(Imu_Msg imu_msg_raw,
                                                Mag_Msg mag_msg)
    {
        a = imu_msg_raw.linear_acceleration;
        w = imu_msg_raw.angular_velocity;
        m = mag_msg;
        const float time = imu_msg_raw.time;

        // Initialize.
        if (!initialized_filter_)
        {
            time_prev_ = time;
            initialized_filter_ = true;
            return;
        }

        // Calculate dt.
        double dt = (time - time_prev_);
        time_prev_ = time;

        // ros::Time t_in, t_out;
        // t_in = ros::Time::now();
        // Update the filter.
        if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z))
            filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
        else
            filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);

        // t_out = ros::Time::now();
        // float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
        // printf("%.6f\n", dt_tot);
        // Publish state.
        getOrientation(imu_msg_raw);
    }

    void ComplementaryFilterROS::getOrientation(Imu_Msg imu_msg_raw)
    {
        // Get the orientation:
        double q0, q1, q2, q3;
        filter_.getOrientation(q0, q1, q2, q3);

        // Create and publish fitlered IMU message.
//        ImuMsg::SharedPtr imu_msg_raw = std::make_shared<Imu_Msg>(imu_msg_raw);

        imu_msg_raw.orientation.x = q1;
        imu_msg_raw.orientation.y = q2;
        imu_msg_raw.orientation.z = q3;
        imu_msg_raw.orientation.w = q0;

        imu_msg_raw.orientation_covariance[0] = orientation_variance_;
        imu_msg_raw.orientation_covariance[1] = 0.0;
        imu_msg_raw.orientation_covariance[2] = 0.0;
        imu_msg_raw.orientation_covariance[3] = 0.0;
        imu_msg_raw.orientation_covariance[4] = orientation_variance_;
        imu_msg_raw.orientation_covariance[5] = 0.0;
        imu_msg_raw.orientation_covariance[6] = 0.0;
        imu_msg_raw.orientation_covariance[7] = 0.0;
        imu_msg_raw.orientation_covariance[8] = orientation_variance_;

        // Account for biases.
        if (filter_.getDoBiasEstimation())
        {
            imu_msg_raw.angular_velocity.x -= filter_.getAngularVelocityBiasX();
            imu_msg_raw.angular_velocity.y -= filter_.getAngularVelocityBiasY();
            imu_msg_raw.angular_velocity.z -= filter_.getAngularVelocityBiasZ();
        }

        quat[0] = q0 ;
        quat[1] = q1 ;
        quat[2] = q2 ;
        quat[3] = q3 ;
    }

ComplementaryFilterROS::ComplementaryFilterROS() {

}

ComplementaryFilterROS::~ComplementaryFilterROS() {

}

void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
