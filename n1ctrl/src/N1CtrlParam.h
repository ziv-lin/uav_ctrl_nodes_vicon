#ifndef __N1CTRLPARAM_H
#define __N1CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
  public:
    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kvi0, Kvi1, Kvi2;
        double Ka0, Ka1, Ka2;
        double Kr0, Kr1, Kr2;
        double Kw0, Kw1, Kw2;
        double Omega_pid_p_roll, Omega_pid_p_pitch, Omega_pid_p_yaw;
        double Omega_pid_d_roll, Omega_pid_d_pitch, Omega_pid_d_yaw;
        double Ctrl_limit_pos_norm  =0.2;
        double Ctrl_limit_spd_norm  = 0.5 ;
        double Ctrl_limit_f_norm = 10.0;
        double Switch_angle = 30.0;
    };

    struct Idling
    {
        double desired_height_limit;
        double desired_velo_limit;
        double feedback_velo_limit;
        double js_thrust_limit;
        double landing_timeout;
        double landing_thrust_percent;
        double lowest_thrust;
    };

    struct RC
    {
        double hori_velo_scale;
        double vert_velo_scale;
        double yaw_scale;
        double attitude_scale;

        double deadzone;
        double exit_cmd_deadzone;
    };

    struct Hover
    {
        bool set_hov_percent_to_zero;
        bool use_hov_percent_kf;

        double vert_velo_limit_for_update;
        double vert_height_limit_for_update;

        double percent_lower_limit;
        double percent_higher_limit;
    };

    struct MsgTimeout
    {
        double odom;
        double rc;
        double cmd;
    };

    Gain       hover_gain, track_gain;
    Idling     idling;
    RC         rc;
    Hover      hover;
    MsgTimeout msg_timeout;

    double mass;
    double gra;
    double hov_percent;
    double full_thrust;

    double ctrl_rate;

    double js_ctrl_lock_velo;

    std::string work_mode;
    std::string js_ctrl_mode;

    Parameter_t();
    void config_from_ros_handle( const ros::NodeHandle &nh );
    void init();
    void config_full_thrust( double hov );

  private:
    template < typename TName, typename TVal >
    void read_essential_param( const ros::NodeHandle &nh, const TName &name, TVal &val )
    {
        if ( nh.getParam( name, val ) )
        {
            // pass
        }
        else
        {
            ROS_ERROR_STREAM( "Read param: " << name << " failed." );
            ROS_BREAK();
        }
    };
};

#endif
