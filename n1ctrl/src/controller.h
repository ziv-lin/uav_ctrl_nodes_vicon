#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <uav_utils/geometry_utils.h>

#include "input.h"
#define CONTROL_ANGLE 0 // 1=control angle , 0 = control rate
struct Desired_State_t
{
    Eigen::Vector3d    p;
    Eigen::Vector3d    v;
    double             yaw;
    Eigen::Quaterniond q;
    Eigen::Vector3d    a;
};

struct Controller_Output_t
{
    static constexpr double CTRL_ANGULAR_RATE = -1.0;
    static constexpr double CTRL_ANGLE = 1.0;
    double                  roll;
    double                  pitch;
    double                  yaw;
    double                  thrust;
    double                  mode; // if mode > 0, thrust = 0~100%;
    // if mode < 0, thrust = -? m/s ~ +? m/s
};

struct SO3_Controller_Output_t
{
    Eigen::Matrix3d Rdes;
    Eigen::Vector3d Fdes;
    double          net_force;
};

class Controller
{
  public:
    Parameter_t &param;

    ros::Publisher ctrl_pub;
    // ros::Publisher ctrl_so3_pub;
    ros::Publisher ctrl_so3_attitude_pub;
    ros::Publisher ctrl_so3_thrust_pub;
    ros::Publisher ctrl_vis_pub;
    ros::Publisher ctrl_dbg_pub;
    ros::Publisher ctrl_val_dbg_pub;
    ros::Publisher ctrl_dbg_p_pub;
    ros::Publisher ctrl_dbg_v_pub;
    ros::Publisher ctrl_dbg_a_pub;
    ros::Publisher ctrl_dbg_att_des_pub;
    ros::Publisher ctrl_dbg_att_real_pub;
    ros::Publisher ctrl_omega_world;
    ros::Publisher ctrl_corilolis_force;


    ros::Publisher ctrl_omega_p;
    ros::Publisher ctrl_omega_d;

    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kvi;
    Eigen::Matrix3d Ka;
    Eigen::Matrix3d Kr;
    Eigen::Matrix3d Kw;
    Eigen::Vector3d Omega_pid_p;
    Eigen::Vector3d Omega_pid_d;
    double Ctrl_limit_pos_norm  =0.2;
    double Ctrl_limit_spd_norm  = 0.5 ;
    double Ctrl_limit_f_norm = 10.0;
    double                  switch_ctrl_angle;
    double                  maximum_attitude_angle[3];
    //    Eigen::Vector3d Omega_pid_d;

    Eigen::Vector3d int_e_v;

    Eigen::Vector3d m_last_F_des;
    Eigen::Matrix3d m_last_R_des = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d m_rot_diff;
    double          m_last_ctrl_time;

    Controller( Parameter_t & );
    void update_attitude_limit( const  geometry_msgs::Vector3StampedConstPtr & in_data);
    void config_gain( const Parameter_t::Gain &gain );
    void config();
    void update( const Desired_State_t &des, const Odom_Data_t &odom,
                 Controller_Output_t &u, SO3_Controller_Output_t &u_so3 );

    void output_visualization( const Controller_Output_t &u );
    void publish_ctrl( const Controller_Output_t &u, const ros::Time &stamp, const ros::Time &extra_stamp );
    void publish_so3_ctrl( const SO3_Controller_Output_t &u_so3, const ros::Time &stamp );

  private:
    bool is_configured;
};

#endif
