#include "controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <n1ctrl/ControllerDebug.h>
#include <uav_utils/converters.h>
#define IF_VERVPSE_INTERGRATION 1
#define IF_LIMIT_MAXIMUM_ATTITUDE 0
#define MAXIMUM_ERR_NORM 100000000

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller( Parameter_t &param_ ) : param( param_ )
{
    is_configured = false;
    int_e_v.setZero();
}

void Controller::config()
{
    config_gain( param.hover_gain );
    is_configured = true;
}

void Controller::config_gain( const Parameter_t::Gain &gain )
{
    Kp.setZero();
    Kv.setZero();
    Kvi.setZero();
    Ka.setZero();
    Kr.setZero();
    Kw.setZero();
    Omega_pid_p.setZero();
    Kp( 0, 0 ) = gain.Kp0;
    Kp( 1, 1 ) = gain.Kp1;
    Kp( 2, 2 ) = gain.Kp2;
    Kv( 0, 0 ) = gain.Kv0;
    Kv( 1, 1 ) = gain.Kv1;
    Kv( 2, 2 ) = gain.Kv2;
    Kvi( 0, 0 ) = gain.Kvi0;
    Kvi( 1, 1 ) = gain.Kvi1;
    Kvi( 2, 2 ) = gain.Kvi2;
    Ka( 0, 0 ) = gain.Ka0;
    Ka( 1, 1 ) = gain.Ka1;
    Ka( 2, 2 ) = gain.Ka2;
    Kr( 0, 0 ) = gain.Kr0;
    Kr( 1, 1 ) = gain.Kr1;
    Kr( 2, 2 ) = gain.Kr2;
    Kw( 0, 0 ) = gain.Kw0;
    Kw( 1, 1 ) = gain.Kw1;
    Kw( 2, 2 ) = gain.Kw2;

    Omega_pid_p( 0 ) = gain.Omega_pid_p_roll;
    Omega_pid_p( 1 ) = gain.Omega_pid_p_pitch;
    Omega_pid_p( 2 ) = gain.Omega_pid_p_yaw;

    Omega_pid_d( 0 ) = gain.Omega_pid_d_roll;
    Omega_pid_d( 1 ) = gain.Omega_pid_d_pitch;
    Omega_pid_d( 2 ) = gain.Omega_pid_d_yaw;
    Ctrl_limit_pos_norm = gain.Ctrl_limit_pos_norm;
    Ctrl_limit_spd_norm = gain.Ctrl_limit_spd_norm;
    Ctrl_limit_f_norm = gain.Ctrl_limit_f_norm;
    switch_ctrl_angle = gain.Switch_angle;
    cout << "===== Kp =====" << endl;
    cout << Kp << endl;
    cout << "===== Kv =====" << endl;
    cout << Kv << endl;
    cout << "===== Kvi =====" << endl;
    cout << Kvi << endl;
    cout << "===== Ka =====" << endl;
    cout << Ka << endl;
    cout << " ===== Omega p =====" << endl;
    cout << Omega_pid_p << endl;
    cout << " ===== Omega d =====" << endl;
    cout << Omega_pid_d << endl;
    cout << "====== Ctrl limit ====" << endl;
    cout << "Ctrl_limit_pos_norm  = " << Ctrl_limit_pos_norm <<endl;
    cout << "Ctrl_limit_spd_norm  = "<< Ctrl_limit_spd_norm <<endl;
    cout << "Ctrl_limit_f_norm = "<< Ctrl_limit_f_norm <<endl;
    cout << "Switch angle  = " << switch_ctrl_angle ;
}

void Controller::update_attitude_limit( const geometry_msgs::Vector3StampedConstPtr &in_data )
{
    maximum_attitude_angle[ 0 ] = ( *in_data ).vector.x; // max  roll
    maximum_attitude_angle[ 1 ] = ( *in_data ).vector.y; // max pitch
    maximum_attitude_angle[ 2 ] = ( *in_data ).vector.z; // max yaw
    ROS_INFO( "Update ctrl attitude  limit , rpy =  [ %.2f,  %.2f,  %.2f ] ", maximum_attitude_angle[ 0 ] * 57.3,
              maximum_attitude_angle[ 1 ] * 57.3, maximum_attitude_angle[ 2 ] * 57.3 );
}

void Controller::update( const Desired_State_t &des, const Odom_Data_t &odom,
                         Controller_Output_t &    u,
                         SO3_Controller_Output_t &u_so3 )
{
    ROS_ASSERT_MSG( is_configured,
                    "Gains for controller might not be initialized!" );
    std::string constraint_info( "" );
    Vector3d    e_p, e_v, F_des;

    geometry_msgs::Vector3Stamped m;
    m.header = odom.msg.header;
    Vector3d                      d;

    if ( des.v( 0 ) != 0.0 || des.v( 1 ) != 0.0 || des.v( 2 ) != 0.0 )
    {
        // ROS_INFO("Reset integration");
        int_e_v.setZero();
    }
    //    cout << des.p <<endl;
    e_p = des.p - odom.p;

    if ( e_p.norm() > Ctrl_limit_pos_norm )
    {
        // cout << "Reach max norm, normalize " <<endl;
        e_p = e_p * Ctrl_limit_pos_norm / e_p.norm();
    }

    Eigen::Vector3d u_p = Kp * e_p;

    e_v =des.v  - odom.v;
    if ( e_v.norm() > Ctrl_limit_spd_norm )
    {
        // cout << "Reach max norm, normalize " <<endl;
        e_v = e_v * Ctrl_limit_spd_norm / e_v.norm();
    }

    e_v = e_v + u_p;

    const std::vector< double > integration_enable_limits = { 0.1, 0.1, 0.1 };
    for ( size_t k = 0; k < 3; ++k )
    {
        if ( std::fabs( e_v( k ) ) < 0.2 )
        {
            int_e_v( k ) += e_v( k ) * 1.0 / 50.0;
        }
    }

    Eigen::Vector3d u_v_p = Kv * e_v;
    //    const std::vector< double > integration_output_limits = { 0.4, 0.4, 0.4 };
    const std::vector< double > integration_output_limits = { 0.4, 0.4, 0.4 };
    Eigen::Vector3d             u_v_i = Kvi * int_e_v;
    for ( size_t k = 0; k < 3; ++k )
    {
        if ( std::fabs( u_v_i( k ) ) > integration_output_limits[ k ] )
        {
            uav_utils::limit_range( u_v_i( k ), integration_output_limits[ k ] );
            if ( IF_VERVPSE_INTERGRATION == 0 )
                ROS_INFO( "Integration saturate for axis %zu, value=%.3f", k, u_v_i( k ) );
        }
    }

    Eigen::Vector3d u_v = u_v_p + u_v_i;

    double yaw_des = des.yaw;
    double yaw_curr = get_yaw_from_quaternion( odom.q );


    Matrix3d wRc = rotz( yaw_curr );

    F_des = u_v * param.mass + Vector3d( 0, 0, param.mass * param.gra ) +
            Ka * param.mass * des.a;

    if ( F_des.norm() > Ctrl_limit_f_norm )
    {
        // cout << "Reach max norm, normalize " <<endl;
        F_des = F_des * Ctrl_limit_f_norm / F_des.norm();
    }

    if ( F_des( 2 ) < 0.05 * param.mass * param.gra )  // limit z axis must be upward and  not to be zero.
    {
        constraint_info =
            boost::str( boost::format( "thrust too low F_des(2)=%.3f; " ) % F_des( 2 ) );
        F_des = F_des / F_des( 2 ) * ( 0.05 * param.mass * param.gra );
        F_des( 2 ) = ( 0.05 * param.mass * param.gra );
    }

    // Limit pitch
    if ( std::fabs( F_des( 0 ) / F_des( 2 ) ) > std::tan(  maximum_attitude_angle[1]  ) )
    {
        constraint_info += boost::str( boost::format( "x(%f) too tilt; " ) %
                                       toDeg( std::atan2( F_des( 0 ), F_des( 2 ) ) ) );
        F_des( 0 ) =
            F_des( 0 ) / std::fabs( F_des( 0 ) ) * F_des( 2 ) * std::tan( maximum_attitude_angle[1]  );
        ROS_INFO("Pitch limit, maximum is %.2f", maximum_attitude_angle[1]*57.3);
    }

    // Limit roll
    if ( std::fabs( F_des( 1 ) / F_des( 2 ) ) > std::tan(  maximum_attitude_angle[0] ) )
    {
        constraint_info += boost::str( boost::format( "y(%f) too tilt; " ) %
                                       toDeg( std::atan2( F_des( 1 ), F_des( 2 ) ) ) );
        F_des( 1 ) =
            F_des( 1 ) / std::fabs( F_des( 1 ) ) * F_des( 2 ) * std::tan(  maximum_attitude_angle[0] );
        ROS_INFO("Roll limit, maximum is %.2f", maximum_attitude_angle[0]*57.3);
    }

    m.vector.x = F_des( 0 );
    m.vector.y = F_des( 1 );
    m.vector.z = F_des( 2 );
    ctrl_dbg_a_pub.publish( m );

    if ( IF_LIMIT_MAXIMUM_ATTITUDE )
    {
        if ( F_des( 2 ) < 0.05 * param.mass * param.gra )
        {
            constraint_info =
                boost::str( boost::format( "thrust too low F_des(2)=%.3f; " ) % F_des( 2 ) );
            F_des = F_des / F_des( 2 ) * ( 0.05 * param.mass * param.gra );
        }
        else if ( F_des( 2 ) > 3.0 * param.mass * param.gra )
        {
            constraint_info =
                boost::str( boost::format( "thrust too high F_des(2)=%.3f; " ) % F_des( 2 ) );
            F_des = F_des / F_des( 2 ) * ( 3.0 * param.mass * param.gra );
        }

        if ( std::fabs( F_des( 0 ) / F_des( 2 ) ) > std::tan( toRad( 50.0 ) ) )
        {
            constraint_info += boost::str( boost::format( "x(%f) too tilt; " ) %
                                           toDeg( std::atan2( F_des( 0 ), F_des( 2 ) ) ) );
            F_des( 0 ) =
                F_des( 0 ) / std::fabs( F_des( 0 ) ) * F_des( 2 ) * std::tan( toRad( 30.0 ) );
        }

        if ( std::fabs( F_des( 1 ) / F_des( 2 ) ) > std::tan( toRad( 50.0 ) ) )
        {
            constraint_info += boost::str( boost::format( "y(%f) too tilt; " ) %
                                           toDeg( std::atan2( F_des( 1 ), F_des( 2 ) ) ) );
            F_des( 1 ) =
                F_des( 1 ) / std::fabs( F_des( 1 ) ) * F_des( 2 ) * std::tan( toRad( 30.0 ) );
        }
    }
    // }

    {
        std_msgs::Header msg;
        msg = odom.msg.header;

        std::stringstream ss;

        if ( constraint_info == "" )
            constraint_info = "constraint no effect";
        ss << std::endl
           << constraint_info << std::endl;
        ss << "ep0 " << e_p( 0 ) << " | ";
        ss << "ep1 " << e_p( 1 ) << " | ";
        ss << "ep2 " << e_p( 2 ) << " | ";
        ss << "ev0 " << e_v( 0 ) << " | ";
        ss << "ev1 " << e_v( 1 ) << " | ";
        ss << "ev2 " << e_v( 2 ) << " | ";
        ss << "Fdes0 " << F_des( 0 ) << " | ";
        ss << "Fdes1 " << F_des( 1 ) << " | ";
        ss << "Fdes2 " << F_des( 2 ) << " | ";

        msg.frame_id = ss.str();
        ctrl_dbg_pub.publish( msg );

        d = -Kp * e_p;
        m.vector.x = d( 0 );
        m.vector.y = d( 1 );
        m.vector.z = d( 2 );
        ctrl_dbg_p_pub.publish( m );

        d = -Kv * e_v;
        m.vector.x = d( 0 );
        m.vector.y = d( 1 );
        m.vector.z = d( 2 );
        ctrl_dbg_v_pub.publish( m );
#if 0
        d = param.mass * des.a;
        m.vector.x = d( 0 );
        m.vector.y = d( 1 );
        m.vector.z = d( 2 );
        ctrl_dbg_a_pub.publish( m );
#endif
        n1ctrl::ControllerDebug dbg_msg;
        dbg_msg.header = odom.msg.header;
        dbg_msg.des_p = uav_utils::to_vector3_msg( des.p );
        dbg_msg.u_p_p = uav_utils::to_vector3_msg( u_p );
        dbg_msg.u_p_i = uav_utils::to_vector3_msg( Eigen::Vector3d::Zero() );
        dbg_msg.u_p = uav_utils::to_vector3_msg( u_p );
        dbg_msg.des_v = uav_utils::to_vector3_msg( des.v );
        dbg_msg.u_v_p = uav_utils::to_vector3_msg( u_v_p );
        dbg_msg.u_v_i = uav_utils::to_vector3_msg( u_v_i );
        dbg_msg.u_v = uav_utils::to_vector3_msg( u_v );

        dbg_msg.k_p_p = uav_utils::to_vector3_msg( Kp.diagonal() );
        dbg_msg.k_p_i = uav_utils::to_vector3_msg( Eigen::Vector3d::Zero() );
        dbg_msg.k_v_p = uav_utils::to_vector3_msg( Kv.diagonal() );
        dbg_msg.k_v_i = uav_utils::to_vector3_msg( Kvi.diagonal() );

        ctrl_val_dbg_pub.publish( dbg_msg );
    }

    Vector3d z_b_des = F_des / F_des.norm();

    /////////////////////////////////////////////////
    // Z-X-Y Rotation Sequence
    // Vector3d x_c_des = Vector3d(std::cos(yaw_des), sin(yaw_des), 0.0);
    // Vector3d y_b_des = z_b_des.cross(x_c_des) /
    // z_b_des.cross(x_c_des).norm();
    // Vector3d x_b_des = y_b_des.cross(z_b_des);
    /////////////////////////////////////////////////

    /////////////////////////////////////////////////
    // Z-Y-X Rotation Sequence
    Vector3d y_c_des = Vector3d( -std::sin( yaw_des ), std::cos( yaw_des ), 0.0 );
    Vector3d x_b_des = y_c_des.cross( z_b_des ) / y_c_des.cross( z_b_des ).norm();
    Vector3d y_b_des = z_b_des.cross( x_b_des );
    /////////////////////////////////////////////////

    Matrix3d R_des1; // it's wRb
    R_des1 << x_b_des, y_b_des, z_b_des;

    Matrix3d R_des2; // it's wRb
    R_des2 << -x_b_des, -y_b_des, z_b_des;
#if 0
    Vector3d e1 = R_to_ypr( R_des1.transpose() * odom.q.toRotationMatrix() );
    Vector3d e2 = R_to_ypr( R_des2.transpose() * odom.q.toRotationMatrix() );

    Matrix3d R_des; // it's wRb
    cout << "=======" << endl;
    if ( e1.norm() < e2.norm() )
    {
        R_des = R_des1;
        cout << "R_des =  1"<<endl;
    }
    else
    {
        R_des = R_des2;
        cout << "R_des =  2"<<endl;
    }
#else
    double e1 = Eigen::AngleAxisd( Eigen::Quaterniond( R_des1.transpose() * odom.q.toRotationMatrix() ) ).angle();
    double e2 = Eigen::AngleAxisd( Eigen::Quaterniond( R_des2.transpose() * odom.q.toRotationMatrix() ) ).angle();

    Matrix3d R_des; // it's wRb
    //cout << "=======" << endl;
    if ( e1 < e2 )
    {
        R_des = R_des1;
        //cout << "R_des =  1" << endl;
    }
    else
    {
//        R_des = R_des2;
//        cout << "R_des =  2" << endl;
    }
#endif
    //    R_des = R_des1;

    { // so3 control
        u_so3.Rdes = R_des;
        u_so3.Fdes = F_des;

        Matrix3d wRb_odom = odom.q.toRotationMatrix();
        Vector3d z_b_curr = wRb_odom.col( 2 );
        u_so3.net_force = F_des.dot( z_b_curr );
    }

    { // n1 api control in forward-left-up frame
        Vector3d F_c = wRc.transpose() * F_des;
        Matrix3d wRb_odom = odom.q.toRotationMatrix();
        Vector3d z_b_curr = wRb_odom.col( 2 );
        double   u1 = F_des.dot( z_b_curr );
        //double   u1 = F_des.norm();
        double fx = F_c( 0 );
        double fy = F_c( 1 );
        double fz = F_c( 2 );

        u.roll = std::atan2( -fy, fz );
        u.pitch = std::atan2( fx, fz );
        u.thrust = u1 / param.full_thrust;
        u.yaw = des.yaw;

        int      if_show_debug_info = 0;
        Vector3d ypr_real = R_to_ypr( odom.q.toRotationMatrix() );
        if ( if_show_debug_info )
        {
            cout << "Current_ypr  " << ypr_real.transpose() * 57.3 << endl;
            cout << "Target_ypr  " << Vector3d( u.yaw, u.pitch, u.roll ).transpose() * 57.3 << endl;
        }

        //if ( CONTROL_ANGLE ) // Control using attitude
        if ( ( fabs( u.roll ) < switch_ctrl_angle / 57.3 ) &&
             ( fabs( u.pitch ) < switch_ctrl_angle / 57.3 ) ) // Control using attitude
        {
            u.mode = Controller_Output_t::CTRL_ANGLE;
        }
        else // Control using angular rate.
        {
            u.mode = Controller_Output_t::CTRL_ANGULAR_RATE ;

            //Eigen::Quaterniond q_desire( u_so3.Rdes );
            Eigen::Quaterniond q_desire( R_des1 );
            Eigen::Quaterniond q_diff = odom.q.inverse() * q_desire;

            //=== Rotaion rate, d item ===
            m_rot_diff = (  R_des1*m_last_R_des.transpose() );
            Eigen::Quaterniond q_diff_last_current( m_rot_diff );
            q_diff_last_current = q_diff_last_current ;
            Eigen::Vector3d r_diff_vec_last_current = Eigen::AngleAxisd( q_diff_last_current ).axis();
            double          r_diff_angle_last_current = Eigen::AngleAxisd( q_diff_last_current ).angle();
            r_diff_vec_last_current = r_diff_vec_last_current * r_diff_angle_last_current /
                                      ( m.header.stamp.toSec() - m_last_ctrl_time );

            m_last_R_des = R_des1;
            m_last_ctrl_time = m.header.stamp.toSec();
            r_diff_vec_last_current = odom.omega;
            if(r_diff_vec_last_current.norm() < 30/57.3)
                r_diff_vec_last_current *=0;
            //=== Rotaion rate, d item ===

            Eigen::Vector3d R_error_vec = Eigen::AngleAxisd( q_diff ).axis();
            double          R_error_angle = Eigen::AngleAxisd( q_diff ).angle();

            if ( if_show_debug_info )
            {
                cout << "Q_diff  = " << Eigen::AngleAxisd( q_diff ).axis().transpose() << endl;
                cout << "R_error_vec = " << R_error_vec.transpose() << endl;
                cout << "R_error_vec_norm = " << R_error_vec.squaredNorm() << endl;
            }

            //cout << "Yaw desire  = " << yaw_des*57.3 <<endl;
            //cout << "R_error_angle = "<< R_error_angle*57.3 <<  "   , R_error_vec = " << R_error_vec.transpose() <<endl;

            if ( R_error_angle > M_PI )
            {
                R_error_angle = R_error_angle - 2 * M_PI;
            }
            else if ( R_error_angle < -M_PI )
            {
                R_error_angle += 2 * M_PI;
            }

            R_error_vec = R_error_vec * R_error_angle;

            Eigen::Vector3d R_error_vec_body = R_error_vec;
            if ( if_show_debug_info )
            {
                cout << "Angle = " << R_error_angle * 57.3 << endl;
                cout << "R_error_vec = " << R_error_vec.transpose() << endl;
                cout << "R_error_vec_norm = " << R_error_vec.squaredNorm() << endl;
            }

//            u.roll = R_error_vec_body( 0 ) * Omega_pid_p( 0 );  // -  ypr_para_d*odom.msg.twist.twist.angular.x ;
//           u.pitch = R_error_vec_body( 1 ) * Omega_pid_p( 1 ); //-  ypr_para_d*odom.msg.twist.twist.angular.y ;
//            u.yaw = R_error_vec_body( 2 ) * Omega_pid_p( 2 );   // -  ypr_para_d*odom.msg.twist.twist.angular.z ;

            u.roll = ( R_error_vec_body( 0 ) * Omega_pid_p( 0 ) - r_diff_vec_last_current( 0 )* Omega_pid_d( 0 ) ) ;  // -  ypr_para_d*odom.msg.twist.twist.angular.x ;
            u.pitch = ( R_error_vec_body( 1 ) * Omega_pid_p( 1 ) - r_diff_vec_last_current( 1 ) * Omega_pid_d( 1 )) ; //-  ypr_para_d*odom.msg.twist.twist.angular.y ;
            u.yaw = ( R_error_vec_body( 2 ) * Omega_pid_p( 2 ) - r_diff_vec_last_current( 2 ) * Omega_pid_d( 2 )) ;   // -  ypr_para_d*odom.msg.twist.twist.angular.z ;

            if ( 1 )
            {
                geometry_msgs::Vector3Stamped omega_dbg;
                omega_dbg.header = odom.msg.header;
                omega_dbg.vector.x = R_error_vec_body( 0 ) * Omega_pid_p( 0 );
                omega_dbg.vector.y = R_error_vec_body( 1 ) * Omega_pid_p( 1 );
                omega_dbg.vector.z = R_error_vec_body( 2 ) * Omega_pid_p( 2 );
                ctrl_omega_p.publish( omega_dbg );

                omega_dbg.vector.x = r_diff_vec_last_current( 0 ) * Omega_pid_d( 0 );
                omega_dbg.vector.y = r_diff_vec_last_current( 1 ) * Omega_pid_d( 1 );
                omega_dbg.vector.z = r_diff_vec_last_current( 2 ) * Omega_pid_d( 2 );
                ctrl_omega_d.publish( omega_dbg );
            }
        }
    }

    {
        Vector3d                      ypr_des = R_to_ypr( R_des );
        Vector3d                      ypr_real = R_to_ypr( odom.q.toRotationMatrix() );
        geometry_msgs::Vector3Stamped m;
        m.header = odom.msg.header;
        m.vector.x = ypr_des( 2 );
        m.vector.y = ypr_des( 1 );
        m.vector.z = ypr_des( 0 );
        ctrl_dbg_att_des_pub.publish( m );
        m.header = odom.msg.header;
        m.vector.x = ypr_real( 2 );
        m.vector.y = ypr_real( 1 );
        m.vector.z = ypr_real( 0 );
        ctrl_dbg_att_real_pub.publish( m );
    }
    //cout << __FILE__ << " " <<__LINE__ <<endl;
    //cout << "Roll, Pitch, Yaw output is " << Eigen::Vector3d(u.roll, u.pitch, u.yaw).transpose()*57.3 <<endl;
    output_visualization( u );
};

void Controller::publish_ctrl( const Controller_Output_t &u,
                               const ros::Time &          stamp,
                               const ros::Time &          extra_stamp )
{
    sensor_msgs::Joy msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = std::string( "FRD" );

    // need to translate to forward-right-down frame
    msg.axes.push_back( toDeg( u.roll ) );
    msg.axes.push_back( toDeg( -u.pitch ) );
    if ( u.mode < 0 )
    {
        msg.axes.push_back( u.thrust * 100 );
    }
    else
    {
        msg.axes.push_back( u.thrust * 100 );
    }
    msg.axes.push_back( toDeg( -u.yaw ) );
    msg.axes.push_back( u.mode );

    // add time stamp for debug
    msg.buttons.push_back( 100000 );
    msg.buttons.push_back( extra_stamp.sec / msg.buttons[ 0 ] );
    msg.buttons.push_back( extra_stamp.sec % msg.buttons[ 0 ] );
    msg.buttons.push_back( extra_stamp.nsec / msg.buttons[ 0 ] );
    msg.buttons.push_back( extra_stamp.nsec % msg.buttons[ 0 ] );
    //    printf("ctrl_pub [%lf, %lf, %lf ]\n", msg.axes[1],  msg.axes[2],  msg.axes[3] );
    ctrl_pub.publish( msg );
}

void Controller::publish_so3_ctrl( const SO3_Controller_Output_t &u_so3,
                                   const ros::Time &              stamp )
{
    //    Eigen::Vector3d    T_w = u_so3.Fdes;
    Eigen::Quaterniond q( u_so3.Rdes );

    geometry_msgs::QuaternionStamped att_msg;

    att_msg.header.stamp = stamp;
    att_msg.header.frame_id = std::string( "world" );
    att_msg.quaternion.x = q.x();
    att_msg.quaternion.y = q.y();
    att_msg.quaternion.z = q.z();
    att_msg.quaternion.w = q.w();

    ctrl_so3_attitude_pub.publish( att_msg );

    geometry_msgs::WrenchStamped thr_msg;

    thr_msg.header.stamp = stamp;
    thr_msg.header.frame_id = std::string( "body" );
    thr_msg.wrench.force.z = u_so3.net_force / param.full_thrust;

    ctrl_so3_thrust_pub.publish( thr_msg );

    // quadrotor_msgs::SO3Command msg;
    // msg.header.stamp = stamp;
    // msg.header.frame_id = std::string("body");

    // msg.force.x = T_w(0);
    // msg.force.y = T_w(1);
    // msg.force.z = T_w(2);

    // msg.orientation.x = q.x();
    // msg.orientation.y = q.y();
    // msg.orientation.z = q.z();
    // msg.orientation.w = q.w();

    // msg.kR[0] = Kr(0,0);
    // msg.kR[1] = Kr(1,1);
    // msg.kR[2] = Kr(2,2);

    // msg.kOm[0] = Kw(0,0);
    // msg.kOm[1] = Kw(1,1);
    // msg.kOm[2] = Kw(2,2);

    // msg.aux.kf_correction = 0.0;
    // msg.aux.angle_corrections[0] = 0.0;
    // msg.aux.angle_corrections[1] = 0.0;
    // msg.aux.enable_motors = true;
    // msg.aux.use_external_yaw = false;

    // ctrl_so3_pub.publish(msg);
}

void Controller::output_visualization( const Controller_Output_t &u )
{
    double fn = u.thrust;
    double tan_r = std::tan( u.roll );
    double tan_p = std::tan( u.pitch );
    double fz = std::sqrt( fn * fn / ( tan_r * tan_r + tan_p * tan_p + 1 ) );
    double fx = fz * tan_p;
    double fy = -fz * tan_r;

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string( "intermediate" );
    msg.linear_acceleration.x = fx;
    msg.linear_acceleration.y = fy;
    msg.linear_acceleration.z = fz;

    ctrl_vis_pub.publish( msg );
};
