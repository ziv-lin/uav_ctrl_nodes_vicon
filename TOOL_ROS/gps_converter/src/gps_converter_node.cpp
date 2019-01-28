#include <algorithm>
#include <iostream>
#include <queue>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
//#include <quadrotor_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/format.hpp>

#include "GPSUtils.hpp"
#include <uav_utils/utils.h>

using bfmt = boost::format;
using namespace std;
Eigen::Quaterniond q_imu;
enum class SignalProcessState
{
    Init,
    Sampling,
    Normal
};

ros::Publisher height_pub;
ros::Publisher gpsodom_pub;
ros::Publisher velo_pub;

double        earth_radius = 6378.137 * 1.0e3;
ros::Duration signal_sample_duration( 5.0 );
double        std_factor = 2;

SignalProcessState signal_process_state = SignalProcessState::Init;

struct Coordinate_t
{
    double latitude;
    double longitude;
    Coordinate_t( double x, double y ) : latitude( x ), longitude( y ){};
    Coordinate_t() : Coordinate_t( 0, 0 ){};
};

Coordinate_t gps_origin;
// double origin_x = 0.0;
// double origin_y = 0.0;
double origin_yaw = 0.0;
double current_yaw = 0.0;

std::vector< Coordinate_t > coordList;
std::vector< double >       yawList;
ros::Time                   start_time;


bool init_ok = false;
Eigen::Vector3d    init_P, last_P;
Eigen::Quaterniond init_Q, last_Q, vins_Q;
ros::Time          now_t, last_odom_t, last_path_t;
Eigen::Vector3d    Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;
nav_msgs::Path run_path;
nav_msgs::Odometry odom_from_grad;

// From Tianbo.Liu
void pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
    if ( !init_ok )
    {
        init_ok = true;
        init_Q.w() = msg->pose.orientation.w;
        init_Q.x() = msg->pose.orientation.x;
        init_Q.y() = msg->pose.orientation.y;
        init_Q.z() = msg->pose.orientation.z;
        init_P.x() = msg->pose.position.x;
        init_P.y() = msg->pose.position.y;
        init_P.z() = msg->pose.position.z;
        last_P = init_P;
        last_Q = init_Q;
        last_odom_t = msg->header.stamp;
    }
    else
    {
        now_t = msg->header.stamp;

        Eigen::Vector3d    now_P, P_w;
        Eigen::Quaterniond now_Q, Q_w;
        now_P.x() = msg->pose.position.x;
        now_P.y() = msg->pose.position.y;
        now_P.z() = msg->pose.position.z;
        now_Q.w() = msg->pose.orientation.w;
        now_Q.x() = msg->pose.orientation.x;
        now_Q.y() = msg->pose.orientation.y;
        now_Q.z() = msg->pose.orientation.z;

        //        std::cout << "x :" << msg->pose.position.x << "y:" << msg->pose.position.y
        //                  << " z :" << msg->pose.position.z << std::endl;
        // Q_w = init_Q.normalized().toRotationMatrix().transpose() *
        // now_Q.normalized().toRotationMatrix();
        Q_w = now_Q.normalized().toRotationMatrix();
        P_w = now_P - init_P;

        Eigen::Vector3d now_vel;
        now_vel = ( P_w - last_P ) / ( now_t - last_odom_t ).toSec();
        //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
        //        std::cout << " now_vel " << now_vel << std::endl;

        /** velocity filter **/
        Vi0 = now_vel;
        Vo0 = ( Vi0 + Vi1 + Vi2 + Vi3 + Vi4 ) * 0.2;
        Vi4 = Vi3;
        Vi3 = Vi2;
        Vi2 = Vi1;
        Vi1 = Vi0;

        /*********************/

        odom_from_grad.header.stamp = now_t;
        odom_from_grad.header.frame_id = "world";
        odom_from_grad.pose.pose.position.x = P_w.x();
        odom_from_grad.pose.pose.position.y = P_w.y();
        odom_from_grad.pose.pose.position.z = P_w.z();
        odom_from_grad.pose.pose.orientation.w = Q_w.w();
        odom_from_grad.pose.pose.orientation.x = Q_w.x();
        odom_from_grad.pose.pose.orientation.y = Q_w.y();
        odom_from_grad.pose.pose.orientation.z = Q_w.z();
        odom_from_grad.twist.twist.linear.x = Vo0.x(); // now_vel.x();
        odom_from_grad.twist.twist.linear.y = Vo0.y(); // now_vel.y();
        odom_from_grad.twist.twist.linear.z = Vo0.z(); // now_vel.z();

        last_odom_t = now_t;
        last_P = P_w;
        last_Q = Q_w;

        ros::Duration delta_t = now_t - last_path_t;
        if ( delta_t.toSec() > 0.1 )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = now_t;
            pose.header.frame_id = "world";
            pose.pose.orientation.x = odom_from_grad.pose.pose.orientation.x;
            pose.pose.orientation.y = odom_from_grad.pose.pose.orientation.y;
            pose.pose.orientation.z = odom_from_grad.pose.pose.orientation.z;
            pose.pose.orientation.w = odom_from_grad.pose.pose.orientation.w;
            pose.pose.position.x = odom_from_grad.pose.pose.position.x;
            pose.pose.position.y = odom_from_grad.pose.pose.position.y;
            pose.pose.position.z = odom_from_grad.pose.pose.position.z;

            run_path.header.stamp = now_t;
            run_path.header.frame_id = "world";
            run_path.poses.push_back( pose );

            last_path_t = now_t;
        }
    }
}

nav_msgs::Odometry* get_velo_from_gps( nav_msgs::Odometry & odom)
{
    geometry_msgs::PoseStamped pose_stamp;
    odom_from_grad = odom;
    pose_stamp.pose = odom.pose.pose;
    pose_stamp.header = odom.header;
    pose_callback(boost::make_shared< geometry_msgs::PoseStamped const >( pose_stamp ) );
    return &odom_from_grad;

}

void from_gps_to_metric( double lati, double longti, double &x, double &y )
{
    // double la = lati / 180.0 * M_PI;
    // double lo = longti / 180.0 * M_PI;
    // return Coordinate_t(std::cos(la) * std::cos(lo) * earth_radius,
    //                     std::cos(la) * std::sin(lo) * earth_radius);
    double x0, y0, z0;
    GPSUtils::GeodeticToEnu(
        lati, longti, 1.0, gps_origin.latitude, gps_origin.longitude, 1.0, x0, y0, z0 );

    // Transform from ENU to NWU
    x = y0;
    y = -x0;
}

void imu_callback( const sensor_msgs::ImuConstPtr &pMsg )
{
    Eigen::Quaterniond q(
        pMsg->orientation.w, pMsg->orientation.x, pMsg->orientation.y, pMsg->orientation.z
                );

    double yaw = uav_utils::get_yaw_from_quaternion( q );
    q_imu = q;
    if ( signal_process_state == SignalProcessState::Sampling )
    {
//        cout << "SignalProcessState::Sampling" <<endl;
        yawList.push_back( yaw );
    }
    else if ( signal_process_state == SignalProcessState::Normal )
    {
//        cout << "SignalProcessState::Normal" <<endl;
        current_yaw = yaw;
    }
}

void gps_callback( const sensor_msgs::NavSatFixConstPtr &pMsg )
{
    if ( pMsg->status.status >= 2 )
    { // health flag from flight controller
        if ( signal_process_state == SignalProcessState::Init )
        {
            start_time = pMsg->header.stamp;
            signal_process_state = SignalProcessState::Sampling;
            ROS_INFO( "[GPS] Initializing..." );
        }
        else if ( signal_process_state == SignalProcessState::Sampling )
        {
            coordList.emplace_back( pMsg->latitude, pMsg->longitude );

            if ( ( pMsg->header.stamp - start_time ) > signal_sample_duration )
            {
                // Data is enough
                ROS_ASSERT( yawList.size() );
                ROS_ASSERT( coordList.size() );

                double sum_yaw = 0;
                for ( auto y : yawList )
                {
                    sum_yaw += y;
                }
                origin_yaw = sum_yaw / yawList.size();

                double sum_x = 0;
                double sum_y = 0;
                for ( auto &c : coordList )
                {
                    sum_x += c.latitude;
                    sum_y += c.longitude;
                }
                gps_origin.latitude = sum_x / coordList.size();
                gps_origin.longitude = sum_y / coordList.size();

                ROS_INFO( "[GPS] Inited. lati:%.3f longti:%.3f Yaw:%.2f",
                          gps_origin.latitude,
                          gps_origin.longitude,
                          origin_yaw );

                current_yaw = origin_yaw;
                signal_process_state = SignalProcessState::Normal;
            }
        }
        else if ( signal_process_state == SignalProcessState::Normal )
        {
            double x, y;
            from_gps_to_metric( pMsg->latitude, pMsg->longitude, x, y );
            // Current point in ground frame (North-West-Sky)
            Eigen::Vector3d G_p( x, y, 0.0 );

            // Rotation from local frame to ground frame
            Eigen::Matrix3d L_R_G = uav_utils::rotz( origin_yaw ).transpose();

            // Current point in local frame
            Eigen::Vector3d L_p = L_R_G * G_p;

            // Quaternion represents ground frame in local frame
            Eigen::Quaterniond q( L_R_G.transpose() );
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = pMsg->header.stamp;
            odom_msg.header.frame_id = std::string( "world" );
            odom_msg.pose.pose.position.x = L_p( 0 );
            odom_msg.pose.pose.position.y = L_p( 1 );
            odom_msg.pose.pose.position.z = pMsg->altitude;

            q_imu = q_imu*q.inverse();
            odom_msg.pose.pose.orientation.w = q_imu.w();
            odom_msg.pose.pose.orientation.x = q_imu.x();
            odom_msg.pose.pose.orientation.y = q_imu.y();
            odom_msg.pose.pose.orientation.z = q_imu.z();
//            odom_msg.pose.pose.orientation.w = q.w();
//            odom_msg.pose.pose.orientation.x = q.x();
//            odom_msg.pose.pose.orientation.y = q.y();
//            odom_msg.pose.pose.orientation.z = q.z();

//            cout <<"==== GPS ====" <<endl;
//            cout << L_R_G <<endl;
//            cout <<  odom_msg.pose.pose.orientation <<endl;

            double stdGPS = std_factor * 5.0 / static_cast< double >( pMsg->status.status );
            odom_msg.pose.covariance[ 0 + 0 * 6 ] = stdGPS * stdGPS;
            odom_msg.pose.covariance[ 1 + 1 * 6 ] = stdGPS * stdGPS;
            odom_msg.pose.covariance[ 2 + 2 * 6 ] = stdGPS * stdGPS;


           odom_msg =  *get_velo_from_gps( odom_msg);

            if ( static_cast< double >( pMsg->status.status ) < 2.0 )
            {
                odom_msg.header.frame_id = std::string( "invalid" );
            }

            gpsodom_pub.publish( odom_msg );
        }
    }
    else
    {
        ROS_INFO_THROTTLE( 5.0, "[GPS] Low signal quality..." );
        return;
    }

    geometry_msgs::Vector3Stamped height_msg;
    height_msg.header.stamp = pMsg->header.stamp;
    height_msg.header.frame_id = std::string( "world" );
    height_msg.vector.x = 0.0;
    height_msg.vector.y = 0.0;
    height_msg.vector.z = pMsg->altitude;

    height_pub.publish( height_msg );
}

void velo_callback( const geometry_msgs::Vector3StampedConstPtr &veloMsg,
                    const sensor_msgs::ImuConstPtr &             imuMsg )
{
    Eigen::Vector3d    g_v = uav_utils::from_vector3_msg( veloMsg->vector );
    Eigen::Quaterniond w_q_b = uav_utils::from_quaternion_msg( imuMsg->orientation );

    // dji velocity is in north-east-ground, transform it to north-west-sky
    g_v.y() *= -1;
    g_v.z() *= -1;

    Eigen::Vector3d b_v = w_q_b.conjugate() * g_v;

    geometry_msgs::Vector3Stamped msg;
    msg.header = veloMsg->header;
    msg.header.frame_id = "body";
    msg.vector = uav_utils::to_vector3_msg( b_v );

    velo_pub.publish( msg );
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "gps_converter" );
    ros::NodeHandle nh( "~" );

    double dur;
    nh.param( "signal_sample_duration", dur, signal_sample_duration.toSec() );
    nh.param( "std_factor", std_factor, std_factor );
    signal_sample_duration = ros::Duration( dur );

    ros::Subscriber gps_sub = nh.subscribe( "gps", 10, gps_callback );
    ros::Subscriber imu_sub = nh.subscribe( "imu", 10, imu_callback );

    message_filters::Subscriber< geometry_msgs::Vector3Stamped > vel_sub( nh, "velo", 10, ros::TransportHints().tcpNoDelay() );
    message_filters::Subscriber< sensor_msgs::Imu >              ori_sub( nh, "imu", 10, ros::TransportHints().tcpNoDelay() );
    message_filters::TimeSynchronizer< geometry_msgs::Vector3Stamped, sensor_msgs::Imu > sync(
        vel_sub, ori_sub, 10 );
    sync.registerCallback( boost::bind( &velo_callback, _1, _2 ) );

    height_pub = nh.advertise< geometry_msgs::Vector3Stamped >( "height", 10 );
    gpsodom_pub = nh.advertise< nav_msgs::Odometry >( "odom", 10 );
    velo_pub = nh.advertise< geometry_msgs::Vector3Stamped >( "body_velocity", 10 );

    ros::spin();

    return 0;
}
