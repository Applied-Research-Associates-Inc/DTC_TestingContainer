#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

#include <carla_interfaces/msg/carla_multirotor_control.hpp>

#include <mutex>
#include <queue>

#include <sensor_msgs/msg/imu.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
// #include <Eigen/Eigen>

#include "libAP_JSON.h"

#include <math.h>

#define EQUAL_THRESHOLD 0.1
#define ARDU_PWM_INPUT_MIN 1100.0
#define ARDU_PWM_INPUT_MAX 1950.0


class ArduController : public rclcpp::Node
{
  public:
    ArduController( std::string topicPrefix,
                    int vehicle_number,
                    double worldRotation,
                    int rotors ) :
        Node("dtc_quadcopter"),
        m_controlPublisher{ this->create_publisher<carla_interfaces::msg::CarlaMultirotorControl>(
            topicPrefix + "/multirotor_control_cmd", 30 ) },
        m_shutdownSubscriber{ this->create_subscription<std_msgs::msg::Empty>(
                                                   topicPrefix + "/shutdown",
                                                   1,
                                                   [this]( const std_msgs::msg::Empty& msg ) {
                                                       (void)msg;
                                                    //    std::lock_guard<std::mutex> guard( m_dataMutex );
                                                       this->shutdown();
                                                   } ) },
        m_odometrySubscriber{ this->create_subscription<nav_msgs::msg::Odometry>(
                                                   topicPrefix + "/odometry",
                                                   30,
                                                   [this]( const nav_msgs::msg::Odometry& msg ) {
                                                    //    std::lock_guard<std::mutex> guard( m_dataMutex );
                                                       m_odometryMsgQueue.push( msg );
                                                       manageMessageQueues();
                                                   } ) },
        m_vehicleImuSubscriber{ this->create_subscription<sensor_msgs::msg::Imu>(
            topicPrefix + "/imu", 30, [this]( const sensor_msgs::msg::Imu& msg ) {
                // std::lock_guard<std::mutex> guard( m_dataMutex );
                m_vehicleImuMsgQueue.push( msg );
                manageMessageQueues();
            } ) }
    {
        m_ardupilotConnection.InitSockets( "0.0.0.0", 9002 + ( 10 * vehicle_number ) );
        m_worldRotation = worldRotation;
        m_vehicleRotors = rotors;
    }

    /// @brief Keeps the timestamps of the front of the imu and odometry queues equal.
    /// Calls update control with an imu and odometry message pair and removes that pair from their queues.
    void manageMessageQueues()
    {
        // RCLCPP_WARN(this->get_logger(), "odom %d imu: %d", m_odometryMsgQueue.size(), m_vehicleImuMsgQueue.size());

        if ( m_odometryMsgQueue.size() > 0 && m_vehicleImuMsgQueue.size() > 0 )
        {
            rclcpp::Time imuStamp = rclcpp::Time(m_vehicleImuMsgQueue.front().header.stamp);
            rclcpp::Time odomStamp = rclcpp::Time(m_odometryMsgQueue.front().header.stamp);

            // The front of each queue contains the oldest message.
            // Remove any messages which will never have a matching message from the other topic.

            while ( ( odomStamp - imuStamp ).seconds() > EQUAL_THRESHOLD )
            {
                RCLCPP_WARN(this->get_logger(), "pop imu %f", ( odomStamp - imuStamp ).seconds());
                m_vehicleImuMsgQueue.pop();
                if ( m_vehicleImuMsgQueue.size() <= 0 )
                {
                    return;
                }
                imuStamp = rclcpp::Time(m_vehicleImuMsgQueue.front().header.stamp);
            }

            while ( ( imuStamp - odomStamp ).seconds() > EQUAL_THRESHOLD )
            {
                RCLCPP_WARN(this->get_logger(), "pop odom %f", ( imuStamp - odomStamp ).seconds());
                m_odometryMsgQueue.pop();
                if ( m_odometryMsgQueue.size() <= 0 )
                {
                    return;
                }
                odomStamp = rclcpp::Time(m_odometryMsgQueue.front().header.stamp);
            }

            // If we have a message pair call updateControl with that pair and remove them from the queues.
            if ( m_odometryMsgQueue.size() > 0 && m_vehicleImuMsgQueue.size() > 0 &&
                 abs( ( imuStamp - odomStamp ).seconds() ) <= EQUAL_THRESHOLD )
            {
                updateControl( m_vehicleImuMsgQueue.front(), m_odometryMsgQueue.front() );
                m_odometryMsgQueue.pop();
                m_vehicleImuMsgQueue.pop();
                while ( m_odometryMsgQueue.size() > 100 && m_vehicleImuMsgQueue.size() > 100 )
                {
                    // Make sure the queue lengths aren't too long
                    m_odometryMsgQueue.pop();
                    m_vehicleImuMsgQueue.pop();
                }
            }
        }
    }

    inline double stepLerp( double tt, double start, double stop )
    {
        return start + tt * ( stop - start );
    }

    template<typename T>
    inline T stepLerpVector3( double tt, T start, T stop )
    {
        T result;
        result.x = stepLerp( tt, start.x, stop.x );
        result.y = stepLerp( tt, start.y, stop.y );
        result.z = stepLerp( tt, start.z, stop.z );
        return result;
    }

    void sendStateInterpolated(double start_timestamp, double end_timestamp, 
        geometry_msgs::msg::Vector3 start_gyro, geometry_msgs::msg::Vector3 end_gyro, 
        geometry_msgs::msg::Vector3 start_accel, geometry_msgs::msg::Vector3 end_accel, 
        geometry_msgs::msg::Point start_pos, geometry_msgs::msg::Point end_pos, 
        geometry_msgs::msg::Vector3 start_euler, geometry_msgs::msg::Vector3 end_euler, 
        geometry_msgs::msg::Vector3 start_vel, geometry_msgs::msg::Vector3 end_vel, 
        int increments)
    {
        // RCLCPP_WARN(this->get_logger(), "interpolate from %f", start_timestamp);

        for (int i = 0; i < increments; i++){
            double tt = static_cast<double>(i) / increments;

            double lerp_timestamp = stepLerp(tt, start_timestamp, end_timestamp);
            auto lerp_gyro = stepLerpVector3(tt, start_gyro, end_gyro);
            auto lerp_acceleration = stepLerpVector3(tt, start_accel, end_accel);
            auto lerp_position = stepLerpVector3(tt, start_pos, end_pos);
            auto lerp_euler = stepLerpVector3(tt, start_euler, end_euler);
            auto lerp_vel = stepLerpVector3(tt, start_vel, end_vel);

            // RCLCPP_WARN(this->get_logger(), "time %f", lerp_timestamp);

            m_ardupilotConnection.SendState( lerp_timestamp,
                                            -lerp_gyro.x, //
                                            -lerp_gyro.y, // 
                                            lerp_gyro.z,
                                            lerp_acceleration.x, 
                                            -lerp_acceleration.y, 
                                            lerp_acceleration.z,
                                            lerp_position.x,
                                            lerp_position.y,
                                            lerp_position.z,
                                            lerp_euler.x,
                                            lerp_euler.y,
                                            lerp_euler.z + m_worldRotation,
                                            lerp_vel.x,
                                            lerp_vel.y,
                                            lerp_vel.z );

            // rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(0.9 * (end_timestamp - start_timestamp) * 1e9 / increments)));
            rclcpp::sleep_for(std::chrono::nanoseconds(5000));
        }
        
        // RCLCPP_WARN(this->get_logger(), "finished at %f", end_timestamp);

    }

    /// @brief Compute controls and write to the simulator
    /// Update each wheel's PIQD controller with the latest command and
    /// measurement. Apply thrusts and brakes for each wheel.
    void updateControl( const sensor_msgs::msg::Imu& imuMsg, const nav_msgs::msg::Odometry& odomMsg )
    {
        // RCLCPP_WARN(this->get_logger(), "Update Control");
        // if (m_interpThread)
        // {
        //     m_interpThread->join();
        // }

        m_ardupilotConnection.ReceiveServoPacket( m_servo_out );

        if ( !m_ardupilotConnection.ap_online )
        {
            carla_interfaces::msg::CarlaMultirotorControl control;
            control.throttle = std::vector<float>( m_vehicleRotors, 0.0 );
            m_controlPublisher->publish( control );
            return;
        }
        carla_interfaces::msg::CarlaMultirotorControl control;
        control.throttle = std::vector<float>( m_vehicleRotors );
        for ( size_t i = 0; i < m_vehicleRotors; i++ )
        {
            // ArduPilot outputs the pwm values of the controller,
            // convert these to throttle values for the motors
            control.throttle[i] =
                _interp1D( m_servo_out[i], ARDU_PWM_INPUT_MIN, ARDU_PWM_INPUT_MAX, 0.0, 1.0 );
        }

        m_controlPublisher->publish( control );

        auto stamp       = imuMsg.header.stamp;
        double timestamp = stamp.sec + ( stamp.nanosec * 1e-9 );
        auto acceleration{ imuMsg.linear_acceleration };
        auto position{ odomMsg.pose.pose.position };
        auto gyro{ imuMsg.angular_velocity };
        auto orientation{ imuMsg.orientation };
        double roll;
        double pitch;
        double yaw;

        tf2::Quaternion tf2_quat(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
        geometry_msgs::msg::Vector3 euler;
        euler.x = roll;
        euler.y = pitch;
        euler.z = yaw;

        position.z = -position.z;

        double worldAngleInRads = -m_worldRotation * ( M_PI / 180.0 );
        double xNew             = position.x * cos( worldAngleInRads ) - position.y * sin( worldAngleInRads );
        double yNew             = position.x * sin( worldAngleInRads ) + position.y * cos( worldAngleInRads );

        position.x = xNew;
        position.y = -yNew;

        acceleration.y = -acceleration.y;
        acceleration.z = -acceleration.z;

        double deltaTime = timestamp - m_lastTimestamp;
        if ( deltaTime <= 0.000001 )
        {
            return;
        }

        double velocityX = ( position.x - m_lastPosition.x ) / deltaTime;
        double velocityY = ( position.y - m_lastPosition.y ) / deltaTime;
        double velocityZ = ( position.z - m_lastPosition.z ) / deltaTime;

        geometry_msgs::msg::Vector3 velocity;
        velocity.x = velocityX;
        velocity.y = velocityY;
        velocity.z = velocityZ;

        // copy values and capture them for lambda function
        double prev_timestamp = m_lastTimestamp;
        geometry_msgs::msg::Vector3 prevGyro = m_prevGyro;
        geometry_msgs::msg::Vector3 prevAcceleration = m_prevAcceleration;
        geometry_msgs::msg::Point prevPosition = m_prevPosition;
        geometry_msgs::msg::Vector3 prevEuler = m_prevEuler;
        geometry_msgs::msg::Vector3 prevVelocity = m_prevVelocity;

        m_interpThread = std::thread( [this, prev_timestamp, timestamp, prevGyro, gyro, prevAcceleration, acceleration, prevPosition, position, prevEuler, euler, prevVelocity, velocity] {
            sendStateInterpolated(prev_timestamp, timestamp, 
                prevGyro, gyro, 
                prevAcceleration, acceleration, 
                prevPosition, position, 
                prevEuler, euler, 
                prevVelocity, velocity, 
            20);
            }
        );

        m_interpThread->detach();

        m_prevGyro = gyro;
        m_prevAcceleration = acceleration;
        m_prevPosition = position;
        m_prevEuler = euler;
        m_prevVelocity = velocity;

        
        // m_ardupilotConnection.SendState( timestamp,
        //                                  gyro.x,
        //                                  -gyro.y,
        //                                  -gyro.z,
        //                                  acceleration.x,
        //                                  acceleration.y,
        //                                  acceleration.z,
        //                                  position.y,
        //                                  position.x,
        //                                  position.z,
        //                                  roll,
        //                                  pitch,
        //                                  yaw + m_worldRotation,
        //                                  velocityY,
        //                                  velocityX,
        //                                  velocityZ );

        m_lastTimestamp = timestamp;
        m_lastPosition  = position;
    }

    /// @brief Shuts down node
    void shutdown() { rclcpp::shutdown(); }

  private:
    std::mutex m_dataMutex;
    std::queue<nav_msgs::msg::Odometry> m_odometryMsgQueue;
    std::queue<sensor_msgs::msg::Imu> m_vehicleImuMsgQueue;

    rclcpp::Publisher<carla_interfaces::msg::CarlaMultirotorControl>::SharedPtr m_controlPublisher;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_shutdownSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometrySubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_vehicleImuSubscriber;
    libAP_JSON m_ardupilotConnection;
    double m_lastTimestamp = 0.0;
    geometry_msgs::msg::Point m_lastPosition;
    uint16_t m_servo_out[16];
    double m_worldRotation = 0.0;
    size_t m_vehicleRotors;

    geometry_msgs::msg::Vector3 m_prevGyro;
    geometry_msgs::msg::Vector3 m_prevAcceleration;
    geometry_msgs::msg::Point m_prevPosition;
    geometry_msgs::msg::Vector3 m_prevEuler;
    geometry_msgs::msg::Vector3 m_prevVelocity;
    std::optional<std::thread> m_interpThread;

    double _interp1D(
        const double& x, const double& x0, const double& x1, const double& y0, const double& y1 )
    {
        double retVal = ( ( y0 * ( x1 - x ) ) + ( y1 * ( x - x0 ) ) ) / ( x1 - x0 );
        retVal        = std::max( retVal, y0 );
        retVal        = std::min( retVal, y1 );
        return retVal;
    }
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    const std::string topicPrefix = "/carla/dtc_vehicle"; 
    const int vehicleNum = 0;
    const int worldRotation = 0;
    const int rotorCount = 4;

    auto nodeHandle = std::make_shared<ArduController>(topicPrefix, vehicleNum, worldRotation, rotorCount);

    while (rclcpp::ok()) {
        rclcpp::spin(nodeHandle);
    }

    rclcpp::shutdown();

    return 0;
}
