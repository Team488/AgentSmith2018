#include <iomanip>

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/array.hpp>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <can_msgs/Frame.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <xbot_robot_comms/RobotWheelOdomPacket.h>
#include <xbot_robot_comms/DriveCommandPacket.h>
#include <xbot_robot_comms/ElevatorPositionCommandPacket.h>
#include <xbot_robot_comms/CommandResultPacket.h>
#include <xbot_obstacle_detector/ObstacleInfo.h>
#include <xbot_vision/CubeTargetInfo.h>

#define ARBID_ROOT_MASK        0xFFFF0000
#define ARBID_ROOT             0x1E040000
#define ARBID_SOURCE_ID_MASK   0x0000FF00
#define ARBID_PACKET_TYPE_MASK 0x000000FF

#define PACKET_TYPE_WHEEL_ODOM 0x01
#define PACKET_TYPE_ORIENTATION 0x02
#define PACKET_TYPE_SET_MODE 0x03
#define PACKET_TYPE_HEADING 0x04
#define PACKET_TYPE_SET_COMMAND 0x06
#define PACKET_TYPE_SCORING_PLACEMENT 0x08

#define PACKET_TYPE_DRIVE_POWER_COMMAND 0x05
#define PACKET_TYPE_DRIVE_VEL_COMMAND 0x09
#define PACKET_TYPE_COMMAND_FINISHED 0x07
#define PACKET_TYPE_DISTANCE_FORWARD 0x08
#define PACKET_TYPE_ELEVATOR_POSITION_COMMAND 0x0A
#define PACKET_TYPE_DETECTED_CUBE 0x0B

#define SOURCE_ID_LOCAL 0x00
#define SOURCE_ID_RIO 0x01

class CommsNode
{
private:
    ros::NodeHandle node;

    ros::Publisher wheel_odometry_publisher;
    ros::Publisher orientation_publisher;
    ros::Publisher heading_publisher;
    ros::Publisher mode_publisher;
    ros::Publisher command_from_rio;
    ros::Publisher scoring_placement_publisher;
    ros::Subscriber command_to_rio;
    ros::Subscriber distance_forward_subscriber;
    ros::Subscriber cube_point_subscriber;

    ros::Subscriber drive_power_command_subscriber;
    ros::Subscriber drive_vel_command_subscriber;

    ros::Publisher can_send;
    ros::Subscriber can_recv;

    template <uint8_t count, uint8_t offset> uint16_t get_bits(uint64_t& data)
    {
        return (data >> offset) & ((1 << count) - 1);
    }

    void handle_wheel_odom_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 6) {
            ROS_WARN_STREAM("Bad size for wheel odometry packet; expected 6, was " << packet_data_array.size());
            return;
        }

        const uint16_t* items = (uint16_t*)packet_data_array.c_array();
        int16_t left_drive_raw = be16toh(items[0]);
        int16_t right_drive_raw = be16toh(items[1]);
        int16_t time_raw = be16toh(items[2]);

        const double left_drive_delta = double(left_drive_raw) / 1000; // 1_000
        const double right_drive_delta = double(right_drive_raw) / 1000; // 1_000
        const double time_delta = double(time_raw) / 10000; // 10_000

        xbot_robot_comms::RobotWheelOdomPacket message;
        message.left_drive_delta = left_drive_delta;
        message.right_drive_delta = right_drive_delta;
        message.time_delta = time_delta;

        wheel_odometry_publisher.publish(message);
    }

    void handle_orientation_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 8) {
            ROS_WARN_STREAM("Bad size for orientation packet; expected 8, was " << packet_data_array.size());
            return;
        }

        const int16_t* items = (int16_t*)packet_data_array.c_array();

        const double w = double(items[0]) / 10000; // 10_000
        const double x = double(items[1]) / 10000; // 10_000
        const double y = double(items[2]) / 10000; // 10_000
        const double z = double(items[3]) / 10000; // 10_000

        geometry_msgs::Quaternion message;
        message.w = w;
        message.x = x;
        message.y = y;
        message.z = z;
        orientation_publisher.publish(message);
    }

    void handle_heading_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 2) {
            ROS_WARN_STREAM("Bad size for heading packet; expected 4, was " << packet_data_array.size());
            return;
        }

        const int16_t* items = (int16_t*)packet_data_array.c_array();

        const float heading = float(items[0]) / 100;

        // TODO: Timestamp messages and handle old data
        std_msgs::Float32 message;
        message.data = heading;
        heading_publisher.publish(message);
    }

    void handle_set_mode_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 1) {
            ROS_WARN_STREAM("Bad size for mode packet; expected 1, was " << packet_data_array.size());
            return;
        }
        
        std_msgs::UInt8 message;
        message.data = packet_data_array[0];
        mode_publisher.publish(message);
    }

        
    void handle_set_command_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 1) {
            ROS_WARN_STREAM("Bad size for set command packet; expected 1, was " << packet_data_array.size());
            return;
        }
        // Command ID -- 0 means stop
        std_msgs::UInt8 message;
        message.data = packet_data_array[0];

        ROS_INFO_STREAM("Received set command packet for command id " << std::to_string(message.data));
        command_from_rio.publish(message);
    }


    void handle_scoring_placement_packet(uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
        if(size != 1) {
            ROS_WARN_STREAM("Bad size for scoring placement packet; expected 1, was " << packet_data_array.size());
            return;
        }

        std_msgs::UInt8 message;
        message.data = packet_data_array[0];
        scoring_placement_publisher.publish(message);

    }

    void handle_packet(uint8_t packet_type, uint8_t size, boost::array<uint8_t, 8UL> packet_data_array)
    {
	    ROS_DEBUG_STREAM("Received packet with type " << packet_type);
        switch(packet_type) {
            case PACKET_TYPE_WHEEL_ODOM:
                handle_wheel_odom_packet(size, packet_data_array);
                break;
            case PACKET_TYPE_ORIENTATION:
                handle_orientation_packet(size, packet_data_array);
                break;
            case PACKET_TYPE_SCORING_PLACEMENT:
                handle_scoring_placement_packet(size, packet_data_array);
                break;
            case PACKET_TYPE_SET_MODE:
                handle_set_mode_packet(size, packet_data_array);
                break;
            case PACKET_TYPE_HEADING:
                handle_heading_packet(size, packet_data_array);
                break;
            case PACKET_TYPE_SET_COMMAND:
                handle_set_command_packet(size, packet_data_array);
                break;
            default:
                ROS_WARN_STREAM("Unknown packet type " << packet_type);
        }
    }

    void can_receive_callback(const can_msgs::Frame& can_frame)
    {
        ROS_DEBUG_STREAM("Received packet with id " << can_frame.id);
        if(can_frame.is_error) {
            std::cout << "Error data: ";
            for (int i = 0; i < can_frame.data.size(); ++i) {
                //std::cout << "0x";
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << can_frame.data.at(i) << " ";
                std::cout << can_frame.data.at(i) << " ";
            }
            std::cout << std::endl;
            return;
        }

        if (can_frame.is_rtr) {
            return;
        }

        if((can_frame.id & ARBID_ROOT_MASK) == ARBID_ROOT) {
            uint8_t source_id = (can_frame.id & ARBID_SOURCE_ID_MASK) >> 8;

            if(source_id != SOURCE_ID_RIO) {
                return;
            }

            uint8_t packet_type = can_frame.id & ARBID_PACKET_TYPE_MASK;
            handle_packet(packet_type, can_frame.dlc, can_frame.data);
        }
    }

    void send_packet(uint8_t packet_type, boost::array<uint8_t, 8> data, uint8_t data_length)
    {
        uint32_t id = ARBID_ROOT | (SOURCE_ID_LOCAL << 8) | packet_type;

        can_msgs::Frame can_frame;
        can_frame.header.stamp = ros::Time::now();
        can_frame.header.frame_id = "";
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = data_length;
        can_frame.id = id;
        can_frame.data = data;

        this->can_send.publish(can_frame);
    }

    void send_distance_forward_packet(const xbot_obstacle_detector::ObstacleInfo& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;
        int16_t distance_forward = (int16_t)packet.distance;
        uint8_t distance_forward_data = htobe16(distance_forward);

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        *((int16_t*)(&packet_bytes[0])) = distance_forward_data;

        send_packet(PACKET_TYPE_DISTANCE_FORWARD, packet_data, 2);
    }

    void send_drive_power_command_packet(const xbot_robot_comms::DriveCommandPacket& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;

        int16_t left_drive_int = (int16_t)(packet.left_drive * 3000);
        int16_t right_drive_int = (int16_t)(packet.right_drive * 3000);

        uint16_t left_drive_data = htobe16(left_drive_int);
        uint16_t right_drive_data = htobe16(right_drive_int);

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        packet_bytes[0] = packet.owner_command;
        *((uint16_t*)(&packet_bytes[1])) = left_drive_data;
        *((int16_t*)(&packet_bytes[3])) = right_drive_data;

        send_packet(PACKET_TYPE_DRIVE_POWER_COMMAND, packet_data, 5);
    }

    void send_drive_vel_command_packet(const xbot_robot_comms::DriveCommandPacket& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;

        int16_t left_drive_int = (int16_t)(packet.left_drive * 100);
        int16_t right_drive_int = (int16_t)(packet.right_drive * 100);

        uint16_t left_drive_data = htobe16(left_drive_int);
        uint16_t right_drive_data = htobe16(right_drive_int);

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        packet_bytes[0] = packet.owner_command;
        *((uint16_t*)(&packet_bytes[1])) = left_drive_data;
        *((int16_t*)(&packet_bytes[3])) = right_drive_data;

        send_packet(PACKET_TYPE_DRIVE_VEL_COMMAND, packet_data, 5);
    }

    void send_elevator_goal_command_packet(const xbot_robot_comms::ElevatorPositionCommandPacket& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        packet_bytes[0] = packet.goal_elevator_position;

        send_packet(PACKET_TYPE_ELEVATOR_POSITION_COMMAND, packet_data, 1);
    }

    void send_command_result_packet(const xbot_robot_comms::CommandResultPacket& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        packet_bytes[0] = packet.command_id;
        packet_bytes[1] = packet.status_code;

        send_packet(PACKET_TYPE_COMMAND_FINISHED, packet_data, 2);
    }

    void send_cube_target_packet(const xbot_vision::CubeTargetInfo& packet)
    {
        boost::array<uint8_t, 8UL> packet_data;

        int16_t x_int = (int16_t)(packet.target_cube_point.point.x * 300);
        int16_t y_int = (int16_t)(packet.target_cube_point.point.y * 300);
        int16_t z_int = (int16_t)(packet.target_cube_point.point.z * 300);

        uint16_t x_data = htobe16(x_int);
        uint16_t y_data = htobe16(y_int);
        uint16_t z_data = htobe16(z_int);

        uint8_t *packet_bytes = (uint8_t*)packet_data.c_array();
        packet_bytes[0] = packet.has_tracked_cube ? 1 : 0;
        *((uint16_t*)(&packet_bytes[1])) = x_data;
        *((int16_t*)(&packet_bytes[3])) = y_data;
        *((int16_t*)(&packet_bytes[5])) = z_data;

        send_packet(PACKET_TYPE_DETECTED_CUBE, packet_data, 7);
    }

public:
    void subscribe_and_publish()
    {
        wheel_odometry_publisher = node.advertise<xbot_robot_comms::RobotWheelOdomPacket>("/comms/wheel_odometry", 5);
        orientation_publisher = node.advertise<geometry_msgs::Quaternion>("/comms/orientation", 5);
        heading_publisher = node.advertise<std_msgs::Float32>("/comms/heading", 5, true); //Currently in degrees
        scoring_placement_publisher = node.advertise<std_msgs::UInt8>("/comms/scoring_placement",5,true);
        mode_publisher = node.advertise<std_msgs::UInt8>("/comms/set_mode", 5, true);

        command_from_rio = node.advertise<std_msgs::UInt8>("/comms/current_command_from_rio", 1, true);
        command_to_rio = node.subscribe("/comms/command_result_to_rio", 1, &CommsNode::send_command_result_packet, this);
        
        distance_forward_subscriber = node.subscribe("/comms/distance_forward",2,&CommsNode::send_distance_forward_packet,this);
        cube_point_subscriber = node.subscribe("/vision/tracked_cube_target",2,&CommsNode::send_cube_target_packet,this);
        drive_power_command_subscriber = node.subscribe("/comms/send_drive_power", 2, &CommsNode::send_drive_power_command_packet, this);
        drive_vel_command_subscriber = node.subscribe("/comms/send_drive_vel", 2, &CommsNode::send_drive_vel_command_packet, this);

        can_send = node.advertise<can_msgs::Frame>("/sent_messages", 5);
        can_recv = node.subscribe("/received_messages", 5, &CommsNode::can_receive_callback, this);
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "xbot_robot_comms_node");

    CommsNode node;
    node.subscribe_and_publish();
    ros::spin();
}
