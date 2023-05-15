#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <main/hololens_sub.hpp>
#include <moveit_ur5_interface/moveit_pose_client.hpp>

#include <pnp_actionlib/pnp_action.hpp>
#include <pnp_actionlib/robot_action.hpp>
#include <pnp_actionlib/gripper_action.hpp>

#include <registration_service/registration_client.hpp>
#include <registration_msgs/srv/registration.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <robotiq_2f_msgs/msg/command_state.hpp>
#include <main/gripper_pub.hpp>

#include <signal.h>

//TODO
//error handling for failed planning 
//use orientation of cube

volatile sig_atomic_t stop;

void inthand(int signum) {
    stop = 1;
}

using namespace std::chrono_literals;

// states:
// INIT -> REG -> IDLE -> PNP -> IDLE -> ...
enum STATE
{
    INIT,
    REGISTRATION,
    IDLE,
    PNP,
    ERROR
};
STATE state = INIT;
std::string error_msg;

int main(int argc, char **argv)
{
    signal(SIGINT, inthand);
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(1);
    auto node = std::make_shared<rclcpp::Node>("main_node");
    RCLCPP_INFO_STREAM(node->get_logger(), "Initialising main node");

    // clients and subscribers
    std::shared_ptr<moveit_ur5::client> moveit_client = std::make_shared<moveit_ur5::client>("robot_client");
    rclcpp::executors::MultiThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_client);

    std::shared_ptr<registration_client> reg_client = std::make_shared<registration_client>("reg_client");
    rclcpp::executors::MultiThreadedExecutor reg_executor;
    reg_executor.add_node(reg_client);

    std::shared_ptr<hololensSubscriber> hololens_reg_sub = std::make_shared<hololensSubscriber>("marker_pose");
    rclcpp::executors::MultiThreadedExecutor reg_sub_executor;
    reg_sub_executor.add_node(hololens_reg_sub);

    std::shared_ptr<hololensSubscriber> hololens_pick_sub = std::make_shared<hololensSubscriber>("pick_pose");
    std::shared_ptr<hololensSubscriber> hololens_place_sub = std::make_shared<hololensSubscriber>("place_pose");
    rclcpp::executors::MultiThreadedExecutor pnp_sub_executor;
    pnp_sub_executor.add_node(hololens_pick_sub);
    pnp_sub_executor.add_node(hololens_place_sub);

    auto pnpc = std::make_shared<rsp::pnp_client>("pnp");
    rclcpp::executors::MultiThreadedExecutor pnp_executor;
    pnp_executor.add_node(pnpc);

    // tf2 listener
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // gripper publisher
    gripper_publisher gripper_publisher("robotiq_2f_command");

    // registration configuration file
    std::string main_share_dir = ament_index_cpp::get_package_share_directory("main");
    std::ifstream reg_cfg_file(main_share_dir + "/json/reg_cfg.json");
    if (!reg_cfg_file.is_open())
    {
        state = ERROR;
        error_msg = "failed to read config json file";
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Reading registration config file");
    Json::Reader reader;
    Json::Value reg_cfg;
    reader.parse(reg_cfg_file, reg_cfg);
    int reg_pos_count = reg_cfg["reg_pos_count"].asInt();

    std::vector<geometry_msgs::msg::Pose> robot_positions(reg_pos_count);
    std::vector<geometry_msgs::msg::Pose> hololens_positions(reg_pos_count);
    geometry_msgs::msg::Pose robot_home;

    for (int i = 0; i < reg_pos_count; i++)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = reg_cfg["position"][i]["x"].asFloat();
        p.position.y = reg_cfg["position"][i]["y"].asFloat();
        p.position.z = reg_cfg["position"][i]["z"].asFloat();
        p.orientation.x = reg_cfg["orientation"]["x"].asFloat();
        p.orientation.y = reg_cfg["orientation"]["y"].asFloat();
        p.orientation.z = reg_cfg["orientation"]["z"].asFloat();
        p.orientation.w = reg_cfg["orientation"]["w"].asFloat();
        robot_positions[i] = p;

        std::cout << "position " << i << " [ "
                  << p.position.x << " "
                  << p.position.y << " "
                  << p.position.z << " ]" << std::endl;
    }
    geometry_msgs::msg::PointStamped marker_offset;
    marker_offset.point.x = reg_cfg["marker_offset"]["x"].asFloat();;
    marker_offset.point.y = reg_cfg["marker_offset"]["y"].asFloat();;
    marker_offset.point.z = reg_cfg["marker_offset"]["z"].asFloat();;

    robot_home.position.x = reg_cfg["home_position"]["x"].asFloat();
    robot_home.position.y = reg_cfg["home_position"]["y"].asFloat();
    robot_home.position.z = reg_cfg["home_position"]["z"].asFloat();
    robot_home.orientation.x = reg_cfg["home_orientation"]["x"].asFloat();
    robot_home.orientation.y = reg_cfg["home_orientation"]["y"].asFloat();
    robot_home.orientation.z = reg_cfg["home_orientation"]["z"].asFloat();
    robot_home.orientation.w = reg_cfg["home_orientation"]["w"].asFloat();

    RCLCPP_INFO_STREAM(node->get_logger(), "Registration file read complete");

    // pick and place pose
    geometry_msgs::msg::Pose place_goal;
    geometry_msgs::msg::Pose pick_goal;

    // start state machine
    while (rclcpp::ok() && state != ERROR)
    {
        switch (state)
        {

        case INIT:
        {
            gripper_publisher.publish("open");
            char choice = 'y';
            std::cout << "Perform registration? (y/n): ";
            choice = std::getchar();
            if (choice == 'n' || choice == 'N')
            {
                state = IDLE;
            }
            else
            {
                state = REGISTRATION;
            }
            break;
        }
        case REGISTRATION:
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "REGISTRATION state");
            auto moveit_result = moveit_client->call(robot_home);
            moveit_executor.spin_until_future_complete(moveit_result);
            auto response = moveit_result.get();
            if (response->result != "success")
            {
                state = ERROR;
                error_msg = "failed to reach home pos";
            }
            std::cout << "Place the marker in the gripper and then press enter..........";
            std::getchar();
            std::getchar();
            gripper_publisher.publish("close");
            std::this_thread::sleep_for(5000ms);

            // move the robot to registration poses and wait for pose message from hololens
            for (int i = 0; i < reg_pos_count; i++)
            {
                // moveit goal
                geometry_msgs::msg::Pose reg_goal;
                reg_goal = robot_positions[i];

                auto moveit_result = moveit_client->call(reg_goal);
                moveit_executor.spin_until_future_complete(moveit_result);
                auto response = moveit_result.get();
                if (response->result != "success")
                {
                    state = ERROR;
                    error_msg = "failed to reach reg goal";
                }

                // read from hololens
                while (!hololens_reg_sub->isMessageReady() && !stop)
                {
                    reg_sub_executor.spin_once();
                }
                hololens_reg_sub->getPose(hololens_positions[i]);
                // RCLCPP_INFO_STREAM(node->get_logger(), "Pose message received : (" << hololens_positions[i].position.x << ", "
                //                                                                    << hololens_positions[i].position.y << ", " << hololens_positions[i].position.z << ")");
            }

            // send two pose arrays to registration server once all registration poses have been collected
            auto reg_result = reg_client->send_request(robot_positions, hololens_positions, marker_offset.point);
            reg_executor.spin_until_future_complete(reg_result);
            auto reg_response = reg_result.get();
            if (reg_response->info != "success")
            {
                state = ERROR;
                error_msg = "failed to compute registration";
            }
            else
            {
                RCLCPP_INFO_STREAM(node->get_logger(), "Registration Accuracy: " << reg_response->error);
                RCLCPP_INFO_STREAM(node->get_logger(), "Computed Transform XYZ: (" << reg_response->transform.position.x << ", "
                                                                                   << reg_response->transform.position.y << ", " << reg_response->transform.position.z << ")");
            }

            moveit_result = moveit_client->call(robot_home);
            moveit_executor.spin_until_future_complete(moveit_result);
            response = moveit_result.get();
            if (response->result != "success")
            {
                state = ERROR;
                error_msg = "failed to reach home pos";
            }

            std::cout << "Press enter to open gripper and remove marker..........";
            std::getchar();
            gripper_publisher.publish("open");
            std::this_thread::sleep_for(5000ms);

            state = IDLE;
            break;
        }

        case IDLE:
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "IDLE state");
            std::this_thread::sleep_for(2000ms);

            // recieve pick and place positions
            while ( ( !hololens_pick_sub->isMessageReady() || !hololens_place_sub->isMessageReady() ) && !stop)
            {
                pnp_sub_executor.spin_once();
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "Pick and Place positions received");
            hololens_pick_sub->getPose(pick_goal);
            hololens_place_sub->getPose(place_goal);

            // transform pick and place goal to robot frame using tf
            geometry_msgs::msg::PointStamped initial_pt;
            geometry_msgs::msg::PointStamped transformed_pt;
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer->lookupTransform("holo_lens_home", "base_link", tf2::TimePointZero);

            initial_pt.point = pick_goal.position;
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            pick_goal.position = transformed_pt.point;

            initial_pt.point = place_goal.position;
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            place_goal.position = transformed_pt.point;

            // orientation is always gripper down
            pick_goal.orientation.x = -1;
            pick_goal.orientation.y = 0;
            pick_goal.orientation.z = 0;
            pick_goal.orientation.w = 0;

            place_goal.orientation.x = -1;
            place_goal.orientation.y = 0;
            place_goal.orientation.z = 0;
            place_goal.orientation.w = 0;

            state = PNP;
            break;
        }

        case PNP:
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "PNP state");

            // send a request to pnp action server and wait for robot to finish pnp
            auto pnp_result = pnpc->pnp_call(pick_goal, place_goal);
            pnp_executor.spin_until_future_complete(pnp_result);

            std::this_thread::sleep_for(2000ms);
            state = IDLE;
            break;
        }
        case ERROR:
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR message: " << error_msg);
            // code to handle error needs to go here
            std::getchar();
            break;
        }
        }

        rate.sleep();
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Shutting down main node");
    rclcpp::shutdown();
    return 0;
}
