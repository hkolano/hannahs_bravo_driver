#include <rsa_bravo_driver/bravo_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace bravo_base
{
    BravoHWInterface::BravoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        ROS_DEBUG("Initializing node!");

        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        // error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "bravo/debug/hardware_interface", debug_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        // Initialize the hardware interface
        init(nh_, nh_);

        // Wait for encoder messages being published
        isReceivingMeasuredJointStates(ros::Duration(10));

        ROS_INFO_STREAM("Initialized RobotHW!");
    }

 
    bool BravoHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing Bravo Comms Client ...");

        const std::string ipAddress("192.168.2.3"); // Default IP address for Bravo
        const unsigned int port = 6789;             // Default port for Bravo

        client_ =  libbpl_protocol::SyncClient (ipAddress, port);

        const uint8_t deviceId = 0x01;
        float serial_number = client.queryFloat(PacketTypes::SERIAL_NUMBER, deviceId);
        float velocity = client.velocity(deviceId);
        float position = client.position(deviceId);
        float current = client.current(deviceId);


        ROS_INFO("Initializing Bravo Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing Bravo Hardware Interface");

        return true;
    }

    void BravoHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        //ROS_INFO_THROTTLE(1, "Read");
        ros::Duration elapsed_time = period;

        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            joint_positions_[i] = measured_joint_states_[i].angular_position_;
            joint_velocities_[i] = measured_joint_states_[i].angular_velocity_;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }

        ROS_INFO_STREAM_THROTTLE(1, "read()");

        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            ss << std::left << std::setw(width) << std::setfill(sep) << "Read" << std::left << std::setw(width) << std::setfill(sep) << "ticks" << std::left << std::setw(width) << std::setfill(sep) << "angle" << std::left << std::setw(width) << std::setfill(sep) << "velocity" << std::endl;
            ROS_INFO_STREAM(std::endl << ss.str());
            //printState();
        }
    }

    void BravoHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s

        ROS_INFO_STREAM_THROTTLE(1, "write()");


        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            // Header
            ss << std::left << std::setw(width) << std::setfill(sep) << "Write"
            << std::left << std::setw(width) << std::setfill(sep) << "velocity"
            << std::left << std::setw(width) << std::setfill(sep) << "percent"
            << std::endl;
            for (int i = 0; i < NUM_JOINTS; ++i)
            {

                // Joint i
                std::string j = "j" + std::to_string(i) + ":";
                ss << std::left << std::setw(width) << std::setfill(sep) << j
                << std::left << std::setw(width) << std::setfill(sep) << joint_velocity_commands_[i]
                << std::endl;
            }
            ROS_INFO_STREAM(std::endl << ss.str());
        }
    }

    bool BravoHWInterface::isReceivingMeasuredJointStates(const ros::Duration &timeout)
    {
        ROS_INFO("Get number of measured joint states publishers");

        int num_publishers = 0;
        return (num_publishers > 0);
    }

    void BravoHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void BravoHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string BravoHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string BravoHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_velocity_commands_[i] << "\t ";
        }
        return ss.str();
    }

    void BravoHWInterface::measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states)
    {
        /// Update current encoder ticks in encoders array
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            measured_joint_states_[i].angular_position_ = msg_joint_states->position[i];
            measured_joint_states_[i].angular_velocity_ = msg_joint_states->velocity[i];
        }
        //ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        //ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }

};
