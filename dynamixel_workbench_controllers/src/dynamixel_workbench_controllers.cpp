/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"
#include <signal.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Global flag for motor shutdown behavior
bool g_shutdown_motors_on_exit = false;
// Global controller instance for signal handler access
DynamixelController* g_controller_instance = nullptr;

DynamixelController::DynamixelController()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_joint_state_topic_(false),
   is_cmd_vel_topic_(false),
   use_moveit_(false),
   wheel_separation_(0.0f),
   wheel_radius_(0.0f),
   is_moving_(false),
   group_sync_read_(nullptr),
   sync_read_start_address_(0),
   sync_read_data_length_(0)
{
  is_joint_state_topic_ = priv_node_handle_.param<bool>("use_joint_states_topic", true);
  is_cmd_vel_topic_ = priv_node_handle_.param<bool>("use_cmd_vel_topic", false);
  use_moveit_ = priv_node_handle_.param<bool>("use_moveit", false);

  read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
  write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
  pub_period_ = priv_node_handle_.param<double>("publish_period", 0.010f);

  if (is_cmd_vel_topic_ == true)
  {
    wheel_separation_ = priv_node_handle_.param<double>("mobile_robot_config/seperation_between_wheels", 0.0);
    wheel_radius_ = priv_node_handle_.param<double>("mobile_robot_config/radius_of_wheel", 0.0);
  }

  dxl_wb_ = new DynamixelWorkbench;
  jnt_tra_ = new JointTrajectory;
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
}

DynamixelController::~DynamixelController()
{
  if (group_sync_read_ != nullptr)
  {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }
}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (!dynamixel.IsDefined() || dynamixel.IsNull())
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool DynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool DynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

bool DynamixelController::initOptimizedSyncRead(void)
{
  bool result = false;
  const char* log = NULL;

  // Only initialize for Protocol 2.0
  if (dxl_wb_->getProtocolVersion() != 2.0f)
  {
    ROS_WARN("Optimized GroupSyncRead is only available for Protocol 2.0");
    return true; // Not an error, just not applicable
  }

  // Calculate the start address and length for reading position, velocity, and current
  sync_read_start_address_ = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);
  
  // Calculate total length including potential gaps between registers
  uint16_t pos_addr = control_items_["Present_Position"]->address;
  uint16_t vel_addr = control_items_["Present_Velocity"]->address;
  uint16_t cur_addr = control_items_["Present_Current"]->address;
  
  uint16_t max_addr = std::max({pos_addr, vel_addr, cur_addr});
  uint16_t max_length = 0;
  
  if (max_addr == pos_addr) max_length = control_items_["Present_Position"]->data_length;
  else if (max_addr == vel_addr) max_length = control_items_["Present_Velocity"]->data_length;
  else max_length = control_items_["Present_Current"]->data_length;
  
  sync_read_data_length_ = (max_addr - sync_read_start_address_) + max_length;

  // Clean up any existing GroupSyncRead
  if (group_sync_read_ != nullptr)
  {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }

  // Create new optimized GroupSyncRead instance
  group_sync_read_ = new dynamixel::GroupSyncRead(dxl_wb_->getPortHandler(),
                                                  dxl_wb_->getPacketHandler(),
                                                  sync_read_start_address_,
                                                  sync_read_data_length_);

  if (group_sync_read_ == nullptr)
  {
    ROS_ERROR("Failed to create optimized GroupSyncRead instance");
    return false;
  }

  // Add all dynamixel IDs to the sync read group
  for (auto const& dxl : dynamixel_)
  {
    result = group_sync_read_->addParam((uint8_t)dxl.second);
    if (result == false)
    {
      ROS_ERROR("Failed to add Dynamixel ID %d to optimized sync read group", dxl.second);
      return false;
    }
  }

  ROS_INFO("Optimized GroupSyncRead initialized successfully for %zu servos", dynamixel_.size());
  ROS_INFO("Start address: %d, Data length: %d", sync_read_start_address_, sync_read_data_length_);
  
  return true;
}

bool DynamixelController::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;

  int32_t get_position[dxl_name.size()];

  uint8_t id_array[dxl_name.size()];
  uint8_t id_cnt = 0;

  for (auto const& name:dxl_name)
  {
    id_array[id_cnt++] = dynamixel_[name];
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               id_array,
                               dxl_name.size(),
                               &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    WayPoint wp;

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                      id_array,
                                      id_cnt,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length,
                                      get_position,
                                      &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      for(uint8_t index = 0; index < id_cnt; index++)
      {
        wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
        pre_goal_.push_back(wp);
      }
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     control_items_["Present_Position"]->data_length,
                                     &read_position,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      pre_goal_.push_back(wp);
    }
  }

  return result;
}

void DynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  if (is_joint_state_topic_) joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber()
{
  trajectory_sub_ = priv_node_handle_.subscribe("joint_trajectory", 100, &DynamixelController::trajectoryMsgCallback, this);
  if (is_cmd_vel_topic_) cmd_vel_sub_ = priv_node_handle_.subscribe("cmd_vel", 10, &DynamixelController::commandVelocityCallback, this);
}

void DynamixelController::initServer()
{
  dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::readCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_read_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  // Optimize: Use static/cached arrays to avoid repeated allocation
  static dynamixel_workbench_msgs::DynamixelState dynamixel_state[20]; // Max reasonable servo count
  static bool arrays_initialized = false;
  static uint8_t id_array[20];
  static uint8_t servo_count = 0;
  
  // Only clear and rebuild arrays if servo configuration changed
  if (!arrays_initialized || servo_count != dynamixel_.size()) 
  {
    servo_count = dynamixel_.size();
    uint8_t id_cnt = 0;
    for (auto const& dxl : dynamixel_)
    {
      dynamixel_state[id_cnt].name = dxl.first;
      dynamixel_state[id_cnt].id = (uint8_t)dxl.second;
      id_array[id_cnt] = (uint8_t)dxl.second;
      id_cnt++;
    }
    arrays_initialized = true;
  }
  
  // Fast clear of message list
  dynamixel_state_list_.dynamixel_state.clear();
  dynamixel_state_list_.dynamixel_state.reserve(servo_count);
#ifndef DEBUG
  // CRITICAL: Always read for emergency stop functionality
  // Removed is_moving_ check to enable emergency stop during trajectory execution
  {
#endif
    if (dxl_wb_->getProtocolVersion() == 2.0f && group_sync_read_ != nullptr)
    {
      // Use optimized direct GroupSyncRead - single communication cycle
      int comm_result = group_sync_read_->txRxPacket();
      if (comm_result != COMM_SUCCESS)
      {
        ROS_ERROR_THROTTLE(1.0, "GroupSyncRead txRxPacket failed: %s", dxl_wb_->getPacketHandler()->getTxRxResult(comm_result));
        // Fall back to original method on communication failure
        goto fallback_sync_read;
      }

      // Fast data extraction - single availability check for the entire data block
      uint8_t index = 0;
      for (auto const& dxl : dynamixel_)
      {
        uint8_t servo_id = (uint8_t)dxl.second;
        
        // Ultra-fast: Skip availability check for maximum speed (assumes reliable communication)
        // Use this ONLY if communication is stable, otherwise uncomment the check below
        // if (group_sync_read_->isAvailable(servo_id, sync_read_start_address_, sync_read_data_length_))
        if (true)  // FAST PATH: Assume data is always available
        {
          // Extract all data directly using calculated offsets - much faster
          dynamixel_state[index].present_position = group_sync_read_->getData(servo_id,
                                                                              control_items_["Present_Position"]->address,
                                                                              control_items_["Present_Position"]->data_length);
                                                                              
          dynamixel_state[index].present_velocity = group_sync_read_->getData(servo_id,
                                                                              control_items_["Present_Velocity"]->address,
                                                                              control_items_["Present_Velocity"]->data_length);
                                                                              
          dynamixel_state[index].present_current = group_sync_read_->getData(servo_id,
                                                                             control_items_["Present_Current"]->address,
                                                                             control_items_["Present_Current"]->data_length);
        }
        else
        {
          // Use previous values to avoid gaps
          if (index < dynamixel_state_list_.dynamixel_state.size())
          {
            dynamixel_state[index].present_position = dynamixel_state_list_.dynamixel_state[index].present_position;
            dynamixel_state[index].present_velocity = dynamixel_state_list_.dynamixel_state[index].present_velocity;
            dynamixel_state[index].present_current = dynamixel_state_list_.dynamixel_state[index].present_current;
          }
          else
          {
            dynamixel_state[index].present_position = 0;
            dynamixel_state[index].present_velocity = 0;
            dynamixel_state[index].present_current = 0;
          }
        }
        
        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
        index++;
      }
    }
    else if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      fallback_sync_read:
      // Fallback to original method if optimized version is not available
      static int32_t get_current[20];
      static int32_t get_velocity[20]; 
      static int32_t get_position[20];
      
      result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                  id_array,
                                  servo_count,
                                  &log);
      if (result == false)
      {
        ROS_ERROR_THROTTLE(1.0, "%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    servo_count,
                                                    control_items_["Present_Current"]->address,
                                                    control_items_["Present_Current"]->data_length,
                                                    get_current,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    servo_count,
                                                    control_items_["Present_Velocity"]->address,
                                                    control_items_["Present_Velocity"]->data_length,
                                                    get_velocity,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR_THROTTLE(1.0, "%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    servo_count,
                                                    control_items_["Present_Position"]->address,
                                                    control_items_["Present_Position"]->data_length,
                                                    get_position,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR_THROTTLE(1.0, "%s", log);
      }

      for(uint8_t index = 0; index < servo_count; index++)
      {
        dynamixel_state[index].present_current = get_current[index];
        dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
      }
    }
    else if(dxl_wb_->getProtocolVersion() == 1.0f)
    {
      uint16_t length_of_data = control_items_["Present_Position"]->data_length + 
                                control_items_["Present_Velocity"]->data_length + 
                                control_items_["Present_Current"]->data_length;
      uint32_t get_all_data[length_of_data];
      uint8_t dxl_cnt = 0;
      for (auto const& dxl:dynamixel_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                       control_items_["Present_Position"]->address,
                                       length_of_data,
                                       get_all_data,
                                       &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }

        dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
        dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
        dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
        dxl_cnt++;
      }
    }
#ifndef DEBUG
  }
#endif

#ifdef DEBUG
  ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
  priv_read_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::publishCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);

  if (is_joint_state_topic_)
  {
    joint_state_msg_.header.stamp = ros::Time::now();

    joint_state_msg_.name.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;

      joint_state_msg_.name.push_back(dxl.first);

      if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        if (strcmp(dxl_wb_->getModelName((uint8_t)dxl.second), "XL-320") == 0) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        else  effort = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      }
      else if (dxl_wb_->getProtocolVersion() == 1.0f) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);

      velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
      position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_.publish(joint_state_msg_);
  }

#ifdef DEBUG
  ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  double robot_lin_vel = msg->linear.x;
  double robot_ang_vel = msg->angular.z;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

//  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
//       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

  double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
    {
      if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
      else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

      if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
      else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
    }
    else
    {
      dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;
      dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
    else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
    else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

    if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
    else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
    else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
  }

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void DynamixelController::writeCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_position[dynamixel_.size()];

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  for (auto const& joint:jnt_tra_msg_->joint_names)
  {
    id_array[id_cnt] = (uint8_t)dynamixel_[joint];
    id_cnt++;
  }

  if (is_moving_ == true)
  {
    for (uint8_t index = 0; index < id_cnt; index++)
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], jnt_tra_msg_->points[point_cnt].positions.at(index));

    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    position_cnt++;
    if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    {
      point_cnt++;
      position_cnt = 0;
      if (point_cnt >= jnt_tra_msg_->points.size())
      {
        is_moving_ = false;
        point_cnt = 0;
        position_cnt = 0;

        ROS_INFO("Complete Execution");
      }
    }
  }

#ifdef DEBUG
  ROS_WARN("[writeCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;

  if (is_moving_ == false)
  {
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);
    if (result == false)
      ROS_ERROR("Failed to get Present Position");

    for (auto const& joint:msg->joint_names)
    {
      ROS_INFO("'%s' is ready to move", joint.c_str());

      jnt_tra_msg_->joint_names.push_back(joint);
      id_cnt++;
    }

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }

        if (use_moveit_ == true)
        {
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(goal[id_num].position);
            jnt_tra_point_msg.velocities.push_back(goal[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(goal[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);

          cnt++;
        }
        else
        {
          jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

          double move_time = 0.0f;
          if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
          else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();

          jnt_tra_->init(move_time,
                         write_period_,
                         pre_goal_,
                         goal);

          std::vector<WayPoint> way_point;
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (double index = 0.0; index < move_time; index = index + write_period_)
          {
            way_point = jnt_tra_->getJointWayPoint(index);

            for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
            {
              jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
              jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
              jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
            }

            jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
            jnt_tra_point_msg.positions.clear();
            jnt_tra_point_msg.velocities.clear();
            jnt_tra_point_msg.accelerations.clear();
          }

          pre_goal_ = goal;
          cnt++;
        }
      }
      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Dynamixel is moving");
  }
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}


// Signal handler function for motor safety shutdown
void motorSafetySignalHandler(int signum)
{
  ROS_INFO("Signal %d received, calling motor shutdown callback", signum);
  
  // Check if motor shutdown is enabled
  if (!g_shutdown_motors_on_exit)
  {
    ROS_INFO("Motor shutdown on exit is disabled - skipping motor shutdown");
    ros::shutdown();
    exit(signum);
  }
  
  // Immediately disable motors using fresh connection before port issues
  try 
  {
    if (g_controller_instance != nullptr)
    {
      PortHandler* portHandler = PortHandler::getPortHandler("/dev/ttyUSB0");
      PacketHandler* packetHandler = PacketHandler::getPacketHandler(2.0);
      
      if (portHandler->openPort() && portHandler->setBaudRate(4000000))
      {
        // Use the dynamixel_ map to get all initialized motors
        for (auto const& dxl : g_controller_instance->getDynamixelMap())
        {
          uint32_t motor_id = dxl.second;
          int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, 64, 0);
          if (dxl_comm_result == COMM_SUCCESS)
          {
            ROS_INFO("MOTOR SAFETY: Emergency torque disabled for motor %s (ID: %d)", dxl.first.c_str(), motor_id);
          }
          else
          {
            ROS_WARN("MOTOR SAFETY: Failed to disable motor %s (ID: %d)", dxl.first.c_str(), motor_id);
          }
        }
        portHandler->closePort();
        ROS_INFO("MOTOR SAFETY: Motor shutdown complete");
      }
      else
      {
        ROS_ERROR("MOTOR SAFETY: Failed to open port for emergency motor shutdown");
      }
      
      delete portHandler;
      delete packetHandler;
    }
    else
    {
      ROS_WARN("MOTOR SAFETY: No controller instance available for motor shutdown");
    }
  }
  catch (...)
  {
    ROS_ERROR("MOTOR SAFETY: Exception during emergency motor shutdown");
  }
  
  // Standard shutdown
  ros::shutdown();
  exit(signum);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_workbench_controllers");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 57600;

  if (argc < 2)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  // Read ROS parameter for motor shutdown behavior
  ros::NodeHandle private_nh("~");
  g_shutdown_motors_on_exit = private_nh.param<bool>("shutdown_motors_on_exit", true);
  ROS_INFO("Motor shutdown on exit: %s", g_shutdown_motors_on_exit ? "ENABLED" : "DISABLED");

  DynamixelController dynamixel_controller;
  
  // Set global controller instance for signal handler
  g_controller_instance = &dynamixel_controller;

  // Register signal handlers with motor shutdown callback
  struct sigaction sa;
  sa.sa_handler = motorSafetySignalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  ROS_INFO("Registered motor safety shutdown handlers");

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = dynamixel_controller.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dynamixel_controller.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dynamixel_controller.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dynamixel_controller.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  result = dynamixel_controller.initOptimizedSyncRead();
  if (result == false)
  {
    ROS_ERROR("Failed to initialize optimized sync read");
    return 0;
  }

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelController::readCallback, &dynamixel_controller);
  ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()), &DynamixelController::writeCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixel_controller);

  ros::spin();

  // Clear global controller pointer before exit
  g_controller_instance = nullptr;

  return 0;
}
