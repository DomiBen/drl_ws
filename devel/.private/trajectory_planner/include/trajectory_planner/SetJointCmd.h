// Generated by gencpp from file trajectory_planner/SetJointCmd.msg
// DO NOT EDIT!


#ifndef TRAJECTORY_PLANNER_MESSAGE_SETJOINTCMD_H
#define TRAJECTORY_PLANNER_MESSAGE_SETJOINTCMD_H

#include <ros/service_traits.h>


#include <trajectory_planner/SetJointCmdRequest.h>
#include <trajectory_planner/SetJointCmdResponse.h>


namespace trajectory_planner
{

struct SetJointCmd
{

typedef SetJointCmdRequest Request;
typedef SetJointCmdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetJointCmd
} // namespace trajectory_planner


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::trajectory_planner::SetJointCmd > {
  static const char* value()
  {
    return "b5f68cbcaa5e7a8e8c34ee9959541272";
  }

  static const char* value(const ::trajectory_planner::SetJointCmd&) { return value(); }
};

template<>
struct DataType< ::trajectory_planner::SetJointCmd > {
  static const char* value()
  {
    return "trajectory_planner/SetJointCmd";
  }

  static const char* value(const ::trajectory_planner::SetJointCmd&) { return value(); }
};


// service_traits::MD5Sum< ::trajectory_planner::SetJointCmdRequest> should match
// service_traits::MD5Sum< ::trajectory_planner::SetJointCmd >
template<>
struct MD5Sum< ::trajectory_planner::SetJointCmdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::trajectory_planner::SetJointCmd >::value();
  }
  static const char* value(const ::trajectory_planner::SetJointCmdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::trajectory_planner::SetJointCmdRequest> should match
// service_traits::DataType< ::trajectory_planner::SetJointCmd >
template<>
struct DataType< ::trajectory_planner::SetJointCmdRequest>
{
  static const char* value()
  {
    return DataType< ::trajectory_planner::SetJointCmd >::value();
  }
  static const char* value(const ::trajectory_planner::SetJointCmdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::trajectory_planner::SetJointCmdResponse> should match
// service_traits::MD5Sum< ::trajectory_planner::SetJointCmd >
template<>
struct MD5Sum< ::trajectory_planner::SetJointCmdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::trajectory_planner::SetJointCmd >::value();
  }
  static const char* value(const ::trajectory_planner::SetJointCmdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::trajectory_planner::SetJointCmdResponse> should match
// service_traits::DataType< ::trajectory_planner::SetJointCmd >
template<>
struct DataType< ::trajectory_planner::SetJointCmdResponse>
{
  static const char* value()
  {
    return DataType< ::trajectory_planner::SetJointCmd >::value();
  }
  static const char* value(const ::trajectory_planner::SetJointCmdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TRAJECTORY_PLANNER_MESSAGE_SETJOINTCMD_H
