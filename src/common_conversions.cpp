#include <robofleet_client/common_conversions.hpp>

// time and duration primitives don't follow the usual pattern,
// so handle them explicitly
ros::Time FbtoRos(const fb::RosTime* fb)
{{
  return ros::Time(fb->sec(), fb->nsec());
}}

const fb::RosTime* RostoFb(flatbuffers::FlatBufferBuilder& fbb, const ros::Time& msg)
{{
  (void)fbb;
  const fb::RosTime* time = new fb::RosTime(msg.sec, msg.nsec);
  return time;
}}

ros::Duration FbtoRos(const fb::RosDuration* fb)
{{
  return ros::Duration(fb->sec(), fb->nsec());
}}

const fb::RosDuration* RostoFb(flatbuffers::FlatBufferBuilder& fbb, const ros::Duration& msg)
{{
  (void)fbb;
  const fb::RosDuration* duration = new fb::RosDuration(msg.sec, msg.nsec);
  return duration;
}}