#ifndef _ROS_behaviortree_ros_NodeStatus_h
#define _ROS_behaviortree_ros_NodeStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace behaviortree_ros
{

  class NodeStatus : public ros::Msg
  {
    public:
      typedef int8_t _value_type;
      _value_type value;
      enum { IDLE =  0 };
      enum { RUNNING =  1 };
      enum { SUCCESS =  2 };
      enum { FAILURE =  3 };

    NodeStatus():
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return "behaviortree_ros/NodeStatus"; };
    const char * getMD5(){ return "15f84edc2fb0ebdb6285fd5a96f59259"; };

  };

}
#endif
