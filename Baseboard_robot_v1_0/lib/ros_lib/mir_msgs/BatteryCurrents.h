#ifndef _ROS_mir_msgs_BatteryCurrents_h
#define _ROS_mir_msgs_BatteryCurrents_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class BatteryCurrents : public ros::Msg
  {
    public:
      typedef float _battery1_current_type;
      _battery1_current_type battery1_current;
      typedef float _battery2_current_type;
      _battery2_current_type battery2_current;

    BatteryCurrents():
      battery1_current(0),
      battery2_current(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->battery1_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery2_current);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery1_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery2_current));
     return offset;
    }

    const char * getType(){ return "mir_msgs/BatteryCurrents"; };
    const char * getMD5(){ return "99e76fe5e1c8183e9d7ded8c13ebdf16"; };

  };

}
#endif
