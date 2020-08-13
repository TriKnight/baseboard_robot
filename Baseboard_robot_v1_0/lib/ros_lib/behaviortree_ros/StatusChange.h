#ifndef _ROS_behaviortree_ros_StatusChange_h
#define _ROS_behaviortree_ros_StatusChange_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "behaviortree_ros/NodeStatus.h"
#include "ros/time.h"

namespace behaviortree_ros
{

  class StatusChange : public ros::Msg
  {
    public:
      typedef uint16_t _uid_type;
      _uid_type uid;
      typedef behaviortree_ros::NodeStatus _prev_status_type;
      _prev_status_type prev_status;
      typedef behaviortree_ros::NodeStatus _status_type;
      _status_type status;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    StatusChange():
      uid(0),
      prev_status(),
      status(),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->uid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uid >> (8 * 1)) & 0xFF;
      offset += sizeof(this->uid);
      offset += this->prev_status.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->uid =  ((uint16_t) (*(inbuffer + offset)));
      this->uid |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->uid);
      offset += this->prev_status.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
     return offset;
    }

    const char * getType(){ return "behaviortree_ros/StatusChange"; };
    const char * getMD5(){ return "e692396d837057ff6ef88785fdfeb748"; };

  };

}
#endif
