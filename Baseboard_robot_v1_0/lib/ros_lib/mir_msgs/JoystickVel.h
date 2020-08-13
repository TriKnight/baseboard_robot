#ifndef _ROS_mir_msgs_JoystickVel_h
#define _ROS_mir_msgs_JoystickVel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace mir_msgs
{

  class JoystickVel : public ros::Msg
  {
    public:
      typedef const char* _joystick_token_type;
      _joystick_token_type joystick_token;
      typedef geometry_msgs::Twist _speed_command_type;
      _speed_command_type speed_command;

    JoystickVel():
      joystick_token(""),
      speed_command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joystick_token = strlen(this->joystick_token);
      varToArr(outbuffer + offset, length_joystick_token);
      offset += 4;
      memcpy(outbuffer + offset, this->joystick_token, length_joystick_token);
      offset += length_joystick_token;
      offset += this->speed_command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joystick_token;
      arrToVar(length_joystick_token, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joystick_token; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joystick_token-1]=0;
      this->joystick_token = (char *)(inbuffer + offset-1);
      offset += length_joystick_token;
      offset += this->speed_command.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mir_msgs/JoystickVel"; };
    const char * getMD5(){ return "55dfb41e13ae3da5456e9941cea0cbc0"; };

  };

}
#endif
