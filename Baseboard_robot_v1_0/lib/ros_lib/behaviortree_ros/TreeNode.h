#ifndef _ROS_behaviortree_ros_TreeNode_h
#define _ROS_behaviortree_ros_TreeNode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "behaviortree_ros/NodeStatus.h"
#include "behaviortree_ros/NodeParameter.h"

namespace behaviortree_ros
{

  class TreeNode : public ros::Msg
  {
    public:
      typedef uint16_t _uid_type;
      _uid_type uid;
      uint32_t children_uid_length;
      typedef uint16_t _children_uid_type;
      _children_uid_type st_children_uid;
      _children_uid_type * children_uid;
      typedef int8_t _type_type;
      _type_type type;
      typedef behaviortree_ros::NodeStatus _status_type;
      _status_type status;
      typedef const char* _instance_name_type;
      _instance_name_type instance_name;
      typedef const char* _registration_name_type;
      _registration_name_type registration_name;
      typedef behaviortree_ros::NodeParameter _params_type;
      _params_type params;
      enum { UNDEFINED =  0 };
      enum { ACTION =  1 };
      enum { CONDITION =  2 };
      enum { CONTROL =  3 };
      enum { DECORATOR =  4 };
      enum { SUBTREE =  5 };

    TreeNode():
      uid(0),
      children_uid_length(0), children_uid(NULL),
      type(0),
      status(),
      instance_name(""),
      registration_name(""),
      params()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->uid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uid >> (8 * 1)) & 0xFF;
      offset += sizeof(this->uid);
      *(outbuffer + offset + 0) = (this->children_uid_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->children_uid_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->children_uid_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->children_uid_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->children_uid_length);
      for( uint32_t i = 0; i < children_uid_length; i++){
      *(outbuffer + offset + 0) = (this->children_uid[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->children_uid[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->children_uid[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->status.serialize(outbuffer + offset);
      uint32_t length_instance_name = strlen(this->instance_name);
      varToArr(outbuffer + offset, length_instance_name);
      offset += 4;
      memcpy(outbuffer + offset, this->instance_name, length_instance_name);
      offset += length_instance_name;
      uint32_t length_registration_name = strlen(this->registration_name);
      varToArr(outbuffer + offset, length_registration_name);
      offset += 4;
      memcpy(outbuffer + offset, this->registration_name, length_registration_name);
      offset += length_registration_name;
      offset += this->params.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->uid =  ((uint16_t) (*(inbuffer + offset)));
      this->uid |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->uid);
      uint32_t children_uid_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      children_uid_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      children_uid_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      children_uid_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->children_uid_length);
      if(children_uid_lengthT > children_uid_length)
        this->children_uid = (uint16_t*)realloc(this->children_uid, children_uid_lengthT * sizeof(uint16_t));
      children_uid_length = children_uid_lengthT;
      for( uint32_t i = 0; i < children_uid_length; i++){
      this->st_children_uid =  ((uint16_t) (*(inbuffer + offset)));
      this->st_children_uid |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_children_uid);
        memcpy( &(this->children_uid[i]), &(this->st_children_uid), sizeof(uint16_t));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->status.deserialize(inbuffer + offset);
      uint32_t length_instance_name;
      arrToVar(length_instance_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_instance_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_instance_name-1]=0;
      this->instance_name = (char *)(inbuffer + offset-1);
      offset += length_instance_name;
      uint32_t length_registration_name;
      arrToVar(length_registration_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_registration_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_registration_name-1]=0;
      this->registration_name = (char *)(inbuffer + offset-1);
      offset += length_registration_name;
      offset += this->params.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "behaviortree_ros/TreeNode"; };
    const char * getMD5(){ return "52b35b9d5a9a6ccf21ff774a2f69e4cc"; };

  };

}
#endif
