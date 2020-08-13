#ifndef _ROS_behaviortree_ros_BehaviorTree_h
#define _ROS_behaviortree_ros_BehaviorTree_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "behaviortree_ros/TreeNode.h"

namespace behaviortree_ros
{

  class BehaviorTree : public ros::Msg
  {
    public:
      typedef uint16_t _root_uid_type;
      _root_uid_type root_uid;
      uint32_t nodes_length;
      typedef behaviortree_ros::TreeNode _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    BehaviorTree():
      root_uid(0),
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->root_uid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->root_uid >> (8 * 1)) & 0xFF;
      offset += sizeof(this->root_uid);
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->root_uid =  ((uint16_t) (*(inbuffer + offset)));
      this->root_uid |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->root_uid);
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (behaviortree_ros::TreeNode*)realloc(this->nodes, nodes_lengthT * sizeof(behaviortree_ros::TreeNode));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(behaviortree_ros::TreeNode));
      }
     return offset;
    }

    const char * getType(){ return "behaviortree_ros/BehaviorTree"; };
    const char * getMD5(){ return "e36ae6bd61313885da34913162caf9f3"; };

  };

}
#endif
