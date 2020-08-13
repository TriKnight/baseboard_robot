#ifndef _ROS_behaviortree_ros_StatusChangeLog_h
#define _ROS_behaviortree_ros_StatusChangeLog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "behaviortree_ros/BehaviorTree.h"
#include "behaviortree_ros/StatusChange.h"

namespace behaviortree_ros
{

  class StatusChangeLog : public ros::Msg
  {
    public:
      typedef behaviortree_ros::BehaviorTree _behavior_tree_type;
      _behavior_tree_type behavior_tree;
      uint32_t state_changes_length;
      typedef behaviortree_ros::StatusChange _state_changes_type;
      _state_changes_type st_state_changes;
      _state_changes_type * state_changes;

    StatusChangeLog():
      behavior_tree(),
      state_changes_length(0), state_changes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->behavior_tree.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->state_changes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_changes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_changes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_changes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_changes_length);
      for( uint32_t i = 0; i < state_changes_length; i++){
      offset += this->state_changes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->behavior_tree.deserialize(inbuffer + offset);
      uint32_t state_changes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_changes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_changes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_changes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_changes_length);
      if(state_changes_lengthT > state_changes_length)
        this->state_changes = (behaviortree_ros::StatusChange*)realloc(this->state_changes, state_changes_lengthT * sizeof(behaviortree_ros::StatusChange));
      state_changes_length = state_changes_lengthT;
      for( uint32_t i = 0; i < state_changes_length; i++){
      offset += this->st_state_changes.deserialize(inbuffer + offset);
        memcpy( &(this->state_changes[i]), &(this->st_state_changes), sizeof(behaviortree_ros::StatusChange));
      }
     return offset;
    }

    const char * getType(){ return "behaviortree_ros/StatusChangeLog"; };
    const char * getMD5(){ return "39f6d1feed012a36f11ff73621704c13"; };

  };

}
#endif
