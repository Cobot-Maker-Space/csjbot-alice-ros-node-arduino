#ifndef _ROS_csjbot_alice_JointPosition_h
#define _ROS_csjbot_alice_JointPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace csjbot_alice
{

  class JointPosition : public ros::Msg
  {
    public:
      int16_t neck;
      int16_t left_arm;
      int16_t right_arm;

    JointPosition():
      neck(0),
      left_arm(0),
      right_arm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_neck;
      u_neck.real = this->neck;
      *(outbuffer + offset + 0) = (u_neck.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_neck.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->neck);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm;
      u_left_arm.real = this->left_arm;
      *(outbuffer + offset + 0) = (u_left_arm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm;
      u_right_arm.real = this->right_arm;
      *(outbuffer + offset + 0) = (u_right_arm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_arm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_neck;
      u_neck.base = 0;
      u_neck.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_neck.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->neck = u_neck.real;
      offset += sizeof(this->neck);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm;
      u_left_arm.base = 0;
      u_left_arm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_arm = u_left_arm.real;
      offset += sizeof(this->left_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm;
      u_right_arm.base = 0;
      u_right_arm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_arm = u_right_arm.real;
      offset += sizeof(this->right_arm);
     return offset;
    }

    const char * getType(){ return "csjbot_alice/JointPosition"; };
    const char * getMD5(){ return "4ceab461e9b6a2387352aeeefe318e44"; };

  };

}
#endif