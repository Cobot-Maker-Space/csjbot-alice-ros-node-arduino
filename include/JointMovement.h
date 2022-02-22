#ifndef _ROS_lily_JointMovement_h
#define _ROS_lily_JointMovement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lily
{

  class JointMovement : public ros::Msg
  {
    public:
      bool neck;
      int16_t neck_to;
      int16_t neck_speed;
      bool left_arm;
      int16_t left_arm_to;
      int16_t left_arm_speed;
      bool right_arm;
      int16_t right_arm_to;
      int16_t right_arm_speed;

    JointMovement():
      neck(0),
      neck_to(0),
      neck_speed(0),
      left_arm(0),
      left_arm_to(0),
      left_arm_speed(0),
      right_arm(0),
      right_arm_to(0),
      right_arm_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_neck;
      u_neck.real = this->neck;
      *(outbuffer + offset + 0) = (u_neck.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->neck);
      union {
        int16_t real;
        uint16_t base;
      } u_neck_to;
      u_neck_to.real = this->neck_to;
      *(outbuffer + offset + 0) = (u_neck_to.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_neck_to.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->neck_to);
      union {
        int16_t real;
        uint16_t base;
      } u_neck_speed;
      u_neck_speed.real = this->neck_speed;
      *(outbuffer + offset + 0) = (u_neck_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_neck_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->neck_speed);
      union {
        bool real;
        uint8_t base;
      } u_left_arm;
      u_left_arm.real = this->left_arm;
      *(outbuffer + offset + 0) = (u_left_arm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm_to;
      u_left_arm_to.real = this->left_arm_to;
      *(outbuffer + offset + 0) = (u_left_arm_to.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_to.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_arm_to);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm_speed;
      u_left_arm_speed.real = this->left_arm_speed;
      *(outbuffer + offset + 0) = (u_left_arm_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_arm_speed);
      union {
        bool real;
        uint8_t base;
      } u_right_arm;
      u_right_arm.real = this->right_arm;
      *(outbuffer + offset + 0) = (u_right_arm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm_to;
      u_right_arm_to.real = this->right_arm_to;
      *(outbuffer + offset + 0) = (u_right_arm_to.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_to.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_arm_to);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm_speed;
      u_right_arm_speed.real = this->right_arm_speed;
      *(outbuffer + offset + 0) = (u_right_arm_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_arm_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_neck;
      u_neck.base = 0;
      u_neck.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->neck = u_neck.real;
      offset += sizeof(this->neck);
      union {
        int16_t real;
        uint16_t base;
      } u_neck_to;
      u_neck_to.base = 0;
      u_neck_to.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_neck_to.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->neck_to = u_neck_to.real;
      offset += sizeof(this->neck_to);
      union {
        int16_t real;
        uint16_t base;
      } u_neck_speed;
      u_neck_speed.base = 0;
      u_neck_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_neck_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->neck_speed = u_neck_speed.real;
      offset += sizeof(this->neck_speed);
      union {
        bool real;
        uint8_t base;
      } u_left_arm;
      u_left_arm.base = 0;
      u_left_arm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_arm = u_left_arm.real;
      offset += sizeof(this->left_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm_to;
      u_left_arm_to.base = 0;
      u_left_arm_to.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_to.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_arm_to = u_left_arm_to.real;
      offset += sizeof(this->left_arm_to);
      union {
        int16_t real;
        uint16_t base;
      } u_left_arm_speed;
      u_left_arm_speed.base = 0;
      u_left_arm_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_arm_speed = u_left_arm_speed.real;
      offset += sizeof(this->left_arm_speed);
      union {
        bool real;
        uint8_t base;
      } u_right_arm;
      u_right_arm.base = 0;
      u_right_arm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_arm = u_right_arm.real;
      offset += sizeof(this->right_arm);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm_to;
      u_right_arm_to.base = 0;
      u_right_arm_to.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_to.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_arm_to = u_right_arm_to.real;
      offset += sizeof(this->right_arm_to);
      union {
        int16_t real;
        uint16_t base;
      } u_right_arm_speed;
      u_right_arm_speed.base = 0;
      u_right_arm_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_arm_speed = u_right_arm_speed.real;
      offset += sizeof(this->right_arm_speed);
     return offset;
    }

    const char * getType(){ return "lily/JointMovement"; };
    const char * getMD5(){ return "f2a569bc893dff1897c77386bfc811d0"; };

  };

}
#endif
