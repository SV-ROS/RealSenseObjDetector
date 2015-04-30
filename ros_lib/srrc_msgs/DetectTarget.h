#ifndef _ROS_SERVICE_DetectTarget_h
#define _ROS_SERVICE_DetectTarget_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace srrc_msgs
{

static const char DETECTTARGET[] = "srrc_msgs/DetectTarget";

  class DetectTargetRequest : public ros::Msg
  {
    public:
      float match_threshold;
      float object_width;
      float object_height;
      float min_distance;
      float max_distance;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_match_threshold;
      u_match_threshold.real = this->match_threshold;
      *(outbuffer + offset + 0) = (u_match_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_match_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_match_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_match_threshold.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->match_threshold);
      union {
        float real;
        uint32_t base;
      } u_object_width;
      u_object_width.real = this->object_width;
      *(outbuffer + offset + 0) = (u_object_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_object_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_object_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_object_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_width);
      union {
        float real;
        uint32_t base;
      } u_object_height;
      u_object_height.real = this->object_height;
      *(outbuffer + offset + 0) = (u_object_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_object_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_object_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_object_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_height);
      union {
        float real;
        uint32_t base;
      } u_min_distance;
      u_min_distance.real = this->min_distance;
      *(outbuffer + offset + 0) = (u_min_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_distance);
      union {
        float real;
        uint32_t base;
      } u_max_distance;
      u_max_distance.real = this->max_distance;
      *(outbuffer + offset + 0) = (u_max_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_match_threshold;
      u_match_threshold.base = 0;
      u_match_threshold.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_match_threshold.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_match_threshold.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_match_threshold.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->match_threshold = u_match_threshold.real;
      offset += sizeof(this->match_threshold);
      union {
        float real;
        uint32_t base;
      } u_object_width;
      u_object_width.base = 0;
      u_object_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_object_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_object_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_object_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->object_width = u_object_width.real;
      offset += sizeof(this->object_width);
      union {
        float real;
        uint32_t base;
      } u_object_height;
      u_object_height.base = 0;
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->object_height = u_object_height.real;
      offset += sizeof(this->object_height);
      union {
        float real;
        uint32_t base;
      } u_min_distance;
      u_min_distance.base = 0;
      u_min_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_distance = u_min_distance.real;
      offset += sizeof(this->min_distance);
      union {
        float real;
        uint32_t base;
      } u_max_distance;
      u_max_distance.base = 0;
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_distance = u_max_distance.real;
      offset += sizeof(this->max_distance);
     return offset;
    }

    const char * getType(){ return DETECTTARGET; };
    const char * getMD5(){ return "53910fefb63d7a8de898b1330f1bf73b"; };

  };

  class DetectTargetResponse : public ros::Msg
  {
    public:
      sensor_msgs::RegionOfInterest roi;
      float confidence;
      float distance;
      float pan;
      float tilt;
      float fov_x;
      float fov_y;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->roi.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.real = this->pan;
      *(outbuffer + offset + 0) = (u_pan.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan);
      union {
        float real;
        uint32_t base;
      } u_tilt;
      u_tilt.real = this->tilt;
      *(outbuffer + offset + 0) = (u_tilt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tilt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tilt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tilt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tilt);
      union {
        float real;
        uint32_t base;
      } u_fov_x;
      u_fov_x.real = this->fov_x;
      *(outbuffer + offset + 0) = (u_fov_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fov_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fov_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fov_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fov_x);
      union {
        float real;
        uint32_t base;
      } u_fov_y;
      u_fov_y.real = this->fov_y;
      *(outbuffer + offset + 0) = (u_fov_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fov_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fov_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fov_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fov_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->roi.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.base = 0;
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan = u_pan.real;
      offset += sizeof(this->pan);
      union {
        float real;
        uint32_t base;
      } u_tilt;
      u_tilt.base = 0;
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tilt = u_tilt.real;
      offset += sizeof(this->tilt);
      union {
        float real;
        uint32_t base;
      } u_fov_x;
      u_fov_x.base = 0;
      u_fov_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fov_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fov_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fov_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fov_x = u_fov_x.real;
      offset += sizeof(this->fov_x);
      union {
        float real;
        uint32_t base;
      } u_fov_y;
      u_fov_y.base = 0;
      u_fov_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fov_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fov_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fov_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fov_y = u_fov_y.real;
      offset += sizeof(this->fov_y);
     return offset;
    }

    const char * getType(){ return DETECTTARGET; };
    const char * getMD5(){ return "d590c5f8254e33acbed1bce735d267f7"; };

  };

  class DetectTarget {
    public:
    typedef DetectTargetRequest Request;
    typedef DetectTargetResponse Response;
  };

}
#endif
