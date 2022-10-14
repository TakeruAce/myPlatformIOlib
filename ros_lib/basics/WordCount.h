#ifndef _ROS_SERVICE_WordCount_h
#define _ROS_SERVICE_WordCount_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace basics
{

static const char WORDCOUNT[] = "basics/WordCount";

  class WordCountRequest : public ros::Msg
  {
    public:
      typedef const char* _words_type;
      _words_type words;

    WordCountRequest():
      words("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_words = strlen(this->words);
      varToArr(outbuffer + offset, length_words);
      offset += 4;
      memcpy(outbuffer + offset, this->words, length_words);
      offset += length_words;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_words;
      arrToVar(length_words, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_words; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_words-1]=0;
      this->words = (char *)(inbuffer + offset-1);
      offset += length_words;
     return offset;
    }

    const char * getType(){ return WORDCOUNT; };
    const char * getMD5(){ return "6f897d3845272d18053a750c1cfb862a"; };

  };

  class WordCountResponse : public ros::Msg
  {
    public:
      typedef const char* _header_type;
      _header_type header;
      typedef uint32_t _count_type;
      _count_type count;

    WordCountResponse():
      header(""),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_header = strlen(this->header);
      varToArr(outbuffer + offset, length_header);
      offset += 4;
      memcpy(outbuffer + offset, this->header, length_header);
      offset += length_header;
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_header;
      arrToVar(length_header, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_header; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_header-1]=0;
      this->header = (char *)(inbuffer + offset-1);
      offset += length_header;
      this->count =  ((uint32_t) (*(inbuffer + offset)));
      this->count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return WORDCOUNT; };
    const char * getMD5(){ return "160c28f81218ab049aa6bc70973de2c7"; };

  };

  class WordCount {
    public:
    typedef WordCountRequest Request;
    typedef WordCountResponse Response;
  };

}
#endif
