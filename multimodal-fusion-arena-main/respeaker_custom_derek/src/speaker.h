#ifndef SPEAKER_H
#define SPEAKER_H

#include <string>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int16MultiArray.h"


#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (1024)
#define NUM_SECONDS (1)
#define NUM_CHANNELS (6)
/* Select sample format. */
#if 0
#define PA_SAMPLE_TYPE paFloat32
typedef float SAMPLE;
#define SAMPLE_SILENCE (0.0f)
#define PRINTF_S_FORMAT "%.8f"
#elif 1
#define PA_SAMPLE_TYPE paInt16
typedef short SAMPLE;
#define SAMPLE_SILENCE (0)
#define PRINTF_S_FORMAT "%d"
#elif 0
#define PA_SAMPLE_TYPE paInt8
typedef char SAMPLE;
#define SAMPLE_SILENCE (0)
#define PRINTF_S_FORMAT "%d"
#else
#define PA_SAMPLE_TYPE paUInt8
typedef unsigned char SAMPLE;
#define SAMPLE_SILENCE (128)
#define PRINTF_S_FORMAT "%d"
#endif

const int NUM_SPEAKERS = 2;

class Speaker {
public:
  int id;
  std::string publisher_name;
  std::string file_name;
  ros::Publisher audio_pub;

  Speaker(ros::NodeHandle n, int id, std::string publisher_name, std::string file_name) {
    audio_pub =
        n.advertise<std_msgs::Int16MultiArray>("/audio/" + publisher_name, 50);
    this->id = id;
    this->publisher_name = publisher_name;
    this->file_name = file_name;
  }
};



#endif