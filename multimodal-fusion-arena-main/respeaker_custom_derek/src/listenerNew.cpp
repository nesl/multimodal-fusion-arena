#include "ros/ros.h"
#include "speaker.h"
#include "std_msgs/Int16MultiArray.h"
#include <sndfile.h>
#include <stdio.h>
#include <vector>

#define NUM_RECORDING_SECONDS (10)

/** Set to 1 if you want to capture the recording to a file. */
#define WRITE_TO_FILE (1)

using namespace std;

SF_INFO sfinfo;

int totalFrames =
    NUM_RECORDING_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
int numSamples = totalFrames * NUM_CHANNELS;
int numBytes = numSamples * sizeof(SAMPLE);

vector<SAMPLE> data1;
vector<SAMPLE> data2;

void speakerCallback1(const std_msgs::Int16MultiArray::ConstPtr &msg) {

  data1.insert(data1.end(), msg->data.begin(), msg->data.end());
  ROS_INFO("vector size: %d", (int) data1.size());
  if (data1.size() == 960000) {
    ROS_INFO("writing data");
    SNDFILE *outfile = sf_open("audio_files/speaker1.flac", SFM_WRITE, &sfinfo);
    const SAMPLE *file_data = &data1[0];
    long wr = sf_write_short(outfile, file_data, numSamples);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);
  }
}

void speakerCallback2(const std_msgs::Int16MultiArray::ConstPtr &msg) {

  data2.insert(data2.end(), msg->data.begin(), msg->data.end());
  ROS_INFO("vector size: %d", (int) data2.size());
  if (data2.size() == 960000) {
    ROS_INFO("writing data");
    SNDFILE *outfile = sf_open("audio_files/speaker2.flac", SFM_WRITE, &sfinfo);
    const SAMPLE *file_data = &data2[0];
    long wr = sf_write_short(outfile, file_data, numSamples);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);
  }
}

int main(int argc, char **argv) {
  sfinfo.channels = NUM_CHANNELS;
  sfinfo.samplerate = SAMPLE_RATE;
  sfinfo.format = SF_FORMAT_FLAC | SF_FORMAT_PCM_16;

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/audio/speaker1", 1000, speakerCallback1);
  ros::Subscriber sub2 = n.subscribe("/audio/speaker2", 1000, speakerCallback2);

  ros::spin();

  return 0;
}