
#include "portaudio.h"
#include <sndfile.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>

#include "speaker.h"

// ros includes
#include "ros/ros.h"
#include <rosbag/bag.h>
//#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"

using namespace std;

// portaudio
int main(int argc, char **argv) {
  ros::init(argc, argv, "speakers");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  Speaker speakers[] = {
      Speaker(n, 24, "speaker1", "audio_files/multithread1.flac"),
      Speaker(n, 25, "speaker2", "audio_files/multithread2.flac")};
  PaStreamParameters inputParametersArr[NUM_SPEAKERS];
  PaStream *streamArr[NUM_SPEAKERS];
  PaError err = paNoError;

  int totalFrames = NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
  int numSamples = totalFrames * NUM_CHANNELS;
  int numBytes = numSamples * sizeof(SAMPLE);

  err = Pa_Initialize(); // initializes Port Audio
  if (err != paNoError) {
    ROS_INFO("Could not initialize the portaudio.\n");
    return 1;
  }

  int i;
  for (i = 0; i < NUM_SPEAKERS; i++) {
    PaStreamParameters inputParameters;
    inputParameters.device = speakers[i].id;
    if (inputParameters.device == paNoDevice) {
      fprintf(stderr, "Error: No default input device.\n");
    }
    inputParameters.channelCount = 6; /* stereo input */
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency =
        Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    inputParametersArr[i] =
        inputParameters; // stores input parameter into inputParametersArr

    //---------------------------------------------
    err = Pa_OpenStream(&streamArr[i], &inputParametersArr[i], NULL,
                        SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, NULL, NULL);
    ROS_INFO("one stream opened");
  }

  for (i = 0; i < NUM_SPEAKERS; i++) {
    err = Pa_StartStream(streamArr[i]);
  }

  printf("\n=== Now recording!! Please speak into the microphone. ===\n");
  fflush(stdout);

  int count = 0;
  SAMPLE *sampleBlockArr[2];
  for (i = 0; i < NUM_SPEAKERS; i++) {
    sampleBlockArr[i] = (SAMPLE *)malloc(numBytes);
  }

  ROS_INFO("INITIALIZED SAMPLEBLOCK WITH %d BYTES", numBytes);
  std_msgs::Int16MultiArray ary[NUM_SPEAKERS];

  while (ros::ok()) {
    ROS_INFO("READING STREAMS");
    thread th1(Pa_ReadStream, streamArr[0], sampleBlockArr[0], totalFrames);
    thread th2(Pa_ReadStream, streamArr[1], sampleBlockArr[1], totalFrames);
    th1.join();
    th2.join();
    ROS_INFO("FINISHED READING STREAMS");

    for (i = 0; i < NUM_SPEAKERS; i++) {
      ary[i].data.clear();
    }
    for (i = 0; i < NUM_SPEAKERS; i++) {
      for (int s = 0; s < numSamples; s++) {
        ary[i].data.push_back(sampleBlockArr[i][s]);
      }
    }

    for (i = 0; i < NUM_SPEAKERS; i++) {
      speakers[i].audio_pub.publish(ary[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
    //++count;
    // ROS_INFO("count: %d", count);
  }

  for (i = 0; i < NUM_SPEAKERS; i++) {
    err = Pa_StopStream(streamArr[i]);
    free(sampleBlockArr[i]);
  }

  Pa_Terminate();
  return 0;
}