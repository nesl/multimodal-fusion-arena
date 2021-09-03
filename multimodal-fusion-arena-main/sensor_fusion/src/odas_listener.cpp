#include "audio_utils/AudioFrame.h"
#include "odas_ros/OdasSslArrayStamped.h"
#include "odas_ros/OdasSst.h"
#include "odas_ros/OdasSstArrayStamped.h"
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <sndfile.h>
#include <stdio.h>
#include <vector>

// custom nesl message stuff
#include "NESLMessages/NeslCoord.h"
#include "NESLMessages/Person.h"
#include "NESLMessages/PersonArr.h"

// message filter stuff
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;

vector<uint8_t> data;
SF_INFO sfinfo;

#define PI 3.14159265

#define THRESHOLD 0.05

#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (1024)
#define NUM_SECONDS (1)
#define NUM_CHANNELS (4)
#define NUM_RECORDING_SECONDS (10)

int written_flag = 0;

int totalFrames =
    NUM_RECORDING_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
int numSamples = totalFrames * NUM_CHANNELS;
int numBytes = numSamples * sizeof(short);
vector<odas_ros::OdasSst> odas_sources;

ros::Publisher pub;

double distance(double alpha, double beta){
  double phi = fmod(abs(beta - alpha), (2 * PI));
  double distance = phi > PI ? (2 * PI) - phi : phi;
  return distance;
}

void sssCallback(const audio_utils::AudioFrame::ConstPtr &msg) {
  // ROS_INFO("channel count: %d \n sampling frequency: %d \n",
  // msg->channel_count, msg->sampling_frequency);
  if (written_flag == 0) {
    // ROS_INFO("message size: %d", (int)msg->data.size());
    data.insert(data.end(), msg->data.begin(), msg->data.end());
    ROS_INFO("vector size: %d", (int)data.size());
  }
  if ((int)data.size() >= numBytes && written_flag == 0) {

    written_flag = 1;

    const short *file_data = (short *)&data[0];
    vector<short> channel1;
    vector<short> channel2;
    vector<short> channel3;
    vector<short> channel4;

    for (int i = 0; i < totalFrames; i++) {
      int four_times_i = i * 4;
      channel1.push_back(file_data[four_times_i]);
      channel2.push_back(file_data[four_times_i + 1]);
      channel3.push_back(file_data[four_times_i + 2]);
      channel4.push_back(file_data[four_times_i + 3]);

      // cout << file_data[i] << ", ";
    }
    const short *channel1data = &channel1[0];
    const short *channel2data = &channel2[0];
    const short *channel3data = &channel3[0];
    const short *channel4data = &channel4[0];

    ROS_INFO("writing data, data size: %d", (int)data.size());

    SNDFILE *outfile =
        sf_open("audio_files/odas_test_channel1.flac", SFM_WRITE, &sfinfo);
    long wr = sf_write_short(outfile, channel1data, totalFrames);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);

    outfile =
        sf_open("audio_files/odas_test_channel2.flac", SFM_WRITE, &sfinfo);
    wr = sf_write_short(outfile, channel2data, totalFrames);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);

    outfile =
        sf_open("audio_files/odas_test_channel3.flac", SFM_WRITE, &sfinfo);
    wr = sf_write_short(outfile, channel3data, totalFrames);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);

    outfile =
        sf_open("audio_files/odas_test_channel4.flac", SFM_WRITE, &sfinfo);
    wr = sf_write_short(outfile, channel4data, totalFrames);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);
  }
}

void sstCallback(const odas_ros::OdasSstArrayStamped::ConstPtr &msg) {
  odas_sources = msg->sources;
  /*
  for (int i = 0; i < odas_sources.size(); i++) {
    float angle = fmod(atan2(odas_sources[i].y, odas_sources[i].x) * 180 / PI + 360, 360);
    // float angle = atan2(sources[i].y, sources[i].x);
    ROS_INFO("id: %d, x:%f, y:%f, angle: %f degrees", odas_sources[i].id,
             odas_sources[i].x, odas_sources[i].y, angle);
  }
  ROS_INFO("----------");
  */
}

void sslCallback(const odas_ros::OdasSslArrayStamped::ConstPtr &msg) {
  vector<odas_ros::OdasSsl> sources = msg->sources;
  for (int i = 0; i < sources.size(); i++) {
    float angle = atan2(sources[i].y, sources[i].x);
    ROS_INFO("x:%f, y:%f, angle: %f", sources[i].x, sources[i].y, angle);
  }
  ROS_INFO("----------");
}

void personCallback(const NESLMessages::PersonArr::ConstPtr &person_msg) {
  vector<NESLMessages::Person> personArr = person_msg->personArr;
  if (personArr.size() == 0) {
    return;
  }

  for (int i = 0; i < odas_sources.size(); i++) {
    if (odas_sources[i].activity > THRESHOLD) {
      float sound_angle = atan2(odas_sources[i].y, odas_sources[i].x);
      float min_angle_dif = 2 * PI;
      int closest_person_index = 0;

      for (int j = 0; j < personArr.size(); j++) {
        float person_angle = atan2(personArr[j].personCoord.x, personArr[j].personCoord.z); // flip z and x since coordinate system in arena is different 
        //ROS_INFO("person angl: %f", person_angle * 180/PI);
        if (distance(person_angle, sound_angle) < min_angle_dif) {
          min_angle_dif = distance(person_angle, sound_angle);
          closest_person_index = j;
        }
      }
      personArr[closest_person_index].talking = true;
    }
  }
  NESLMessages::PersonArr to_publish;
  to_publish.personArr = personArr;
  pub.publish(to_publish);
}

int main(int argc, char **argv) {
  
  sfinfo.channels = 1;
  sfinfo.samplerate = SAMPLE_RATE;
  sfinfo.format = SF_FORMAT_FLAC | SF_FORMAT_PCM_16;
  
  //("numBytes: %d", numBytes);

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // ros::Subscriber sub1 = n.subscribe("/odas/sss", 1000, sssCallback);
  ros::Subscriber sub2 = n.subscribe("/odas/sst", 1, sstCallback);
  // ros::Subscriber sub3 = n.subscribe("/odas/ssl", 1000, sslCallback);
  /*
  message_filters::Subscriber<odas_ros::OdasSstArrayStamped> odas_sub(
      n, "/odas/sst", 100);
  message_filters::Subscriber<NESLMessages::PersonArr> person_sub(
      n, "/camera3/people", 100);
  message_filters::TimeSynchronizer<odas_ros::OdasSstArrayStamped,
                                    NESLMessages::PersonArr>
      sync(odas_sub, person_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  */
  ros::Subscriber person_sub = n.subscribe("/camera3/people", 1, personCallback);
  pub = n.advertise<NESLMessages::PersonArr>("/NESL/combined", 10);
  ROS_INFO("spinning");

  ros::spin();

  return 0;
}
