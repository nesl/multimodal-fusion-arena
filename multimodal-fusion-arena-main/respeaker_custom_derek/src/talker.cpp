#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/ByteMultiArray.h>

#include "portaudio.h"
#include <sndfile.h>
#include <stdio.h>
#include <stdlib.h>

#include <sstream>
#include <string>
#include <thread>

using namespace std;
// libsndfile

const int MAX_SPEAKERS = 6;

#define READSIZE 1024
// SF_VIRTUAL_IO *virtualFile;
static unsigned int bytes_written = 0;
static unsigned int current_pos = 0;

// ros bag file

// static rosbag::Bag bag;
// std_msgs::Int16MultiArray ary;
// static std_msgs::ByteMultiArray ary;

static sf_count_t buffer_get_filelen(
    void *user_data) { // std::vector<char> myData = (std::vector<char> *)
  // unsigned char *charBuf = (unsigned char*) user_data ;
  const unsigned char *charBuffer = (unsigned char *)user_data;
  // Then we create the vector (named vectorBuffer) by copying the contents of
  // charBuffer to the vector std::vector<unsigned char>
  // vectorBuffer(charBuffer, charBuffer + bytes_written);
  return current_pos; // vectorBuffer.size(); //myData.size();
}

static sf_count_t buffer_seek(sf_count_t offset, int whence, void *user_data) {
  const unsigned char *charBuffer = (unsigned char *)user_data;
  static sf_count_t position;
  switch (whence) {
  case SEEK_SET:
    position = offset;
    // current_pos = position;
    break;

  case SEEK_CUR:
    // buff->seek(buff->pos()+offset);
    position = current_pos + offset;
    // current_pos = position;
    break;

  case SEEK_END:
    // buff->seek(buff->size()+offset);
    position = bytes_written + offset;
    // current_pos = position;
    break;
  default:
    break;
  };
  current_pos = position;
  return position;
}

static sf_count_t buffer_read(void *ptr, sf_count_t count, void *user_data) {

  // unsigned char *charBuffer = (unsigned char *) ptr; //needs to allocate
  // memory from the current position

  user_data = ((char *)user_data) + current_pos;
  // unsigned char *charBuffer = (unsigned char *) user_data;
  memcpy(ptr, &user_data, count);
  current_pos = current_pos + count;
  return count; // vectorBuffer.read((char*)ptr,count);
}

static sf_count_t buffer_write(const void *ptr, sf_count_t count,
                               void *user_data) {
  user_data = ((char *)user_data) + current_pos;
  memcpy(&user_data, ptr, count);
  current_pos = current_pos + count;
  return count;
}

static sf_count_t buffer_tell(void *user_data) {
  // QBuffer *buff = (QBuffer *) user_data ;
  return current_pos;
}

// linsndfile

// portaudio

/* #define SAMPLE_RATE  (17932) // Test failure to open with this value. */
#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (1024)
#define NUM_SECONDS (10)
#define NUM_CHANNELS (6)
/* #define DITHER_FLAG     (paDitherOff) */
#define DITHER_FLAG (0) /**/
/** Set to 1 if you want to capture the recording to a file. */
#define WRITE_TO_FILE (1)

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

typedef struct {
  int frameIndex; /* Index into sample array. */
  int maxFrameIndex;
  SAMPLE *recordedSamples;
} paTestData;

static int recordCallback(const void *inputBuffer, void *outputBuffer,
                          unsigned long framesPerBuffer,
                          const PaStreamCallbackTimeInfo *timeInfo,
                          PaStreamCallbackFlags statusFlags, void *userData) {
  paTestData *data = (paTestData *)userData;
  const SAMPLE *rptr = (const SAMPLE *)inputBuffer;
  SAMPLE *wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
  long framesToCalc;
  long i;
  int finished;
  unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

  (void)outputBuffer; /* Prevent unused variable warnings. */
  (void)timeInfo;
  (void)statusFlags;
  (void)userData;

  if (framesLeft < framesPerBuffer) {
    framesToCalc = framesLeft;
    finished = paComplete;
  } else {
    framesToCalc = framesPerBuffer;
    finished = paContinue;
  }

  if (inputBuffer == NULL) {
    for (i = 0; i < framesToCalc; i++) {
      for (int j = 0; j < NUM_CHANNELS; j++) {
        *wptr++ = SAMPLE_SILENCE;
      }
    }
  } else {
    for (i = 0; i < framesToCalc; i++) {
      for (int j = 0; j < NUM_CHANNELS; j++) {
        *wptr++ = *rptr++;
      }
    }
  }
  data->frameIndex += framesToCalc;
  return finished;
}

/* This routine will be called by the PortAudio engine when audio is needed.
** It may be called at interrupt level on some machines so don't do anything
** that could mess up the system like calling malloc() or free().
*/
static int playCallback(const void *inputBuffer, void *outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo *timeInfo,
                        PaStreamCallbackFlags statusFlags, void *userData) {
  paTestData *data = (paTestData *)userData;
  SAMPLE *rptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
  SAMPLE *wptr = (SAMPLE *)outputBuffer;
  unsigned int i;
  int finished;
  unsigned int framesLeft = data->maxFrameIndex - data->frameIndex;

  (void)inputBuffer; /* Prevent unused variable warnings. */
  (void)timeInfo;
  (void)statusFlags;
  (void)userData;

  if (framesLeft < framesPerBuffer) {
    /* final buffer... */
    for (i = 0; i < framesLeft; i++) {
      for (int j = 0; j < NUM_CHANNELS; j++){
        *wptr++ = *rptr++; 
      }
    }
    for (; i < framesPerBuffer; i++) {
      for (int j = 0; j < NUM_CHANNELS; j++){
        *wptr++ = 0;
      }
    }
    data->frameIndex += framesLeft;
    finished = paComplete;
  } else {
    for (i = 0; i < framesPerBuffer; i++) {
      for (int j = 0; j < NUM_CHANNELS; j++){
        *wptr++ = *rptr++; /* left */
      }
    }
    data->frameIndex += framesPerBuffer;
    finished = paContinue;
  }
  return finished;
}

class Speaker {
public:
  int id;
  string publisher_name;
  string file_name;
  ros::Publisher audio_pub;

  Speaker(ros::NodeHandle n, int id, string publisher_name, string file_name) {
    audio_pub = n.advertise<std_msgs::ByteMultiArray>(
        publisher_name, 50); //<my_custom_msg_package::MyCustomMsg>
    this->id = id;
    this->publisher_name = publisher_name;
    this->file_name = file_name;
  }
};

int publish(Speaker speakers[], int num_speakers) {
  // portaudio
  // PaStreamParameters inputParameters, outputParameters, inputParameters2;
  PaStreamParameters inputParametersArr[MAX_SPEAKERS];
  // PaStreamParameters outputParameters; // output parameters are never used?
  // PaStream *stream;
  // PaStream *stream2;
  PaStream *streamArr[MAX_SPEAKERS];
  PaError err = paNoError;
  // paTestData data, data2;
  paTestData dataArr[MAX_SPEAKERS];
  int i;
  int totalFrames;
  int numSamples;
  int numBytes;
  SAMPLE max, val;
  double average;

  // printf("patest_record.c\n"); fflush(stdout);
  // printf("sample size is %ld\n",sizeof(SAMPLE));

  // libsndfile
  // SF_VIRTUAL_IO buffer_virtualio ;

  // printf("Could not allocate record array.\n");
  ROS_INFO("initializing");
  err = Pa_Initialize(); // initializes Port Audio
  ROS_INFO("finished initializing");
  if (err != paNoError) {
    ROS_INFO("Could not initialize the portaudio.\n");
    return 1;
  }
  for (int i = 0; i < num_speakers; i++) {
    paTestData data;
    data.maxFrameIndex = totalFrames =
        NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
    data.frameIndex = 0;
    numSamples = totalFrames * NUM_CHANNELS;
    numBytes = numSamples * sizeof(SAMPLE);
    data.recordedSamples = (SAMPLE *)malloc(
        numBytes); /* From now on, recordedSamples is initialised. */
    if (data.recordedSamples == NULL) {
      printf("Could not allocate record array.\n");
    }
    for (int j = 0; j < numSamples; j++)
      data.recordedSamples[j] = 0;

    dataArr[i] = data; // stores data into dataArr

    //--------------------------------------------
    PaStreamParameters inputParameters;
    inputParameters.device = speakers[i].id;
    if (inputParameters.device == paNoDevice) {
      fprintf(stderr, "Error: No default input device.\n");
      // goto done;
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
                        SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff,
                        recordCallback, &dataArr[i]);
    ROS_INFO("one stream openned");
  }

  for (int i = 0; i < num_speakers; i++) {
    err = Pa_StartStream(streamArr[i]);
  }

  printf("\n=== Now recording!! Please speak into the microphone. ===\n");
  fflush(stdout);

  while ((err = Pa_IsStreamActive(streamArr[0])) == 1) {
    Pa_Sleep(1000);
    //ROS_INFO("index = %d\n", data.frameIndex);
    // fflush(stdout);
  }

  // get the recording
  // convert to flac
  current_pos = 0;

  SF_INFO sfinfo;
  sfinfo.channels = NUM_CHANNELS;
  sfinfo.samplerate = SAMPLE_RATE;
  sfinfo.format = SF_FORMAT_FLAC | SF_FORMAT_PCM_16;
  // printf("Could not allocate record array2.\n");
  for (int i = 0; i < num_speakers; i++) {
    //char *buf = (char *)malloc(numBytes);
    SNDFILE *outfile = sf_open(speakers[i].file_name.c_str(), SFM_WRITE,
                               &sfinfo); // open to file
    long wr = sf_write_short(outfile, dataArr[i].recordedSamples, numSamples);
    printf("Bytes written %ld\n", wr);
    sf_close(outfile);

    FILE *fp1;
    fp1 = fopen(speakers[i].file_name.c_str(), "r"); // opening an existing file
    if (fp1 == NULL) {
      // printf(("Could not open file " + speaker.file_name).c_str());
      printf("Could not open file");
      return 1;
    }
    std_msgs::ByteMultiArray ary;
    ary.data.clear();
    // std_msgs::UInt8MultiArray msg;
    for (int i = 0; i < (wr * 2); i++) {
      // val = data.recordedSamples[i];
      // do rosbag conversion
      ary.data.push_back(getc(fp1));
    }
    ROS_INFO("Sending message");
    speakers[i].audio_pub.publish(ary);
    Pa_Terminate();
    fclose(fp1);
//    free(buf);
  }
}

// portaudio
int main(int argc, char **argv) {
  ros::init(argc, argv, "speakers");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);
  Speaker speakers[] = {
      Speaker(n, 24, "speaker1", "audio_files/multithread1.flac"),
      Speaker(n, 25, "speaker2", "audio_files/multithread2.flac")};
  //printf("sample size is %ld\n", sizeof(SAMPLE));
  int count = 0;
  while (ros::ok()) {
    publish(speakers, 2);
    cout << "finished publishing" << endl;
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    cout << "count: " << count << endl;
  }

  /*

done:
  Pa_Terminate();
  if( data.recordedSamples )       // Sure it is NULL or valid.
      free( data.recordedSamples );
  if( err != paNoError )
  {
      fprintf( stderr, "An error occurred while using the portaudio stream\n" );
      fprintf( stderr, "Error number: %d\n", err );
      fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
      err = 1;          // Always return 0 or 1, but no other return codes.
  }
  return err;
*/
  return 0;
}
