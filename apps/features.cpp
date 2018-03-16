/**
 * Copyright 2017 PerceptIn
 *
 * This End-User License Agreement (EULA) is a legal agreement between you
 * (the purchaser) and PerceptIn regarding your use of
 * PerceptIn Robotics Vision System (PIRVS), including PIRVS SDK and
 * associated documentation (the "Software").
 *
 * IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THIS EULA, DO NOT INSTALL,
 * USE OR COPY THE SOFTWARE.
 */

#include <mutex>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

#include <opencv2/highgui.hpp>

#include <pirvs_ironsides.h>

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit.
 */
void exit_handler(int s){
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

/**
 * This sample app visualizes the 2d features + 3d points detected from a device.
 */

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Not enough input argument.\nUsage:\n%s [calib JSON] \n", argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial state for feature detection + matching + triangulation.
  std::shared_ptr<PIRVS::FeatureState> state;
  if (!PIRVS::InitFeatureState(file_calib, &state)){
    printf("Failed to InitFeatureState.\n");
    return -1;
  }

  // Create an interface to stream the PerceptIn V1 device.
  if (!PIRVS::CreatePerceptInDevice(
        &gDevice, PIRVS::PerceptInDevice::RAW_MODE, true) ||
      !gDevice) {
    printf("Failed to create device.\n");
    return -1;
  }
  // Start streaming from the device.
  if (!gDevice->StartDevice()) {
    printf("Failed to start device.\n");
    return -1;
  }

  // Image and window to draw 2d features.
  PIRVS::Mat img_2d;
  cv::namedWindow("Detected features");
  // Image and window to draw features with depth.
  PIRVS::Mat img_depth;
  cv::namedWindow("Sparse depth");


  // Stream data from the device and update the feature state.
  while (1) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }
    // RunFeature only accept StereoData.
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (!stereo_data) {
      continue;
    }

    // Update feature state according to the stereo data.
    // Note, the last input argument of RunFeature() is a flag to specify
    // whether or not to match + triangulate features between the two sensors.
    // Set get_3d = false, if 3d features are not required.
    const bool get_3d = true;
    PIRVS::RunFeature(stereo_data, state, get_3d);

    // Get the 2d features from both sensor in the stereo camera to do all sorts
    // of cool stuff.
    //
    // Sample code:
    //     std::vector<PIRVS::Point2d> features_l;
    //     std::vector<PIRVS::Point2d> features_r;
    //     if (state->Get2dFeatures(&features_l, &features_r)) {
    //       // Your cool stuff here.
    //     }
    // Get the 3d features from the stereo camera to do all sorts of cool stuff.
    // Note, 3d features are only available if get_3d is true.
    //     std::vector<PIRVS::StereoFeature> features;
    //     if (get_3d && state->GetStereoFeatures(&features)) {
    //       // Your cool stuff here.
    //     }

    // Visualize the 2d detected features on both sensors in the stereo camera.
    if (PIRVS::Draw2dFeatures(stereo_data, state, &img_2d)) {
      cv::Mat cv_img_2d(img_2d.height,img_2d.width, CV_8UC3);
      memcpy(cv_img_2d.data, img_2d.data,
             cv_img_2d.total() * cv_img_2d.elemSize());
      cv::imshow("Detected features", cv_img_2d);
    }
    // Visualize the 3d depth of the features.
    if (get_3d && PIRVS::DrawStereoFeatures(stereo_data, state, &img_depth)) {

      cv::Mat cv_img_depth(img_depth.height,img_depth.width, CV_8UC3);
      memcpy(cv_img_depth.data, img_depth.data,
             cv_img_depth.total() * cv_img_depth.elemSize());
      cv::imshow("Sparse depth", cv_img_depth);
    }

    // Press ESC to stop.
    char key = cv::waitKey(1);
    if (key == 27) {
      printf("Stopped.\n");
      break;
    }
  }

  gDevice->StopDevice();
  cv::destroyAllWindows();

  return 0;
}
