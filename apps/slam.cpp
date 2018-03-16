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

#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

// Note: Use OpenCV for GUI.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

// PIRVS header.
#include <pirvs_ironsides.h>

/// Declare a header to device interface.
std::shared_ptr<PIRVS::PerceptInDevice> gDevice = nullptr;

/**
 * Gracefully exit when CTRL-C is hit.
 */
void exit_handler(int s) {
  if (!gDevice) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

/**
 * This sample app streams raw images (using RAW_MODE mode) from the device
 * and tracks the pose of a device while building the map in real-time.
 */
int main(int argc, char **argv) {
  if (argc < 3) {
    printf(
        "Not enough input argument.\n"
        "Usage:\n"
        "  %s <calib JSON> <voc JSON>\n",
        argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);
  const std::string file_voc(argv[2]);

  // Log the poses.
  std::ofstream pose_output("/tmp/PerceptIn_pose_output.txt");

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial SLAM state.
  std::shared_ptr<PIRVS::SlamState> slam_state;
  if (!PIRVS::InitState(file_calib, &slam_state)) {
    printf("Failed to InitState.\n");
    return -1;
  }

  // Create an initial (empty) map. The map will be incrementally built as
  // we feed in new data to RunSlam().
  std::shared_ptr<PIRVS::Map> map;
  if (!PIRVS::InitMap(file_calib, file_voc, &map)) {
    printf("Failed to InitMap.\n");
    return -1;
  }

  // Prepare a drawer to visualize the tracked pose while SLAM runs.
  PIRVS::TrajectoryDrawer drawer;
  PIRVS::Mat img_draw;
  cv::namedWindow("Trajectory");
  // Create an window to show the raw image.
  cv::namedWindow("Left image");

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

  bool stereo_data_available = false;
  // Stream data from the device and update the SLAM state and the map.
  while (1) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }

    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (stereo_data) {
      stereo_data_available = true;
    }

    if (!stereo_data_available) {
      continue;
    }

    // Update the SLAM state and the map according to the data.
    if (!PIRVS::RunSlam(data, map, slam_state)) {
      printf("SLAM failed.\n");
      break;
    }
    // Get the tracking pose from the updated SLAM state and do all sorts of
    // cool stuff with it. Reminder, if the cool stuff takes too long, the
    // device will drop frame which will hurt SLAM (making it more likely to
    // fail). A good practice is to get the pose here, and do the cool stuff at
    // a different thread.
    // Sample code:
    // PIRVS::Affine3d global_T_device;
    // if (slam_state->GetPose(&global_T_device)) {
    //  // Cool stuff here.
    // }

    // Visualize the pose after updating the pose with an StereoData.
    if (stereo_data) {
      if (drawer.Draw(slam_state, &img_draw)) {
        cv::Mat cv_img(img_draw.height, img_draw.width, CV_8UC3);
        memcpy(cv_img.data, img_draw.data, cv_img.total() * cv_img.elemSize());
        cv::imshow("Trajectory", cv_img);

        // Log the poses.
        PIRVS::PoseInQuaternion pose_in_quaternion;
        if (slam_state->GetPoseInQuaternion(&pose_in_quaternion)) {
          pose_output << pose_in_quaternion.timestamp << " "
                      << pose_in_quaternion.tx << " "
                      << pose_in_quaternion.ty << " "
                      << pose_in_quaternion.tz << " "
                      << pose_in_quaternion.qx << " "
                      << pose_in_quaternion.qy << " "
                      << pose_in_quaternion.qz << " "
                      << pose_in_quaternion.qw << "\n";
        }
      }
      cv::Mat cv_img_left =
          cv::Mat(stereo_data->img_l.height, stereo_data->img_l.width, CV_8UC1);
      memcpy(cv_img_left.data, stereo_data->img_l.data,
             cv_img_left.total() * cv_img_left.elemSize());
      cv::imshow("Left image", cv_img_left);

      cv::Mat cv_img_right =
          cv::Mat(stereo_data->img_r.height, stereo_data->img_r.width, CV_8UC1);
      memcpy(cv_img_right.data, stereo_data->img_r.data,
             cv_img_right.total() * cv_img_right.elemSize());
      cv::imshow("Right image", cv_img_right);

      // Press ESC to stop.
      char key = cv::waitKey(1);
      if (key == 27) {
        printf("Stopped.\n");
        break;
      }
    }
  }

  // Save the final map to disk. (This may take a while.)
  // Note, save the map even if SLAM failed because the map may still be usable.
  // if (!PIRVS::SaveMap("/tmp/sparse_map.json", map)) {
  //   printf("Failed to save map to disk.\n");
  //   return -1;
  // }

  gDevice->StopDevice();
  cv::destroyAllWindows();

  return 0;
}
