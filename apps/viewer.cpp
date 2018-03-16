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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pirvs_ironsides.h>

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit.
 */
void exit_handler(int s) {
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  exit(1);
}

/**
 * This sample app provides a GUI to show the live stream of a device.
 *
 * When running in DEPTH_MODE mode, A trackbar will show up on the top to
 * adjust the color code of the depth image.
 *
 * The GUI is also an interface to record from the device. Press space bar to
 * start recording a sequence, and then, press space bar again to stop
 * recording. The recorded sequence will be stored under the specified folder,
 * and the name of the sequence is the timestamp where the sequence begins.
 */

int main(int argc, char **argv) {
  // Install SIGNAL handler.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Parse input arguments.
  if (argc < 2) {
    printf(
        "Not enough input argument.\n"
        "Usage:\n  "
        "  %s <mode>\n"
        "Modes:\n"
        "  raw_mode   -- stream raw images and IMU readings\n"
        "  depth_mode -- stream depth and rectified images and IMU readings\n",
        argv[0]);
    return -1;
  }
  const std::string mode(argv[1]);

  // Create the device according to the mode.
  if (mode == "raw_mode") {
    if (!PIRVS::CreatePerceptInDevice(
            &gDevice, PIRVS::PerceptInDevice::RAW_MODE, false) ||
        !gDevice) {
      printf("Failed to create device.\n");
      return -1;
    }
  } else if (mode == "depth_mode") {
    if (!PIRVS::CreatePerceptInDevice(&gDevice,
                                      PIRVS::PerceptInDevice::DEPTH_MODE) ||
        !gDevice) {
      printf("Failed to create device.\n");
      return -1;
    }
  } else {
    printf("[%s] is not a valid mode.\n", mode.c_str());
    return -1;
  }

  // Start the GUI.
  // The recorded sequence will go to /tmp/PerceptIn_device_recordings/.
  gDevice->GUI("/tmp/PerceptIn_device_recordings/");

  // Alternatively, you can interact with the device without using the GUI.
  //
  // Here's a sample code for streaming data from the PerceptInDevice.
  //    gDevice->StartDevice();
  //    std::shared_ptr<const PIRVS::Data> data;
  //    while (1) {
  //      gDevice->GetData(&data);
  //
  //      // In RAW_MODE mode, the device outputs PIRVS::ImuData and
  //      // PIRVS::StereoData. In DEPTH_MODE mode, the device outputs
  //      // PIRVS::ImuData and PIRVS::StereoDepthData.
  //
  //      std::shared_ptr<const PIRVS::ImuData> imu_data =
  //          std::dynamic_pointer_cast<const PIRVS::ImuData>(data);
  //      if (imu_data) {
  //        printf("IMU reading: %zd (%f, %f, %f) (%f, %f, %f)\n",
  //               imu_data->timestamp,
  //               imu_data->accel.x, imu_data->accel.y, imu_data->accel.z,
  //               imu_data->ang_v.x, imu_data->ang_v.y, imu_data->ang_v.z);
  //        continue;
  //      }
  //
  //      std::shared_ptr<const PIRVS::StereoData> stereo_data =
  //          std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
  //      if (stereo_data) {
  //        int type = stereo_data->img_l.channels == 1 ? CV_8UC1 : CV_8UC3;
  //        cv::Mat cv_img_left(
  //            stereo_data->img_l.height, stereo_data->img_l.width, type);
  //        memcpy(cv_img_left.data, stereo_data->img_l.data,
  //               cv_img_left.total() * cv_img_left.elemSize());
  //        cv::imshow("Left image", cv_img_left);
  //
  //        cv::Mat cv_img_right(
  //            stereo_data->img_r.height, stereo_data->img_r.width, type);
  //        memcpy(cv_img_right.data, stereo_data->img_r.data,
  //               cv_img_right.total() * cv_img_right.elemSize());
  //        cv::imshow("Right image", cv_img_right);
  //
  //        cv:waitKey(1);
  //        continue;
  //      }
  //
  //      std::shared_ptr<const PIRVS::StereoDepthData> depth_stereo_data =
  //          std::dynamic_pointer_cast<const PIRVS::StereoDepthData>(data);
  //      cv::Mat_<uchar> cv_img_left(
  //          depth_stereo_data->img_l.height, depth_stereo_data->img_l.width);
  //      memcpy(cv_img_left.data, depth_stereo_data->img_l.data,
  //             cv_img_left.total() * cv_img_left.elemSize());
  //      cv::imshow("Left (rectified) image", cv_img_left);
  //
  //      cv::Mat_<uchar> cv_img_right(
  //          depth_stereo_data->img_r.height, depth_stereo_data->img_r.width);
  //      memcpy(cv_img_right.data, depth_stereo_data->img_r.data,
  //             cv_img_right.total() * cv_img_right.elemSize());
  //      cv::imshow("Right (rectified) image", cv_img_right);
  //
  //      cv::Mat_<uint16_t> cv_img_depth(
  //          depth_stereo_data->img_depth.height,
  //          depth_stereo_data->img_depth.width);
  //      memcpy(cv_img_depth.data, depth_stereo_data->img_depth.data,
  //             cv_img_depth.total() * cv_img_depth.elemSize());
  //      cv::imshow("Depth image", cv_img_depth);
  //
  //      cv:waitKey(1);
  //      continue;
  //    }
  //    gDevice->StopDevice();
  //    cv::destroyAllWindows();
  //
  //
  // You can also record data directly from the PerceptInDevice.
  // Example code:
  //    gDevice->StartDevice();
  //    gDevice->StartRecording("/tmp/PerceptIn_V1_device_recordings/");
  //    usleep(3000000);  // Record data for 3 sec.
  //    std::string dir;
  //    size_t num_imu = 0;
  //    size_t num_stereo = 0;
  //    gDevice->StopRecording(&dir, &num_imu, &num_stereo);
  //    printf("Folder: %s\nIMU data: %zd\nStereo data: %zd\n",
  //           dir.c_str(), num_imu, num_stereo);
  //    gDevice->StopDevice();

  return 0;
}
