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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <pirvs_ironsides.h>

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit
 */
void exit_handler(int s){
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  exit(1);
}

/**
 * This file shows how to create a shared library based on PerceptIn SDK library.
 * See viewer_lib.cpp which links against this shared library for online viewer.
 */

int pirvs_shared_lib(int argc, char **argv) {

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create the interface to stream from a PerceptIn device.
  if (!PIRVS::CreatePerceptInDevice(&gDevice) || !gDevice) {
    printf("Failed to create device.\n");
    return -1;
  }

  // Start the GUI.
  // The recorded sequence will go to /tmp/PerceptIn_device_recordings/.
  gDevice->GUI("/tmp/PerceptIn_device_recordings/");
  return 0;
}
