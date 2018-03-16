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

#include <stdlib.h>

extern int pirvs_shared_lib(int argc, char **argv);

/**
 * This app demonstrates how to create an app using a shared library
 * based on PerceptIn SDK library.
 */

int main(int argc, char **argv) {

  return pirvs_shared_lib(argc, argv);
}
