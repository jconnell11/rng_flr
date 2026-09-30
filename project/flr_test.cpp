// flr_test.cpp : test of pseudo-range inference from color images
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2026 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

#ifndef __linux__
  #include <windows.h>                 // needed for timeGetTime
  #include <stdio.h>
  #pragma comment(lib, "winmm.lib")    // for timeGetTime
#else
  #include <stdlib.h>                  // for atoi
#endif

#include "jhc_conio.h"

#include "vid_ocv.h"                   
#include "rng_flr.h"


//= Test of pseudo-range inference from color images.
// shows floor area image and grayscale version of depth
// no args = ESP32Cam wifi streaming with warp from Baijiu robot
// else argument is camera unit number (no warping applied)
// NOTE: needs rng_flr.dll, ocv_vid.dll, and opencv_world4100.dll!

int main (int argc, char *argv[])
{
  char ipname[40] = "http://192.168.0.200:81/stream";
  const unsigned char *vbuf;
  unsigned char *gbuf = NULL, *nbuf = NULL;
  double fps, flen = 219.0, ht = 5.5, tilt = 4.2;
  int rc, unit, drop, cnt = 0;

  // announce default camera pose
  printf("Assuming power-on camera pose:\n");
  printf("  ht = %3.1f\", tilt = %3.1f degs\n", ht, tilt);

  // connect to camera
  if (argc > 1)
  {
    unit = atoi(argv[1]);
    printf("Opening camera %d ... ", unit);
    rc = ocv_cam(unit, 1);
    flen = 554.0;                                // HFOV = 60 deg
  }
  else
  {
    printf("Opening %s ... ", ipname);
    rc = ocv_open(ipname, 1);
  }
  if (rc <= 0)
  {
    printf("\n  Failed to open video source!\n");
    return 0;
  }
  printf("\n");

  // set standard geometric correction and create two display windows
  if (argc < 2)
    ocv_warp(0.14, -0.13, 0.024, flen, 1, 0, 313, 242);   
  rng_init(flen, 640, 480);          
  ocv_win(0, "Floor", 0, 0);     
  ocv_win(1, "Depth", 650, 0);     

  // create buffers on heap for debugging images
  gbuf = new unsigned char [640 * 480 * 3];
  nbuf = new unsigned char [640 * 480 * 3];

  // continuously framegrab
  printf("Streaming video (hit any key to exit) ...\n");
  while (!_kbhit())
  {
    // get next video frame (blocks)
    if (ocv_get(&vbuf, 1) <= 0)      
    {
      printf("Video connection lost!\n");
      break;
    }

    // perform depth inference and wait for completion
    rng_est(vbuf, ht, tilt);
    if (rng_rdy(200) <= 0)
      continue;
    rng_d16(NULL, NULL);               // to reset flag
    rng_gnd(gbuf);
    rng_nite(nbuf);

    // show newest images
    ocv_queue(0, gbuf, 640, 480);
    ocv_queue(1, nbuf, 640, 480);    
    ocv_show(); 
    cnt++;
    printf("\r  %d ", cnt);
    fflush(stdout);
  }

  // report speed 
  drop = ocv_done();
  ocv_rate(fps);
  if (cnt > 0)
    printf("  %d frames in %3.1f secs = %3.1f fps (%d dropped)\n", cnt, cnt / fps, fps, drop);
  else
    printf("  0 frames in 0.0 secs = 0.0 fps\n");

  // clean up
  ocv_done();
  delete [] nbuf;
  delete [] gbuf;

  // keep terminal window visible
  while (_kbhit())
    rc = _getch();
  printf("Hit any key to exit ...\n");
  rc = _getch();
  _kbdone();
  return 1;
}

