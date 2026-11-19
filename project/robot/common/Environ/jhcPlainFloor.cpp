// jhcPlainFloor.cpp : makes synthetic depth image for an untextured floor
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

#include <math.h>

#include "Interface/jhcMessage.h"      // common video

#include "Environ/jhcPlainFloor.h"


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcPlainFloor::~jhcPlainFloor ()
{
}


//= Default constructor initializes certain values.

jhcPlainFloor::jhcPlainFloor ()
{
  // max blobs in image
  boxes.SetSize(1000);

  // region finding 
  sc   =  4.0;               // Colored edge scaling
  msc  =  4.0;               // Monochrome edge scaling
  eth  = 50;                 // Edge threshold
  bd   =  3;                 // Border blanking
  blk  = 20;                 // Dark area threshold
  smin = 30;                 // Min separator area
  rsm  =  3;                 // Region smoothing

  // depth inference 
  amin  = 50;                // Min object area
  claim =  6;                // Blob expansion amt
  bot   = 10;                // Nearness to bottom
  frac  =  0.3;              // Floor blob wrt max
  gap   = 50;                // Max sep to object cliff
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Configure system for camera input of some size with some focal length.
// assumes fx = fy and optical center is middle of image
// up is fake wall height (in inches) to add at edge of floor area  

void jhcPlainFloor::Init (double f, int w, int h, double up)
{
  // configure internal images
  boost.SetSize(w, h, 3);
  comps.SetSize(w, h, 2);
  seeds.SetSize(comps);
  mono.SetSize(w, h, 1);
  tmp.SetSize(mono);
  tmp2.SetSize(mono);
  edge.SetSize(mono);
  sep.SetSize(mono);
  reg.SetSize(mono);
  floor.SetSize(mono);

  // remember focal length, image size, nd bounding wall
  flen = f;
  iw = w;
  ih = h;
  rim = up;
}


//= Creates synthetic depth image registered to color image based on camera pose.
// h is camera height in inches, t is tilt is in degs, assumes no roll 
// d16 pixels are 16 bit offsets from image plane in 0.25mm steps (like Kinect)

int jhcPlainFloor::Range (jhcImg& d16, const jhcImg& rgb, double h, double t)
{
  if (!mono.SameSize(rgb, 3) || !mono.SameSize(d16, 2))
    return Fatal("Bad images to jhcPlainFloor::Range");

  // cache image parameters
  ht = h;
  trads = D2R * t;

  // find non-textured regions and choose some as floor
  combo_edges(edge, mono, rgb);
  bland_areas(reg, edge, mono);
  pick_floor(floor, comps, reg);                      

  // convert floor and obstacles to frontal depth image
  ground_d16(d16, floor);                  
  obst_scan(d16, comps, floor);     
  return 1;
}


//= Create an version of the input with a green floor region.
// uses cached "trads" value and internal "mono" and "floor" images 

int jhcPlainFloor::Grass (jhcImg& dest)
{
  int horiz = (int)(0.5 * (ih - 1) - flen * tan(trads)), dev = 80;

  if (!boost.SameFormat(dest))
    return Fatal("Bad images to jhcPlainFloor::Grass");
  Threshold(tmp, floor, 128, dev);
  ClipDiff(tmp2, mono, tmp);
  ClipSum(tmp, mono, tmp);
  MergeRGB(dest, tmp2, tmp, tmp2);
  DrawLine(dest, 0, horiz, iw, horiz, 1, 0, 255, 255);
  return 1;
}


///////////////////////////////////////////////////////////////////////////
//                          Scene Segmentation                           //
///////////////////////////////////////////////////////////////////////////

//= Combine edge maps from individual channels with grayscale edges.
// also fills internal image "boost" 

void jhcPlainFloor::combo_edges (jhcImg& sob, jhcImg& bw, const jhcImg& rgb)
{
  ForceMono(bw, rgb);               
  SobelEdge(sob, bw, msc);
  MaxColor(boost, rgb);
  SobelMaxRGB(sob, boost, sc, 0);
}


//= Find regions between cleaned-up edges.
// also fills internal image "sep" 

void jhcPlainFloor::bland_areas (jhcImg& proto, const jhcImg& sob, const jhcImg& bw)
{
  // add very dark regions to edges
  Threshold(sep, sob, eth);             
  Border(sep, -bd);
  OverGate(sep, sep, bw, blk, 255);

  // clean up to get homogeneous regions
  BoxAvg(tmp, sep, rsm);
  RemSmall(proto, tmp, 0.0, smin, 80);
  BoxThresh(proto, proto, rsm, 80, 0, 255);    
}


//= Choose biggest non-textured region(s) near the bottom to comprise the floor.
// also fills internal image "seeds", sets "boxes" data 

void jhcPlainFloor::pick_floor (jhcImg& gnd, jhcImg& cc, const jhcImg& proto)
{
  int big;

  // find components
  CComps4(seeds, proto, amin);
  Expand(cc, seeds, claim);                  

  // keep only areas touching bottom of image
  boxes.FindBBox(cc);
  boxes.YBotThresh(-bot);

  // keep only biggest components to form floor
  big = boxes.Biggest();
  boxes.AreaThresh(ROUND(frac * boxes.PixelCnt(big)));
  boxes.MarkOver(gnd, cc);
}


///////////////////////////////////////////////////////////////////////////
//                           Depth Estimation                            //
///////////////////////////////////////////////////////////////////////////

// BASIC GEOMETRY
// --------------
// transform from world coordinates to camera coordinates:
// camera at origin looking along y axis for tilt = 0 (d is forward)
//   cx = wx
//   cy = -st * wy + ct * wz      where st = sin(tilt) 
//    d =  ct * wy + st * wz            ct = cos(tilt)
// and its inverse:
//   wx = cx;
//   wy = -st * cy + ct * d
//   wz =  ct * cy + st * d
// use perspective projection to get pixel positions (wrt middle):
// ix and iy flipped for normal rightside-up image (using d = -cz)
//   ix = f * cx / d
//   iy = f * cy / d
// and its inverse:
//   cx = d * ix / f
//   cy = d * iy / f
// substituting gives world coordinates from pixel positions:
//   wx = d * ix / f
//   wy = d * (f * ct - iy * st) / f
//   wz = d * (f * st + iy * ct) / f


//= Assign range values to floor pixels based on camera height and tilt.
// same image plane offset for all floor pixels on a particular scan line
// given pixel iy on ground plane with wz = -ht, find camera plane offset d
//   -ht = d * (f * st + iy * ct) / f
// so finally:
//   d = -ht * f / (iy * ct + f * st)
// depth values are in 0.25 mm steps (ht is in inches)
// only considers image rays that are below horizontal
// uses cached "ht" and "trads" member variable values

void jhcPlainFloor::ground_d16 (jhcImg& d16, const jhcImg& gnd) const
{
  double nhf = -101.6 * ht * flen, ymid = 0.5 * (ih - 1), iy = -ymid;
  double mm4, ct = cos(trads), fst = flen * sin(trads);
  int x, y;
  US16 dz;
  US16 *d = (US16 *) d16.PxlDest();
  const UC8 *g = gnd.PxlSrc();

  d16.FillArr(0xFF);                             // default = 0xFFFF (53') 
  for (y = 0; y < ih; y++, iy += 1.0)            
  {
    mm4 = nhf / (ct * iy + fst);
    if (mm4 > 65535.0)                           // too close to horizon
      break;
    dz = (US16) mm4;
    for (x = 0; x < iw; x++, g++, d++)           
      if (*g != 0)
        *d = dz;
  }     
}


//= Assume all components at boundary of floor region rise vertically straight up.
// so scan in world up-direction until exit of first component above boundary
// last valid floor pixel has known (ix0 iy0) and d0 so the planar position is:
//   wx0 = d0 * ix0 / f
//   wy0 = d0 * (f * ct - iy0 * st) / f
// inverting the original wx equation and keeping ground point wx0 constant:
//   ix = wx0 * f / d
// solving the original wy equation for d and keeping ground point wy0 constant:
//   d = wy0 * f / (f * ct - iy * st)
// substituting d into the ix equation gives ix as a function of iy:
//   ix = wx0 * f / [wy0 * f / (f * ct - iy * st)]
//      = (wx0 / wy0) * (f * ct - iy * st)
// using the original definitions of wx0 and wy0:
//   ix = [ix0 / (f * ct - iy0 * st)] * (iy * st - f * ct)
//   d  = [d0  * (f * ct - iy0 * st)] / (iy * st - f * ct)
// finally, letting mix0 = (f * ct - iy0 * st) and mix = (iy * st - f * ct):
//   ix = [ix0 / mix0] * mix = xm0 * mix
//   d  = [d0  * mix0] / mix = dm0 / mix
// this gives corresponding straight-up ix and d from successively incrementing iy
// Note the derived d can also be used to calculate the world height of the point:
//   wz = d * (st + iy * ct / f)
// uses cached "ht", "trads", and "rim" member variable values

void jhcPlainFloor::obst_scan (jhcImg& d16, const jhcImg& cc, const jhcImg& gnd) const
{
  double xmid = 0.5 * (iw - 1), ymid = 0.5 * (ih - 1), wall = -101.6 * (ht - rim);
  double st = sin(trads), ct = cos(trads), fct = flen * ct, ctdf = ct / flen;
  double mix0, dm0, xm0, iy, mix, d, wz = 0.0;
  int flat = (int)(ymid - flen * tan(trads)) - 1, ylim = __min(flat, ih - 1);
  int x, y, sx, sy, mm4, lab, miss, obj, ej, ej0 = ((rim > 0.0) ? 1 : 0);
  const UC8 *g = gnd.PxlSrc();

  // scan up from bottom for pixels on boundary of floor region
  for (y = 0; y < ylim; y++)
    for (x = 0; x < iw; x++, g++)
      if ((*g != 0) && (*(g + iw) == 0))         // not floor above
      {
        // compute coefficients for new ix and d based on new iy
        mix0 = fct - (y - ymid) * st;
        xm0 = (x - xmid) / mix0;
        dm0 = d16.ARef16(x, y) * mix0;

        // start generating streak angled upwards from boundary
        miss = 0;
        obj = 0;
        ej = ej0;                                // possibly disabled
        for (sy = y + 1; sy < ih; sy++)
        {
          // check that streak has not exited image or encountered floor again
          iy = sy - ymid;
          mix = fct - iy * st;
          sx = ROUND(xm0 * mix + xmid);
          if ((sx < 0) || (sx >= iw))          
            break;
          if (gnd.ARef(sx, sy) != 0)
            break;                     

          // make sure pixel depth is not beyond 16 bit limit
          d = dm0 / mix;
          mm4 = ROUND(d);
          if (mm4 > 65535)           
            break;       

          // SET EJ - determine whether edge guard wall is needed
          if (ej > 0)     
          {
            wz = d * (st + iy * ctdf);           // units of mm x 4
            if ((wz > wall) && (sy > (y + 1)))   // at least 1 pixel
              ej = 0;
          }

          // SET OBJ - allow small gap then mark first object found 
          lab = cc.ARef16(sx, sy);          
          if (obj == 0)                // nothing found yet          
          {
            if (lab == 0)              // still in gap above floor
              miss++;
            else if (miss <= gap)      // select object if gap is small
              obj = lab;                         
          }
          else if (lab != obj)         // no longer in selected object
            obj = -1;

          // record depth of vertical face if in rim or object 
          // stop if beyond rim and either object finished or gap too big
          if ((ej > 0) || (obj > 0))
            d16.ASet16(sx, sy, (US16) mm4);
          else if ((ej <= 0) && ((obj < 0) || (miss >= gap)))
            break;
        }
      }
}

