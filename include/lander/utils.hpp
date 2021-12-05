#pragma once

// Copyright 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Some generic utility functions

inline double getDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI (for radians) or 180.0 (for degrees);
  double d =  fabs(fmod(fabs(x - y), 2*c));
  double r = d > c ? c*2 - d : d;
  
  double sign = ((x-y >= 0.0) && (x-y <= c)) || ((x-y <= -c) && (x-y> -2*c)) ? 1.0 : -1.0;
  return sign * r;                                           

}