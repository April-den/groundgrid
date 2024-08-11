/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <Eigen/Eigen>

namespace groundgrid {
class GroundSegmentation {
  public:
    GroundSegmentation() {};
    void init(const size_t dimension, const float& resolution){
      const size_t cellCount = std::round(dimension/resolution);
      expectedPoints.resize(cellCount, cellCount);
      for(size_t i=0; i<cellCount; ++i){
        for(size_t j=0; j<cellCount; ++j){
            const float& dist = std::hypot(i-cellCount/2.0,j-cellCount/2.0);
            expectedPoints(i,j) = std::atan(1/dist)/verticalPointAngDist;
        }
      }
      Eigen::initParallel();
    };
    

  protected:
    Eigen::MatrixXf expectedPoints;

    // velodyne 128: Average distance in rad on the unit circle of the appr. 220k points per round/128 Lasers
    const float verticalPointAngDist = 0.00174532925*2; // 0.2 degrees HDL-64e //0.00174532925; // 0.1 degrees //(2*M_PI)/(220000.0/128.0); // ca. 0.00365567:
    const float minDistSquared = 12.0f;
  };
}
