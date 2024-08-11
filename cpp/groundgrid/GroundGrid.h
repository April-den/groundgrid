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

// Grid map
#include "GridMap.h"

class tPoint{
    public:
        float poseX, poseY, poseZ, intensity;
        uint32_t ring;
        std::string frame_id;
        tPoint(){};
        virtual ~tPoint(){};
        tPoint& operator=(const tPoint& other) {
            if (this == &other)
                return *this;
            poseX = other.poseX;
            poseY = other.poseY;
            poseZ = other.poseZ;
            intensity = other.intensity;
            ring = other.ring;
            frame_id = other.frame_id;
            return *this;
        }
};

class tPose{
    public:
        tPoint point;
        float orientationX, orientationY, orientationZ, orientationW;
        std::string frame_id;
        tPose(){};
        virtual ~tPose(){};
        tPose& operator=(const tPose& other) {
            if (this == &other)
                return *this;
            point = other.point;
            orientationX = other.orientationX;
            orientationY = other.orientationY;
            orientationZ = other.orientationZ;
            orientationW = other.orientationW;
            frame_id = other.frame_id;
            return *this;
        }
};

class GroundGrid {
   public:

    /** Constructor.
     */
    GroundGrid(){};

    /** Destructor.
     */
    virtual ~GroundGrid(){};

    /** Sets the current dynamic configuration.
     **
     ** @param config
     */

    void initGroundGrid(const tPose &inOdom){
        mMap_ptr = std::make_shared<grid_map::GridMap, const std::vector< std::string >>({"points", "ground", "groundpatch", "minGroundHeight", "maxGroundHeight"});
        tPose odomPose;
        grid_map::GridMap& map = *mMap_ptr;
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(static_cast<double>(mDimension),static_cast<double>(mDimension)),static_cast<double>(mResolution),grid_map::Position(static_cast<double>(inOdom.point.poseX),static_cast<double>(inOdom.point.poseY)));
        odomPose = inOdom; 
        std::vector<grid_map::BufferRegion> damage;
        map.move(grid_map::Position(static_cast<double>(odomPose.point.poseX),static_cast<double>(odomPose.point.poseY)),damage);
        map["points"].setZero();
        map["ground"].setConstant(inOdom.point.poseZ);
        map["groundpatch"].setConstant(0.0000001);
        map["minGroundHeight"].setConstant(100.0);
        map["maxGroundHeight"].setConstant(-100.0);
        mLastPose = inOdom;
    };

    std::shared_ptr<grid_map::GridMap> update(const tPose &inOdom){
        if(!mMap_ptr){
            initGroundGrid(inOdom);
            return mMap_ptr;
        }
        grid_map::GridMap& map = *mMap_ptr;
        tPose poseDiff;
        poseDiff.point.poseX = inOdom.point.poseX-mLastPose.point.poseX;
        poseDiff.point.poseY = inOdom.point.poseY-mLastPose.point.poseY;
        std::vector<grid_map::BufferRegion> damage;
        map.move(grid_map::Position(inOdom.point.poseX,inOdom.point.poseY),damage);
        return mMap_ptr;
    };

    private:
        const float mResolution = .33f;
        const float mDimension = 120.0f;
        std::shared_ptr<grid_map::GridMap> mMap_ptr;
        tPose mLastPose;
};

