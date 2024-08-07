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