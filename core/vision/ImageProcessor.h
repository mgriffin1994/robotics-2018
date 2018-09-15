#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <vision/structures/VisionParams.h>

typedef struct Run {
    Run* lead_parent;
    int start;
    int end;
    int row;
    uint8_t color;
    std::vector<Run *> possible_parents;
    int blobnum;
} Run;

typedef struct BlobRegion {
    int centerx;
    int centery;
    int minx;
    int miny;
    int maxx;
    int maxy;
    int numRuns;
    int blobSize; // Actually the bounding box size
    int numPixels; // Actually the number of pixels in all runs of this blob
    float density;
    uint8_t color;
} BlobRegion;

bool checkNearBeacon(BlobRegion* blob1, BlobRegion* blob2, int thresholdx, int thresholdy);

class BallDetector;
class Classifier;
class BeaconDetector;

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    ~ImageProcessor();
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    std::unique_ptr<BeaconDetector> beacon_detector_;
    std::unique_ptr<Classifier> color_segmenter_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(const RobotCalibration& calibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    
    void detectBall(std::map<uint8_t, std::vector<BlobRegion *>> &blobs);
    void detectGoal(std::map<uint8_t, std::vector<BlobRegion *>> &blobs);
    bool findBall(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, int& imageX, int& imageY, int& radius);
    bool findGoal(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, int& imageX, int& imageY);
    void findBlob(std::map<uint8_t, std::vector<BlobRegion *>> &blobs);

  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;

    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    std::unique_ptr<RobotCalibration> calibration_;
    bool enableCalibration_;

    //void saveImg(std::string filepath);
    int topFrameCounter_ = 0;
    int bottomFrameCounter_ = 0;
};

#endif
