#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ImageProcessor.h>
#include <vector>

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacons(std::vector<BlobRegion *> &blobs);
 private:
  TextLogger* textlogger;
};
