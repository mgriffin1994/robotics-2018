#pragma once

#include <vision/ObjectDetector.h>

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
