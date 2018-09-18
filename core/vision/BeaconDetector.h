#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ImageProcessor.h>
#include <vector>

class TextLogger;

#define BEACON_NAME(c) ( \
    c == WO_BEACON_YELLOW_BLUE ? "YELLOW_BLUE" \
    : c == WO_BEACON_YELLOW_PINK ? "YELLOW_PINK" \
    : c == WO_BEACON_PINK_YELLOW ? "PINK_YELLOW" \
    : c == WO_BEACON_PINK_BLUE ? "PINK_BLUE" \
    : c == WO_BEACON_BLUE_PINK ? "BLUE_PINK" \
    : c == WO_BEACON_BLUE_YELLOW ? "BLUE_YELLOW" \
    : "INVALID")

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacon(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, WorldObjectType beacon, std::vector<int> &coordinates);
  void findBeacons(std::map<uint8_t, std::vector<BlobRegion *>> &blobs);
 private:
  TextLogger* textlogger;
};
