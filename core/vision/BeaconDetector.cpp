#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(std::vector<BlobRegion *> &blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_BLUE, 300 }
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }
  };

  //assuming first color is bottom color, right above white


  //static map<WorldObjectType,vector<int>> beacons = {
  //  { WO_BEACON_YELLOW_BLUE, { 24, 15, 74, 83} },
  //  { WO_BEACON_YELLOW_PINK, { 104, 41, 138, 96 } },
  //  { WO_BEACON_PINK_YELLOW, { 187, 38, 212, 90 } },
  //  { WO_BEACON_BLUE_PINK, { 246, 36, 268, 86 } }
  //};
  //TODO add the other 2 beacons (above in heights) here too

  //TODO: find centerx and centery for each of the 6 beacons (containing 4 pink, 4 yellow, 4 blue, and 6 white blobs amongst them)
  // need to also make sure that the two beacon squares are above a white blob too (shouldn't detect upside down beacons)
  // also may not have the top k size blobs belong to the beacons (obviously not for blue because of the goal, 
  // but also pink and yellow because of spurious legs in the shot too that could be larger than beacon colors)
  // would need to only consider pink, yellow, blue, and white blobs that have very similiar x values (and correct ordering of y values)


  auto fid = vblocks_.frame_info->frame_id;
  if(fid >= 6150) return;
  for(auto beacon : beacons) {
    auto& object = vblocks_.world_object->objects_[beacon.first];
    auto box = beacon.second;
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
