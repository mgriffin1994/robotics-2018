#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>


using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(std::map<uint8_t, std::vector<BlobRegion *>> &blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    // { WO_BEACON_PINK_BLUE, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    // { WO_BEACON_BLUE_YELLOW, 300 }
  };

  //assuming first color is bottom color, right above white


  static map<WorldObjectType,vector<int>> beacons = {
    { WO_BEACON_YELLOW_BLUE, { 24, 15, 74, 83} },
    { WO_BEACON_YELLOW_PINK, { 104, 41, 138, 96 } },
    { WO_BEACON_PINK_YELLOW, { 187, 38, 212, 90 } },
    // { WO_BEACON_PINK_BLUE, { } },
    { WO_BEACON_BLUE_PINK, { 246, 36, 268, 86 } },
    // { WO_BEACON_BLUE_YELLOW, {} }
  };

  

//  for (int i = 0; i < blobs.size(); i++) {
//      BlobRegion blob = *blobs[i];
//      if (blob.color == c_PINK) {
//          // TODO: more heuristics here?
//          if (((camera_ == Camera::TOP) && (blob.blobSize > 50 && blob.blobSize < 200)) || 
//                  ((camera_ != Camera::TOP) && (blob.blobSize > 1000 && blob.blobSize < 2000))) {
//              imageX = blob.centerx;
//              imageY = blob.centery;
//              // TODO: Get thresholds for correct ball radius
//              if (std::abs(1.0 - (float)(blob.maxx - blob.minx) / (float)(blob.maxy-blob.miny)) < 0.15) {
//                  printf("centerx %d, centery %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d\n",
//                          blob.centerx, blob.centery, blob.minx, blob.miny, blob.maxx, blob.maxy, blob.numRuns, blob.blobSize, blob.color);
//                  break;
//
//              }
//          } else {
//              return;
//          }
//      }
//  }
//  return;

  //aspect ratio to distinguish goal from beacon blue 
  //(could also use ratio of first largest blue to second largest blue and if way higher than 1 first one is goal)
  //ignore white and make sure high enough
  //trim blobs list after finding them to only include blobs above a certain size also
  //then o(n*6) for each beacon look across all found blobs (using n*6 table of blobs) for one with similar x value, size, and the correct higher/lower y value
  //and then use that combo to find the center of the corresponding beacon


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
