#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>


using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacon(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, WorldObjectType beacon, std::vector<int> &coordinates) {
  static map<WorldObjectType, vector<uint8_t>> colors = {
    { WO_BEACON_YELLOW_BLUE, {c_YELLOW, c_BLUE} },
    { WO_BEACON_YELLOW_PINK, {c_YELLOW, c_PINK} },
    { WO_BEACON_PINK_YELLOW, {c_PINK, c_YELLOW} },
    { WO_BEACON_PINK_BLUE, {c_PINK, c_BLUE} },
    { WO_BEACON_BLUE_PINK, {c_BLUE, c_PINK} },
    { WO_BEACON_BLUE_YELLOW, {c_BLUE, c_YELLOW} }
  };

  // TODO: Add white check below
  uint8_t color1 = colors[beacon][0];
  uint8_t color2 = colors[beacon][1];
  std::vector<BlobRegion *> *color1_blobs = &blobs[color1];
  std::vector<BlobRegion *> *color2_blobs = &blobs[color2];

  // TODO:
  //aspect ratio to distinguish goal from beacon blue 
  //(could also use ratio of first largest blue to second largest blue and if way higher than 1 first one is goal)
  //ignore white and make sure high enough
  //trim blobs list after finding them to only include blobs above a certain size also
  //then o(n*6) for each beacon look across all found blobs (using n*6 table of blobs) for one with similar x value, size, and the correct higher/lower y value
  //and then use that combo to find the center of the corresponding beacon

  for (int i = 0; i < color1_blobs->size(); i++) {
      BlobRegion *color1_blob = (*color1_blobs)[i];
      if (color1_blob->blobSize > 0) {
          for (int j = 0; j < color2_blobs->size(); j++) {
              BlobRegion * color2_blob = (*color2_blobs)[j];
              // TODO: Check white below and reasonable height
              if ((color2_blob->blobSize > 0) && checkNearBeacon(color1_blob, color2_blob, 20, 100)) {
                  coordinates = {color1_blob->minx, color1_blob->miny, color2_blob->maxx, color2_blob->maxy};
                  printf("coord: %d, %d, %d, %d\n", coordinates[0], coordinates[1], coordinates[2], coordinates[3]);
                  return;
              }
          }
      } 
  }
}

void BeaconDetector::findBeacons(std::map<uint8_t, std::vector<BlobRegion *>> &blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_PINK_BLUE, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_BLUE_YELLOW, 300 }
  };

  //assuming first color is bottom color, right above white

  static map<WorldObjectType,vector<int>> beacons = {
    { WO_BEACON_YELLOW_BLUE, {0, 0, 0, 0} },
    { WO_BEACON_YELLOW_PINK, {0, 0, 0, 0} },
    { WO_BEACON_PINK_YELLOW, {0, 0, 0, 0} },
    { WO_BEACON_PINK_BLUE, {0, 0, 0, 0} },
    { WO_BEACON_BLUE_PINK, {0, 0, 0, 0} },
    { WO_BEACON_BLUE_YELLOW, {0, 0, 0, 0} }
  };

  auto fid = vblocks_.frame_info->frame_id;
  if(fid >= 6150) return;
  for(auto beacon : beacons) {
    findBeacon(blobs, beacon.first, beacon.second);
    auto& object = vblocks_.world_object->objects_[beacon.first];
    auto box = beacon.second;
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
    printf("Box: %d, %d, %d, %d\n", box[0], box[1], box[2], box[3]);
    printf("Beacon centerx: %d, centery: %d\n", object.imageCenterX, object.imageCenterY);
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
