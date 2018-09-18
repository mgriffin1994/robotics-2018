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
  int vstep = 1 << 1;
  int hstep = 1 << 2;
  uint8_t color1 = colors[beacon][0];
  uint8_t color2 = colors[beacon][1];
  std::vector<BlobRegion *> *color1_blobs = &blobs[color1];
  std::vector<BlobRegion *> *color2_blobs = &blobs[color2];
  std::vector<BlobRegion *> *white_blobs = &blobs[c_WHITE];

  // TODO:
  //aspect ratio to distinguish goal from beacon blue 
  //(could also use ratio of first largest blue to second largest blue and if way higher than 1 first one is goal)
  //ignore white and make sure high enough
  //trim blobs list after finding them to only include blobs above a certain size also
  //then o(n*6) for each beacon look across all found blobs (using n*6 table of blobs) for one with similar x value, size, and the correct higher/lower y value
  //and then use that combo to find the center of the corresponding beacon

  for (int i = 0; i < color1_blobs->size(); i++) {
      BlobRegion *color1_blob = (*color1_blobs)[i];
      float color1_aspect = 1.0 - std::abs((color1_blob->maxx - color1_blob->minx + hstep) / (color1_blob->maxy - color1_blob->miny + vstep));
      if ((color1_blob->blobSize > 200) && (color1_blob->density > 0.6)){
          for (int j = 0; j < color2_blobs->size(); j++) {
              BlobRegion * color2_blob = (*color2_blobs)[j];
              float color2_aspect = 1.0 - std::abs((color2_blob->maxx - color2_blob->minx + hstep) / (color2_blob->maxy - color2_blob->miny + vstep));
              if ((color2_blob->blobSize > 200) && checkNearBeacon(color1_blob, color2_blob, 20, 1000 + 200, 20, 20) && 
                      (color2_blob->density > 0.6)) {
                  for (int k = 0; k < white_blobs->size(); k++) {
                      BlobRegion *white_blob = (*white_blobs)[k];
                      if ((white_blob->blobSize > 200) && checkNearBeacon(color2_blob, white_blob, 100, 1000 + 200, 100, 100)) {
                          // TODO: Still need some sort of white check here to avoid detecting upside-down beacons
                          //TODO change these coordinates for the beacon
                          //get the color1_blob or color2_blob with the better aspect ratio (1 to 1) and use it's width (and calculate the correct height)

                          int min_width = MIN(color1_blob->maxx - color1_blob->minx, color2_blob->maxx - color2_blob->minx);
                          int min_height = MIN(color1_blob->maxy - color1_blob->miny, color2_blob->maxy - color2_blob->miny);
                          int minx, miny, maxx, maxy;

                          if (min_width == (color1_blob->maxx - color1_blob->minx)) {
                              minx = color1_blob->minx;
                              maxx = color1_blob->maxx;
                          } else {
                              minx = color2_blob->minx;
                              maxx = color2_blob->maxx;
                          }

                          if (min_height == (color1_blob->maxy - color1_blob->miny)) {
                              miny = color1_blob->miny;
                              maxy = color1_blob->maxy + min_height;
                          } else {
                              miny = color2_blob->miny - min_height;
                              maxy = color2_blob->maxy;
                          }

                          coordinates = {minx, miny, maxx, maxy};
                          printf("BEACON: Found beacon %s\n", BEACON_NAME(beacon));
                          printf("BLOB1: centerX %d, centerY %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d, density = %lf\n",
                                  color1_blob->centerx, color1_blob->centery, color1_blob->minx, color1_blob->miny, color1_blob->maxx, color1_blob->maxy, 
                                  color1_blob->numRuns, color1_blob->blobSize, color1_blob->color, color1_blob->density);
                          printf("BLOB2: centerX %d, centerY %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d, density = %lf\n",
                                  color2_blob->centerx, color2_blob->centery, color2_blob->minx, color2_blob->miny, color2_blob->maxx, color2_blob->maxy, 
                                  color2_blob->numRuns, color2_blob->blobSize, color2_blob->color, color2_blob->density);
                          return;
                      }
                  }
              }
          }
      }
  }
}

void BeaconDetector::findBeacons(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, HorizonLine &horizon) {
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
    { WO_BEACON_YELLOW_BLUE, {-1, -1, -1, -1} },
    { WO_BEACON_YELLOW_PINK, {-1, -1, -1, -1} },
    { WO_BEACON_PINK_YELLOW, {-1, -1, -1, -1} },
    { WO_BEACON_PINK_BLUE,   {-1, -1, -1, -1} },
    { WO_BEACON_BLUE_PINK,   {-1, -1, -1, -1} },
    { WO_BEACON_BLUE_YELLOW, {-1, -1, -1, -1} }
  };

  // auto fid = vblocks_.frame_info->frame_id;
  int beacon_count = 0;
  // if(fid >= 6150) return;
  for(auto beacon : beacons) {
      findBeacon(blobs, beacon.first, beacon.second);
      if (beacon.second[0] != -1) {
          beacon_count++;
          auto& object = vblocks_.world_object->objects_[beacon.first];
          auto box = beacon.second;
          object.imageCenterX = (box[0] + box[2]) / 2;
          object.imageCenterY = (box[1] + box[3]) / 2;
          auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
          object.visionDistance = cmatrix_.groundDistance(position);
          object.visionBearing = cmatrix_.bearing(position);
          object.visionElevation = cmatrix_.elevation(position);
          printf("BEACON %s: centerx %d, centery %d, elevation: %lf, distance: %lf\n",
                  BEACON_NAME(beacon.first), object.imageCenterX, object.imageCenterY, object.visionElevation, object.visionDistance);
          object.seen = true;
          object.fromTopCamera = camera_ == Camera::TOP;
          tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
      }
  }
  printf("Found %d beacons\n", beacon_count);
}
