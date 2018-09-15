#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <map>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera)
{
  enableCalibration_ = false;
  color_segmenter_ = std::make_unique<Classifier>(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = std::make_unique<BeaconDetector>(DETECTOR_PASS_ARGS);
  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

//void ImageProcessor::saveImg(std::string filepath) {
//  cv::Mat mat;
//  int xstep_ = 1 << iparams_.defaultHorizontalStepScale;
//  int ystep_ = 1 << iparams_.defaultVerticalStepScale;
//  cv::resize(color_segmenter_->img_grayscale(), mat, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST);

//  cv::imwrite(filepath, mat);
//}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_.data(), NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_.data(), NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_.data(), *abs_parts = vblocks_.body_model->abs_parts_.data();
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->isLoaded();
  return vblocks_.image->isLoaded();
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(const RobotCalibration& calibration){
  *calibration_ = calibration;
}

void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  tlog(30, "Process Frame camera %i", camera_);
  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;

  std::map<uint8_t, std::vector<BlobRegion *>> blobs;
  findBlob(blobs);
  detectBall(blobs);
  detectGoal(blobs);
  beacon_detector_->findBeacons(blobs);


  std::map<uint8_t, std::vector<BlobRegion *>>::iterator it;
  for(it = blobs.begin(); it != blobs.end(); it++) {
      std::vector<BlobRegion *> colored_blobs = it->second;
      for(int i = 0; i < colored_blobs.size(); i++) {
          delete(colored_blobs[i]);
      }
  }

  blobs.clear();
}

void ImageProcessor::detectGoal(std::map<uint8_t, std::vector<BlobRegion *>> &blobs) {
  // Code taken from https://github.com/LARG/robotics-2018/blob/master/documentation/codebase_tutorial.md
  int imageX, imageY;
  if(!findGoal(blobs, imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL]; //josiah wrote code to visualize goals but needs to be unknown_goal

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY, 250); //Height of center of goal in mm, same code used in BeaconDetector.cpp
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  if(goal->visionElevation > 500){
    goal->seen = false;
  } else {
    goal->seen = true;
  }

  goal->fromTopCamera = camera_ == Camera::TOP;
}

void ImageProcessor::detectBall(std::map<uint8_t, std::vector<BlobRegion *>> &blobs){
  // Code taken from https://github.com/LARG/robotics-2018/blob/master/documentation/codebase_tutorial.md
  int imageX, imageY, radius;
  if(!findBall(blobs, imageX, imageY, radius)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;
  ball->radius = radius;

  Position p = cmatrix_.getWorldPosition(imageX, imageY, 20);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  printf("visionDistance: %f\n", ball->visionDistance);

    // TODO: Get elevation thresholds
//  if(ball->visionElevation > 100){
//    ball->seen = false;
//  } else {
//    ball->seen = true;
//  }  
  ball->seen = true;
  ball->fromTopCamera = camera_ == Camera::TOP;
}

bool compareBlobs(const BlobRegion *a, const BlobRegion *b){
	return a->blobSize > b->blobSize;
}

// TODO: Create lookup table for blobsize and visionDistance to reliably draw overlay
bool ImageProcessor::findBall(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, int& imageX, int& imageY, int& radius) {
    imageX = imageY = radius = 0;
    int vstep = 1 << 1;
    int hstep = 1 << 2;
    std::vector<BlobRegion *> *orange_blobs = &blobs[c_ORANGE];
    for (int i = 0; i < orange_blobs->size(); i++) {
        BlobRegion *blob = (*orange_blobs)[i];
        // TODO: more heuristics here?
        if (((camera_ == Camera::TOP) && (blob->blobSize > 50)) || 
                ((camera_ != Camera::TOP) && (blob->blobSize > 1000))) {
            imageX = blob->centerx;
            imageY = blob->centery;
            // TODO: Why +hstep +vstep?
            radius = std::max(blob->maxx - blob->minx + hstep, blob->maxy - blob->miny + vstep) / 2;
            // TODO: Get thresholds for correct ball radius
            printf("aspect ratio: %lf\n", std::abs((float)(blob->maxx - blob->minx) / (float)(blob->maxy-blob->miny)));
            if ((std::abs(1.0 - (float)(blob->maxx - blob->minx) / (float)(blob->maxy-blob->miny)) < 0.25) &&
                    ((radius > 1) && (radius < 100)) && (blob->density > 0.5)){
                printf("centerx %d, centery %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d, radius = %d, density = %lf\n",
                        blob->centerx, blob->centery, blob->minx, blob->miny, blob->maxx, blob->maxy, blob->numRuns, blob->blobSize, blob->color, radius, blob->density);
                return true;

            }
        } else {
            return false;
        }
    }
    return false;
}

bool ImageProcessor::findGoal(std::map<uint8_t, std::vector<BlobRegion *>> &blobs, int& imageX, int& imageY) {
    imageX = imageY = 0;
    std::vector<BlobRegion *> *blue_blobs = &blobs[c_BLUE];
    for (int i = 0; i < blue_blobs->size(); i++) {
        BlobRegion *blob = (*blue_blobs)[i];
        // TODO: more heuristics here?
        if (camera_ == Camera::TOP && blob->blobSize > 500){
            imageX = blob->centerx;
            imageY = blob->centery;
            if (std::abs(1.7 - (float)(blob->maxx - blob->minx) / (float)(blob->maxy - blob->miny)) < 0.40){
                //                  printf("centerx %d, centery %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d\n",
                //                      blob->centerx, blob->centery, blob->minx, blob->miny, blob->maxx, blob->maxy, blob->numRuns, blob->blobSize, blob->color);
                return true;

            }
        } else {
            return false;
        }
    }
    return false;
}

// Top camera: 1280 x 960
// Bottom camera: 320 x 240
void ImageProcessor::findBlob(std::map<uint8_t, std::vector<BlobRegion *>> &blobs) {

    auto segImg = getSegImg();
    int vstep = 1 << 1;
    int hstep = 1 << 2;

    std::vector<std::vector<Run *>> all_runs;
    int headx;
    std::vector<Run *> hor_runs;

    for(int y = 0; y < iparams_.height; y+=vstep) {
        for(int x = 0; x < iparams_.width; x+=hstep) {
            // Retrieve the segmented color of the pixel at (x,y)
            if(x == 0) {
                headx = x;
                continue;
            }

            uint8_t c = segImg [y * iparams_.width + x]; //color at current step
            uint8_t old_col = segImg[y * iparams_.width + (x - hstep)]; //color at previous step
            //TODO: We may need to change color table to better allow for shadows


            //detect a run whenever the color changes and when just finished a run of orange, blue, yellow, or pink color pixels
            if((old_col != c && (old_col == c_ORANGE || old_col == c_BLUE || old_col == c_YELLOW || old_col == c_PINK || old_col == c_WHITE))
                    || (x == iparams_.width - hstep && (c == c_ORANGE || c == c_BLUE || c == c_YELLOW || c == c_PINK || c == c_WHITE))) { 
                 //x will never equal iparams_.width
                 //need to detect white too, since have to detect white for beacons (can't detect beacon if upside down so need to know where white blob is)


                //create Run object when detect color change from orange
                Run *cur_run_ptr = new Run();
                cur_run_ptr->start = headx; //head x is either start of row or last time the color changed (below)
                cur_run_ptr->end = (x == iparams_.width - hstep) ? x : x - hstep; //end of run should be previous x value
                //if end of row (x == iparams_.width - hstep) end should be the current pixel not the previous one


                cur_run_ptr->lead_parent = cur_run_ptr; //start as self pointer
                cur_run_ptr->color = old_col; //color of run, color before color switched to new value c
                cur_run_ptr->row = y; //row is current row y
                cur_run_ptr->blobnum = -1; //default blobnum is -1 (later assigned to index within blobregion vector)
                cur_run_ptr->possible_parents = {}; //list of non-lead parents (all parents to right of lead_parent)

                //don't look for parents if in first row
                if(y != 0) {
                    //look at all runs in previous row and find parents
                    int row_above = (y/vstep) - 1; // y/step is current iteration through outer loop
                    for(int i=0; i < all_runs[row_above].size(); i++){
                        Run *run_ptr = all_runs[row_above][i];
                        if(run_ptr->end < cur_run_ptr->start){
                          continue; //ignore runs that are entirely before this one
                        }
                        if(run_ptr->color == cur_run_ptr->color &&
                                ((run_ptr->start >= cur_run_ptr->start && run_ptr->start <= cur_run_ptr->end)
                                    || (run_ptr->end >= cur_run_ptr->start && run_ptr->end <= cur_run_ptr->end))){
                            if(cur_run_ptr->lead_parent == cur_run_ptr){ // if current run is pointing to itself
                                cur_run_ptr->lead_parent = run_ptr->lead_parent;
                            } else { // occurs if found more than one overlapping run from previous runs above
                                cur_run_ptr->possible_parents.push_back(run_ptr);
                            }
                        }
                        if(run_ptr->start > cur_run_ptr->end){
                            break; //stop looking if got to run beyond this one
                        }
                    }
                }
                hor_runs.push_back(cur_run_ptr);
            }

            //on any color change, change head of next potential run
            if (old_col != c) {
                headx = x;
            }
        }
        all_runs.push_back(hor_runs);
        hor_runs.clear();
    }

    //path compression
    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            Run *run_ptr = all_runs[i][j];
            if(run_ptr->possible_parents.size() > 0){
                for(int k=0; k < run_ptr->possible_parents.size(); k++){
                    Run *parent_ptr = (run_ptr->possible_parents)[k];
                    Run *grand_parent_ptr = parent_ptr->lead_parent;
                    if(grand_parent_ptr->lead_parent == grand_parent_ptr){ //default value, then self pointer
                        grand_parent_ptr->lead_parent = run_ptr->lead_parent; //change grandparent's lead_parent to my lead_parent
                    }
                }
            }
        }
    }

    //create blobs
    int numblobs = 0;
    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            Run *run_ptr = all_runs[i][j];

            if(run_ptr->end - run_ptr->start + hstep <= 2*hstep){
              //if very short run don't use it to create new blob or adjust statistics of existing blobs
              //should reduce the impact of noise and create more accurate blobs
              continue;

            }

            //create blobs whenever encouter root node or any of it's children
            Run *parent_ptr = run_ptr->lead_parent;
            Run *grand_parent_ptr = parent_ptr->lead_parent;

            // TODO: Maybe add a sparsity calculation using the aggregate of run sizes?
            BlobRegion *blob_ptr;
            if(grand_parent_ptr->blobnum == -1){ //if my grandparent (root of this blob) hasn't been added yet, create a blob for it
                blob_ptr = new BlobRegion(); //create new blob in here and not every time outside (was bug)

                //create blob with my statistics, not the root's statistics
                blob_ptr->centerx = (run_ptr->start + run_ptr->end) / 2; //technically don't know if run ends at end or up to hstep-1 after end, close enough
                blob_ptr->centery = run_ptr->row;
                blob_ptr->minx = run_ptr->start;
                blob_ptr->miny = run_ptr->row;
                blob_ptr->maxx = run_ptr->end;
                blob_ptr->maxy = run_ptr->row;
                blob_ptr->numRuns = 1;
                blob_ptr->numPixels = (run_ptr->end - run_ptr->start) * vstep;
                blob_ptr->blobSize = (blob_ptr->maxx - blob_ptr->minx + hstep) * vstep; //actual size in pixels in original image
                blob_ptr->color = run_ptr->color;
                grand_parent_ptr->blobnum = blobs[run_ptr->color].size();
                std::vector<BlobRegion *> *colored_blobs = &blobs[run_ptr->color];
                colored_blobs->push_back(blob_ptr);
            } else {
                //find correct relevant blob already existing and update it's values
                std::vector<BlobRegion *> *colored_blobs = &blobs[run_ptr->color];
                blob_ptr = (*colored_blobs)[grand_parent_ptr->blobnum];
                blob_ptr->minx = MIN(blob_ptr->minx, run_ptr->start);
                blob_ptr->maxx = MAX(blob_ptr->maxx, run_ptr->end);
                blob_ptr->miny = MIN(blob_ptr->miny, run_ptr->row);
                blob_ptr->maxy = MAX(blob_ptr->maxy, run_ptr->row);
                blob_ptr->centerx += (int) ((((run_ptr->start + run_ptr->end) / 2) - blob_ptr->centerx + hstep) / blob_ptr->numRuns);
                blob_ptr->centery += (int) ((run_ptr->row - blob_ptr->centery + hstep) / blob_ptr->numRuns);
                blob_ptr->numRuns += 1;
                blob_ptr->numPixels += (run_ptr->end - run_ptr->start) * vstep;
                // TODO: Why +hstep +vstep?
                blob_ptr->blobSize = (blob_ptr->maxx - blob_ptr->minx + hstep) * (blob_ptr->maxy - blob_ptr->miny + vstep); //actual size in pixels in original image
            }
        }
    }

    std::map<uint8_t, std::vector<BlobRegion *>>::iterator it;
    for(it = blobs.begin(); it != blobs.end(); it++) {
        std::vector<BlobRegion *> *colored_blobs = &(it->second);
        for(int i = 0; i < colored_blobs->size(); i++) {
            (*colored_blobs)[i]->density = ((float) (*colored_blobs)[i]->numPixels / (float) (*colored_blobs)[i]->blobSize);
        }
        std::sort(colored_blobs->begin(), colored_blobs->end(), compareBlobs); //should sort in decreasing order of size (in absolute size not downsampled size)
    }

    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            all_runs[i][j]->possible_parents.clear();
            delete(all_runs[i][j]);
        }
        all_runs[i].clear();
    }
    all_runs.clear();
}


int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  return std::vector<BallCandidate*>();

}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
