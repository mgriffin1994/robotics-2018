#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>

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
  //printf("camera bottom: %i, camera name: %i\n", camera_ == Camera::BOTTOM, camera_);
  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;

  std::vector<Region *> blobs;
  findBlob(blobs);
  detectBall(blobs);
  detectGoal(blobs);
  beacon_detector_->findBeacons();

  // TODO: debug prints
  for (int i = 0; i < blobs.size(); i++) {
      Region blob = *blobs[i];
      printf("==============================");
      printf("blob.centerx: %d\n", blob.centerx);
      printf("blob.centery: %d\n", blob.centery);
      printf("blob.minx: %d\n", blob.minx);
      printf("blob.miny: %d\n", blob.miny);
      printf("blob.maxx: %d\n", blob.maxx);
      printf("blob.maxy: %d\n", blob.maxy);
      printf("blob.numRuns: %d\n", blob.numRuns);
      printf("blob.blobSize: %d\n", blob.blobSize);
      printf("blob.color: %d\n", blob.color);
  }

  for(int i = 0; i < blobs.size(); i++) {
      delete(blobs[i]);
  }
  blobs.clear();
}

void ImageProcessor::detectGoal(std::vector<Region *> &blobs) {
  // Code taken from https://github.com/LARG/robotics-2018/blob/master/documentation/codebase_tutorial.md
  int imageX, imageY;
  if(!findGoal(blobs, imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OWN_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;
}

void ImageProcessor::detectBall(std::vector<Region *> &blobs) {
  // Code taken from https://github.com/LARG/robotics-2018/blob/master/documentation/codebase_tutorial.md
  int imageX, imageY;
  if(!findBall(blobs, imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
}

bool compareBlobs(const Region *a, const Region *b){
	return a->blobSize < b->blobSize;
}

bool ImageProcessor::findBall(std::vector<Region *> &blobs, int& imageX, int& imageY) {
    imageX = imageY = 0;
    for (int i = 0; i < blobs.size(); i++) {
        Region blob = *blobs[i];
        if (blob.color == c_ORANGE) {
            // TODO: more heuristics here?
            if (blob.blobSize > 100) {
                imageX = blob.centerx;
                imageY = blob.centery;
                return true;
            }
        }
    }
    return false;
}

bool ImageProcessor::findGoal(std::vector<Region *> &blobs, int& imageX, int& imageY) {
    imageX = imageY = 0;
    for (int i = 0; i < blobs.size(); i++) {
        Region blob = *blobs[i];
        if (blob.color == c_BLUE) {
            // TODO: more heuristics here?
            if (blob.blobSize > 2000) {
                imageX = blob.centerx;
                imageY = blob.centery;
                return true;
            }
        }
    }
    return false;
}

void ImageProcessor::findBlob(std::vector<Region *>& blobs) {

    auto segImg = getSegImg();
    int step = iparams_.width / 320;

    std::vector<std::vector<Run *>> all_runs;

    // Process from left to right
    for(int x = 0; x < iparams_.width; x+=step) {
        // Process from top to bottom
        std::vector<Run *> hor_runs;
        for(int y = 0; y < iparams_.height; y+=step) {
            // Retrieve the segmented color of the pixel at (x,y)
            uint8_t c = segImg [y * iparams_.width + x];
            int oldx, headx, headcol;
            uint8_t oldcol;
            if(x==0) {
                oldx = x;
                oldcol = c;
                headx = oldx;
                headcol = c;
                continue;
            }
            if(oldcol != c && (c == c_ORANGE || c == c_BLUE || c == c_YELLOW || c== c_PINK)) {

                //create Run object when detect c(lor change from orange
                Run *cur_run_ptr = new Run();
                Run cur_run = *cur_run_ptr;
                cur_run.start = headx;
                cur_run.end = oldx;
                cur_run.lead_parent = &cur_run;
                cur_run.color = oldcol;
                cur_run.row = y;
                cur_run.blobnum = -1;

                headx = x;
                headcol = c;

                if(y != 0) {
                    //look at all runs in previous row and find parents
                    for(int i=0; i < all_runs[y-1].size(); i++){
                        Run run = *all_runs[y-1][i];
                        if(run.color == cur_run.color && ((run.start >= cur_run.start && run.start <= cur_run.end) || (run.end >= cur_run.start && run.end <= cur_run.end))){
                            if(cur_run.lead_parent == &cur_run){
                                cur_run.lead_parent = run.lead_parent;
                            } else {
                                cur_run.possible_parents.push_back(&run);
                            }
                        }
                        if(run.start > cur_run.end){
                            break;
                        }
                    }
                }
                hor_runs.push_back(cur_run_ptr);
            }
            oldx = x;
            oldcol = c;
        }
        all_runs.push_back(hor_runs);
    }

    //path compression
    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            Run run = *all_runs[i][j];
            if(run.possible_parents.size() > 0){
                for(int k=0; k < run.possible_parents.size(); k++){
                    Run parent = *run.possible_parents[k];
                    if((*(parent.lead_parent)).lead_parent == parent.lead_parent){
                        (*(parent.lead_parent)).lead_parent = run.lead_parent;
                    }
                }
            }
        }
    }

    //create blobs
    int numblobs = 0;
    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            Run run = *all_runs[i][j];
            //create blobs whenever encouter root node or any of it's children
            Run parent = *(run.lead_parent);
            Run grand_parent = *(parent.lead_parent);

            Region *blob_ptr = new Region();
            Region blob = *blob_ptr;
            if(grand_parent.blobnum == -1){
                blob.centerx = (run.start + run.end + 1) / 2;
                blob.centery = run.row;
                blob.minx = run.start;
                blob.miny = run.row;
                blob.maxx = run.end;
                blob.maxy = run.row;
                blob.numRuns = 1;
                blob.blobSize = (blob.maxx - blob.minx+1);
                blob.color = run.color;
                grand_parent.blobnum = numblobs++;
                blobs.push_back(blob_ptr);
            } else {
                //find correct relevant blob already existing and update it's values
                blob = *blobs[grand_parent.blobnum];
                blob.minx = ((blob.minx < run.start) ? blob.minx : run.start);
                blob.maxx = ((blob.maxx > run.end) ? blob.maxx : run.end);
                blob.miny = ((blob.miny < run.row) ? blob.minx : run.row);
                blob.maxy = ((blob.maxy > run.row) ? blob.minx : run.row);
                blob.centerx += (int) ((((run.start + run.end + 1) / 2) - blob.centerx) / blob.numRuns);
                blob.centery += (int) ((run.row - blob.centery + 1) / blob.numRuns);
                blob.numRuns++;
                blob.blobSize = (blob.maxx - blob.minx+1) * (blob.maxy - blob.miny+1);
            }
        }
    }

    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            delete(all_runs[i][j]);
        }
        all_runs[i].clear();
    }
    all_runs.clear();

    std::sort(blobs.begin(), blobs.end(), compareBlobs);
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
