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
  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;

  std::vector<BlobRegion *> blobs;
  findBlob(blobs);
  detectBall(blobs);
  detectGoal(blobs);
  beacon_detector_->findBeacons();

  for(int i = 0; i < blobs.size(); i++)
      delete(blobs[i]);

  blobs.clear();
}

void ImageProcessor::detectGoal(std::vector<BlobRegion *> &blobs) {
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
  goal->fromTopCamera = camera_ == Camera::TOP;
}

void ImageProcessor::detectBall(std::vector<BlobRegion *> &blobs) {
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
  ball->fromTopCamera = camera_ == Camera::TOP;
}

bool compareBlobs(const BlobRegion *a, const BlobRegion *b){
	return a->blobSize < b->blobSize;
}

bool ImageProcessor::findBall(std::vector<BlobRegion *> &blobs, int& imageX, int& imageY) {
    imageX = imageY = 0;
    for (int i = 0; i < blobs.size(); i++) {
        BlobRegion blob = *blobs[i];
        if (blob.color == c_ORANGE) {
            // TODO: more heuristics here?
            if (blob.blobSize > 100) {
                imageX = blob.centerx;
                imageY = blob.centery;
                printf("centerx %d, centery %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d\n", 
                    blob.centerx, blob.centery, blob.minx, blob.miny, blob.maxx, blob.maxy, blob.numRuns, blob.blobSize, blob.color);
                return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

bool ImageProcessor::findGoal(std::vector<BlobRegion *> &blobs, int& imageX, int& imageY) {
    imageX = imageY = 0;
    for (int i = 0; i < blobs.size(); i++) {
        BlobRegion blob = *blobs[i];
        if (blob.color == c_BLUE) {
            // TODO: more heuristics here?
            if (blob.blobSize > 200) {
                imageX = blob.centerx;
                imageY = blob.centery;
                printf("centerx %d, centery %d, minx %d, miny %d, maxx %d, maxy %d, numRuns %d, blobSize %d, color %d\n", 
                    blob.centerx, blob.centery, blob.minx, blob.miny, blob.maxx, blob.maxy, blob.numRuns, blob.blobSize, blob.color);
                return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

void ImageProcessor::findBlob(std::vector<BlobRegion *>& blobs) {

    auto segImg = getSegImg();
    int step = iparams_.width / 320;

    std::vector<std::vector<Run *>> all_runs;

    // Process from left to right
    for(int y = 0; y < iparams_.height; y+=step) {
        // Process from top to bottom
        std::vector<Run *> hor_runs;
        for(int x = 0; x < iparams_.width; x+=step) {
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
            if(oldcol != c && (oldcol == c_ORANGE || oldcol == c_BLUE || oldcol == c_YELLOW || oldcol == c_PINK)) {
                //create Run object when detect color change from orange
                Run *cur_run_ptr = new Run();
                cur_run_ptr->start = headx;
                cur_run_ptr->end = oldx;
                cur_run_ptr->lead_parent = cur_run_ptr;
                cur_run_ptr->color = oldcol;
                cur_run_ptr->row = y;
                cur_run_ptr->blobnum = -1;
                cur_run_ptr->possible_parents = {};

                headx = x;
                headcol = c;

                if(y != 0) {
                    //look at all runs in previous row and find parents
                    for(int i=0; i < all_runs[(y / step) - 1].size(); i++){
                        Run *run_ptr = all_runs[(y / step) - 1][i];
                        if(run_ptr->color == cur_run_ptr->color && 
                                ((run_ptr->start >= cur_run_ptr->start && run_ptr->start <= cur_run_ptr->end) 
                                    || (run_ptr->end >= cur_run_ptr->start && run_ptr->end <= cur_run_ptr->end))){
                            if(cur_run_ptr->lead_parent == cur_run_ptr){
                                cur_run_ptr->lead_parent = run_ptr->lead_parent;
                            } else {
                                cur_run_ptr->possible_parents.push_back(run_ptr);
                            }
                        }
                        if(run_ptr->start > cur_run_ptr->end){
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
            Run *run_ptr = all_runs[i][j];
            if(run_ptr->possible_parents.size() > 0){
                for(int k=0; k < run_ptr->possible_parents.size(); k++){
                    Run *parent_ptr = run_ptr->possible_parents[k];
                    Run *grand_parent_ptr = parent_ptr->lead_parent;
                    if(grand_parent_ptr->lead_parent == parent_ptr->lead_parent){
                        grand_parent_ptr->lead_parent = run_ptr->lead_parent;
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

            //create blobs whenever encouter root node or any of it's children
            Run *parent_ptr = run_ptr->lead_parent;
            Run *grand_parent_ptr = parent_ptr->lead_parent;

            BlobRegion *blob_ptr = new BlobRegion();
            if(grand_parent_ptr->blobnum == -1){
                blob_ptr->centerx = (run_ptr->start + run_ptr->end + 1) / 2;
                blob_ptr->centery = run_ptr->row;
                blob_ptr->minx = run_ptr->start;
                blob_ptr->miny = run_ptr->row;
                blob_ptr->maxx = run_ptr->end;
                blob_ptr->maxy = run_ptr->row;
                blob_ptr->numRuns = 1;
                blob_ptr->blobSize = (blob_ptr->maxx - blob_ptr->minx+1);
                blob_ptr->color = run_ptr->color;
                grand_parent_ptr->blobnum = numblobs++;
                blobs.push_back(blob_ptr);
            } else {
                //find correct relevant blob already existing and update it's values
                blob_ptr = blobs[grand_parent_ptr->blobnum];
                blob_ptr->minx = ((blob_ptr->minx < run_ptr->start) ? blob_ptr->minx : run_ptr->start);
                blob_ptr->maxx = ((blob_ptr->maxx > run_ptr->end) ? blob_ptr->maxx : run_ptr->end);
                blob_ptr->miny = ((blob_ptr->miny < run_ptr->row) ? blob_ptr->minx : run_ptr->row);
                blob_ptr->maxy = ((blob_ptr->maxy > run_ptr->row) ? blob_ptr->minx : run_ptr->row);
                blob_ptr->centerx += (int) ((((run_ptr->start + run_ptr->end + 1) / 2) - blob_ptr->centerx + 1) / blob_ptr->numRuns);
                blob_ptr->centery += (int) ((run_ptr->row - blob_ptr->centery + 1) / blob_ptr->numRuns);
                blob_ptr->numRuns++;
                blob_ptr->blobSize = (blob_ptr->maxx - blob_ptr->minx+1) * (blob_ptr->maxy - blob_ptr->miny+1);
            }
        }
    }

    for(int i=0; i < all_runs.size(); i++){
        for(int j=0; j < all_runs[i].size(); j++){
            all_runs[i][j]->possible_parents.clear();
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
