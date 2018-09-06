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
  detectBall();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {
  // Code taken from https://github.com/LARG/robotics-2018/blob/master/documentation/codebase_tutorial.md
  int imageX, imageY;
  //return;
  if(!findBall(imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;
 
  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
}

typedef struct Run {
	Run* lead_parent;
    int start;
	int end;
	int row;
	uint8_t color;
    std::vector<Run *> possible_parents;
	bool addedToBlob;
	int blobnum;
} Run;

typedef struct Region {
	int centerx;
	int centery;
	int minx;
	int miny;
	int maxx;
	int maxy;
	int numRuns;
	int blobSize;
} Region;

bool compareBlobs(const Region *a, const Region *b){
	return a->blobSize < b->blobSize;
}

bool ImageProcessor::findBall(int& imageX, int& imageY) {
    imageX = imageY = 0;
    float centerX, centerY = 0.0;
    
    int total = 1;
    // Process from left to right
    for(int x = 0; x < iparams_.width; x++) {
        // Process from top to bottom
        for(int y = 0; y < iparams_.height; y++) {
            // Retrieve the segmented color of the pixel at (x,y)
            auto c = getSegImg()[y * iparams_.width + x];
            //printf("color: %i\n", c);
            //printf(c == c_ORANGE);
            //printf("%d\n", c == c_ORANGE);
            if(c == c_ORANGE){
                centerX = centerX + (((float)x-centerX) / (float)total);
                centerY = centerY + (((float)y-centerY) / (float)total);
                total++;
            }
        }
    }
    imageX = (int) centerX;
    imageY = (int) centerY;

    imageX = (imageX < 0.1*iparams_.width) ? 0.1*iparams_.width : imageX;
    imageX = (imageX > 0.9*iparams_.width) ? 0.9*iparams_.width : imageX;
    
    imageY = (imageY < 0.1*iparams_.height) ? 0.1*iparams_.height : imageY;
    imageY = (imageY > 0.9*iparams_.height) ? 0.9*iparams_.height : imageY;
    
    
    //printf("total orange pixels: %i\n", total);
    //printf("center of orange pixels: %i, %i\n", imageX, imageY);
    if(total > 100){
        return true;
    }
    return false;

    

  // TODO: temp
//   printf(" ============================================== hi ====================\n");
// 
//   imageX = imageY = 0;
//   auto segImg = getSegImg();
// 
// 	std::vector<std::vector<Run *>> all_runs;
// 
// 
// 	// Process from left to right
// 	for(int x = 0; x < 320; x++) {
// 	  // Process from top to bottom
// 		std::vector<Run *> hor_runs;
// 	  for(int y = 0; y < 240; y++) {
// 		// Retrieve the segmented color of the pixel at (x,y)
// 				uint8_t c = segImg [y * iparams_.width + x];
// 				int oldx, headx, headcol;
// 				uint8_t oldcol;
// 				if(x==0) {
// 					oldx = x;
// 					oldcol = c;
// 					headx = oldx;
// 					headcol = c;
// 					continue;
// 				}
// 				if(oldcol != c && oldcol == c_ORANGE) {
// 					//create Run object when detect c(lor change from orange
// 					Run cur_run;
// 					cur_run.start = headx;
// 					cur_run.end = oldx;
// 					cur_run.lead_parent = &cur_run;
// 					cur_run.color = oldcol;
// 					cur_run.addedToBlob = false;
// 					cur_run.row = y;
// 					cur_run.blobnum = -1;
// 
// 					if(y != 0) {
// 						//look at all runs in previous row and find parents
// 						for(int i=0; i < all_runs[y-1].size(); i++){
// 							Run run = *all_runs[y-1][i];
// 							if(run.color == cur_run.color && ((run.start >= cur_run.start && run.start <= cur_run.end) || (run.end >= cur_run.start && run.end <= cur_run.end))){
// 								if(cur_run.lead_parent == &cur_run){
// 									cur_run.lead_parent = run.lead_parent;
// 								} else {
// 									cur_run.possible_parents.push_back(&run);
// 								}
// 							}
// 							if(run.start > cur_run.end){
// 								break;
// 							}
// 						}
// 					}
// 					hor_runs.push_back(&cur_run);
// 				}
// 				if(oldcol != c && c == c_ORANGE){
// 					headx = x;
// 					headcol = c;
// 				}
// 				oldx = x;
// 				oldcol = c;
// 	  }
// 		all_runs.push_back(hor_runs);
// 	}
// 	//path compression
// 	for(int i=0; i < all_runs.size(); i++){
// 		for(int j=0; j < all_runs[i].size(); j++){
// 			Run run = *all_runs[i][j];
// 			if(run.possible_parents.size() > 0){
// 				for(int k=0; k < run.possible_parents.size(); k++){
// 					Run parent = *run.possible_parents[k];
// 					if((*(parent.lead_parent)).lead_parent == parent.lead_parent){
// 						(*(parent.lead_parent)).lead_parent = run.lead_parent;
// 					}
// 				}
// 			}
// 		}
// 	}
// 
// 	//create blobs
// 	std::vector<Region *> blobs;
// 	int numblobs = 0;
// 	for(int i=0; i < all_runs.size(); i++){
// 		for(int j=0; j < all_runs[i].size(); j++){
// 			Run run = *all_runs[i][j];
// 			//create blobs whenever encouter root node or any of it's children
// 			Run parent = *(run.lead_parent);
// 			Run grand_parent = *(parent.lead_parent);
// 
// 
// 			if(!grand_parent.addedToBlob){
// 				grand_parent.addedToBlob = true;
// 				Region blob;
// 				blob.centerx = (run.start + run.end) / 2;
// 				blob.centery = run.row;
// 				blob.minx = run.start;
// 				blob.miny = run.row;
// 				blob.maxx = run.end;
// 				blob.maxy = run.row;
// 				blob.numRuns = 1;
// 				blob.blobSize = (blob.maxx - blob.minx) * (blob.maxy - blob.miny);
// 				grand_parent.blobnum = numblobs++;
// 			} else {
// 				//find correct relevant blob already existing and update it's values
// 				Region blob = *blobs[grand_parent.blobnum];
// 				blob.minx = ((blob.minx < run.start) ? blob.minx : run.start);
// 				blob.maxx = ((blob.maxx > run.end) ? blob.maxx : run.end);
// 				blob.miny = ((blob.miny < run.row) ? blob.minx : run.row);
// 				blob.maxy = ((blob.maxy > run.row) ? blob.minx : run.row);
// 				blob.centerx = blob.centerx + (int)((((run.start + run.end) / 2) - blob.centerx) / blob.numRuns);
// 				blob.numRuns++;
// 				blob.blobSize = (blob.maxx - blob.minx) * (blob.maxy - blob.miny);
// 			}
// 		}
// 
// 	}
// 	std::sort(blobs.begin(), blobs.end(), compareBlobs);
// 	if(blobs.size() > 0 && blobs[0]->blobSize > 225){
// 		imageX = blobs[0]->centerx;
// 		imageY = blobs[0]->centery;
// 		return true;
// 	}
//     return false;
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
