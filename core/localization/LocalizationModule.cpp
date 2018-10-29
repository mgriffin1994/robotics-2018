#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>
#include <Eigen/Eigenvalues>
#include <math.h>
#include <memory/JointBlock.h>
#include <memory/BodyModelBlock.h>


// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("vision_joint_angles");
#ifdef MEAT
  requiresMemoryBlock("body_model");
#endif
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  getOrAddMemoryBlock(cache_.joint,"vision_joint_angles");

#ifdef MEAT
  getOrAddMemoryBlock(cache_.body_model,"body_model");
#endif
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  pfilter_->init(self.loc, self.orientation);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  pfilter_->init(Point2D(-750,0), 0.0f);
  cache_.localization_mem->player_ = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::processFrame() {
  double currTime = double(clock());
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  //auto sloc = cache_.localization_mem->player_;
  //self.loc = Point2D(-1250, 0);
  //self.orientation = 0.0;

//  float friction = 0.5;
  float friction = 0.966;
  //float noiseR = 0.1;
  //float noiseQ = 0.1;

  double timeDelta = double((currTime - prevTime) + 0.000001) / CLOCKS_PER_SEC;
  //timeDelta = min(timeDelta, 1.0 / 20.);
  timeDelta = 1.0 / 30;

  static int num_frames_not_seen = 0;
  int max_frames = 15;

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  pfilter_->processFrame();
  self.loc = pfilter_->pose().translation;
  self.orientation = pfilter_->pose().rotation;
  
  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);

  if(ball.seen) {

    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> xk = cache_.localization_mem->state;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Pk = cache_.localization_mem->covariance;

    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> xkBar = Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign>::Zero();


    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> eye = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Qk = eye;
    Qk(0,0) = 2;
    Qk(1,1) = 60;
    Qk(2,2) = 2;
    Qk(3,3) = 60;    //TODO: change this to have noiser position and less noisy velocity? (or all same but different noiseQ?)



    //Move back inside if doing EKF
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Ak = eye;
    Ak(0, 1) = timeDelta;
    Ak(2, 3) = timeDelta;
    Ak(1, 1) = friction;
    Ak(3, 3) = friction;

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);


    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;


    Eigen::Matrix<float, 2, 1, Eigen::DontAlign> zk = Eigen::Matrix<float, 2, 1, Eigen::DontAlign>::Zero();

    zk[0] = ball.visionDistance;
    zk[1] = ball.visionBearing;


    // std::cout << std::endl;

    // std::cout << "Measurement:\n" << zk << std::endl;
    // std::cout << "Ak:\n" << Ak << std::endl;     //Believe this is created and set correctly


    xkBar = Ak*xk;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> PkBar = (Ak*Pk)*Ak.transpose() + Qk;


    // std::cout << "State Bar:\n" << xkBar << std::endl;
    // std::cout << "Covariance Bar:\n" << PkBar << std::endl;

    float headpan = cache_.joint->getJointValue(0);

    // std::cout << "Angles: " << self.orientation << " " << headpan << std::endl;
    // std::cout << "Robot Position:\n" << self.loc.x << " " << self.loc.y << std::endl;


    float distance = sqrt(pow(xkBar[0] - self.loc.x, 2) + pow(xkBar[2] - self.loc.y, 2));
    float bearing = atan2(xkBar[2] - self.loc.y, xkBar[0] -  self.loc.x) - self.orientation;

    //form Hk using xkbar?
    Eigen::Matrix<float, 2, 1, Eigen::DontAlign> zkBar = Eigen::Matrix<float, 2, 1, Eigen::DontAlign>::Zero();

    zkBar[0] = distance;
    zkBar[1] = bearing;


    // std::cout << "Predicted Measurement:\n" << zkBar << std::endl;


    Eigen::Matrix<float, 2, STATE_SIZE, Eigen::DontAlign> Hk = Eigen::Matrix<float, 2, STATE_SIZE, Eigen::DontAlign>::Zero();
    Hk(0, 0) = (xkBar[0] - self.loc.x) / distance;
    Hk(1, 2) = (xkBar[0] - self.loc.x) / (distance*distance);
    Hk(0, 2) = (xkBar[2] - self.loc.y) / distance;
    Hk(1, 0) = (self.loc.y - xkBar[2]) / (distance*distance);

    // std::cout << "Hk:\n" << Hk << std::endl;


    //TODO: get R when ball bit farther away? (like how far scorer going to kick from or bit more?)
    Eigen::Matrix<float, 2, 2, Eigen::DontAlign> Rk;
    // Rk(0,0) = 7.33937503;
    // Rk(0,1) = -0.0000902340085;
    // Rk(1,0) = -0.0000902340085;
    // Rk(1,1) = 0.000000924242939;
    Rk(0,0) = 167.567;
    Rk(0,1) = 0.000477;
    Rk(1,0) = 0.000477;
    Rk(1,1) = 0.0000021556;

    Eigen::Matrix<float, STATE_SIZE, 2, Eigen::DontAlign> Kk = (PkBar*Hk.transpose())*((Hk*PkBar*Hk.transpose() + Rk).inverse());

    // std::cout << "Kalman Gain:\n" << Kk << std::endl;

    xk = xkBar + Kk*(zk - zkBar);
    Pk = (eye - Kk*Hk)*PkBar;


    // std::cout << "State:\n" << xk << std::endl;
    // std::cout << "Covariance:\n" << Pk << std::endl;


    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = xk[0];
    cache_.localization_mem->state[1] = xk[1];
    cache_.localization_mem->state[2] = xk[2];
    cache_.localization_mem->state[3] = xk[3];
    //cache_.localization_mem->state[4] = xk[4];
    cache_.localization_mem->covariance = Pk;

    //TODO use this???
    if(num_frames_not_seen >= max_frames){
        cache_.localization_mem->state[0] = ball.loc.x;
        cache_.localization_mem->state[1] = 0;
        cache_.localization_mem->state[2] = ball.loc.y;
        cache_.localization_mem->state[3] = 0;
        cache_.localization_mem->covariance = eye;
        num_frames_not_seen = 0;
    }

  } else {
      num_frames_not_seen++;

/*      if (num_frames_not_seen >= max_frames) {
        cache_.localization_mem->state[0] = self.loc.x - 50;
        cache_.localization_mem->state[1] = 0;
        cache_.localization_mem->state[2] = 0;
        cache_.localization_mem->state[3] = 0;
        //cache_.localization_mem->state[4] = xk[4];
        cache_.localization_mem->covariance = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();
      } else {

    //
    //    xkBar = Ak*xk;
    //    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> PkBar = (Ak*Pk)*Ak.transpose() + Qk;
    //
    //
    //    //cout << "Missed ball" << std::endl;
    //    //std::cout << "State Bar:\n" << xkBar << std::endl;
    //    //std::cout << "Covariance Bar:\n" << PkBar << std::endl;
    //
    //    cache_.localization_mem->state[0] = xkBar[0];
    //    cache_.localization_mem->state[1] = xkBar[1];
    //    cache_.localization_mem->state[2] = xkBar[2];
    //    cache_.localization_mem->state[3] = xkBar[3];
    //    //cache_.localization_mem->state[4] = xk[4];
    //    cache_.localization_mem->covariance = PkBar;
    //
    //    xk.resize(0, 0);
    //    Pk.resize(0, 0);
    //    xkBar.resize(0, 0);
    //    PkBar.resize(0, 0);
    //    eye.resize(0, 0);
    //    Ak.resize(0, 0);
    //    Qk.resize(0, 0);
    //
      }*/
  }

  prevTime = currTime;

}
