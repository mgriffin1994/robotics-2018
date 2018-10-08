#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>

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
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
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
  auto sloc = cache_.localization_mem->player_;
  self.loc = sloc;

  float friction = 0.5;
  float noiseR = 0.05;
  float noiseQ = 0.05;

    
  if(ball.seen) {
    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // double timeDelta = double((currTime - prevTime) + 0.000001) / CLOCKS_PER_SEC;
    double timeDelta = 0.03;

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    int x_diff = abs(ball.loc.x - prevPoint.x);
    int y_diff = abs(ball.loc.y - prevPoint.y);

    // TODO: Fine tune values here
    int x_diff_thresh = 10;
    int y_diff_thresh = 5;

//    if (x_diff < x_diff_thresh && y_diff < y_diff_thresh) {
//        ball.absVel = Point2D(0.0, 0.0);
//    } else if (x_diff < x_diff_thresh) {
//        ball.absVel = Point2D(0.0, (ball.loc.y - prevPoint.y) / timeDelta);
//    } else if (y_diff < y_diff_thresh) {
//        ball.absVel = Point2D((ball.loc.x - prevPoint.x) / timeDelta, 0.0);
//    }

    ball.absVel = Point2D((ball.loc.x - prevPoint.x) / timeDelta, 
            (ball.loc.y - prevPoint.y) / timeDelta);

    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> zk = Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign>::Zero();
    zk[0] = ball.loc.x;
    zk[1] = ball.loc.y;
    zk[2] = ball.absVel.x;
    zk[3] = ball.absVel.y;
    zk[4] = friction;

    // std::cout << "Measurement:\n" << zk << std::endl;
    //This is good



    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> xk = cache_.localization_mem->state;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Pk = cache_.localization_mem->covariance;

    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> xkBar = Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign>::Zero();
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> eye = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Ak = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();

    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Rk = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();
    Rk *= noiseR;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Qk = Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>::Identity();
    Qk *= noiseQ;

    Ak(0, 2) = timeDelta;
    Ak(1, 3) = timeDelta;
    Ak(2, 2) = xk[4];
    Ak(2, 4) = xk[2];
    Ak(3, 3) = xk[4];
    Ak(3, 4) = xk[3];

    // std::cout << "Ak:\n" << Ak << std::endl;     //Believe this is created and set correctly
    // std::cout << "Time Delta:\n" << timeDelta << std::endl; //This should be fine


    //Bug probably below this part???
    xkBar[0] = xk[0] + xk[2]*timeDelta;
    xkBar[1] = xk[1] + xk[3]*timeDelta;
    xkBar[2] = xk[2]*xk[4];
    xkBar[3] = xk[3]*xk[4];
    xkBar[4] = xk[4];
    // std::cout << "State Bar:\n" << xkBar << std::endl;




    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> PkBar = Ak*Pk*Ak.transpose() + Qk;
    // std::cout << "Covariance Bar:\n" << PkBar << std::endl;


    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Kk = PkBar*((PkBar + Rk).inverse());
    // std::cout << "Kalman Gain:\n" << Kk << std::endl;

    xk = xkBar + Kk*(zk - xkBar);
    Pk = (eye - Kk)*PkBar;
    // std::cout << "State:\n" << xk << std::endl;
    // std::cout << "Covariance:\n" << Pk << std::endl;


    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = xk[0];
    cache_.localization_mem->state[1] = xk[1];
    cache_.localization_mem->state[2] = xk[2];
    cache_.localization_mem->state[3] = xk[3];
    cache_.localization_mem->state[4] = xk[4];
  	cache_.localization_mem->covariance = Pk;


    zk.resize(0, 0);
    xk.resize(0, 0);
    Pk.resize(0, 0);
    xkBar.resize(0, 0);
    PkBar.resize(0, 0);
    eye.resize(0, 0);
    Ak.resize(0, 0);
    Rk.resize(0, 0);
    Kk.resize(0, 0);

  } 
  //TODO: How do we handle not seeing the ball?
  else {
    ball.distance = 10000.0f;
    ball.bearing = 0.0f;
  }
  prevPoint = ball.loc;
  prevTime = currTime;
}
