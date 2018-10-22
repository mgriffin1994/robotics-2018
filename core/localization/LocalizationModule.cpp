#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>
#include <Eigen/Eigenvalues>

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
  auto sloc = cache_.localization_mem->player_;
  self.loc = sloc;

//  float friction = 0.5;
  float friction = 1.0;
  float noiseR = 0.1;
  float noiseQ = 0.1;

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
    Qk *= noiseQ;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Hk = eye;

    //Move back inside if doing EKF
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Ak = eye;
    Ak(0, 2) = timeDelta;
    Ak(1, 3) = timeDelta;
    Ak(2, 2) = friction;
    Ak(3, 3) = friction;

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);


    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    int x_diff = abs(ball.loc.x - prevPoint.x);
    int y_diff = abs(ball.loc.y - prevPoint.y);

    // TODO: Fine tune values here
    int x_diff_thresh = 15;
    int y_diff_thresh = 8;

//    if (x_diff < x_diff_thresh && y_diff < y_diff_thresh) {
//        ball.absVel = Point2D(0.0, 0.0);
//    } else if (x_diff < x_diff_thresh) {
//        ball.absVel = Point2D(0.0, (ball.loc.y - prevPoint.y) / timeDelta);
//    } else if (y_diff < y_diff_thresh) {
//        ball.absVel = Point2D((ball.loc.x - prevPoint.x) / timeDelta, 0.0);
//    }

    //std::cout << "*** C++ prints" << std::endl;

    ball.absVel = Point2D((ball.loc.x - prevPoint.x) / timeDelta,
            (ball.loc.y - prevPoint.y) / timeDelta);

    prevPoint = ball.loc;

    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> zk = Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign>::Zero();
    zk[0] = ball.loc.x;
    zk[1] = ball.loc.y;
    zk[2] = ball.absVel.x;
    zk[3] = ball.absVel.y;
    //zk[4] = friction;

    //std::cout << "Measurement:\n" << zk << std::endl;
    //This is good

    //Ak(0, 2) = timeDelta;
    //Ak(1, 3) = timeDelta;
    //Ak(2, 2) = xk[4];
    //Ak(2, 4) = xk[2];
    //Ak(3, 3) = xk[4];
    //Ak(3, 4) = xk[3];

    //std::cout << "Ak:\n" << Ak << std::endl;     //Believe this is created and set correctly


    //xkBar[0] = xk[0] + xk[2]*timeDelta;
    //xkBar[1] = xk[1] + xk[3]*timeDelta;
    //xkBar[2] = xk[2]*xk[4];
    //xkBar[3] = xk[3]*xk[4];
    //xkBar[4] = xk[4];

    xkBar = Ak*xk;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> PkBar = (Ak*Pk)*Ak.transpose() + Qk;

    //std::cout << "State Bar:\n" << xkBar << std::endl;
    //std::cout << "Covariance Bar:\n" << PkBar << std::endl;

    //Eigen::Matrix<float, STATE_SIZE-1, STATE_SIZE-1, Eigen::DontAlign> Rk = eye.block<STATE_SIZE-1, STATE_SIZE-1>(0, 0);
    //Rk *= noiseR;

    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Rk;

//    Rk << 2.77834922e+02, -6.01047446e+00, 6.58618318e+03, -1.44504580e+02,
//         -6.01047446e+00, 1.61867445e+01, -9.81230369e+01, 3.07608693e+02,
//          6.58618318e+03, -9.81230369e+01, 3.95272535e+05, -7.27628877e+03,
//         -1.44504580e+02, 3.07608693e+02, -7.27628877e+03, 1.84565405e+04;

//    Rk = 5*eye;

//    Rk <<  1.19153971e+00,  -2.08927445e-01,   1.62362367e+01,  -3.33271650e+00,
//           -2.08927445e-01,   1.00013832e-01,  -3.55263424e+00,   2.48750581e+00,
//           1.62362367e+01,  -3.55263424e+00,   9.74348930e+02,  -2.06537843e+02,
//           -3.33271650e+00,   2.48750581e+00,  -2.06537843e+02,   1.49250474e+02;

    Rk = eye;
    Rk(0,0) = 0.2;
    Rk(1,1) = 0.1;
    Rk(2,2) = 1.5;
    Rk(3,3) = 1.0;

    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> Kk = (PkBar*Hk.transpose())*((Hk*PkBar*Hk.transpose() + Rk).inverse());
    //std::cout << "Kalman Gain:\n" << Kk << std::endl;

    xk = xkBar + Kk*(zk - Hk*xkBar);
    Pk = (eye - Kk*Hk)*PkBar;

    //Pk = 0.5*Pk + 0.5*Pk.transpose(); //make sure it's symmetric (already pretty close so won't change values too much)
    //float minEig = Pk.eigenvalues().real().minCoeff()
    //if(minEig <= 0) Pk += (0.00000001 + minEig)*eye;
    //These two force Pk to be PD


    //std::cout << "PkBar Eigenvalues:\n" << PkBar.eigenvalues() << std::endl;
    //std::cout << "Pk Eigenvalues:\n" << Pk.eigenvalues() << std::endl;


    //std::cout << "State:\n" << xk << std::endl;
    //std::cout << "Covariance:\n" << Pk << std::endl;


    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = xk[0];
    cache_.localization_mem->state[1] = xk[1];
    cache_.localization_mem->state[2] = xk[2];
    cache_.localization_mem->state[3] = xk[3];
    //cache_.localization_mem->state[4] = xk[4];
  	cache_.localization_mem->covariance = Pk;


    if(num_frames_not_seen >= max_frames){
        cache_.localization_mem->state[0] = ball.loc.x;
        cache_.localization_mem->state[1] = ball.loc.y;
        cache_.localization_mem->state[2] = 0;
        cache_.localization_mem->state[3] = 0;
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
