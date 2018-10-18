#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
//#include <vision/BeaconDetector.h>

using namespace Eigen;



ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger)
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;

  // Generate random particles for demonstration
  particles().resize(num_particles);

  // Prior distribution
  // TODO: Revert to this if robot kidnapped
  for(auto& p : particles()) {
    //±2500,±1250
    p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
    p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
    p.t = Random::inst().sampleU() * M_PI - M_PI/2.0;  //0., M_PI / 4);
    p.w = 1.0/num_particles;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "New frame" << std::endl;

  dirty_ = true;

  float current_angle = 0.0;

  static map<WorldObjectType, Point2D> beacon_locs = {
        { WO_BEACON_BLUE_YELLOW,    Point2D(1500, 1000) },
        { WO_BEACON_YELLOW_BLUE,    Point2D(1500, -1000) },
        { WO_BEACON_BLUE_PINK,      Point2D(0, 1000) },
        { WO_BEACON_PINK_BLUE,      Point2D(0, -1000) },
        { WO_BEACON_PINK_YELLOW,    Point2D(-1500, 1000) },
        { WO_BEACON_YELLOW_PINK,    Point2D(-1500, -1000) }
    };

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  std::vector<Particle>& new_particles = particles();
  std::vector<Particle> resampled;
  float c[num_particles] = {};
  c[0] = new_particles[0].w;

  // std::cout << "This is c" << std::endl;

  for (int i = 1; i < num_particles; ++i) {
      c[i] = c[i - 1] + new_particles[i].w;
      // std::cout << c[i] << std::endl;
  }

  int i = 0;

  float u = Random::inst().sampleU(1.0/num_particles);

  // std::cout << "This is u" << std::endl;
  // std::cout << u << std::endl;

  // std::cout << "This is new_p and u" << std::endl;

  for (int j = 0; j < num_particles; ++j) {
      while (u > c[i]) {
          i++;
      }

      Particle new_p = new_particles[i];
      new_p.w = 1.0/num_particles;
      resampled.push_back(new_p);
      u += 1.0/num_particles;
      // std::cout << new_p.x << " "<< new_p.y << " "<< new_p.t << " "<< new_p.w << std::endl;
      // std::cout << u << std::endl;
  }

  float eta = 0.0;
  std::cout << num_particles << std::endl;

  // Generate new samples from resampled particles
  for (int i = 0; i < num_particles; ++i) {
      Particle& p = resampled[i];

      // TODO: Mess with noise
      // NOTE: new_pT is absolute angle for the particle, theta is relative angle from the robot, theta_pred is relative angle from the particle
      float new_pX = Random::inst().sampleN()*20 + p.x + disp.translation.x*cos(p.t) + disp.translation.y*sin(p.t);
      float new_pY = Random::inst().sampleN()*20 + p.y + disp.translation.x*sin(p.t) - disp.translation.y*cos(p.t);
      float new_pT = Random::inst().sampleN()*(M_PI/16) + disp.rotation + p.t;

      float w = 1.0;

      for(auto beacon : beacon_locs) {
        auto& object = cache_.world_object->objects_[beacon.first];
        float beaconX = beacon.second.x;
        float beaconY = beacon.second.y;

        if(object.seen){
          float d_pred = sqrt(pow(new_pX - beaconX, 2) + pow(new_pY-beaconY, 2));
          float d = object.visionDistance;
          float theta = object.visionBearing;

//           float theta_pred = atan2(beaconY-new_pY, beaconX-new_pX);
          float theta_pred = atan2(beaconY-new_pY, beaconX-new_pX) - new_pT; // TODO: pretty sure this is right - Justin and the goose man

          Eigen::Vector2f z;
          z << d, theta;

          Eigen::Vector2f mean;
          mean << d_pred, theta_pred;

          Eigen::Matrix2f covar;
          covar << 500, 0, 0, 1.25;

          float prob = (1 / (2*M_PI*sqrt(covar.determinant())))*exp((-0.5*(z-mean).transpose()*covar.inverse()*(z-mean)));

          // std::cout << "prob one-liner: " << prob << std::endl;

          w *= prob;

          if((p.x > 0 && p.x < 1500) && (p.y > 0 && p.y < 1000)){
              std::cout << "=======================================" << std::endl;
              std::cout << "z - mean: " << z - mean << std::endl;
              //std::cout << p.x << ", "<< p.y << ", "<< p.t << ", "<< p.w << std::endl;
              std::cout << "particle x, y, t degrees, w:  " << new_pX << " | "<< new_pY << " | "<< new_pT * 180 / M_PI << " | "<< p.w << std::endl;
              //std::cout << disp.translation.x << ", "<< disp.translation.y << ", "<< disp.rotation << std::endl;
              //std::cout << disp.translation.x*cos(p.t) << ", "<< disp.translation.y*sin(p.t) << std::endl;
              std::cout << "=======================================" << std::endl;
              std::cout << "object d, theta degrees, d_pred, theta_pred degrees:  " << d << " | " << theta * 180 / M_PI << " | " << d_pred << " | " << theta_pred * 180 / M_PI << std::endl;
              std::cout << "prob:  " << prob << std::endl;
              std::cout << "=======================================" << std::endl;
              std::cout << std::endl;
          }

        }
      }

      eta += w;

      new_particles[i] = {new_pX, new_pY, new_pT, w};
  }

  // std::cout << "ETA: " << eta << std::endl;

  // Normalize weights
  for (auto& p : new_particles) {
      p.w /= eta;
      // std::cout << "Normalized p.w: " << p.w << std::endl;
  }
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}
