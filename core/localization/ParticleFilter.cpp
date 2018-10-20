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
  for(auto& p : particles()) {
      //±2500,±1250
      p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
      p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
      p.t = Random::inst().sampleU() * 2*M_PI - M_PI;  //0., M_PI / 4);
      p.w = 1.0/num_particles;
  }
  kidnapped = false;
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated

  /*std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "New frame" << std::endl;*/

  dirty_ = true;

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

  // TODO: If average liklihood of estimate becomes too low,
  // reseed (either with hypothesis from visible beacons (or history of beacons))
  // or just reseed with uniform particles everywhere again
  // Reseed periodically to prevent against kidnapped robot?
  // Resample only if no movement

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
  float avg_prob = 0;
  //std::cout << num_particles << std::endl;
  float avg_px, avg_py, avg_pt = 0;

  // Generate new samples from resampled particles
  for (int i = 0; i < num_particles; ++i) {
      Particle& p = resampled[i];

      // NOTE: new_pT is absolute angle for the particle, theta is relative angle from the robot, theta_pred is relative angle from the particle
      float new_pX = Random::inst().sampleN()*30 + p.x + disp.translation.x*cos(p.t) + disp.translation.y*sin(p.t);
      float new_pY = Random::inst().sampleN()*30 + p.y + disp.translation.x*sin(p.t) - disp.translation.y*cos(p.t);
      float new_pT = Random::inst().sampleN()*(M_PI/4) + disp.rotation + p.t;

      // Tighten it up! if completely stationary
      // TODO: make sure these actually become 0 when turn in place, if not zero the add back angle stuff above (disp.translation.x*cos(p.t) etc)
      if (disp.translation.x == 0 && disp.translation.y == 0) {
           new_pX = Random::inst().sampleN()*1 + p.x;
           new_pY = Random::inst().sampleN()*1 + p.y;
       } else if (disp.translation.x == 0) {
           new_pX = Random::inst().sampleN()*10 + p.x;
           new_pY = Random::inst().sampleN()*10 + p.y;
      } else if (disp.translation.y == 0) {
          new_pX = Random::inst().sampleN()*10 + p.x;
          new_pY = Random::inst().sampleN()*10 + p.y;
      }

      // Tighten it up! if not turning
      // TODO: make sure these actually become 0 when moving and not turning
       if (disp.rotation == 0) {
           new_pT = Random::inst().sampleN()*(M_PI/8) + p.t;
       }

      float w = 1.0;

      for(auto beacon : beacon_locs) {
        auto& object = cache_.world_object->objects_[beacon.first];
        float beaconX = beacon.second.x;
        float beaconY = beacon.second.y;

        if(object.seen){
          float d_pred = sqrt(pow(new_pX - beaconX, 2) + pow(new_pY-beaconY, 2));
          float d = object.visionDistance;
          float theta = object.visionBearing;
          float theta_pred = atan2(beaconY-new_pY, beaconX-new_pX) - new_pT;

          // TODO: Check for case when both are close to pi, -pi
          theta = std::fmod(theta + M_PI, 2 * M_PI);
		  if (theta < 0)
              theta += 2 * M_PI;
          theta -= M_PI;

          theta_pred = std::fmod(theta_pred + M_PI, 2 * M_PI);
          if (theta_pred < 0)
              theta_pred += 2 * M_PI;
          theta_pred -= M_PI;

          Eigen::Vector2f z;
          z << d, theta;

          Eigen::Vector2f mean;
          mean << d_pred, theta_pred;

          Eigen::Matrix2f covar;
          covar << 10000, 0, 0, .1; // TODO: Change theta variance

          Eigen::Vector2f diff;
          diff[0] = d - d_pred;
          // diff[1] = cos(theta - theta_pred);
          diff[1] = theta - theta_pred;

          diff[1] = std::fmod(diff[1] + M_PI, 2 * M_PI);
          if (diff[1] < 0)
              diff[1] += 2 * M_PI;
          diff[1] -= M_PI;

          //covar << 9.53023146e+03, -2.99987068e-02, -2.99987068e-02, 3.16690503e-06;

          //covar << 9.53023146e+03, -2.99987068e-02, -2.99987068e-02, 3.16690503e-06;

          /*[[  7.47232300e+02   1.13351948e-02]
            [  1.13351948e-02   1.96148594e-06]]*/

          //In the paper referenced in class, just uses two univariate gaussians, one for each of distance and  angle
          //  and then averages the two liklihoods together
          //  this way is probably better, since can handle covariances, but need to make sure ratio of variances is appropriate

          float dist_prob = 1/(sqrt(2*M_PI*covar(0, 0)))*exp(-0.5*pow(d_pred - d, 2) / covar(0, 0));
          float theta_prob = 1/(sqrt(2*M_PI*covar(1, 1)))*exp(-0.5*pow(theta_pred - theta, 2) / covar(1, 1));

          float prob = (1 / (2*M_PI*sqrt(covar.determinant())))*exp((-0.5*(diff).transpose()*covar.inverse()*(diff)));

          // std::cout << "prob one-liner: " << prob << std::endl;

           w *= prob;
//           avg_prob += prob;
        }
      }

      avg_prob += w;

//      if((p.x > -1500 && p.x < 1500) && (p.y > -1000 && p.y < 1000) && w >= 0){
//          std::cout << "=======================================" << std::endl;
//          //std::cout << "z - mean: " << z - mean << std::endl;
//          //std::cout << p.x << ", "<< p.y << ", "<< p.t << ", "<< p.w << std::endl;
//          std::cout << "particle x, y, t degrees, w:  " << new_pX << " | "<< new_pY << " | "<< new_pT * 180 / M_PI << " | "<< p.w << std::endl;
//          //std::cout << disp.translation.x << ", "<< disp.translation.y << ", "<< disp.rotation << std::endl;
//          //std::cout << disp.translation.x*cos(p.t) << ", "<< disp.translation.y*sin(p.t) << std::endl;
//          //std::cout << "=======================================" << std::endl;
//          std::cout << "object d, theta degrees, d_pred, theta_pred degrees:  " << d << " | " << theta * 180 / M_PI << " | " << d_pred << " | " << theta_pred * 180 / M_PI << std::endl;
//          std::cout << "total prob:  " << prob << std::endl;
//          std::cout << "dist prob, theta prob:  " << dist_prob << " | " << theta_prob << std::endl;
//          std::cout << "=======================================" << std::endl;
//          std::cout << std::endl;
//      }

      eta += w;

      avg_px += new_pX / num_particles;
      avg_py += new_pY / num_particles;
      avg_pt += new_pT / num_particles;

      new_particles[i] = {new_pX, new_pY, new_pT, w};
  }

  // std::cout << "ETA: " << eta << std::endl;

  avg_prob /= num_particles;
  std::cout << "=======================================" << std::endl;
  std::cout << "avg prob:  " << avg_prob << std::endl;
  std::cout << "=======================================" << std::endl;

  // TODO: Check if kidnapped works in meatspace
  BodyModelBlock* body_model = cache_.body_model;

  if (!body_model->feet_on_ground_) {
      kidnapped = true;
  }

  if (body_model->feet_on_ground_ && kidnapped) {
      // Prior distribution
      for(auto& p : particles()) {
          //±2500,±1250
          p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
          p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
          p.t = Random::inst().sampleU() * 2*M_PI - M_PI;  //0., M_PI / 4);
          p.w = 1.0/num_particles;
      }
      kidnapped = false;
  }

  // Resample some particles if avg_prob is too low
  // TODO: This works but we might have to mess with the noise values
  // Also, we should only do this if our avg_prob has been 0 for too long
  // sampleN or sampleU? From current particle position or entire field?
  if (avg_prob == 1.0) {
      for (auto& p : new_particles) {
           p.x = Random::inst().sampleN()*40 + avg_px;
           p.y = Random::inst().sampleN()*40 + avg_py;
           p.t = Random::inst().sampleN()*(M_PI/8) + avg_pt;
          // p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
          // p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
          // p.t = Random::inst().sampleU() * M_PI - M_PI/2.0;  //0., M_PI / 4);
          // How to normalize weights after this?
          // p.w = 1.0/num_particles;
      }
  }

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
      mean_.translation += T(p.x,p.y) * p.w;
      mean_.rotation += p.t * p.w;
    }
//    if(particles().size() > 0)
//        mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}

