#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory/GameStateBlock.h>

//#include <vision/BeaconDetector.h>

using namespace Eigen;


float max3( float a, float b, float c )
{
   float max = ( a < b ) ? b : a;
   return ( ( max < c ) ? c : max );
}


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

      // Three probability masses
      float rand_prob = Random::inst().sampleU();

      if(!cache_.game_state->isPenaltyKick){
          p.x = Random::inst().sampleN() * 50 - 1500 + 325;
          p.y = Random::inst().sampleN() * 50;
          p.t = Random::inst().sampleN() * M_PI/8;
          p.w = 1.0/num_particles;
      } else {

        if (rand_prob < 0.5) {
            p.x = Random::inst().sampleN() * 50 + 400;
            p.y = Random::inst().sampleN() * 50 - 850;
            p.t = Random::inst().sampleN() * M_PI/8 + M_PI/2;
            p.w = 1.0/num_particles;
        } else {
            p.x = Random::inst().sampleN() * 50 + 400;
            p.y = Random::inst().sampleN() * 50 + 850;
            p.t = Random::inst().sampleN() * M_PI/8 - M_PI/2;
            p.w = 1.0/num_particles;
        }
      }

      // p.x = 0; //static_cast<int>(frame * 5), 250);
      // p.y = 0; // 0., 250);
      // p.t = 0;  //0., M_PI / 4);
      // p.w = 1.0/num_particles;
  }
#ifdef MEAT
  kidnapped = false;
#endif
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated

  /*std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "New frame" << std::endl;*/

  dirty_ = true;

/*  static map<WorldObjectType, Point2D> beacon_locs = {
        { WO_BEACON_BLUE_YELLOW,    Point2D(1500, 1000) },
        { WO_BEACON_YELLOW_BLUE,    Point2D(1500, -1000) },
        { WO_BEACON_BLUE_PINK,      Point2D(0, 1000) },
        { WO_BEACON_PINK_BLUE,      Point2D(0, -1000) },
        { WO_BEACON_PINK_YELLOW,    Point2D(-1500, 1000) },
        { WO_BEACON_YELLOW_PINK,    Point2D(-1500, -1000) }
    };*/

    static map<WorldObjectType, Point2D> beacon_locs = {
        { WO_BEACON_BLUE_YELLOW,    Point2D(500, 1000) },
        { WO_BEACON_YELLOW_BLUE,    Point2D(500, 1000) },
        { WO_BEACON_BLUE_PINK,      Point2D(0, -1000) },
        { WO_BEACON_PINK_BLUE,      Point2D(0, -1000) },
        { WO_BEACON_PINK_YELLOW,    Point2D(-500, 1000) },
        { WO_BEACON_YELLOW_PINK,    Point2D(-500, 1000) }
    };

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  auto frame = cache_.frame_info->frame_id;
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
  float existing = 1.0;

  /*if (frame % num_frames_random == 0) {
      existing = 0.95;
  }*/

  float u = Random::inst().sampleU(1.0/(existing*num_particles));

  // std::cout << "This is u" << std::endl;
  // std::cout << u << std::endl;

  // std::cout << "This is new_p and u" << std::endl;
  for (int j = 0; j < int(existing*num_particles); ++j) {
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
  for (int j = int(existing*num_particles); j < num_particles; ++j){
      Particle new_p = new_particles[0];
      new_p.x = Random::inst().sampleN() * 200 + mean_.translation.x; //static_cast<int>(frame * 5), 250);
      new_p.y = Random::inst().sampleN() * 200 + mean_.translation.y; // 0., 250);
      new_p.t = Random::inst().sampleN() * 2*M_PI - M_PI;  //0., M_PI / 4);
      new_p.w = 1.0/num_particles;
      resampled.push_back(new_p);
  }

  float eta = 0.0;
  float avg_prob = 0;
  //std::cout << num_particles << std::endl;
  float avg_px = 0.0;
  float avg_py = 0.0;
  float avg_pt = 0.0;

  int threshold = 50;


  if (cache_.game_state->isPenaltyKick && (mean_.translation.x < -1500 - threshold || mean_.translation.x > 500 + 2*threshold || abs(mean_.translation.y) > 1000 + threshold)){
      cout << "Out of bounds" << std::endl;
      for(auto& p : particles()) {
          //±2500,±1250

          float rand_prob = Random::inst().sampleU();

          if (rand_prob < 0.5) {
              p.x = Random::inst().sampleN() * 50 + 400;
              p.y = Random::inst().sampleN() * 50 - 850;
              p.t = Random::inst().sampleN() * M_PI/8 + M_PI/2;
              p.w = 1.0/num_particles;
          } else {
              p.x = Random::inst().sampleN() * 50 + 400;
              p.y = Random::inst().sampleN() * 50 + 850;
              p.t = Random::inst().sampleN() * M_PI/8 - M_PI/2;
              p.w = 1.0/num_particles;
          }

          //TODO: change this to AMCL and sample points based about measurements
      }
      return;
  }
  //TODO see if rotation also bad
  if(!cache_.game_state->isPenaltyKick && (abs(mean_.rotation) > M_PI/2 || abs(mean_.translation.y) > 700 + threshold || mean_.translation.x < -1500 - threshold || mean_.translation.x > -850 + threshold)){
      cout << "Out of bounds" << std::endl;
      //cout << "Bot: " << mean_.translation.x << " " << mean_.translation.y;

      for(auto& p : particles()) {

        p.x = Random::inst().sampleN() * 50 - 1500 + 325;
        p.y = Random::inst().sampleN() * 50;
        p.t = Random::inst().sampleN() * M_PI/8;
        p.w = 1.0/num_particles;

      }
      return;
  }


  // float lambda_x = max3(disp.translation.x*2.0, disp.translation.y*1.0, 2);
  // float lambda_y = max3(disp.translation.x*1.0, disp.translation.y*2.0, 2);
  // float lambda_t = max3(abs(disp.translation.x+disp.translation.y)*0.0, disp.rotation*2.7, 0.072);

//  std::cout << std::endl;
//  std::cout << "Lambda: " << lambda_x << " " << lambda_y << " " << lambda_t << std::endl;


  // Generate new samples from resampled particles
  for (int i = 0; i < num_particles; ++i) {
      Particle& p = resampled[i];


      // NOTE: new_pT is absolute angle for the particle, theta is relative angle from the robot, theta_pred is relative angle from the particle
      //This is fine, the sim just has odometry noise
      float new_pX = Random::inst().sampleN() + p.x + disp.translation.x*cos(p.t) - disp.translation.y*sin(p.t);
      float new_pY = Random::inst().sampleN() + p.y + disp.translation.x*sin(p.t) + disp.translation.y*cos(p.t);
      float new_pT = Random::inst().sampleN()*(M_PI/64) + p.t + disp.rotation;




      // float lambda_x = max3(disp.translation.x*2.0, disp.translation.y*1.0, 25);
      // float lambda_y = max3(disp.translation.x*1.0, disp.translation.y*2.0, 25);
      // float lambda_t = max3(abs(disp.translation.x+disp.translation.y)*0.002, disp.rotation*1.0, 0.1);





      // float delta_x = disp.translation.x + lambda_x*Random::inst().sampleN();
      // float delta_y = disp.translation.y + lambda_y*Random::inst().sampleN();
      // float delta_t = disp.rotation + lambda_t*Random::inst().sampleN();


      // float new_pX = p.x + delta_x*cos(p.t) + delta_y*sin(p.t);
      // float new_pY = p.y - delta_x*sin(p.t) + delta_y*cos(p.t);
      // float new_pT = p.t + delta_t;





      // Tighten it up! if completely stationary
      // if (disp.translation.x == 0 && disp.translation.y == 0) {
      //      new_pX = Random::inst().sampleN() + p.x + disp.translation.x*cos(p.t) + disp.translation.y*sin(p.t);
      //      new_pY = Random::inst().sampleN() + p.y + disp.translation.x*sin(p.t) - disp.translation.y*cos(p.t);
      //      //new_pX = p.x;
      //      //new_pY = p.y;
      //  } /*else if (disp.translation.x == 0 || disp.translation.y == 0) {
      //      new_pX = Random::inst().sampleN()*20 + p.x;
      //      new_pY = Random::inst().sampleN()*20 + p.y;
      // } */

      // if (disp.rotation == 0) {
      //     new_pT = Random::inst().sampleN()*(M_PI/128) + p.t + disp.rotation;
      //     //new_pT = p.t;
      // }






      new_pT = std::fmod(new_pT + M_PI, 2 * M_PI);
      if (new_pT < 0)
          new_pT += 2 * M_PI;
      new_pT -= M_PI;


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
          covar << 9530.23146, 0.0, 0.0, 0.00000316690503; // TODO: Change theta variance

          Eigen::Vector2f diff;
          diff[0] = d - d_pred;
          // diff[1] = cos(theta - theta_pred);
          diff[1] = theta - theta_pred;

          // diff[1] = std::fmod(diff[1] + M_PI, 2 * M_PI);
          // if (diff[1] < 0)
          //     diff[1] += 2 * M_PI;
          // diff[1] -= M_PI;



          //covar << 9.53023146e+03, -2.99987068e-02, -2.99987068e-02, 3.16690503e-06;

          //covar << 9.53023146e+03, -2.99987068e-02, -2.99987068e-02, 3.16690503e-06;

          /*[[  7.47232300e+02   1.13351948e-02]
            [  1.13351948e-02   1.96148594e-06]]*/

          //In the paper referenced in class, just uses two univariate gaussians, one for each of distance and  angle
          //  and then averages the two liklihoods together
          //  this way is probably better, since can handle covariances, but need to make sure ratio of variances is appropriate

          //float dist_prob = 1/(sqrt(2*M_PI*covar(0, 0)))*exp(-0.5*pow(d_pred - d, 2) / covar(0, 0));
          //float theta_prob = 1/(sqrt(2*M_PI*covar(1, 1)))*exp(-0.5*pow(theta_pred - theta, 2) / covar(1, 1));
          //float prob = dist_prob*theta_prob;

          float prob = (1 / (2*M_PI*sqrt(covar.determinant())))*exp((-0.5*(diff).transpose()*covar.inverse()*(diff)));

          // std::cout << "prob one-liner: " << prob << std::endl;

           w *= prob;
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
//  std::cout << "=======================================" << std::endl;
//  std::cout << "avg prob:  " << avg_prob << std::endl;
//  std::cout << "=======================================" << std::endl;

#ifdef MEAT
  // TODO: Check if kidnapped works in meatspace

  if (!cache_.body_model->feet_on_ground_) {
      kidnapped = true;
  }

  if (cache_.body_model->feet_on_ground_ && kidnapped) {
      // cache_.body_model distribution
      std::cout << "Feet back on the ground, reseeding" << std::endl;
      for(auto& p : new_particles) {
          //±2500,±1250
          p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
          p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
          p.t = Random::inst().sampleU() * 2*M_PI - M_PI;  //0., M_PI / 4);
          p.w = 1.0/num_particles;
      }
      kidnapped = false;
  }
#endif

  if (avg_prob == 1.0) {
      num_frames_lost++;
  } else {
      num_frames_lost = 0;
  }

  // TODO: Try messing with noise values
  //       Or try periodically reseeding
  /*if (avg_prob == 1.0 && num_frames_lost > 20) {
      for (auto& p : new_particles) {
           p.x = Random::inst().sampleN()*100 + avg_px;
           p.y = Random::inst().sampleN()*100 + avg_py;
           p.t = Random::inst().sampleN()*(M_PI/2) + avg_pt;
          // p.x = Random::inst().sampleU() * 5000 - 2500; //static_cast<int>(frame * 5), 250);
          // p.y = Random::inst().sampleU() * 2500 - 1250; // 0., 250);
          // p.t = Random::inst().sampleU() * M_PI - M_PI/2.0;  //0., M_PI / 4);
          // How to normalize weights after this?
          // p.w = 1.0/num_particles;
      }
  }*/


  // Normalize weights
  for (auto& p : new_particles) {
      if(eta == 0.0){
        //std::cout << "Eta is 0" << std::endl;
        p.w = 1.0 / num_particles;
      }
      else p.w /= eta;
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

