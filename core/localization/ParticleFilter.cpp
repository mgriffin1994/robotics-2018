#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

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
    p.x = Random::inst().sampleU() * 250; //static_cast<int>(frame * 5), 250);
    p.y = Random::inst().sampleU() * 250; // 0., 250);
    p.t = Random::inst().sampleU() * M_PI / 4;  //0., M_PI / 4);
    p.w = Random::inst().sampleU();
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  auto frame = cache_.frame_info->frame_id;

  std::vector<Particle> resampled;
  float c[num_particles] = {};
  c[0] = particles()[0].w;

  for (int i = 1; i < num_particles; ++i) {
      c[i] = c[i - 1] + particles()[i].w;
  }

  int i = 1;

  float u[num_particles + 1] = {};
  u[0] = Random::inst().sampleU((float) 1/num_particles);

  for (int j = 0; j < num_particles; ++j) {
      while (u[j] > c[i]) {
          i++;
      }

      Particle new_p = particles()[i];
      new_p.w = (float) 1/num_particles;
      resampled.push_back(new_p);
      u[j + 1] = u[j] + (float) 1/num_particles;
  }

  std::vector<Particle> new_particles;
  float eta = 0.0;

  // Generate new samples from resampled particles
  for (int i = 0; i < num_particles; ++i) {
      Particle& p = resampled[i];

      // TODO: Change from 0.5?
      float x = Random::inst().sampleN() * 250 + 0.5 * (disp.translation.x + p.x);
      float y = Random::inst().sampleN() * 250 + 0.5 * (disp.translation.y + p.y);
      float t = Random::inst().sampleN() * M_PI / 4 + 0.5 * (disp.rotation + p.t);

      // TODO: Compute importance weight here from Normal cdf
      float w = 0.5;
      eta += w;

      Particle new_p = {x, y, t, w};
      new_particles.push_back(new_p);
  }

  // Normalize weights
  for (auto& p : new_particles) {
      p.w /= eta;
  }

  particles() = new_particles;
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
