#pragma once

#include <math/Pose2D.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>
#include <memory/WorldObjectBlock.h>
#include <memory/BodyModelBlock.h>

//#define MEAT

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    int num_particles = PARTICLE_NUM;
    int num_frames_random = 5;
    float prev_eta = 0.0;
    int num_frames_lost = 0;
    bool kidnapped;

    mutable Pose2D mean_;
    mutable bool dirty_;
};
