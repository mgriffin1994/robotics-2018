#pragma once

#include <tool/simulation/ObservationGenerator.h>
#include <tool/simulation/IsolatedBehaviorSimulation.h>

class GoalieSimulation : public IsolatedBehaviorSimulation {
  public:
    GoalieSimulation();
    void simulationStep() override;
    void kickBall();
    void resetBall();

  private:
    bool align_;
    int sframe_;
    void teleportPlayer(Point2D position, float orientation, int = 0);
};
