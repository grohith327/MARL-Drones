#ifndef _DEFAULT_SUPERVISOR_H
#define _DEFAULT_SUPERVISOR_H

#include "core/supervisor.h"
#include "util/registerer.h"

class DefaultSupervisor : public Supervisor {
  REGISTER("DefaultSupervisor", Supervisor);

 private:
  bool Initialize() override;
  void Draw(double t, VisualizerWindow* window) override;
  // void Update(double t, double dt) override;
};

#endif
