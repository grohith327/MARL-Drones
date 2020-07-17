#ifndef _TEMPLATE_SUPERVISOR_H
#define _TEMPLATE_SUPERVISOR_H

#include "core/supervisor.h"
#include "util/registerer.h"

class TemplateSupervisor : public Supervisor {
  REGISTER("TemplateSupervisor", Supervisor);

 private:
  bool Initialize() override;
  void Draw(double t, VisualizerWindow* window) override;
  void Update(double t, double dt) override;
};

#endif
