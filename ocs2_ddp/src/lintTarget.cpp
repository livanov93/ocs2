#include <ocs2_ddp/DDP_DataCollector.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_Settings.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/ILQR_Settings.h>

// Riccati equations
#include <ocs2_ddp/riccati_equations/ContinuousTimeRiccatiEquations.h>
#include <ocs2_ddp/riccati_equations/DiscreteTimeRiccatiEquations.h>

#include <ocs2_ddp/riccati_equations/RiccatiModificationBase.h>
#include <ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
