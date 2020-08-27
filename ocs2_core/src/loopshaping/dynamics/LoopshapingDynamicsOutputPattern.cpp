
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsOutputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  return r_filter.getA() * x_filter + r_filter.getB() * u_system;
}

VectorFunctionLinearApproximation LoopshapingDynamicsOutputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = this->computeFlowMap(t, x, u);

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).setZero();
  dynamics.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = r_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topRows(x_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.bottomRows(FILTER_STATE_DIM) = r_filter.getB();

  return dynamics;
}
VectorFunctionLinearApproximation LoopshapingDynamicsOutputPattern::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                               const vector_t& u) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const auto jumpMap_system = systemDynamics_->jumpMapLinearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation jumpMap;
  jumpMap.f = this->computeJumpMap(t, x);

  jumpMap.dfdx.resize(x.rows(), x.rows());
  jumpMap.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = jumpMap_system.dfdx;
  jumpMap.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).setZero();
  jumpMap.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  jumpMap.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();

  jumpMap.dfdu.resize(x.rows(), u.rows());
  jumpMap.dfdu.topRows(x_system.rows()) = jumpMap_system.dfdu;
  jumpMap.dfdu.bottomRows(FILTER_STATE_DIM).setZero();

  return jumpMap;
}

}  // namespace ocs2