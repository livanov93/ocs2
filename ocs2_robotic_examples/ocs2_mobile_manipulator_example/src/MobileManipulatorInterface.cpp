/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/misc/LoadData.h>

#include <ros/package.h>

#include <pinocchio/fwd.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "ocs2_mobile_manipulator_example/MobileManipulatorInterface.h"

namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  urdfPath_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  pinocchioInterface_.reset(new PinocchioInterface<ad_scalar_t>(urdfPath_));

  auto test = std::unique_ptr<PinocchioInterface<scalar_t>>(new PinocchioInterface<scalar_t>(urdfPath_));
  // Debugging the pinocchio model
  auto model = test->getModel().cast<scalar_t>();
  assert(model.nq == STATE_DIM);
  pinocchio::Data data(model);
  Eigen::VectorXd q = pinocchio::neutral(model);
  std::cerr << "Load Pinocchio model from " << urdfPath_ << '\n';
  std::cout << "model.nv = " << model.nv << '\n';
  std::cout << "model.nq = " << model.nq << '\n';
  std::cout << "model.njoints = " << model.njoints << '\n';
  std::cout << "q = " << q.transpose() << '\n';

  pinocchio::forwardKinematics(model, data, q);
  for (int k = 0; k < model.njoints; ++k) {
    std::cerr << "\"" << model.names[k] << "\":\t";
    std::cerr << " t = " << data.oMi[k].translation().transpose();
    std::cerr << '\n';
  }

  /*
   * DDP-MPC settings
   */
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");

  bool recompileLibraries;
  ocs2::loadData::loadCppDataType(taskFile_, "model_settings.recompileLibraries", recompileLibraries);

  /*
   * Dynamics
   */
  dynamicsPtr_.reset(new MobileManipulatorDynamics(*pinocchioInterface_));
  dynamicsPtr_->initialize("mobile_manipulator_dynamics", libraryFolder_, recompileLibraries, true);

  /*
   * Rollout
   */
  auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  costPtr_.reset(new MobileManipulatorCost());
  costPtr_->initialize("mobile_manipulator_cost", libraryFolder_, recompileLibraries, true);

  /*
   * Constraints
   */
  constraintPtr_.reset(new ocs2::ConstraintBase());

  /*
   * Initialization
   */
  operatingPointPtr_.reset(new ocs2::OperatingPoints(initialState_, ocs2::vector_t::Zero(INPUT_DIM)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::MPC_DDP> MobileManipulatorInterface::getMpc() {
  return std::unique_ptr<ocs2::MPC_DDP>(new ocs2::MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                                                          operatingPointPtr_.get(), ddpSettings_, mpcSettings_));
}

}  // namespace mobile_manipulator
