/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "PoseHoldSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Static.hh>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
#include <vector>

using namespace custom;

GZ_ADD_PLUGIN(
	PoseHoldSystem,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(PoseHoldSystem, "custom::PoseHoldSystem")

static constexpr const char *kTopic = "/pose_hold";

// A held vehicle is not subject to physics, so lift it clear of the ground plane.
static constexpr double kMinHoldHeight = 2.0;
static constexpr double kRotateRateRadS = 45. * GZ_PI / 180.;

template <typename Component, typename T>
static void setCommand(gz::sim::EntityComponentManager &ecm, gz::sim::Entity entity, const T &value)
{
	if (ecm.Component<Component>(entity)) {
		ecm.SetComponentData<Component>(entity, value);

	} else {
		ecm.CreateComponent(entity, Component(value));
	}
}

void PoseHoldSystem::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &,
			       gz::sim::EntityComponentManager &, gz::sim::EventManager &)
{
	_world_entity = entity;

	if (!_node.Subscribe(kTopic, &PoseHoldSystem::commandCallback, this)) {
		gzerr << "[PoseHoldSystem] Error subscribing to topic [" << kTopic << "]" << std::endl;
		return;
	}

	gzmsg << "[PoseHoldSystem] Listening on " << kTopic << std::endl;
}

void PoseHoldSystem::commandCallback(const gz::msgs::StringMsg &msg)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_pending_commands.push_back(msg.data());
}

void PoseHoldSystem::setStatic(gz::sim::EntityComponentManager &ecm, bool is_static)
{
	setCommand<gz::sim::components::Static>(ecm, _target, is_static);
}

void PoseHoldSystem::handleCommand(const std::string &command, gz::sim::EntityComponentManager &ecm)
{
	std::vector<std::string> tokens;
	{
		std::istringstream stream(command);
		std::string token;

		while (stream >> token) {
			tokens.push_back(token);
		}
	}

	if (tokens.empty()) {
		return;
	}

	const gz::sim::World world(_world_entity);
	size_t first = 0;

	// An optional leading model name selects the vehicle
	const gz::sim::Entity named = world.ModelByName(ecm, tokens[0]);

	if (named != gz::sim::kNullEntity) {
		if (tokens.size() < 2) {
			gzerr << "[PoseHoldSystem] Missing command after model name [" << tokens[0] << "]" << std::endl;
			return;
		}

		// Do not leave the previous vehicle frozen when switching to another one
		if (_target != gz::sim::kNullEntity && _target != named && _mode != Mode::Free) {
			setStatic(ecm, false);
			_mode = Mode::Free;
		}

		_target = named;
		first = 1;

	} else if (_target == gz::sim::kNullEntity) {
		for (const gz::sim::Entity model : world.Models(ecm)) {
			const auto *is_static = ecm.Component<gz::sim::components::Static>(model);

			if (!is_static || !is_static->Data()) {
				_target = model;
				break;
			}
		}
	}

	if (_target == gz::sim::kNullEntity) {
		gzerr << "[PoseHoldSystem] No target model for command [" << command << "]" << std::endl;
		return;
	}

	std::string verb = tokens[first];
	std::transform(verb.begin(), verb.end(), verb.begin(), [](unsigned char c) { return std::tolower(c); });

	const gz::sim::Model model(_target);
	const gz::math::Pose3d current = gz::sim::worldPose(_target, ecm);

	if (verb == "release") {
		_mode = Mode::Free;
		setStatic(ecm, false);
		gzmsg << "[PoseHoldSystem] Released " << model.Name(ecm) << std::endl;
		return;
	}

	if (verb == "rotate") {
		// Continue from wherever the vehicle currently is, held or not
		if (_mode == Mode::Free) {
			_position = current.Pos();
			_position.Z(std::max(_position.Z(), kMinHoldHeight));
			_orientation = current.Rot();
			_heading = current.Rot().Yaw();
		}

		_mode = Mode::Rotate;
		setStatic(ecm, false);
		gzmsg << "[PoseHoldSystem] Rotating " << model.Name(ecm) << std::endl;
		return;
	}

	// Attitudes are roll/pitch in the FLU body frame, keeping the current heading
	double roll = 0.;
	double pitch = 0.;

	if (verb == "level") {
		// roll and pitch stay zero

	} else if (verb == "nose_down") {
		pitch = GZ_PI / 2.;

	} else if (verb == "tail_down") {
		pitch = -GZ_PI / 2.;

	} else if (verb == "left_down") {
		roll = -GZ_PI / 2.;

	} else if (verb == "right_down") {
		roll = GZ_PI / 2.;

	} else if (verb == "inverted") {
		roll = GZ_PI;

	} else {
		gzerr << "[PoseHoldSystem] Unknown command [" << verb << "]. Use level, nose_down, tail_down, left_down, "
		      "right_down, inverted, rotate or release" << std::endl;
		return;
	}

	// Yaw read back from a pose at +-90 deg pitch is ill-defined, so reuse the tracked heading once held
	if (_mode == Mode::Free) {
		_heading = current.Rot().Yaw();
	}

	_position = current.Pos();
	_position.Z(std::max(_position.Z(), kMinHoldHeight));
	_orientation = gz::math::Quaterniond(roll, pitch, _heading);
	_mode = Mode::Hold;
	setStatic(ecm, true);
	gzmsg << "[PoseHoldSystem] Holding " << model.Name(ecm) << " " << verb << std::endl;
}

void PoseHoldSystem::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	std::vector<std::string> commands;
	{
		std::lock_guard<std::mutex> lock(_mutex);
		commands.swap(_pending_commands);
	}

	for (const std::string &command : commands) {
		handleCommand(command, ecm);
	}

	if (info.paused || _mode == Mode::Free || _target == gz::sim::kNullEntity) {
		return;
	}

	if (_mode == Mode::Rotate) {
		// Rotate about the world vertical axis, which is the axis calibration wants for each held side
		const double dt = std::chrono::duration<double>(info.dt).count();
		const gz::math::Quaterniond delta(0., 0., kRotateRateRadS * dt);
		_orientation = (delta * _orientation).Normalized();
		_heading += kRotateRateRadS * dt;

		// Dynamic body: command the rates too so the IMU sees the rotation, and cancel gravity drift.
		// The angular velocity command is applied in the body frame.
		setCommand<gz::sim::components::LinearVelocityCmd>(ecm, _target, gz::math::Vector3d::Zero);
		setCommand<gz::sim::components::AngularVelocityCmd>(ecm, _target,
				_orientation.Inverse() * gz::math::Vector3d(0., 0., kRotateRateRadS));
	}

	gz::sim::Model(_target).SetWorldPoseCmd(ecm, gz::math::Pose3d(_position, _orientation));
}
