/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMETRYOPENVINS_H_
#define ODOMETRYOPENVINS_H_

#include <rtabmap/core/Odometry.h>

#include <deque>

namespace ov_msckf {
class VioManager;
struct VioManagerOptions;
}

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryOpenVINS : public Odometry
{
public:
	OdometryOpenVINS(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() {return Odometry::kTypeOpenVINS;}
	virtual bool canProcessRawImages() const {return true;}
	virtual bool canProcessAsyncIMU() const {return true;}
	/// 仅在参数开启时声明支持 ROS 无关的外部足式速度观测。
	virtual bool canProcessExternalVelocity() const;
	/// 将一条四维足式速度观测缓存到 OpenVINS，相机线程随后只消费一次。
	virtual void processExternalVelocity(const ExternalVelocityMeasurement & measurement);
	/// 返回足式辅助状态、创新、门控结果和杆臂诊断。
	virtual std::map<std::string, std::string> externalVelocityDiagnostics() const;

private:
	virtual Transform computeTransform(SensorData & image, const Transform & guess = Transform(), OdometryInfo * info = 0);

private:
#ifdef RTABMAP_OPENVINS
	std::unique_ptr<ov_msckf::VioManager> vioManager_;
	std::unique_ptr<ov_msckf::VioManagerOptions> params_;
	bool initGravity_;
	Transform previousPoseInv_;
	Transform imuLocalTransformInv_;
	Eigen::Matrix<double, 6, 6> Phi_;
	std::deque<ExternalVelocityMeasurement> pendingLegVelocity_;
	bool legExtrinsicsDisabled_;
#endif
};

}

#endif /* ODOMETRYOPENVINS_H_ */
