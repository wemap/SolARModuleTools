/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "SolAR3DTransformEstimationFrom3D3D.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3DTransformEstimationFrom3D3D);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolAR3DTransformEstimationFrom3D3D::SolAR3DTransformEstimationFrom3D3D() :ComponentBase(xpcf::toUUID<SolAR3DTransformEstimationFrom3D3D>())
{
	declareInterface<api::solver::pose::I3DTransformFinderFrom3D3D>(this);
}


SolAR3DTransformEstimationFrom3D3D::~SolAR3DTransformEstimationFrom3D3D() {
}

FrameworkReturnCode SolAR3DTransformEstimationFrom3D3D::estimate(const std::vector<Point3Df>& firstPoints3D, const std::vector<Point3Df>& secondPoints3D, Transform3Df & pose)
{	
	// init 3D transformation
	pose.linear() = Eigen::Matrix3f::Identity(3, 3);
	pose.translation() = Eigen::Vector3f::Zero();

	Eigen::MatrixXf in, out;
	in.resize(firstPoints3D.size(), 3);
	out.resize(secondPoints3D.size(), 3);
	if (in.rows() != out.rows())
		return FrameworkReturnCode::_ERROR_;
	for (int i = 0; i < firstPoints3D.size(); i++) {
		in.row(i) = Eigen::Vector3f(firstPoints3D[i].getX(), firstPoints3D[i].getY(), firstPoints3D[i].getZ());
		out.row(i) = Eigen::Vector3f(secondPoints3D[i].getX(), secondPoints3D[i].getY(), secondPoints3D[i].getZ());
	}

	// First find the scale, by finding the ratio of sums of some distances,
	// then bring the datasets to the same scale.
	float dist_in = 0, dist_out = 0;
	for (int row = 0; row < in.rows() - 1; row++) {
		dist_in += (in.row(row + 1) - in.row(row)).norm();
		dist_out += (out.row(row + 1) - out.row(row)).norm();
	}
	if (dist_in <= 0 || dist_out <= 0)
		return FrameworkReturnCode::_ERROR_;

	float scale = dist_out / dist_in;
	out /= scale;

	// Find the centroids then shift to the origin
	Eigen::Vector3f in_ctr = Eigen::Vector3f::Zero();
	Eigen::Vector3f out_ctr = Eigen::Vector3f::Zero();
	for (int row = 0; row < in.rows(); row++) {
		in_ctr += in.row(row);
		out_ctr += out.row(row);
	}
	in_ctr /= in.rows();
	out_ctr /= out.rows();
	for (int row = 0; row < in.rows(); row++) {
		in.row(row) -= in_ctr;
		out.row(row) -= out_ctr;
	}

	// SVD
	Eigen::MatrixXf Cov = in.transpose() * out;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Find the rotation
	float d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
	if (d > 0)
		d = 1.f;
	else
		d = -1.f;
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3, 3);
	I(2, 2) = d;
	Eigen::Matrix3f R = svd.matrixV() * I * svd.matrixU().transpose();

	// The final transform
	pose.linear() = scale * R;
	pose.translation() = scale * (out_ctr - R * in_ctr);

	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
