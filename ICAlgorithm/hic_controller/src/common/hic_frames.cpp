#include "hic_frames.hpp"

#include <algorithm>
#include <cmath>

namespace KDL
{

Rotation operator*(const Rotation& lhs, const Rotation& rhs)
{
	Rotation result = Rotation::Identity();
	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
		{
			double value = 0.0;
			for (int k = 0; k < 3; ++k)
			{
				value += lhs(row, k) * rhs(k, col);
			}
			result(row, col) = value;
		}
	}
	return result;
}

double Vector::Norm() const
{
	return std::sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
}

void Rotation::GetQuaternion(double& x, double& y, double& z, double& w) const
{
	const double trace = data[0] + data[4] + data[8];
	if (trace > 0.0)
	{
		const double s = 0.5 / std::sqrt(trace + 1.0);
		w = 0.25 / s;
		x = (data[7] - data[5]) * s;
		y = (data[2] - data[6]) * s;
		z = (data[3] - data[1]) * s;
		return;
	}

	if (data[0] > data[4] && data[0] > data[8])
	{
		const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + data[0] - data[4] - data[8]));
		w = (data[7] - data[5]) / s;
		x = 0.25 * s;
		y = (data[1] + data[3]) / s;
		z = (data[2] + data[6]) / s;
		return;
	}

	if (data[4] > data[8])
	{
		const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + data[4] - data[0] - data[8]));
		w = (data[2] - data[6]) / s;
		x = (data[1] + data[3]) / s;
		y = 0.25 * s;
		z = (data[5] + data[7]) / s;
		return;
	}

	const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + data[8] - data[0] - data[4]));
	w = (data[3] - data[1]) / s;
	x = (data[2] + data[6]) / s;
	y = (data[5] + data[7]) / s;
	z = 0.25 * s;
}

Frame Frame::DH(double a, double alpha, double d, double theta)
{
	const double ct = std::cos(theta);
	const double st = std::sin(theta);
	const double ca = std::cos(alpha);
	const double sa = std::sin(alpha);

	return Frame(
		Rotation(
			ct, -st * ca, st * sa,
			st, ct * ca, -ct * sa,
			0.0, sa, ca),
		Vector(a * ct, a * st, d));
}

} // namespace KDL
