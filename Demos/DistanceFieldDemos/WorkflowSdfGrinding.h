#ifndef WORKFLOW_SDF_GRINDING_H
#define WORKFLOW_SDF_GRINDING_H

#include "Common/Common.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace PBD
{
namespace WorkflowSdfGrinding
{
	template <typename Step, typename Action, typename DomainPointFn>
	std::vector<Step> buildBoneGrindingPath(
		const Eigen::AlignedBox3d& domain,
		const Action sdfEditAction,
		const DomainPointFn& domainPoint)
	{
		std::vector<Step> path;
		const Real radius = static_cast<Real>(0.085);
		const Real hold = static_cast<Real>(0.06);

		auto appendLine = [&path, &domain, &domainPoint, radius, hold, sdfEditAction](
			const Real x0, const Real y0, const Real z0,
			const Real x1, const Real y1, const Real z1,
			const unsigned int steps)
		{
			const unsigned int count = std::max(steps, 2u);
			const Vector3r start = domainPoint(domain, x0, y0, z0);
			const Vector3r end = domainPoint(domain, x1, y1, z1);
			for (unsigned int i = 0u; i < count; i++)
			{
				const Real t = static_cast<Real>(i) / static_cast<Real>(count - 1u);
				Step step;
				step.tip = start + t * (end - start);
				step.radius = radius;
				step.holdSeconds = hold;
				step.action = sdfEditAction;
				path.push_back(step);
			}
		};

		appendLine(
			static_cast<Real>(0.43), static_cast<Real>(0.47), static_cast<Real>(0.53),
			static_cast<Real>(0.60), static_cast<Real>(0.47), static_cast<Real>(0.53),
			16u);
		appendLine(
			static_cast<Real>(0.60), static_cast<Real>(0.51), static_cast<Real>(0.56),
			static_cast<Real>(0.43), static_cast<Real>(0.51), static_cast<Real>(0.56),
			16u);
		appendLine(
			static_cast<Real>(0.46), static_cast<Real>(0.55), static_cast<Real>(0.50),
			static_cast<Real>(0.58), static_cast<Real>(0.55), static_cast<Real>(0.50),
			12u);

		return path;
	}

	template <typename Step, typename Action>
	std::vector<Step> buildTopBoneGrindingPath(
		const Vector3r& topCenter,
		const Real radius,
		const Action sdfEditAction)
	{
		std::vector<Step> path;
		const Real hold = static_cast<Real>(0.06);
		const Real sweepHalf = static_cast<Real>(0.12);

		auto appendLine = [&path, radius, hold, sdfEditAction](
			const Vector3r& start,
			const Vector3r& end,
			const unsigned int steps)
		{
			const unsigned int count = std::max(steps, 2u);
			for (unsigned int i = 0u; i < count; i++)
			{
				const Real t = static_cast<Real>(i) / static_cast<Real>(count - 1u);
				Step step;
				step.tip = start + t * (end - start);
				step.radius = radius;
				step.holdSeconds = hold;
				step.action = sdfEditAction;
				path.push_back(step);
			}
		};

		const Vector3r approach = topCenter + Vector3r(static_cast<Real>(0), static_cast<Real>(0.18), static_cast<Real>(0));
		appendLine(approach, topCenter, 8u);

		const Vector3r sweepLeft = topCenter + Vector3r(-sweepHalf, static_cast<Real>(0), static_cast<Real>(0));
		const Vector3r sweepRight = topCenter + Vector3r(sweepHalf, static_cast<Real>(0), static_cast<Real>(0));
		appendLine(sweepLeft, sweepRight, 10u);
		appendLine(sweepRight, sweepLeft, 10u);

		const Vector3r sweepFront = topCenter + Vector3r(static_cast<Real>(0), static_cast<Real>(0), sweepHalf);
		const Vector3r sweepBack = topCenter + Vector3r(static_cast<Real>(0), static_cast<Real>(0), -sweepHalf);
		appendLine(sweepFront, sweepBack, 10u);
		appendLine(sweepBack, sweepFront, 10u);

		for (unsigned int i = 0u; i < 4u; i++)
		{
			const Real yOff = static_cast<Real>(-0.05) + static_cast<Real>(0.025) * static_cast<Real>(i);
			const Vector3r cpStart = topCenter + Vector3r(sweepHalf, yOff, static_cast<Real>(0));
			const Vector3r cpEnd = topCenter + Vector3r(-sweepHalf, yOff, static_cast<Real>(0));
			appendLine(cpStart, cpEnd, 6u);
			appendLine(cpEnd, cpStart, 6u);
		}

		return path;
	}
}
}

#endif
