#ifndef WORKFLOW_SOFT_TISSUE_FRACTURE_H
#define WORKFLOW_SOFT_TISSUE_FRACTURE_H

#include "Common/Common.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace PBD
{
namespace WorkflowSoftTissueFracture
{
	template <typename Step, typename Action, typename Phase>
	std::vector<Step> buildScriptedParticlePullPath(
		const Vector3r& start,
		const Vector3r& perStepPullDelta,
		const Real radius,
		const Action moveOnlyAction,
		const Action pullFractureAction,
		const Phase approachPhase,
		const Phase graspPhase,
		const Phase pullPhase,
		const Phase holdPhase)
	{
		std::vector<Step> path;
		const Real hold = static_cast<Real>(0.08);

		auto appendStep = [&path, radius, hold](const Vector3r& tip, const Action action, const Phase phase)
		{
			Step step;
			step.tip = tip;
			step.radius = radius;
			step.holdSeconds = hold;
			step.action = action;
			step.phase = phase;
			path.push_back(step);
		};

		appendStep(start, moveOnlyAction, approachPhase);
		Vector3r tip = start;
		for (unsigned int i = 0u; i < 8u; i++)
		{
			tip += perStepPullDelta;
			appendStep(tip, pullFractureAction, graspPhase);
		}
		for (unsigned int i = 0u; i < 24u; i++)
		{
			tip += perStepPullDelta;
			appendStep(tip, pullFractureAction, pullPhase);
		}
		for (unsigned int i = 0u; i < 32u; i++)
			appendStep(tip, pullFractureAction, holdPhase);

		return path;
	}

	template <typename Step, typename Action, typename Phase, typename DomainPointFn>
	std::vector<Step> buildLigamentPullPath(
		const Eigen::AlignedBox3d& domain,
		const Action moveOnlyAction,
		const Action pullFractureAction,
		const Phase approachPhase,
		const Phase graspPhase,
		const Phase pullPhase,
		const Phase fracturePhase,
		const Phase holdPhase,
		const DomainPointFn& domainPoint)
	{
		std::vector<Step> path;
		const Real radius = static_cast<Real>(0.085);
		const Real hold = static_cast<Real>(0.08);

		auto appendLine = [&path, &domain, &domainPoint, radius, hold](
			const Real x0, const Real y0, const Real z0,
			const Real x1, const Real y1, const Real z1,
			const unsigned int steps,
			const Action action,
			const Phase phase)
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
				step.action = action;
				step.phase = phase;
				path.push_back(step);
			}
		};

		appendLine(
			static_cast<Real>(0.50), static_cast<Real>(0.36), static_cast<Real>(0.50),
			static_cast<Real>(0.50), static_cast<Real>(0.47), static_cast<Real>(0.50),
			8u,
			moveOnlyAction,
			approachPhase);
		appendLine(
			static_cast<Real>(0.47), static_cast<Real>(0.47), static_cast<Real>(0.50),
			static_cast<Real>(0.54), static_cast<Real>(0.58), static_cast<Real>(0.50),
			18u,
			pullFractureAction,
			graspPhase);
		appendLine(
			static_cast<Real>(0.54), static_cast<Real>(0.58), static_cast<Real>(0.50),
			static_cast<Real>(0.47), static_cast<Real>(0.66), static_cast<Real>(0.50),
			16u,
			pullFractureAction,
			pullPhase);
		appendLine(
			static_cast<Real>(0.47), static_cast<Real>(0.66), static_cast<Real>(0.50),
			static_cast<Real>(0.47), static_cast<Real>(0.74), static_cast<Real>(0.50),
			8u,
			pullFractureAction,
			holdPhase);
		return path;
	}

	template <typename Step, typename Action, typename Phase, typename DomainPointFn>
	std::vector<Step> buildDiscPullPath(
		const Eigen::AlignedBox3d& domain,
		const Action moveOnlyAction,
		const Action pullFractureAction,
		const Phase approachPhase,
		const Phase graspPhase,
		const Phase pullPhase,
		const Phase fracturePhase,
		const Phase holdPhase,
		const DomainPointFn& domainPoint)
	{
		std::vector<Step> path;
		const Real radius = static_cast<Real>(0.09);
		const Real hold = static_cast<Real>(0.08);

		auto appendLine = [&path, &domain, &domainPoint, radius, hold](
			const Real x0, const Real y0, const Real z0,
			const Real x1, const Real y1, const Real z1,
			const unsigned int steps,
			const Action action,
			const Phase phase)
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
				step.action = action;
				step.phase = phase;
				path.push_back(step);
			}
		};

		appendLine(
			static_cast<Real>(0.50), static_cast<Real>(0.32), static_cast<Real>(0.46),
			static_cast<Real>(0.50), static_cast<Real>(0.44), static_cast<Real>(0.46),
			10u,
			moveOnlyAction,
			approachPhase);
		appendLine(
			static_cast<Real>(0.50), static_cast<Real>(0.44), static_cast<Real>(0.46),
			static_cast<Real>(0.50), static_cast<Real>(0.58), static_cast<Real>(0.46),
			16u,
			pullFractureAction,
			graspPhase);
		appendLine(
			static_cast<Real>(0.50), static_cast<Real>(0.58), static_cast<Real>(0.46),
			static_cast<Real>(0.50), static_cast<Real>(0.86), static_cast<Real>(0.46),
			28u,
			pullFractureAction,
			pullPhase);
		appendLine(
			static_cast<Real>(0.50), static_cast<Real>(0.86), static_cast<Real>(0.46),
			static_cast<Real>(0.50), static_cast<Real>(0.90), static_cast<Real>(0.46),
			10u,
			pullFractureAction,
			holdPhase);
		return path;
	}
}
}

#endif
