#ifndef INCLUDED_BSPLINE_CURVE_EVALUATOR_H
#define INCLUDED_BSPLINE_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)

#include "CurveEvaluator.h"

//using namespace std;

class BSplineCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
	void pushPoints(std::vector<Point>& ptvEvaluatedCurvePts, std::vector<Point>& pts, const float& fAniLength = 0) const;
	void convertPoints(std::vector<Point>& pts, Point B0, Point B1, Point B2, Point B3) const;
};

#endif