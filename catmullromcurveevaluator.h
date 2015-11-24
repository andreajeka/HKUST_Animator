#ifndef INCLUDED_CATMULLROM_CURVE_EVALUATOR_H
#define INCLUDED_CATMULLROM_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  

#include "CurveEvaluator.h"

//using namespace std;

class CatmullRomCurveEvaluator : public CurveEvaluator {
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
	void pushPoints(std::vector<Point>& ptvEvaluatedCurvePts, std::vector<Point>& pts, const float& fAniLength = 0) const;
	void convertPoints(std::vector<Point>& pts, Point P0, Point P1, Point P2, Point P3) const;
};

#endif