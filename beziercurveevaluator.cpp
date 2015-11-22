#include "beziercurveevaluator.h"
#include "point.h"
#include "vec.h"
#include "mat.h"

const Mat4d M(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	ptvEvaluatedCurvePts.clear();

	// wrapping, 3k control point
	vector<Point> control = ptvCtrlPts;
	if (bWrap) 
		control.push_back(Point(ptvCtrlPts.front().x + fAniLength, ptvCtrlPts.front().y));

	int i;
	for (int i = 0; i + 3 < control.size(); i += 3) {
		// contruct the P vectors 
		const Vec4d Px(control[i].x, control[i + 1].x, control[i + 2].x, control[i + 3].x);
		const Vec4d Py(control[i].y, control[i + 1].y, control[i + 2].y, control[i + 3].y);

		// pushing the points
		ptvEvaluatedCurvePts.push_back(control[i]);
		ptvEvaluatedCurvePts.push_back(control[i + 3]);
		for (int j = 0; j < 50; j++) {
			double t = j / 50.0;
			Vec4d T(1, t, t * t, t * t * t);
			Point ptOnCurve(T * M * Px, T * M * Py);

			if (bWrap && ptOnCurve.x > fAniLength) // point out of screen
				ptOnCurve.x = ptOnCurve.x - fAniLength;

			ptvEvaluatedCurvePts.push_back(ptOnCurve);
		}
	}

	// push the remaining points directly
	for (; i < control.size(); i++) { 
		ptvEvaluatedCurvePts.push_back(control[i]);
	}

	if (!bWrap) {
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts.front().y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts.back().y));
	}
	else { // linear wrap
		const float side_length = ptvCtrlPts.front().x + fAniLength - ptvCtrlPts.back().x;
		const float back_ratio = 1.0f - ptvCtrlPts.front().x / side_length;
		const float y = ptvCtrlPts.back().y + (ptvCtrlPts.front().y - ptvCtrlPts.back().y) * back_ratio;
		ptvEvaluatedCurvePts.push_back(Point(0.0f, y));
	}
}