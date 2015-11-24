#include "bsplinecurveevaluator.h"
#include "point.h"
#include "vec.h"
#include "mat.h"

const Mat4d M(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);

void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {
	ptvEvaluatedCurvePts.clear();
	int iCtrlPtCount = ptvCtrlPts.size();
	if (iCtrlPtCount < 3) return;

	vector<Point> pts;
	if (bWrap) {
		// first curve
		Point B0(ptvCtrlPts[iCtrlPtCount - 1].x - fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y);
		Point B1 = ptvCtrlPts[0];
		Point B2 = ptvCtrlPts[1];
		Point B3 = ptvCtrlPts[2];
		convertPoints(pts, B0, B1, B2, B3);
		pushPoints(ptvEvaluatedCurvePts, pts, 0);

		// last Curve
		pts.clear();
		B0 = ptvCtrlPts[iCtrlPtCount - 3];
		B1 = ptvCtrlPts[iCtrlPtCount - 2];
		B2 = ptvCtrlPts[iCtrlPtCount - 1];
		B3.x = ptvCtrlPts[0].x + fAniLength;
		B3.y = ptvCtrlPts[0].y;
		convertPoints(pts, B0, B1, B2, B3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// middle curve
		int i;
		for (i = 0; i + 3 < iCtrlPtCount; i++) {
			pts.clear();
			B0 = ptvCtrlPts[i];
			B1 = ptvCtrlPts[i + 1];
			B2 = ptvCtrlPts[i + 2];
			B3 = ptvCtrlPts[i + 3];
			convertPoints(pts, B0, B1, B2, B3);
			pushPoints(ptvEvaluatedCurvePts, pts);
		}

		pts.clear();
		B0 = ptvCtrlPts[iCtrlPtCount - 2];
		B1 = ptvCtrlPts[iCtrlPtCount - 1];
		B2.x = ptvCtrlPts[0].x + fAniLength;
		B2.y = ptvCtrlPts[0].y;
		B3.x = ptvCtrlPts[1].x + fAniLength;
		B3.y = ptvCtrlPts[1].y;
		convertPoints(pts, B0, B1, B2, B3);
		pushPoints(ptvEvaluatedCurvePts, pts, fAniLength);
	}
	else
	{
		// first curve
		Point B0 = ptvCtrlPts[0];
		Point B1 = ptvCtrlPts[0];
		Point B2 = ptvCtrlPts[1];
		Point B3 = ptvCtrlPts[2];
		convertPoints(pts, B0, B1, B2, B3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// last curve
		pts.clear();
		B0 = ptvCtrlPts[iCtrlPtCount - 3];
		B1 = ptvCtrlPts[iCtrlPtCount - 2];
		B2 = ptvCtrlPts[iCtrlPtCount - 1];
		B3 = ptvCtrlPts[iCtrlPtCount - 1];
		convertPoints(pts, B0, B1, B2, B3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// middle
		int i;
		for (i = 0; i + 3 < iCtrlPtCount; i++) {
			pts.clear();
			B0 = ptvCtrlPts[i];
			B1 = ptvCtrlPts[i + 1];
			B2 = ptvCtrlPts[i + 2];
			B3 = ptvCtrlPts[i + 3];
			convertPoints(pts, B0, B1, B2, B3);
			pushPoints(ptvEvaluatedCurvePts, pts);
		}

		// straight lines
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[0]);
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[iCtrlPtCount - 1]);
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y));
	}
}

void BSplineCurveEvaluator::pushPoints(std::vector<Point>& ptvEvaluatedCurvePts, std::vector<Point>& pts, const float& fAniLength) const {
	// contruct the P vectors 
	const Vec4d Px(pts[0].x, pts[1].x, pts[2].x, pts[3].x);
	const Vec4d Py(pts[0].y, pts[1].y, pts[2].y, pts[3].y);

	// pushing the points
	ptvEvaluatedCurvePts.push_back(pts[0]);
	ptvEvaluatedCurvePts.push_back(pts[3]);
	for (int j = 0; j < 50; j++) { // draw 50 points
		double t = j / 50.0;
		Vec4d T(1, t, t * t, t * t * t);
		Point ptOnCurve(T * M * Px, T * M * Py);

		if (fAniLength > 0.001f && ptOnCurve.x > fAniLength) // point out of screen
			ptOnCurve.x = ptOnCurve.x - fAniLength;

		ptvEvaluatedCurvePts.push_back(ptOnCurve);
	}
}

void BSplineCurveEvaluator::convertPoints(std::vector<Point>& pts, Point B0, Point B1, Point B2, Point B3) const {
	Point V0((B0.x + B1.x * 4 + B2.x) / 6, (B0.y + B1.y * 4 + B2.y) / 6);
	Point V1((4 * B1.x + 2 * B2.x) / 6, (4 * B1.y + 2 * B2.y) / 6);
	Point V2((2 * B1.x + 4 * B2.x) / 6, (2 * B1.y + 4 * B2.y) / 6);
	Point V3((B1.x + B2.x * 4 + B3.x) / 6, (B1.y + B2.y * 4 + B3.y) / 6);
	pts.push_back(V0);
	pts.push_back(V1);
	pts.push_back(V2);
	pts.push_back(V3);
}