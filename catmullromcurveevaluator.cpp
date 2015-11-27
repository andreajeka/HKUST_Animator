#include "catmullromcurveevaluator.h"
#include "point.h"
#include "vec.h"
#include "mat.h"
#include "modelerui.h"

const Mat4d M(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);
struct myclass {
	bool operator() (Point pt1, Point pt2) { return (pt1.x < pt2.x); }
} myobject;

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {

	ptvEvaluatedCurvePts.clear();
	int iCtrlPtCount = ptvCtrlPts.size();
	if (iCtrlPtCount < 3) return;
	vector<Point> pts;

	if (bWrap) {
		// first
		Point P0(ptvCtrlPts[iCtrlPtCount - 1].x - fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y);
		Point P1 = ptvCtrlPts[0];
		Point P2 = ptvCtrlPts[1];
		Point P3 = ptvCtrlPts[2];
		convertPoints(pts, P0, P1, P2, P3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// last
		pts.clear();
		P0 = ptvCtrlPts[iCtrlPtCount - 3];
		P1 = ptvCtrlPts[iCtrlPtCount - 2];
		P2 = ptvCtrlPts[iCtrlPtCount - 1];
		P3.x = ptvCtrlPts[0].x + fAniLength;
		P3.y = ptvCtrlPts[0].y;
		convertPoints(pts, P0, P1, P2, P3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		pts.clear();
		P0 = ptvCtrlPts[iCtrlPtCount - 2];
		P1 = ptvCtrlPts[iCtrlPtCount - 1];
		P2.x = ptvCtrlPts[0].x + fAniLength;
		P2.y = ptvCtrlPts[0].y;
		P3.x = ptvCtrlPts[1].x + fAniLength;
		P3.y = ptvCtrlPts[1].y;
		convertPoints(pts, P0, P1, P2, P3);
		pushPoints(ptvEvaluatedCurvePts, pts, fAniLength);

		// middle curve
		int i;
		for (i = 0; i + 3 < iCtrlPtCount; i++) {
			pts.clear();
			P0 = ptvCtrlPts[i];
			P1 = ptvCtrlPts[i + 1];
			P2 = ptvCtrlPts[i + 2];
			P3 = ptvCtrlPts[i + 3];
			convertPoints(pts, P0, P1, P2, P3);
			pushPoints(ptvEvaluatedCurvePts, pts);
		}
	}
	else {
		// first
		Point P0 = ptvCtrlPts[0];
		Point P1 = ptvCtrlPts[0];
		Point P2 = ptvCtrlPts[1];
		Point P3 = ptvCtrlPts[2];
		convertPoints(pts, P0, P1, P2, P3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// last
		pts.clear();
		P0 = ptvCtrlPts[iCtrlPtCount - 3];
		P1 = ptvCtrlPts[iCtrlPtCount - 2];
		P2 = ptvCtrlPts[iCtrlPtCount - 1];
		P3 = ptvCtrlPts[iCtrlPtCount - 1];
		convertPoints(pts, P0, P1, P2, P3);
		pushPoints(ptvEvaluatedCurvePts, pts);

		// middle curve
		int i;
		for (i = 0; i + 3 < iCtrlPtCount; i++) {
			pts.clear();
			P0 = ptvCtrlPts[i];
			P1 = ptvCtrlPts[i + 1];
			P2 = ptvCtrlPts[i + 2];
			P3 = ptvCtrlPts[i + 3];
			convertPoints(pts, P0, P1, P2, P3);
			pushPoints(ptvEvaluatedCurvePts, pts);
		}

		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y));
	}
}

void CatmullRomCurveEvaluator::pushPoints(std::vector<Point>& ptvEvaluatedCurvePts, std::vector<Point>& pts, const float& fAniLength) const {
	// contruct the P vectors 
	const Vec4d Px(pts[0].x, pts[1].x, pts[2].x, pts[3].x);
	const Vec4d Py(pts[0].y, pts[1].y, pts[2].y, pts[3].y);

	// pushing the points
	ptvEvaluatedCurvePts.push_back(pts[0]);
	ptvEvaluatedCurvePts.push_back(pts[3]);
	for (int j = 0; j < 50; j++) { // draw 50 points
		double t = j / 50.0;

		Vec4d T0(1, t - 0.01, (t - 0.01) * (t - 0.01), (t - 0.01) * (t - 0.01) * (t - 0.01));
		Point pt0(T0 * M * Px, T0 * M * Py);

		Vec4d T(1, t, t * t, t * t * t);
		Point ptOnCurve(T * M * Px, T * M * Py);

		Vec4d T1(1, t + 0.01, (t + 0.01) * (t + 0.01), (t + 0.01) * (t + 0.01) * (t + 0.01));
		Point pt1(T1 * M * Px, T1 * M * Py);

		if (fAniLength > 0.001f) {
			if (ptOnCurve.x > fAniLength) // point out of screen
				ptOnCurve.x = ptOnCurve.x - fAniLength;
			ptvEvaluatedCurvePts.push_back(ptOnCurve);
		}
		else {
			if (ptOnCurve.x < pts[3].x && ptOnCurve.x > pts[0].x && ((ptOnCurve.x - pt0.x) > 0.01) && ((pt1.x - ptOnCurve.x) > 0.01))
				ptvEvaluatedCurvePts.push_back(ptOnCurve);
		}
	}
}

void CatmullRomCurveEvaluator::convertPoints(std::vector<Point>& pts, Point P0, Point P1, Point P2, Point P3) const {
	Point V0(P1);
	Point V1(Point(P1.x + cat / 3 * (P2.x - P0.x), P1.y + cat / 3 * (P2.y - P0.y)));
	Point V2(Point(P2.x - cat / 3 * (P3.x - P1.x), P2.y - cat / 3 * (P3.y - P1.y)));
	Point V3(P2);
	pts.push_back(V0);
	pts.push_back(V1);
	pts.push_back(V2);
	pts.push_back(V3);
}