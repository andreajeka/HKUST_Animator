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
	int iCtrlPtCount = ptvCtrlPts.size();
	vector<Point> control = ptvCtrlPts;

	int i;
	for (i = 0; i + 3 < control.size(); i += 3) {
		vector<Point> pts;
		pts.push_back(ptvCtrlPts[i]);
		pts.push_back(ptvCtrlPts[i + 1]);
		pts.push_back(ptvCtrlPts[i + 2]);
		pts.push_back(ptvCtrlPts[i + 3]);

		pushPoints(ptvEvaluatedCurvePts, pts, fAniLength);
	}

	if (bWrap && (iCtrlPtCount - i) == 3) { // 3k control points and remaining 3 points
		Point p1, p2, p3, p4;
		p1 = ptvCtrlPts[i];
		p2 = ptvCtrlPts[i + 1];
		p3 = ptvCtrlPts[i + 2];
		p4 = ptvCtrlPts[0];
		p4.x = p4.x + fAniLength;
		vector<Point> pts;
		pts.push_back(p1);
		pts.push_back(p2);
		pts.push_back(p3);
		pts.push_back(p4);

		pushPoints(ptvEvaluatedCurvePts, pts, fAniLength);
	}
	else {
		float x = 0.0;
		float y1;
		// push the remaining points directly
		for (; i < control.size(); i++)
			ptvEvaluatedCurvePts.push_back(control[i]);

		if (bWrap) {
			if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) { // if first and last are equal in x
				y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
			ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) / (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
			}
			else
				y1 = ptvCtrlPts[0].y;
		}
		else {
			y1 = ptvCtrlPts[0].y;
		}

		ptvEvaluatedCurvePts.push_back(Point(x, y1));

		/// set the endpoint based on the wrap flag.
		float y2;
		x = fAniLength;
		if (bWrap)
			y2 = y1;
		else
			y2 = ptvCtrlPts[iCtrlPtCount - 1].y;

		ptvEvaluatedCurvePts.push_back(Point(x, y2));
	}
}

void BezierCurveEvaluator::pushPoints(std::vector<Point>& ptvEvaluatedCurvePts, std::vector<Point>& pts, const float& fAniLength) const {
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

		if (ptOnCurve.x > fAniLength) // point out of screen
			ptOnCurve.x = ptOnCurve.x - fAniLength;

		ptvEvaluatedCurvePts.push_back(ptOnCurve);
	}
}