#ifndef MATH2D_H
#define MATH2D_H

#include <cmath>
#include <array>

#include "utils.h"

const double EPS = 1e-9;

struct Point2D;
struct Vec2D;
struct Segement2D;

/*
 * Point in a 2D Space
 */
struct Point2D
{
	double x;
	double y;

	Point2D() : x(0.0), y(0.0) { }

	Point2D(double _x, double _y) : x(_x), y(_y) { }

	// Unary Point operators
	Point2D& operator+=(const Point2D& p) { x += p.x; y += p.y; return *this; }
	Point2D& operator-=(const Point2D& p) { x -= p.x; y -= p.y; return *this; }

	// Unary Vector operators
	Point2D& operator+=(const Vec2D& v);
	Point2D& operator-=(const Vec2D& v);

	// Scalar operators
	Point2D& operator*=(double s) { x *= s; y *= s; return *this; }
	Point2D& operator/=(double s) { x /= s; y /= s; return *this; }
	
	// Unary minus operator
	Point2D operator-() const { return Point2D(-x, -y);	}
};

// Comparison operators
inline bool operator==(const Point2D& lhs, const Point2D& rhs) {
	return ((fabs(lhs.x - rhs.x) < EPS) && (fabs(lhs.y - rhs.y) < EPS));
}

inline bool operator!=(const Point2D& lhs, const Point2D& rhs) {
	return !(lhs  == rhs);
}

// Binary Point operators
inline Point2D operator+(const Point2D& a, const Point2D& b) {
	return Point2D(a) += b;
}

inline Point2D operator-(const Point2D& a, const Point2D& b) {
	return Point2D(a) -= b;
}

// Binary Vector operators
inline Point2D operator+(const Point2D& a, const Vec2D& v) {
	return Point2D(a) += v;
}

inline Point2D operator-(const Point2D& a, const Vec2D& v) {
	return Point2D(a) -= v;
}

// Binary scalar operators
inline Point2D operator*(const Point2D& a, double s) {
	return Point2D(a) *= s;
}

inline Point2D operator*(double s, const Point2D& a) {
	return Point2D(a) *= s;
}

inline Point2D operator/(const Point2D& a, double s) {
	return Point2D(a) /= s;
}

// Utility functions
inline double dist_sq(const Point2D& lhs, const Point2D& rhs) {
	return (lhs.x - rhs.x) * (lhs.x - rhs.x)
		 + (lhs.y - rhs.y) * (lhs.y - rhs.y);
}

inline double dist(const Point2D& lhs, const Point2D& rhs) {
	return sqrt(dist_sq(lhs, rhs));
}

inline double hypot(const Point2D& lhs, const Point2D& rhs) {
	return hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Point2D lerp(const Point2D& a, const Point2D& b, double t) {
	return Point2D(
		lerp(a.x, b.x, t),
		lerp(a.y, b.y, t)
	);
}

double angle(const Point2D& a, const Point2D& o, const Point2D& b);

/*
 * Vector in a 2D Space
 */
struct Vec2D
{
	double x;
	double y;

	Vec2D() : x(0.0), y(0.0) { }

	Vec2D(double _x, double _y) : x(_x), y(_y) { }

	Vec2D(const Point2D& p) : x(p.x), y(p.y) { }

	Vec2D(const Point2D& a, const Point2D& b) : x(b.x - a.x), y(b.y - a.y) { }

	// Unary Point operators
	Vec2D& operator+=(const Vec2D& v) { x += v.x; y += v.y; return *this; }
	Vec2D& operator-=(const Vec2D& v) { x -= v.x; y -= v.y; return *this; }

	// Scalar operators
	Vec2D& operator*=(double s) { x *= s; y *= s; return *this; }
	Vec2D& operator/=(double s) { x /= s; y /= s; return *this; }

	// Unary minus operator
	Vec2D operator-() const { return Vec2D(-x, -y); }
};

// Comparison operators
inline bool operator==(const Vec2D& lhs, const Vec2D& rhs) {
	return ((fabs(lhs.x - rhs.x) < EPS) && (fabs(lhs.y - rhs.y) < EPS));
}

inline bool operator!=(const Vec2D& lhs, const Vec2D& rhs) {
	return !(lhs == rhs);
}

// Binary Vector operators
inline Vec2D operator+(const Vec2D& a, const Vec2D& b) {
	return Vec2D(a) += b;
}

inline Vec2D operator-(const Vec2D& a, const Vec2D& b) {
	return Vec2D(a) -= b;
}

// Binary scalar operators
inline Vec2D operator*(const Vec2D& a, double s) {
	return Vec2D(a) *= s;
}

inline Vec2D operator*(double s, const Vec2D& a) {
	return Vec2D(a) *= s;
}

inline Vec2D operator/(const Vec2D& a, double s) {
	return Vec2D(a) /= s;
}

// Utility functions
inline double norm_sq(const Vec2D& a) {
	return a.x * a.x + a.y * a.y;
}

inline double norm(const Vec2D& a) {
	return sqrt(norm_sq(a));
}

inline double hypot(const Vec2D& a) {
	return hypot(a.x, a.y);
}

inline double dot(const Vec2D& a, const Vec2D& b) {
	return a.x * b.x + a.y * b.y;
}

inline double cross(const Vec2D& a, const Vec2D& b) {
	return a.x * b.y - a.y * b.x;
}

inline Vec2D normalized(const Vec2D& a) {
	const double n = norm(a);
	return Vec2D(a.x / n, a.y / n);
}

inline Vec2D rotateCCW90(const Vec2D& v) {
	return Vec2D(-v.y, v.x);
}

inline Vec2D rotateCW90(const Vec2D& v) {
	return Vec2D(v.y, -v.x);
}

inline double angle(const Vec2D& oa, const Vec2D& ob) {
	return acos(dot(oa, ob) / sqrt(norm_sq(oa) * norm_sq(ob)));
}

/*
 * Segment in a 2D Space
 */
struct Segment2D
{
	Point2D a;
	Point2D b;

	Segment2D() = default;

	Segment2D(const Point2D& _a, const Point2D& _b) : a(_a), b(_b) { }
};

// Utility functions
inline double length_sq(const Segment2D& s) {
	return dist_sq(s.a, s.b);
}

inline double length(const Segment2D& s) {
	return dist(s.a, s.b);
}

inline Point2D lerp(const Segment2D& s, double t) {
	return lerp(s.a, s.b, t);
}

inline Point2D MidPoint(const Segment2D& s) {
	return Point2D(
		(s.a.x + s.b.x) / 2.0,
		(s.a.y + s.b.y) / 2.0
	);
}

template <size_t N>
std::array<Point2D, N> Subdivide(const Segment2D& s)
{
	std::array<Point2D, N> points;

	for (int n = 0; n < points.size(); n++)
	{
		const double t = double(n + 1) / (N + 1);
		points[n] = lerp(s.a, s.b, t);
	}

	return points;
}

double pointLineProjection(const Point2D& p, const Point2D& a, const Point2D& b);
double pointLineProjection(const Point2D& p, const Segment2D& s);

double pointLineSegmentProjection(const Point2D& p, const Point2D& a, const Point2D& b);
double pointLineSegmentProjection(const Point2D& p, const Segment2D& s);

double distToLine(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c);

double distToLineSegment(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c);
double distToLineSegment(const Point2D& p, const Segment2D& s, Point2D& c);

#endif // MATH2D_H