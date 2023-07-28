#pragma once

#include <cmath>

struct float3
{
	float3() = default;
	float3(const float a, const float b, const float c) : x(a), y(b), z(c) {}
	float3(const float a) : x(a), y(a), z(a) {}
	union
	{
		struct
		{
			float x, y, z;
		};
		float cell[3];
	};
	float &operator[](const int i) { return cell[i]; }
};

static inline float3 make_float3(const float &a, const float &b, const float &c)
{
	float3 f3;
	f3.x = a, f3.y = b, f3.z = c;
	return f3;
}
static inline float3 make_float3(const float &s)
{
	return make_float3(s, s, s);
}

inline float3 operator-(const float3 &a)
{
	return make_float3(-a.x, -a.y, -a.z);
}
inline float3 operator-(const float3 &a, const float3 &b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline float3 operator+(const float3 &a, const float3 &b)
{
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline float3 operator+(const float3 &a, const float b)
{
	return make_float3(a.x + b, a.y + b, a.z + b);
}
inline float3 operator+(float a, const float3 &b)
{
	return make_float3(a + b.x, a + b.y, a + b.z);
}
inline float3 operator*(const float3 &a, float b)
{
	return make_float3(a.x * b, a.y * b, a.z * b);
}
inline float3 operator*(const float &a, const float3 &b)
{
	return make_float3(a * b.x, a * b.y, a * b.z);
}
// inline float3 operator*( const float3 &a, const float3 &b ) { return make_float3( a.x * b.x, a.y * b.y, a.z * b.z );
// }

inline float fminf(float a, float b)
{
	return a < b ? a : b;
}
inline float fmaxf(float a, float b)
{
	return a > b ? a : b;
}
inline float3 fminf(const float3 &a, const float3 &b)
{
	return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), std::fminf(a.z, b.z));
}
inline float3 fmaxf(const float3 &a, const float3 &b)
{
	return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), std::fmaxf(a.z, b.z));
}
inline float3 cross(const float3 &a, const float3 &b)
{
	return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float dot(const float3 &a, const float3 &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline uint dominantAxis(const float3 &v)
{
	float x = fabs(v.x), y = fabs(v.y), z = fabs(v.z);
	float m = std::max(std::max(x, y), z);
	return m == x ? 0 : (m == y ? 1 : 2);
}
