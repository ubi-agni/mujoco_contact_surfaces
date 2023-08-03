/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David Leins */

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
