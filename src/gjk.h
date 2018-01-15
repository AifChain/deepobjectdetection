//  Created by Igor Kroitor on 29/12/15.

#ifndef GJK_H
#define GJK_H

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// Gilbert-Johnson-Keerthi (GJK) collision detection algorithm in 2D
// http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
// http://mollyrocket.com/849
//-----------------------------------------------------------------------------

struct _vec2 { float x; float y; };
typedef struct _vec2 vec2;

//-----------------------------------------------------------------------------
// Basic vector arithmetic operations

vec2 subtract (vec2 a, vec2 b);
vec2 negate (vec2 v);
vec2 perpendicular (vec2 v);
float dotProduct (vec2 a, vec2 b);
float lengthSquared (vec2 v);

//-----------------------------------------------------------------------------
// Triple product expansion is used to calculate perpendicular normal vectors 
// which kinda 'prefer' pointing towards the Origin in Minkowski space

vec2 tripleProduct (vec2 a, vec2 b, vec2 c);

//-----------------------------------------------------------------------------
// This is to compute average center (roughly). It might be different from
// Center of Gravity, especially for bodies with nonuniform density,
// but this is ok as initial direction of simplex search in GJK.

vec2 averagePoint (const vec2 * vertices, size_t count);

//-----------------------------------------------------------------------------
// Get furthest vertex along a certain direction

size_t indexOfFurthestPoint (const vec2 * vertices, size_t count, vec2 d);

//-----------------------------------------------------------------------------
// Minkowski sum support function for GJK

vec2 support (const vec2 * vertices1, size_t count1,
              const vec2 * vertices2, size_t count2, vec2 d);

//-----------------------------------------------------------------------------
// The GJK yes/no test

//int iter_count = 0;

int gjk (const vec2 * vertices1, size_t count1,
         const vec2 * vertices2, size_t count2);

//-----------------------------------------------------------------------------

float Perturbation();

vec2 Jostle(vec2 a);

int collisionDetection(const vec2 * vertices1, const vec2 * vertices2);
bool collisionDetection2(const vec2 * vertices1, const vec2 * vertices2);

#endif

// gjk toolkit