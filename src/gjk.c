#include "gjk.h"

//-----------------------------------------------------------------------------
// Basic vector arithmetic operations

vec2 subtract (vec2 a, vec2 b) { a.x -= b.x; a.y -= b.y; return a; }
vec2 negate (vec2 v) { v.x = -v.x; v.y = -v.y; return v; }
vec2 perpendicular (vec2 v) { vec2 p = { v.y, -v.x }; return p; }
float dotProduct (vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
float lengthSquared (vec2 v) { return v.x * v.x + v.y * v.y; }

//-----------------------------------------------------------------------------
// Triple product expansion is used to calculate perpendicular normal vectors 
// which kinda 'prefer' pointing towards the Origin in Minkowski space

vec2 tripleProduct (vec2 a, vec2 b, vec2 c) {
    
    vec2 r;
    
    float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
    float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)
    
    // perform b * a.dot(c) - a * b.dot(c)
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

//-----------------------------------------------------------------------------
// This is to compute average center (roughly). It might be different from
// Center of Gravity, especially for bodies with nonuniform density,
// but this is ok as initial direction of simplex search in GJK.

vec2 averagePoint (const vec2 * vertices, size_t count) {
    vec2 avg = { 0.f, 0.f };
    for (size_t i = 0; i < count; i++) {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}

//-----------------------------------------------------------------------------
// Get furthest vertex along a certain direction

size_t indexOfFurthestPoint (const vec2 * vertices, size_t count, vec2 d) {
    
    float maxProduct = dotProduct (d, vertices[0]);
    size_t index = 0;
    for (size_t i = 0; i < count; i++) {
        float product = dotProduct (d, vertices[i]);
        if (product > maxProduct) {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

//-----------------------------------------------------------------------------
// Minkowski sum support function for GJK

vec2 support (const vec2 * vertices1, size_t count1,
              const vec2 * vertices2, size_t count2, vec2 d) {

    // get furthest point of first body along an arbitrary direction
    size_t i = indexOfFurthestPoint (vertices1, count1, d);
    
    // get furthest point of second body along the opposite direction
    size_t j = indexOfFurthestPoint (vertices2, count2, negate (d));

    // subtract (Minkowski sum) the two points to see if bodies 'overlap'
    return subtract (vertices1[i], vertices2[j]);
}

//-----------------------------------------------------------------------------
// The GJK yes/no test

int iter_count = 0;

int gjk (const vec2 * vertices1, size_t count1,
         const vec2 * vertices2, size_t count2) {
    
    size_t index = 0; // index of current vertex of simplex
    vec2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];
    
    vec2 position1 = averagePoint (vertices1, count1); // not a CoG but
    vec2 position2 = averagePoint (vertices2, count2); // it's ok for GJK )

    // initial direction from the center of 1st body to the center of 2nd body
    d = subtract (position1, position2);
    
    // if initial direction is zero – set it to any arbitrary axis (we choose X)
    if ((d.x == 0) && (d.y == 0))
        d.x = 1.f;
    
    // set the first support as initial point of the new simplex
    a = simplex[0] = support (vertices1, count1, vertices2, count2, d);
    
    if (dotProduct (a, d) <= 0)
        return 0; // no collision
    
    d = negate (a); // The next search direction is always towards the origin, so the next search direction is negate(a)
    
    while (1) {
		iter_count++;
        
        a = simplex[++index] = support (vertices1, count1, vertices2, count2, d);
        
        if (dotProduct (a, d) <= 0)
            return 0; // no collision
        
        ao = negate (a); // from point A to Origin is just negative A
        
        // simplex has 2 points (a line segment, not a triangle yet)
        if (index < 2) {
            b = simplex[0];
            ab = subtract (b, a); // from point A to B
            d = tripleProduct (ab, ao, ab); // normal to AB towards Origin
            if (lengthSquared (d) == 0)
                d = perpendicular (ab);
            continue; // skip to next iteration
        }
        
        b = simplex[1];
        c = simplex[0];
        ab = subtract (b, a); // from point A to B
        ac = subtract (c, a); // from point A to C
        
        acperp = tripleProduct (ab, ac, ac);
        
        if (dotProduct (acperp, ao) >= 0) {
            
            d = acperp; // new direction is normal to AC towards Origin
            
        } else {
            
            abperp = tripleProduct (ac, ab, ab);
            
            if (dotProduct (abperp, ao) < 0)
                return 1; // collision
            
            simplex[0] = simplex[1]; // swap first element (point C)

            d = abperp; // new direction is normal to AB towards Origin
        }
        
        simplex[1] = simplex[2]; // swap element in the middle (point B)
        --index;
    }
    
    return 0;
}

//-----------------------------------------------------------------------------
float Perturbation()
{
	return ((float)rand() / (float)RAND_MAX) * FLT_EPSILON * 100.0f * ((rand() % 2) ? 1.0f : -1.0f);
}

vec2 Jostle(vec2 a)
{
	vec2 b;
	b.x = a.x + Perturbation();
	b.y = a.y + Perturbation();
	return b;
}

int collisionDetection(const vec2 * vertices1,
                       const vec2 * vertices2) {

    // test case from dyn4j

//    vec2 vertices1[] = {
//        { 6.0f, 14.0f },
//        { 4.0f, 0.0f },
//        { 12.0f, 0.0f },
//        { 10.0f, 14.0f },
//    };
//
//    vec2 vertices2[] = {
//        { 11.0f, 11.0f },
//        { 11.0f, 8.0f },
//        { 15.0f, 8.0f },
//        { 15.0f, 11.0f },
//    };

    size_t count1 = sizeof (vertices1) / sizeof (vec2); // == 3
    size_t count2 = sizeof (vertices2) / sizeof (vec2); // == 4

    vec2 a[sizeof (vertices1) / sizeof (vec2)];
    vec2 b[sizeof (vertices2) / sizeof (vec2)];

    for (size_t i = 0; i < count1; ++i) a[i] = Jostle(vertices1[i]);
    for (size_t i = 0; i < count2; ++i) b[i] = Jostle(vertices2[i]);

    int collisionDetected = gjk (a, count1, b, count2);
    if (!collisionDetected)
    {
        printf("Found failing case:\n\t{%f, %f}, {%f, %f}, {%f, %f}, {%f, %f}\n\t{%f, %f}, {%f, %f}, {%f, %f}, {%f, %f}\n\n",
//            a[0].x, a[0].y, a[1].x, a[1].y, a[2].x, a[2].y, a[3].x, a[3].y,
//            b[0].x, b[0].y, b[1].x, b[1].y, b[2].x, b[2].y, b[3].x, b[3].y
              vertices1[0].x, vertices1[0].y, vertices1[1].x, vertices1[1].y, vertices1[2].x, vertices1[2].y, vertices1[3].x, vertices1[3].y,
              vertices2[0].x, vertices2[0].y, vertices2[1].x, vertices2[1].y, vertices2[2].x, vertices2[2].y, vertices2[3].x, vertices2[3].y
        );
    }
    else
    {
        printf("Collision correctly detected\n");
    }

    return collisionDetected;
}

bool collisionDetection2(const vec2 * vertices1, const vec2 * vertices2)
{
    // only if vertices2 is a rectangle
    int left = vertices2[0].x;
    int right = vertices2[2].x;
    int top = vertices2[0].y;
    int bot = vertices2[1].y;

    //
    int x0 = vertices1[0].x;
    int y0 = vertices1[0].y;
    int x1 = vertices1[1].x;
    int y1 = vertices1[1].y;
    int x2 = vertices1[3].x;
    int y2 = vertices1[3].y;
    int x3 = vertices1[2].x;
    int y3 = vertices1[2].y;

    int dx_left =  abs (x1 - x0), sx_left = x0 < x1 ? 1 : -1;
    int dy_left = -abs (y1 - y0), sy_left = y0 < y1 ? 1 : -1;
    int dx_right =  abs (x3 - x2), sx_right = x2 < x3 ? 1 : -1;
    int dy_right = -abs (y3 - y2), sy_right = y2 < y3 ? 1 : -1;
    int err_left = dx_left + dy_left, e2_left; /* error value e_xy */
    int err_right = dx_right + dy_right, e2_right; /* error value e_xy */

    for (;;){  /* loop */
        for (int x = x0; x <= x2; x++){
            if ((left < x) && (x < right) & (top < y0) && (y0 < bot))
                return true;
        }

        if (x0 == x1 && y0 == y1) break;
        if (x2 == x3 && y2 == y3) break;

        e2_left = 2 * err_left;
        if (e2_left >= dy_left) { err_left += dy_left; x0 += sx_left; } /* e_xy+e_x > 0 */
        if (e2_left <= dx_left) { err_left += dx_left; y0 += sy_left; } /* e_xy+e_y < 0 */

        e2_right = 2 * err_right;
        if (e2_right >= dy_right) { err_right += dy_right; x2 += sx_right; } /* e_xy+e_x > 0 */
        if (e2_right <= dx_right) { err_right += dx_right; y2 += sy_right; } /* e_xy+e_y < 0 */
    }

    return false;
}
