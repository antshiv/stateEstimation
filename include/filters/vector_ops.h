#ifndef VECTOR_OPS_H
#define VECTOR_OPS_H

#include "../types/state_types.h"

// Vector addition: result = v1 + v2
Vector3d vector_add(const Vector3d *v1, const Vector3d *v2);

// Vector subtraction: result = v1 - v2
Vector3d vector_sub(const Vector3d *v1, const Vector3d *v2);

// Dot product: result = v1 • v2
double vector_dot(const Vector3d *v1, const Vector3d *v2);

// Cross product: result = v1 × v2
Vector3d vector_cross(const Vector3d *v1, const Vector3d *v2);

// Normalize a vector
Vector3d vector_normalize(const Vector3d *v);

// Compute magnitude of a vector
double vector_magnitude(const Vector3d *v);

#endif // VECTOR_OPS_H

