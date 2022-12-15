/* SPDX-License-Identifier: BSD-3-Clause
 * Parts adapted from Open Shading Language with this license:
 *
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * Modifications Copyright 2011-2022 Blender Foundation. */

#pragma once

#include <iostream>
CCL_NAMESPACE_BEGIN

/* Spherical coordinates <-> Cartesian direction. */

ccl_device float2 direction_to_spherical(float3 dir)
{
  float theta = safe_acosf(dir.z);
  float phi = atan2f(dir.x, dir.y);

  return make_float2(theta, phi);
}

ccl_device float3 spherical_to_direction(float theta, float phi)
{
  float sin_theta = sinf(theta);
  return make_float3(sin_theta * cosf(phi), sin_theta * sinf(phi), cosf(theta));
}

/* Equirectangular coordinates <-> Cartesian direction */

ccl_device float2 direction_to_equirectangular_range(float3 dir, float4 range)
{
  if (is_zero(dir))
    return zero_float2();

  float u = (atan2f(dir.y, dir.x) - range.y) / range.x;
  float v = (acosf(dir.z / len(dir)) - range.w) / range.z;

  return make_float2(u, v);
}

ccl_device float3 equirectangular_range_to_direction(float u, float v, float4 range)
{
  float phi = range.x * u + range.y;
  float theta = range.z * v + range.w;
  float sin_theta = sinf(theta);
  return make_float3(sin_theta * cosf(phi), sin_theta * sinf(phi), cosf(theta));
}

ccl_device float2 direction_to_equirectangular(float3 dir)
{
  return direction_to_equirectangular_range(dir, make_float4(-M_2PI_F, M_PI_F, -M_PI_F, M_PI_F));
}

ccl_device float3 equirectangular_to_direction(float u, float v)
{
  return equirectangular_range_to_direction(u, v, make_float4(-M_2PI_F, M_PI_F, -M_PI_F, M_PI_F));
}

/* Fisheye <-> Cartesian direction */

ccl_device float2 direction_to_fisheye(float3 dir, float fov)
{
  float r = atan2f(sqrtf(dir.y * dir.y + dir.z * dir.z), dir.x) / fov;
  float phi = atan2f(dir.z, dir.y);

  float u = r * cosf(phi) + 0.5f;
  float v = r * sinf(phi) + 0.5f;

  return make_float2(u, v);
}

ccl_device float3 fisheye_to_direction(float u, float v, float fov)
{
  u = (u - 0.5f) * 2.0f;
  v = (v - 0.5f) * 2.0f;

  float r = sqrtf(u * u + v * v);

  if (r > 1.0f)
    return zero_float3();

  float phi = safe_acosf((r != 0.0f) ? u / r : 0.0f);
  float theta = r * fov * 0.5f;

  if (v < 0.0f)
    phi = -phi;

  return make_float3(cosf(theta), -cosf(phi) * sinf(theta), sinf(phi) * sinf(theta));
}

ccl_device float2 direction_to_fisheye_equisolid(float3 dir, float lens, float width, float height)
{
  float theta = safe_acosf(dir.x);
  float r = 2.0f * lens * sinf(theta * 0.5f);
  float phi = atan2f(dir.z, dir.y);

  float u = r * cosf(phi) / width + 0.5f;
  float v = r * sinf(phi) / height + 0.5f;

  return make_float2(u, v);
}

ccl_device_inline float3
fisheye_equisolid_to_direction(float u, float v, float lens, float fov, float width, float height)
{
  u = (u - 0.5f) * width;
  v = (v - 0.5f) * height;

  float rmax = 2.0f * lens * sinf(fov * 0.25f);
  float r = sqrtf(u * u + v * v);

  if (r > rmax)
    return zero_float3();

  float phi = safe_acosf((r != 0.0f) ? u / r : 0.0f);
  float theta = 2.0f * asinf(r / (2.0f * lens));

  if (v < 0.0f)
    phi = -phi;

  return make_float3(cosf(theta), -cosf(phi) * sinf(theta), sinf(phi) * sinf(theta));
}

ccl_device_inline float3 fisheye_lens_polynomial_to_direction(
    float u, float v, float coeff0, float4 coeffs, float fov, float width, float height)
{
  u = (u - 0.5f) * width;
  v = (v - 0.5f) * height;

  float r = sqrtf(u * u + v * v);
  float r2 = r * r;
  float4 rr = make_float4(r, r2, r2 * r, r2 * r2);
  float theta = -(coeff0 + dot(coeffs, rr));

  if (fabsf(theta) > 0.5f * fov)
    return zero_float3();

  float phi = safe_acosf((r != 0.0f) ? u / r : 0.0f);

  if (v < 0.0f)
    phi = -phi;

  return make_float3(cosf(theta), -cosf(phi) * sinf(theta), sinf(phi) * sinf(theta));
}

ccl_device float2 direction_to_fisheye_lens_polynomial(
    float3 dir, float coeff0, float4 coeffs, float width, float height)
{
  float theta = -safe_acosf(dir.x);

  float r = (theta - coeff0) / coeffs.x;

  for (int i = 0; i < 20; i++) {
    float r2 = r * r;
    float4 rr = make_float4(r, r2, r2 * r, r2 * r2);
    r = (theta - (coeff0 + dot(coeffs, rr))) / coeffs.x;
  }

  float phi = atan2f(dir.z, dir.y);

  float u = r * cosf(phi) / width + 0.5f;
  float v = r * sinf(phi) / height + 0.5f;

  return make_float2(u, v);
}

    ccl_device_inline float3
fisheye_opencv_to_direction(float u, float v, float coeff0, float4 coeffs,
        float fx, float fy, float cx, float cy, float imageWidth, float imageHeight)
{
    // Supersampling for numerical solver
    int solved = 0;
    float3 solutions[4];

    for(int i = 0; i < 5; i++)
    {
        float nu = u + ((i % 2 == 0) ? -1 : 1) * (( float(i)/4.0f ) / (i % 2 == 0) ? 2.0f : 1.0f);
        float nv = v + ((i % 2 == 0) ? -1 : 1) * (( float(i)/4.0f ) / (i % 2 == 0) ? 2.0f : 1.0f);

        nu = (nu - cx) / fx;// * width;
        nv = (nv - cy) / fy;// * height;

        float thetad = sqrtf(nu * nu + nv * nv);

        bool converged = false;
        float eps = 1e-06f;

        if( thetad < -M_PI_F/2.0f) {
            thetad = -M_PI_F/2.0f;
        }

        if( thetad > M_PI_F/2.0f) {
            thetad = M_PI_F/2.0f;
        }

        float theta = thetad;

        for(int i = 0; i < 18; i++)
        {
            float theta2 = theta * theta;
            float theta4 = theta2 * theta2;
            float theta6 = theta4 * theta2;
            float theta8 = theta6 * theta2;

            float k0_theta2 = coeffs[0] * theta2;
            float k1_theta4 = coeffs[1] * theta4;
            float k2_theta6 = coeffs[2] * theta6;
            float k3_theta8 = coeffs[3] * theta8;

            float theta_fix = (theta * (1.0f + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - thetad) /
                ( 1.0f + 3.0f*k0_theta2 + 5.0f*k1_theta4 + 7.0f*k2_theta6 + 9.0f*k3_theta8);

            theta = theta - theta_fix;

            if( fabsf(theta_fix) < eps)
            {
                converged = true;
                break;
            }
        }

        bool theta_flipped = (thetad < 0.0f && theta > 0.0f) || (thetad > 0.0f && theta < 0.0f);

        if ( converged && !theta_flipped)
        {
            float scale = tanf(theta) / thetad;
            float x2 = nu*scale;
            float y2 = nv*scale;
            float r = sqrtf(x2*x2+y2*y2);

            //if (fabsf(theta) > 0.5f * fov)
            //  return zero_float3();

            float phi = safe_acosf((r != 0.0f) ? x2 / r : 0.0f);

            if (y2 < 0.0f)
                phi = -phi;

            //return make_float3(x2,y2, -1);
            //return make_float3(1.0f, x2, y2);
            if( i == 0 )
            {
                return make_float3(cosf(theta), -cosf(phi) * sinf(theta), sinf(phi) * sinf(theta));
            }
            std::cout << "Not on 0 solved" << std::endl;
            solutions[solved++] = make_float3(cosf(theta), -cosf(phi) * sinf(theta), sinf(phi) * sinf(theta));

            // If we have exactly 2 solutions and we are on step 2 we can also break out
            if( solved == 2 && i == 2)
            {
                break;
            }
        }
    }

    // If we do not have a single solution, return 0 vector
    if( solved == 0 )
    {
        return zero_float3();
    }

    float3 acc = zero_float3();

    for(int i = 0; i < solved; i++)
    {
        acc += solutions[i];
    }

    return acc / solved;
}

ccl_device float2 direction_to_fisheye_opencv(float3 dir, float coeff0, float4 coeffs,
        float width, float height)
{
    float theta = -safe_acosf(dir.x);

    float r = (theta - coeff0) / coeffs.x;

    for (int i=0; i<20; i++) {
        float r2 = r*r;
        float4 rr = make_float4(r, r2, r2*r, r2*r2);
        r = (theta - (coeff0 + dot(coeffs, rr))) / coeffs.x;
    }

    float phi = atan2f(dir.z, dir.y);

    float u = r * cosf(phi) / width + 0.5f;
    float v = r * sinf(phi) / height + 0.5f;

    return make_float2(u, v);
}


/* Omnidirectional Model <-> Cartesian direction */

ccl_device float3 omni_to_direction(float u,
        float v,
        float imageWidth,
        float imageHeight,
        float radiusPixels,
        float a0,
        float a1,
        float a2,
        float a3,
        float a4,
        float kC,
        float kD,
        float kE,
        float cx,
        float cy,
        float invDetAffine) {
    // scale coordinates and shift center
    u = u * imageWidth - cx;
    v = imageHeight * (1.f - v) - cy;

    if(radiusPixels > 0.f && u*u + v*v > radiusPixels*radiusPixels)
        return make_float3(0.f, 0.f, 0.f);

    // inverse affine transformation
    const float affine_u = invDetAffine * (kC * u - kE * v);
    const float affine_v = invDetAffine * (-kD * u + v);

    // ray z-direction
    const float rho2 = affine_u * affine_u + affine_v * affine_v;
    const float rho = sqrtf(rho2);
    const float z = a0 + a1*rho + a2*rho2 + a3*rho2*rho + a4*rho2*rho2;
    const float invnorm = 1.f / sqrtf(affine_u*affine_u + affine_v*affine_v + z*z);

    return make_float3(
            - invnorm * z,
            - invnorm * affine_u,
            - invnorm * affine_v);
}

ccl_device float2 direction_to_omni(float3 dir,
        float imageWidth,
        float imageHeight,
        float kC,
        float kD,
        float kE,
        float cx,
        float cy)
{
    // Not implemented yet.
    return make_float2(0.0f, 0.0f);
}



/* Mirror Ball <-> Cartesion direction */

ccl_device float3 mirrorball_to_direction(float u, float v)
{
    /* point on sphere */
    float3 dir;

    dir.x = 2.0f * u - 1.0f;
    dir.z = 2.0f * v - 1.0f;

    if (dir.x * dir.x + dir.z * dir.z > 1.0f)
        return zero_float3();

    dir.y = -sqrtf(max(1.0f - dir.x * dir.x - dir.z * dir.z, 0.0f));

    /* reflection */
    float3 I = make_float3(0.0f, -1.0f, 0.0f);

    return 2.0f * dot(dir, I) * dir - I;
}

ccl_device float2 direction_to_mirrorball(float3 dir)
{
    /* inverse of mirrorball_to_direction */
    dir.y -= 1.0f;

    float div = 2.0f * sqrtf(max(-0.5f * dir.y, 0.0f));
    if (div > 0.0f)
        dir /= div;

    float u = 0.5f * (dir.x + 1.0f);
    float v = 0.5f * (dir.z + 1.0f);

    return make_float2(u, v);
}

/* Single face of a equiangular cube map projection as described in
https://blog.google/products/google-ar-vr/bringing-pixels-front-and-center-vr-video/ */
ccl_device float3 equiangular_cubemap_face_to_direction(float u, float v)
{
    u = (1.0f - u);

    u = tanf(u * M_PI_2_F - M_PI_4_F);
    v = tanf(v * M_PI_2_F - M_PI_4_F);

    return make_float3(1.0f, u, v);
}

ccl_device float2 direction_to_equiangular_cubemap_face(float3 dir)
{
    float u = atan2f(dir.y, dir.x) * 2.0f / M_PI_F + 0.5f;
    float v = atan2f(dir.z, dir.x) * 2.0f / M_PI_F + 0.5f;

    u = 1.0f - u;

    return make_float2(u, v);
}

ccl_device_inline float3 panorama_to_direction(ccl_constant KernelCamera *cam, float u, float v)
{
    switch (cam->panorama_type) {
        case PANORAMA_EQUIRECTANGULAR:
            return equirectangular_range_to_direction(u, v, cam->equirectangular_range);
        case PANORAMA_EQUIANGULAR_CUBEMAP_FACE:
            return equiangular_cubemap_face_to_direction(u, v);
        case PANORAMA_MIRRORBALL:
            return mirrorball_to_direction(u, v);
        case PANORAMA_FISHEYE_EQUIDISTANT:
            return fisheye_to_direction(u, v, cam->fisheye_fov);
        case PANORAMA_FISHEYE_LENS_POLYNOMIAL:
            return fisheye_lens_polynomial_to_direction(u,
                    v,
                    cam->fisheye_lens_polynomial_bias,
                    cam->fisheye_lens_polynomial_coefficients,
                    cam->fisheye_fov,
                    cam->sensorwidth,
                    cam->sensorheight);
        case PANORAMA_FISHEYE_OPENCV:
            return fisheye_opencv_to_direction(u,
                    v,
                    cam->fisheye_lens_polynomial_bias,
                    cam->fisheye_lens_polynomial_coefficients,
                    cam->fisheye_focal_x,
                    cam->fisheye_focal_y,
                    cam->fisheye_optical_sensor_x,
                    cam->fisheye_optical_sensor_y,
                    cam->width,
                    cam->height);
        case PANORAMA_OMNIDIRECTIONAL:
            return omni_to_direction(u,
                    v,
                    cam->sensorwidth,
                    cam->sensorheight,
                    cam->omni_radius * cam->sensorheight / 2.f,
                    cam->fisheye_lens_polynomial_bias,
                    cam->fisheye_lens_polynomial_coefficients[0],
                    cam->fisheye_lens_polynomial_coefficients[1],
                    cam->fisheye_lens_polynomial_coefficients[2],
                    cam->fisheye_lens_polynomial_coefficients[3],
                    cam->omni_c,
                    cam->omni_d,
                    cam->omni_e,
                    cam->sensorwidth / 2.f + cam->omni_shift_cx,
                    cam->sensorheight / 2.f + cam->omni_shift_cy,
                    1.f /( cam->omni_c - cam->omni_d * cam->omni_e));
        case PANORAMA_FISHEYE_EQUISOLID:
        default:
            return fisheye_equisolid_to_direction(
                    u, v, cam->fisheye_lens, cam->fisheye_fov, cam->sensorwidth, cam->sensorheight);
    }
}

ccl_device_inline float2 direction_to_panorama(ccl_constant KernelCamera *cam, float3 dir)
{
    switch (cam->panorama_type) {
        case PANORAMA_EQUIRECTANGULAR:
            return direction_to_equirectangular_range(dir, cam->equirectangular_range);
        case PANORAMA_EQUIANGULAR_CUBEMAP_FACE:
            return direction_to_equiangular_cubemap_face(dir);
        case PANORAMA_MIRRORBALL:
            return direction_to_mirrorball(dir);
        case PANORAMA_FISHEYE_EQUIDISTANT:
            return direction_to_fisheye(dir, cam->fisheye_fov);
        case PANORAMA_FISHEYE_LENS_POLYNOMIAL:
            return direction_to_fisheye_lens_polynomial(dir,
                    cam->fisheye_lens_polynomial_bias,
                    cam->fisheye_lens_polynomial_coefficients,
                    cam->sensorwidth,
                    cam->sensorheight);
        case PANORAMA_FISHEYE_OPENCV:
            return direction_to_fisheye_opencv(dir,
                    cam->fisheye_lens_polynomial_bias,
                    cam->fisheye_lens_polynomial_coefficients,
                    cam->sensorwidth,
                    cam->sensorheight);
        case PANORAMA_OMNIDIRECTIONAL:
            return direction_to_omni(dir,
                    cam->sensorwidth,
                    cam->sensorheight,
                    cam->omni_c,
                    cam->omni_d,
                    cam->omni_e,
                    cam->sensorwidth / 2.f + cam->omni_shift_cx,
                    cam->sensorheight / 2.f + cam->omni_shift_cy);
        case PANORAMA_FISHEYE_EQUISOLID:
        default:
            return direction_to_fisheye_equisolid(
                    dir, cam->fisheye_lens, cam->sensorwidth, cam->sensorheight);
    }
}

ccl_device_inline void spherical_stereo_transform(ccl_constant KernelCamera *cam,
        ccl_private float3 *P,
        ccl_private float3 *D)
{
    float interocular_offset = cam->interocular_offset;

    /* Interocular offset of zero means either non stereo, or stereo without
     * spherical stereo. */
    kernel_assert(interocular_offset != 0.0f);

    if (cam->pole_merge_angle_to > 0.0f) {
        const float pole_merge_angle_from = cam->pole_merge_angle_from,
              pole_merge_angle_to = cam->pole_merge_angle_to;
        float altitude = fabsf(safe_asinf((*D).z));
        if (altitude > pole_merge_angle_to) {
            interocular_offset = 0.0f;
        }
        else if (altitude > pole_merge_angle_from) {
            float fac = (altitude - pole_merge_angle_from) /
                (pole_merge_angle_to - pole_merge_angle_from);
            float fade = cosf(fac * M_PI_2_F);
            interocular_offset *= fade;
        }
    }

    float3 up = make_float3(0.0f, 0.0f, 1.0f);
    float3 side = normalize(cross(*D, up));
    float3 stereo_offset = side * interocular_offset;

    *P += stereo_offset;

    /* Convergence distance is FLT_MAX in the case of parallel convergence mode,
     * no need to modify direction in this case either. */
    const float convergence_distance = cam->convergence_distance;

    if (convergence_distance != FLT_MAX) {
        float3 screen_offset = convergence_distance * (*D);
        *D = normalize(screen_offset - stereo_offset);
    }
}

CCL_NAMESPACE_END
