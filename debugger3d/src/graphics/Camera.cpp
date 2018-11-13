#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <math.h>

#include "Camera.hpp"

#define PI 3.14159265f
#define PI_HALF PI / 2.0f
#define TWO_PI 2.0f * PI

////////////////////////////////////////////////////////////////////////////////
Camera::Camera() {

    pos_      = glm::vec3(0.0f, 0.0f, 0.0f);
    angle_    = glm::vec3(0.0f, 0.0f, 0.0f);

    // Sets up the camera axes (should not be tampered with)
    fwd_      = glm::vec3(0.0f, 0.0f, -1.0f);
    up_       = glm::vec3(0.0f, 1.0f, 0.0f);
    right_    = glm::vec3(1.0f, 0.0f, 0.0f);
    projection_matrix_ = glm::perspective(glm::radians(45.0f), 4.0f/3.0f,
                                          0.05f, 200.0f); 
}

////////////////////////////////////////////////////////////////////////////////
Camera::Camera(glm::vec3 pos, float v_angle, float h_angle) : Camera() {

    SetPosition(pos);
    RotateVertical(v_angle);
    RotateHorizontal(h_angle);
}

////////////////////////////////////////////////////////////////////////////////
const glm::mat4 Camera::ViewMatrix() const {

    return glm::lookAt(pos_, pos_ + fwd_, up_);
}

////////////////////////////////////////////////////////////////////////////////
const glm::mat4 Camera::ProjMatrix() const {

    return projection_matrix_;
}

////////////////////////////////////////////////////////////////////////////////
void Camera::SetPosition(glm::vec3 pos) {

    pos_ = pos;
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::MoveUp(float dist) {

    pos_ += up_ * dist;
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::MoveRight(float dist) {

    pos_ += right_ * dist;
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::MoveForward(float dist) {

    pos_ += fwd_ * dist;
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::RotateHorizontal(float angle) {

    angle_[0] = glm::clamp(angle_[0] + angle, -PI_HALF, PI_HALF);
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::RotateVertical(float angle) {

    angle_[1] = glm::mod(angle_[1] + angle, TWO_PI);
    UpdateViewMatrix();
}

////////////////////////////////////////////////////////////////////////////////
void Camera::UpdateViewMatrix() {

    float cp = cosf(angle_[0]);
    float ct = cosf(angle_[1]);
    float sp = sinf(angle_[0]);
    float st = sinf(angle_[1]);

    fwd_   = glm::vec3(-st * cp, sp, -ct * cp);
    right_ = glm::vec3(ct, 0.0f, -st);
    up_    = glm::cross(right_, fwd_);
}

////////////////////////////////////////////////////////////////////////////////
void Camera::Face(glm::vec3 dir) {

    fwd_ = glm::normalize(dir);
}

////////////////////////////////////////////////////////////////////////////////
const glm::vec3 Camera::Position() const {

    return pos_;
}

////////////////////////////////////////////////////////////////////////////////
const glm::vec3 Camera::Direction() const {

    return glm::normalize(fwd_);
}

/// @file

