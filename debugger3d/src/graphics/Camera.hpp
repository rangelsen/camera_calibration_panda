#ifndef CAMERA_H
#define CAMERA_H

#include <GL/gl.h>
#include <glm/glm.hpp>

class Camera {

public:
    Camera();

    Camera(glm::vec3 pos, float v_angle, float h_angle);

    void SetPosition(glm::vec3 pos); 

    void MoveUp(float dist);

    void MoveRight(float dist);

    void MoveForward(float dist);

    void RotateHorizontal(float angle);

    void RotateVertical(float angle);

    const glm::mat4 ViewMatrix() const;

    const glm::mat4 ProjMatrix() const;

    void UpdateViewMatrix();

    const glm::vec3 Position() const;

    void Face(glm::vec3 dir);

    /**
     * @brief Returns the normalized direction the camera is facing
     */
    const glm::vec3 Direction() const;
    
private:
    glm::vec3 pos_;
    glm::vec3 angle_;
    glm::vec3 fwd_, up_, right_;
    glm::mat4 projection_matrix_;

};

#endif // CAMERA_H

/// @file

