#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <vector>
#include <GL/gl.h>

class Shader {

protected:
    static const unsigned int NUM_SHADERS = 2;
    GLuint id_;
    GLuint shader_[NUM_SHADERS];

    std::string Load(const std::string& filename);
    GLuint CreateShader(const std::string& source, GLenum type);
    void ErrorCheck(
        GLuint shader, GLuint flag,
        bool is_program, const std::string& error_msg
    );
    void Compile(const std::string& filename);
    void Link();
    void Validate();
    void BindAttributes(const std::vector<std::string> attributes);

public:
    
    Shader() { };
    Shader(const std::string& filename, const std::vector<std::string> attributes); 
    virtual ~Shader();

    void Bind();
    void Unbind();
    GLint Id();
};

#endif // SHADER_H

/// @file

