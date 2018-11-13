#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <GL/glew.h>
#include <GL/gl.h>

#include "Shader.hpp"

Shader::Shader(const std::string& filename,
               const std::vector<std::string> attributes) {

    Compile(filename);
    
    BindAttributes(attributes);
    Link();
    Validate();
}

void Shader::Link() {
    glLinkProgram(id_);
    ErrorCheck(id_, GL_LINK_STATUS, true, "Error: Program linking failed: ");
}

void Shader::Validate() {
    glValidateProgram(id_);
    ErrorCheck(id_, GL_VALIDATE_STATUS, true, "Error: Program linking failed: ");
}

void Shader::Compile(const std::string& filename) {
    id_ = glCreateProgram();

    shader_[0] = CreateShader(Load(filename + ".vs"), GL_VERTEX_SHADER);
    shader_[1] = CreateShader(Load(filename + ".fs"), GL_FRAGMENT_SHADER);

    for(size_t i = 0; i < NUM_SHADERS; i++) {
        glAttachShader(id_, shader_[i]);
    }
}

Shader::~Shader() {

    for(size_t i = 0; i < NUM_SHADERS; i++) {
        glDetachShader(id_, shader_[i]);
        glDeleteShader(shader_[i]);
    }

    glDeleteProgram(id_);
}

void Shader::Bind() {
    glUseProgram(id_);
}

void Shader::Unbind() {

    glUseProgram(0);
}

GLuint Shader::CreateShader(const std::string& source, GLenum type) {
    GLuint shader = glCreateShader(type);

    if(shader == 0)
        std::cerr << "Error: Shader creation failed!" << std::endl;

    const GLchar* shaderSourceStrings[1];
    GLint shaderSourceStringLengths[1];

    shaderSourceStrings[0] = source.c_str();
    shaderSourceStringLengths[0] = source.length();

    glShaderSource(shader, 1, shaderSourceStrings, shaderSourceStringLengths);
    glCompileShader(shader);

    ErrorCheck(shader, GL_COMPILE_STATUS, false, "Error: Shader compilation failed: ");

    return shader;
}

std::string Shader::Load(const std::string& filename) {

    std::ifstream input(filename.c_str());

    std::string output;
    std::string line;

    if(input.is_open()) {
        while(std::getline(input, line)) {
            output.append(line + "\n");
        }
    }
    else
        std::cerr << "Unable to load shader: " << filename << std::endl;

    return output;
}

void Shader::ErrorCheck(GLuint shader, GLuint flag, bool is_program, const std::string& error_msg) {

    GLint success = 0;
    GLchar error[1024] = { 0 };

    if(is_program)
        glGetProgramiv(shader, flag, &success);
    else
        glGetShaderiv(shader, flag, &success);

    if(success == GL_FALSE) {
        if(is_program)
            glGetProgramInfoLog(shader, sizeof(error), NULL, error);
        else
            glGetShaderInfoLog(shader, sizeof(error), NULL, error);

        std::cerr << error_msg << ": " << error << "'" << std::endl;
    }
}

GLint Shader::Id() {
    return id_;
}

void Shader::BindAttributes(const std::vector<std::string> attributes) {

    for (int i = 0; i < (int) attributes.size(); i++) {

        glBindAttribLocation(id_, i, attributes[i].c_str());
    }
/*
    for(int i = 0; i < n_attrib; i++) {
        glBindAttribLocation(id_, i, attrib[i].c_str());
    }
*/
}

