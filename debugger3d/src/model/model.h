#ifndef MODEL_H
#define MODEL_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

#include "model/mesh.h"

#include <graphics/texture.h>

using namespace std;

class Model {

public:
    vector<Texture> textures_loaded;
    vector<Mesh> meshes;
    string directory;
    bool gammaCorrection;
	glm::mat4 transform;

    Model(string const &path, bool gamma = false) : gammaCorrection(gamma) {

        loadModel(path);
    }


    void Draw(Shader* shader, Camera* camera, std::vector<glm::mat4>* kinematic_chain) {

		glm::vec3 view_pos = camera->Position();
		glm::mat4 vp = camera->ProjMatrix() * camera->ViewMatrix();

		shader->Bind();

		glUniformMatrix4fv(glGetUniformLocation(shader->Id(), "view_proj"), 1, GL_FALSE, &vp[0][0]);
		glUniform3f(glGetUniformLocation(shader->Id(), "view_pos"), view_pos.x, view_pos.y, view_pos.z);

		glm::mat4 chain_tf = this->transform;


        for(unsigned int ilink = 0; ilink < meshes.size(); ilink++) {

			if (ilink != meshes.size() - 1)
				chain_tf = chain_tf * meshes[ilink].transform;

			if (ilink > 0 && ilink < 7)
				chain_tf = chain_tf * (*kinematic_chain)[ilink - 1];

			meshes[ilink].Draw(shader, chain_tf);
		}

		shader->Unbind();
    }
    
private:
    void loadModel(string const &path) {

        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);

        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
            return;
        }

        directory = path.substr(0, path.find_last_of('/'));

		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < 4; j++) {

				this->transform[j][i] = scene->mRootNode->mTransformation[i][j];
			}
		}

        processNode(scene->mRootNode, scene, glm::mat4(1.0f));
    }

    void processNode(aiNode *node, const aiScene *scene, glm::mat4 parent_tf) {

		glm::mat4 node_tf(1.0f);

		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < 4; j++)
				node_tf[j][i] = node->mTransformation[i][j];
		}

        for(unsigned int i = 0; i < node->mNumMeshes; i++) {

            aiMesh* ai_mesh = scene->mMeshes[node->mMeshes[i]];
			Mesh mesh = processMesh(ai_mesh, scene);

			mesh.transform = node_tf;
            meshes.push_back(mesh);
        }

        for(unsigned int i = 0; i < node->mNumChildren; i++)
			processNode(node->mChildren[i], scene, glm::mat4(1.0f));
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene) {

        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        for(unsigned int i = 0; i < mesh->mNumVertices; i++) {

            Vertex vertex;
            glm::vec3 vector;

            // positions
            vector.x = mesh->mVertices[i].x;
            vector.y = mesh->mVertices[i].y;
            vector.z = mesh->mVertices[i].z;
            vertex.Position = vector;

            // normals
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex.Normal = vector;

            // texture coordinates
            if(mesh->mTextureCoords[0]) {

                glm::vec2 vec;
                vec.x = mesh->mTextureCoords[0][i].x; 
                vec.y = mesh->mTextureCoords[0][i].y;
                vertex.TextureCoordinate = vec;
            }
            else
                vertex.TextureCoordinate = glm::vec2(0.0f, 0.0f);

            // tangent

			/*
            vector.x = mesh->mTangents[i].x;
            vector.y = mesh->mTangents[i].y;
            vector.z = mesh->mTangents[i].z;
            vertex.Tangent = vector;

            // bitangent
            vector.x = mesh->mBitangents[i].x;
            vector.y = mesh->mBitangents[i].y;
            vector.z = mesh->mBitangents[i].z;
            vertex.BiTangent = vector;
			*/
            vertices.push_back(vertex);
        }

        for(unsigned int i = 0; i < mesh->mNumFaces; i++) {

            aiFace face = mesh->mFaces[i];
            for(unsigned int j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    
        
        // 1. diffuse maps
        vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

        // 2. specular maps
        vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());

        // 3. normal maps
        std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
        textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());

        // 4. height maps
        std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
        textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
        
        return Mesh(vertices, indices, textures);
    }

    vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName) {

        vector<Texture> textures;
        for(unsigned int i = 0; i < mat->GetTextureCount(type); i++) {

            aiString str;
            mat->GetTexture(type, i, &str);
            bool skip = false;

            for(unsigned int j = 0; j < textures_loaded.size(); j++) {

                if(std::strcmp(textures_loaded[j].path, str.C_Str()) == 0) {

                    textures.push_back(textures_loaded[j]);
                    skip = true;
                    break;
                }
            }

            if(!skip) {

                Texture texture;
				string texpath = directory + "/" + string(str.C_Str());
                texture.id = texture_load(texpath.c_str());
				strcpy(texture.type, typeName.c_str());
				strcpy(texture.path, str.C_Str());
                textures.push_back(texture);
                textures_loaded.push_back(texture);
            }
        }
        return textures;
    }
};

#endif // MODEL_H

/// @file

