#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <Eigen/Core>
#include <glm/gtc/matrix_transform.hpp>

#include <camerasensor.hpp>
#include <graphics/Display.hpp>
#include <graphics/Camera.hpp>
#include <graphics/coord_frame.hpp>
#include <graphics/grid.hpp>
#include <visualization/visualization.hpp>
#include <input/InputHandler.hpp>
#include <graphics/Shader.hpp>
#include <graphics/ShaderAttributes.hpp>
#include <common/util.hpp>

////////////////////////////////////////////////////////////////////////////////
typedef struct PointCloud {

	GLArray glarray;
	Shader* shader;
	glm::mat4 pose;
} PointCloud;

////////////////////////////////////////////////////////////////////////////////
void renderPointCloud(void* entity, RenderInfo* rinfo) {
	PointCloud* pc = (PointCloud*) entity;

	pc->shader->Bind();

    glm::mat4 vp = rinfo->view_proj;
    glUniformMatrix4fv(glGetUniformLocation(pc->shader->Id(), "view_proj"), 1,
		GL_FALSE, &vp[0][0]);

    glUniformMatrix4fv(glGetUniformLocation(pc->shader->Id(), "model"), 1,
		GL_FALSE, &pc->pose[0][0]);

	glBindVertexArray(pc->glarray.vao);
	glDrawArrays(GL_POINTS, 0, pc->glarray.n_vertices);
	glBindVertexArray(0);
	pc->shader->Unbind();
}

////////////////////////////////////////////////////////////////////////////////
RenderBundle* setupForRendering(PointCloud* pcloud, std::vector<Vertex>* vertices) {
	
	pcloud->glarray = glarray_create(&(*vertices)[0], vertices->size(), "p,c");

    pcloud->shader = new Shader("src/shaders/pointcloud",
        ShaderAttributes::PCLOUD_ATTRIBUTES);

	RenderBundle* rb = new RenderBundle;

	rb->entity = (void*) pcloud;
	rb->render_procedure = &renderPointCloud;
	rb->destroy_procedure = NULL;
	return rb;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<Vertex> depthImageToPointCloud(cv::Mat* image, CameraSensor* camera, glm::vec3 color) {

	std::vector<Vertex> points;

	for (int i = 0; i < image->cols; i++) {

		for (int j = 0; j < image->rows; j++) {

			float depth = camera->MeterScale() * image->at<uint16_t>(j, i);

			if (depth == 0.0f) continue;

			Eigen::Vector3f pos = camera->PixelToPoint(j, i, depth);

			Vertex vertex;
			vertex.Position.x = pos[0];
			vertex.Position.y = pos[1];
			vertex.Position.z = pos[2];
			vertex.Color = color;

			points.push_back(vertex);
		}
	}

	return points;
}

////////////////////////////////////////////////////////////////////////////////
glm::mat4 cvMatToGlm(cv::Mat* pose) {

	glm::mat4 out;

	for (uint8_t i = 0; i < 4; i++) {

		for (uint8_t j = 0; j < 4; j++)
			out[j][i] = (float) pose->at<double>(i, j);
	}

	return out;
}

////////////////////////////////////////////////////////////////////////////////
void printGlmMat(glm::mat4 mat) {

    std::cout << "glm::mat4" << std::endl;
    for (uint8_t i = 0; i < 4; i++) {

        for (uint8_t j = 0; j < 4; j++) {

            std::cout << mat[j][i] << " ";
        }

        std::cout << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	Display* display = new Display();

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(100.0f);

	Camera* vcamera = new Camera(glm::vec3(0.0f, 0.5f, 0.0f), 0.0f, 0.0f);
	
	Grid* grid = grid_create(-0.5f, -0.5f, 1.0f, 1.0f);
	RenderBundle* rbundle_grid = grid_create_renderbundle(grid);

	visualization_setup(vcamera);
	visualization_submit_for_rendering(rbundle_grid);

	glm::mat4 tf(1.0f);
	tf = glm::rotate(tf, (float) -M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));

    CoordFrame* cf_orig = coordframe_create(0.04f);
    cf_orig->pose = tf;
    cf_orig->pose = glm::translate(tf, glm::vec3(0.0f, 0.0f, 0.001f));
    RenderBundle* rbundle_cf_orig = coordframe_create_renderbundle(cf_orig);
    visualization_submit_for_rendering(rbundle_cf_orig);

	std::string res_path_prefix =
		"../res/calib-dataset5/";
		// "/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/calib-dataset5/";

    std::vector<PointCloud*> pclouds;

	CameraSensor::Initialize();
    {
        for (CameraSensor* camera : CameraSensor::connected_devices) {

            std::string serial = camera->SerialNumber();

            cv::Mat depth = cv::imread(
                res_path_prefix + "verification-custom-camera-settings/ver-images-depth-" +
                serial + "/depth0.png",
                cv::IMREAD_UNCHANGED
            );

            cv::Mat cv_pose = Util::readPosesFromFile(res_path_prefix +
                "bTc_" + serial + ".csv", NULL)[0];

            Util::printCvMat(cv_pose);
            glm::mat4 pose = cvMatToGlm(&cv_pose);

            glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);

            if (camera->SerialNumber() == "810512060827")
                color = glm::vec3(0.0f, 1.0f, 0.0f);

            std::vector<Vertex> points = depthImageToPointCloud(&depth, camera, color);

            PointCloud* pcloud = new PointCloud;
            pcloud->pose = tf * pose;
            std::cout << "pcloud pose" << std::endl;
            printGlmMat(pcloud->pose);
            RenderBundle* rbundle_pcloud = setupForRendering(pcloud, &points);
            visualization_submit_for_rendering(rbundle_pcloud);
            pclouds.push_back(pcloud);

            CoordFrame* cf = coordframe_create(0.02f);
            cf->pose = pcloud->pose;
            std::cout << "cf pose" << std::endl;
            printGlmMat(cf->pose);
            RenderBundle* rbundle_cf = coordframe_create_renderbundle(cf);
            visualization_submit_for_rendering(rbundle_cf);
        }
    }
	CameraSensor::Destroy();

	InputHandler::SetRotationSpeed(0.03f);
	InputHandler::SetMovementSpeed(0.07f);

	while (!display->IsClosed()) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		InputHandler::HandleInputs(display, vcamera);

		visualization_render();

        printf("cam pos: [%f, %f, %f]\n", vcamera->Position().x, vcamera->Position().y, vcamera->Position().z);
		display->Update();
	}

	delete vcamera;
	delete display;

	return 0;
}

/// @file

