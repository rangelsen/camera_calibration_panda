#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <Eigen/Core>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

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

			float depth = 0.001f * image->at<uint16_t>(j, i);

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
int main(int argc, char** argv) {

	CameraSensor::Initialize();
	CameraSensor* camera = CameraSensor::connected_devices[0];

	Display* display = new Display();

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(100.0f);

	Camera* vcamera = new Camera(glm::vec3(0.0f, 0.5f, 0.0f), 0.0f, 0.0f);
	
	Grid* grid = grid_create(-0.05f, -0.05f, 0.1f, 0.1f);
	RenderBundle* rbundle_grid = grid_create_renderbundle(grid);

	visualization_setup(vcamera);
	visualization_submit_for_rendering(rbundle_grid);

	std::string res_path_prefix =
		"/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/calib-dataset5/";

	for (CameraSensor* camera : CameraSensor::connected_devices[0]) {

		std::string serial = camera->SerialNumber();

		cv::Mat depth = cv::imread(
			res_path_prefix + "verification-default-camera-settings/ver-images-depth-" +
			serial + "/depth0.png",
			cv::IMREAD_UNCHANGED
		);

		cv::Mat pose = Util::readPosesFromFile(res_path_prefix +
			"bTc_" + serial + ".csv", NULL)[0];

		Util::printCvMat(pose);
		glm::mat4 pose1 = cvMatToGlm(&pose);

		std::vector<Vertex> points = depthImageToPointCloud(&depth, camera, glm::vec3(0.0f, 1.0f, 0.0f));

		PointCloud* pcloud;
		pcloud.pose = glm::inverse(pose);
		RenderBundle* rbundle_pcloud1 = setupForRendering(&pcloud1, &points1);
		visualization_submit_for_rendering(rbundle_pcloud1);
	}

	CameraSensor::Destroy();

	InputHandler::SetRotationSpeed(0.03f);
	InputHandler::SetMovementSpeed(0.07f);

	while (!display->IsClosed()) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		InputHandler::HandleInputs(display, vcamera);

		visualization_render();

		// std::cout << glm::to_string(vcamera->Position()) << std::endl;
		display->Update();
	}

	delete vcamera;
	delete display;

	return 0;
}

/// @file

