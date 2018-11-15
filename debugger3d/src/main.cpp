#include <iostream>
#include <stdio.h>
#include <stddef.h>
#include <cmath>

#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <pthread.h>

#include <graphics/Display.hpp>
#include <graphics/Camera.hpp>
#include <graphics/Shader.hpp>
#include <graphics/ShaderAttributes.hpp>
#include <graphics/Cube.hpp>
#include <graphics/grid.hpp>
#include <graphics/coord_frame.hpp>
#include <input/InputHandler.hpp>
#include <visualization/visualization.hpp>
#include <model/model.h>
#include <robot/robot.hpp>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_opengl3.h>

////////////////////////////////////////////////////////////////////////////////
#define RENDER_CAMER_POSES 				0
#define RENDER_ENDEFF_POSES 			0
#define RENDER_ENDEFF_TO_CHARUCO_POSES 	0
#define RENDER_BOARD_POSES 				0

////////////////////////////////////////////////////////////////////////////////
std::vector<float> split_float(std::string str, std::string delim) {

	std::vector<float> vals;
	uint32_t search_idx = 0;

	bool is_first = true;

	uint32_t n_entries = 0;

	while (n_entries < 16) {

		uint32_t start_cell;

		if (is_first) {

			start_cell = 0;
			is_first = false;
		}
		else
			start_cell = str.find(delim, search_idx) + 1;

		uint32_t end_cell = str.find(delim, start_cell + 1);
		uint32_t len = end_cell - start_cell;

		std::string cell = str.substr(start_cell, len);
		float val = std::stof(cell);

		vals.push_back(val);

		search_idx = end_cell;
		n_entries++;
	}

	return vals;
}

////////////////////////////////////////////////////////////////////////////////
glm::mat4 line_to_pose(std::string line) {


	std::vector<float> vals = split_float(line, ",");

	glm::mat4 pose(1.0f);

	for (uint8 i = 0; i < 4; i++) {

		for (uint8 j = 0; j < 4; j++) {

			pose[j][i] = vals[j + i * 4];
		}
	}

	return pose;
}

////////////////////////////////////////////////////////////////////////////////
/*
std::vector<glm::mat4> poses_from_file(const std::string& filepath) {

	std::ifstream file(filepath);
	std::string line;

	std::vector<glm::mat4> poses;

	while(std::getline(file, line)) {

		glm::mat4 pose = line_to_pose(line);
		poses.push_back(pose);
	}

	return poses;
}
*/

////////////////////////////////////////////////////////////////////////////////
void poses_from_file(const std::string& filepath,
	std::vector<glm::mat4>* poses, std::vector<int>* indices) {

	std::ifstream file(filepath);
	std::string raw_line;

	std::cout << "reading poses: " << filepath << std::endl;

	while(std::getline(file, raw_line)) {

		int i = 0;
		while (raw_line[i++] != ',') ;

		if (indices != NULL) {

			int index = std::atoi(raw_line.substr(0, i).c_str());
			indices->push_back(index);
		}

		glm::mat4 pose = line_to_pose(raw_line.substr(i, raw_line.length()));
		poses->push_back(pose);
	}
}

////////////////////////////////////////////////////////////////////////////////
void render_gui(SDL_Window* window) {

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplSDL2_NewFrame(window);
	ImGui::NewFrame();
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

///////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	Display* display = new Display();

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(100.0f);

	Camera* vcamera = new Camera(glm::vec3(0.0f, 0.5f, 0.0f), 0.0f, 0.0f);
	
	Grid* grid = grid_create(-0.05f, -0.05f, 0.1f, 0.1f);
	RenderBundle* rbundle_grid = grid_create_renderbundle(grid);

	visualization_setup(vcamera);
	visualization_submit_for_rendering(rbundle_grid);

	std::string out_path_prefix = "/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/calib-dataset5/";

	std::vector<glm::mat4> endeff_to_charuco1;
	poses_from_file(out_path_prefix + "../eTch-747612060748.csv", &endeff_to_charuco1, NULL);

	std::vector<glm::mat4> endeff_to_charuco2;
	poses_from_file(out_path_prefix + "../eTch-810512060827.csv", &endeff_to_charuco2, NULL);

	std::vector<glm::mat4> camera_poses1;
	poses_from_file(out_path_prefix + "bTc_747612060748.csv", &camera_poses1, NULL);

	std::vector<glm::mat4> camera_poses2;
	poses_from_file(out_path_prefix + "bTc_810512060827.csv", &camera_poses2, NULL);

	std::vector<std::vector<glm::mat4> > entities {

		camera_poses1,
		camera_poses2,
		endeff_to_charuco1,
		endeff_to_charuco2,
	};

	glm::mat4 tf(1.0f);
	tf = glm::rotate(tf, (float) -M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));

	for (uint32_t i = 0; i < entities.size(); i++) {

		for (uint32_t j = 0; j < entities[i].size(); j++) {

			CoordFrame* cf_entity = coordframe_create(0.005f);
			cf_entity->pose = tf * entities[i][j];
			RenderBundle* rbundle_entity = coordframe_create_renderbundle(cf_entity);
			visualization_submit_for_rendering(rbundle_entity);
		}
	}

	// Render robot base frame
	CoordFrame* cf_base = coordframe_create(0.02f);
	cf_base->pose = tf;
	cf_base->pose = glm::translate(cf_base->pose, glm::vec3(0.0f, 0.0f, 0.001f));
	RenderBundle* rbundle_base = coordframe_create_renderbundle(cf_base);
	visualization_submit_for_rendering(rbundle_base);

	while (!display->IsClosed()) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		InputHandler::HandleInputs(display, vcamera);

		visualization_render();

		display->Update();
	}

	delete vcamera;
	delete display;

    return 0;
}

/// @file

