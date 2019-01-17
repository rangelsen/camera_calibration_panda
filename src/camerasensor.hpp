#ifndef CAMERASENSOR_HPP
#define CAMERASENSOR_HPP

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
/*
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
*/

class CameraSensor {

public:
    CameraSensor();
    
	CameraSensor(rs2::device device);

    ~CameraSensor();

	static void Initialize();

	static void Destroy();

    void AcquireColorImage(cv::Mat* image);

    const Eigen::Matrix4f Extrinsics() const;

    Eigen::Vector3f Position() const;

    Eigen::Matrix3f RotationMatrix() const;

    void SetExtrinsics(Eigen::Matrix4f extrinsics);

    const Eigen::Matrix4f InverseExtrinsics() const;

    float MeterScale() const;

    const Eigen::Vector3f PixelToPoint(int row, int col, float depth) const;

    void PointToPixel(const Eigen::Vector3f& point, int* row, int* col) const;

    /**
     * @brief Transform point in camera frame to world frame
     */
    const Eigen::Vector3f TransformToWorldFrame(
        const Eigen::Vector3f& camera_point) const;

    /**
     * @brief Transform point in world frame to camera frame
     */
    const Eigen::Vector3f TransformToCameraFrame(
        const Eigen::Vector3f& world_point) const;

    rs2::device GetDeviceByName(const std::string dev_name);

    std::string GetDeviceName(const rs2::device& dev);

    void CaptureDepth(cv::Mat* image);

    void CaptureIr(cv::Mat* image);

	void CaptureRgb(cv::Mat* image);

    void RotateX(float angle);

    void RotateY(float angle);

    void RotateZ(float angle);

	cv::Mat Intrinsics(std::string stream);

	cv::Mat DistCoeffs(std::string stream);

	static std::string GetSerialNumber(const rs2::device& dev);

	rs2::device Device();

	static std::vector<CameraSensor*> connected_devices;

	std::string SerialNumber();

	/*
	static pcl::PointCloud<pcl::PointXYZ>::Ptr PointsToCloud(
		const rs2::points& points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr CapturePointCloud();
	*/

	cv::Mat GetIntrinsics(std::string stream_type);

private:
    void ActivateStream(rs2_stream stream, rs2::config* config);

    void GetStreamProfile(rs2::device* device, rs2::sensor& sensor_out,
                          rs2::stream_profile& stream_profile_out,
                          rs2_stream stream_type);

	rs2_intrinsics GetStreamIntrinsics(rs2::pipeline_profile* profile,
		rs2_stream stream_type);

    rs2::frameset Capture();

    bool StreamIsActive(rs2_stream stream_type, rs2::pipeline_profile* profile);

    void Warmup(rs2::pipeline* pipeline, uint32_t n_frames);

	static rs2_stream StringToStreamType(std::string stream_name);

	cv::Mat GetDistortionCoeffs(std::string stream_name);

	void SetupStreams();

    uint32_t res_x_, res_y_;
    Eigen::Matrix4f extrinsics_;
    Eigen::Matrix4f inv_extrinsics_;

    rs2::pipeline pipeline_;
    rs2::pipeline_profile pipeline_profile_;
    rs2::device device_;
    rs2::config config_;
    rs2_intrinsics depth_intrinsics_;
	rs2_intrinsics ir_intrinsics_;
	std::vector<rs2_stream> active_streams_;
	std::map<rs2_stream, cv::Mat> intrinsics_;
	std::map<rs2_stream, cv::Mat> dist_coeffs_;

    float meter_scale_;

	static rs2::context ctx_;
};

#endif // CAMERASENSOR_HPP

/// @file

