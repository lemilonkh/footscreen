#pragma once

#include <opencv2/core/core.hpp>

#include <vector>
#include <string>

class Calibration
{
public:
	Calibration();
	~Calibration();

	void restart();
	bool hasTerminated() const;

	void loop(const cv::Mat &bgrImage, const cv::Mat &depthImage);

	void handleMouseClick(int x, int y, int flags);

	const cv::Mat &physicalToProjector() const;
	const cv::Mat &projectorToPhysical() const;

	const cv::Mat &physicalToCamera() const;
	const cv::Mat &cameraToPhysical() const;

protected:
	void calibrate(const cv::Mat &bgrImage);
	void calibrateProjector();
	void calibrateCamera(const cv::Mat &bgrImage);

	void computeHomography();

	// own (team Y3t1z)
	void logMatrices();
	std::vector<cv::Point2f>* makeRect(width, height)

	bool m_hasTerminated;

	cv::Mat m_calibrationImage;

	std::vector<std::string> m_circleNames;

	bool m_isProjectorCalibrated;
	int m_numberOfProjectorCoordinates;

	// The 4 points for calibrating the projector
	std::vector<cv::Point2f> m_projectorCoordinates;

	bool m_isCameraCalibrated;
	int m_numberOfCameraCoordinates;

	// The 4 points for calibrating the camera
	std::vector<cv::Point2f> m_cameraCoordinates;

	// matrices to convert between physical and projector space
	cv::Mat m_physicalToProjector;
	cv::Mat m_projectorToPhysical;

	// matrices to convert between physical and camera space
	cv::Mat m_physicalToCamera;
	cv::Mat m_cameraToPhysical;
};
