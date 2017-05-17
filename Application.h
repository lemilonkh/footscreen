#pragma once

#include <opencv2/core/core.hpp>
#include <boost/tokenizer.hpp>
#include <XnTypes.h>

class GameClient;
class GameServer;

class DepthCamera;
class KinectMotor;
class SkeletonTracker;

class Calibration;

class Application
{
public:
	Application();
	virtual ~Application();

	void loop();

	void warpImage();
	void processFrame();
	void processSkeleton(XnUserID userId);

	void makeScreenshots();
	void clearOutputImage();
	void flipHorizontally();
	void calibrateTouch();
	cv::Point2f detectTouch();

	bool isFinished();

protected:
	GameClient *m_gameClient;
	GameServer *m_gameServer;

	DepthCamera *m_depthCamera;
	KinectMotor *m_kinectMotor;
	SkeletonTracker *m_skeletonTracker;

	Calibration *m_calibration;

	cv::Mat m_bgrImage;
	cv::Mat m_depthImage;
	cv::Mat m_outputImage;
	cv::Mat m_gameImage;
	cv::Mat m_bgrFlipImage;
	cv::Mat m_depthFlipImage;
	cv::Mat m_gameFlipImage;
	cv::Mat m_calibrationImage;

	bool m_isFinished;
	bool m_isTouchCalibrated;

	double m_groundValue;

	static const int uist_level;
	static const char *uist_server;
};
