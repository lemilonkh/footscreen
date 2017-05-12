#pragma once

#include <set>

#include <opencv2/core/core.hpp>

#include <XnCppWrapper.h>

class DepthCamera;

class SkeletonTracker
{
public:
	SkeletonTracker(DepthCamera *depthCamera);
	~SkeletonTracker();
	
	std::set<XnUserID>& getTrackedUsers();
	xn::SkeletonCapability getSkeletonCapability();
	cv::Point2f getProjectedPosition(XnSkeletonJointPosition realWorldPosition);

	void newUser(XnUserID userId);
	void exitUser(XnUserID userId);
	void reEnterUser(XnUserID userId);
	void lostUser(XnUserID userId);

	void poseDetected(XnUserID userId, const XnChar* pose);

	void calibrationStart(XnUserID userId);
	void calibrationEnd(XnUserID userId, XnCalibrationStatus status);

	void drawSkeleton(cv::Mat &bgrImage, XnUserID userId);

protected:
	void drawLimb(cv::Mat &bgrImage, XnUserID userId, XnSkeletonJoint joint1, XnSkeletonJoint joint2);

	DepthCamera *m_depthCamera;

	xn::UserGenerator m_userGenerator;
	XnChar m_calibrationPose[20];

	std::set<XnUserID> m_trackedUsers;
};

