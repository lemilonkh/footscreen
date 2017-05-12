#include "SkeletonTracker.h"

#include <iostream>

#include "DepthCamera.h"
#include "DepthCameraException.h"

#include <opencv2/imgproc/imgproc.hpp>

void XN_CALLBACK_TYPE newUserCb(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->newUser(userId);
}
void XN_CALLBACK_TYPE exitUserCb(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->exitUser(userId);
}
void XN_CALLBACK_TYPE reEnterUserCb(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->reEnterUser(userId);
}
void XN_CALLBACK_TYPE lostUserCb(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->lostUser(userId);
}

void XN_CALLBACK_TYPE poseDetectedCb(xn::PoseDetectionCapability& capability, const XnChar* pose, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->poseDetected(userId, pose);
}

void XN_CALLBACK_TYPE calibrationStartCb(xn::SkeletonCapability& capability, XnUserID userId, void* cookie)
{
	((SkeletonTracker*)cookie)->calibrationStart(userId);
}
void XN_CALLBACK_TYPE calibrationEndCb(xn::SkeletonCapability& capability, XnUserID userId, XnCalibrationStatus status, void* cookie)
{
	((SkeletonTracker*)cookie)->calibrationEnd(userId, status);
}

SkeletonTracker::SkeletonTracker(DepthCamera* depthCamera)
	: m_depthCamera(depthCamera)
	, m_trackedUsers()
{	
	XnStatus status = XN_STATUS_OK;
	
	std::cout << "Create user generator...";
	xn::Query query;
	query.AddSupportedCapability(XN_CAPABILITY_SKELETON);
	query.AddSupportedCapability(XN_CAPABILITY_POSE_DETECTION);
	status = m_userGenerator.Create(m_depthCamera->m_context, &query);
	if (status != XN_STATUS_OK) throw DepthCameraException("No user generator found", status);

	status = m_userGenerator.GetSkeletonCap().GetCalibrationPose(m_calibrationPose);
	if (status != XN_STATUS_OK) throw DepthCameraException("Calibration pose could not be retrieved", status);

	status = m_userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
	if (status != XN_STATUS_OK) throw DepthCameraException("Skeleton profile could not be set", status);
	std::cout << " done." << std::endl;

	std::cout << "Register user callbacks...";
	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	status = m_userGenerator.GetSkeletonCap().RegisterToCalibrationStart(calibrationStartCb, this, hCalibrationCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("Skeleton calibration start callback could not be registered", status);
	status = m_userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(calibrationEndCb, this, hCalibrationCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("Skeleton calibration end callback could not be registered", status);

	status = m_userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(poseDetectedCb, this, hPoseCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("Pose detection callback could not be registered", status);

	status = m_userGenerator.RegisterUserCallbacks(newUserCb, lostUserCb, this, hUserCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("User generator callbacks could not be registered", status);
	status = m_userGenerator.RegisterToUserExit(exitUserCb, this, hUserCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("User exit callback could not be registered", status);
	status = m_userGenerator.RegisterToUserReEnter(reEnterUserCb, this, hUserCallbacks);
	if (status != XN_STATUS_OK) throw DepthCameraException("User reenter callback could not be registered", status);
	std::cout << " done." << std::endl;

	std::cout << "Start user generator...";
	status = m_userGenerator.StartGenerating();
	if (status != XN_STATUS_OK) throw DepthCameraException("User generator could not be started", status);
	std::cout << " done." << std::endl;
}

void SkeletonTracker::newUser(XnUserID userId)
{
	std::cout << "New user " << userId << " tracked." << std::endl;
	m_userGenerator.GetPoseDetectionCap().StartPoseDetection(m_calibrationPose, userId);
}
void SkeletonTracker::exitUser(XnUserID userId)
{
	std::cout << "Exit user " << userId << "." << std::endl;
	m_userGenerator.GetPoseDetectionCap().StopPoseDetection(userId);
	m_userGenerator.GetSkeletonCap().StopTracking(userId);
	m_trackedUsers.erase(userId);
}
void SkeletonTracker::reEnterUser(XnUserID userId)
{
	std::cout << "ReEnter user " << userId << "." << std::endl;
	if(m_userGenerator.GetSkeletonCap().IsCalibrated(userId))
	{
		m_userGenerator.GetSkeletonCap().StartTracking(userId);
		m_trackedUsers.insert(userId);
	}
	else
		m_userGenerator.GetPoseDetectionCap().StartPoseDetection(m_calibrationPose, userId);
}
void SkeletonTracker::lostUser(XnUserID userId)
{
	std::cout << "Lost user " << userId << "." << std::endl;
	m_userGenerator.GetPoseDetectionCap().StopPoseDetection(userId);
	m_userGenerator.GetSkeletonCap().StopTracking(userId);
	m_trackedUsers.erase(userId);
}

void SkeletonTracker::poseDetected(XnUserID userId, const XnChar* pose)
{
	std::cout << "Pose " << pose << " detected for user " << userId << "." << std::endl;

	m_userGenerator.GetPoseDetectionCap().StopPoseDetection(userId);
	m_userGenerator.GetSkeletonCap().RequestCalibration(userId, true);
}

void SkeletonTracker::calibrationStart(XnUserID userId)
{
	std::cout << "Calibration started for user " << userId << "." << std::endl;
}

void SkeletonTracker::calibrationEnd(XnUserID userId, XnCalibrationStatus status)
{
	if(status == XN_CALIBRATION_STATUS_OK)
	{
		std::cout << "Calibration completed for user " << userId << "." << std::endl;
		m_userGenerator.GetSkeletonCap().StartTracking(userId);
		m_trackedUsers.insert(userId);
	}
	else
	{
		std::cout << "Calibration failed for user " << userId << "." << std::endl;
		m_userGenerator.GetPoseDetectionCap().StartPoseDetection(m_calibrationPose, userId);
	}
}

void SkeletonTracker::drawSkeleton(cv::Mat &bgrImage, XnUserID userId)
{
	drawLimb(bgrImage, userId, XN_SKEL_HEAD, XN_SKEL_NECK);
	drawLimb(bgrImage, userId, XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
	drawLimb(bgrImage, userId, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
	drawLimb(bgrImage, userId, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);

	drawLimb(bgrImage, userId, XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
	drawLimb(bgrImage, userId, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
	drawLimb(bgrImage, userId, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);

	drawLimb(bgrImage, userId, XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
	drawLimb(bgrImage, userId, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);
}

void SkeletonTracker::drawLimb(cv::Mat &bgrImage, XnUserID userId, XnSkeletonJoint joint1, XnSkeletonJoint joint2)
{
	XnSkeletonJointPosition position1, position2;
	m_userGenerator.GetSkeletonCap().GetSkeletonJointPosition(userId, joint1, position1);
	m_userGenerator.GetSkeletonCap().GetSkeletonJointPosition(userId, joint2, position2);

	if (position1.fConfidence < 0.5 && position2.fConfidence < 0.5)
		return;

	cv::Point2f position1Proj = getProjectedPosition(position1);
	cv::Point2f position2Proj = getProjectedPosition(position2);

	// circles for joints
	cv::circle(bgrImage, position1Proj, 5, cv::Scalar::all(255));
	cv::circle(bgrImage, position2Proj, 5, cv::Scalar::all(255));
	cv::line(bgrImage, position1Proj, position2Proj, cv::Scalar::all(255));
}

std::set<XnUserID>& SkeletonTracker::getTrackedUsers()
{
	return m_trackedUsers;
}

xn::SkeletonCapability SkeletonTracker::getSkeletonCapability()
{
	return m_userGenerator.GetSkeletonCap();
}

cv::Point2f SkeletonTracker::getProjectedPosition(XnSkeletonJointPosition realWorldPosition)
{
	XnPoint3D point[1];
	point[0] = realWorldPosition.position;

	m_depthCamera->m_depthGenerator.ConvertRealWorldToProjective(1, point, point);
	return cv::Point2f(point[0].X, point[0].Y);
}

SkeletonTracker::~SkeletonTracker()
{}
