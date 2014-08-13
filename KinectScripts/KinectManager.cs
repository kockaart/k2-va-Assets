using UnityEngine;
using Windows.Kinect;

using System;
using System.Collections;
using System.Collections.Generic;

public class KinectManager : MonoBehaviour 
{
//	// Bool to control whether or not to start Kinect2Server internally
//	public bool startKinectServer = true;
	
	// How high off the ground is the sensor (in meters).
	public float sensorHeight = 1.0f;

	// Public Bool to determine whether to receive and compute the user map
	public bool computeUserMap = false;
	
	// Public Bool to determine whether to receive and compute the color map
	public bool computeColorMap = false;
	
	// Public Bool to determine whether to receive and compute the infrared map
	public bool computeInfraredMap = false;
	
	// Public Bool to determine whether to display user map on the GUI
	public bool displayUserMap = false;
	
	// Public Bool to determine whether to display color map on the GUI
	public bool displayColorMap = false;
	
	// Public Bool to determine whether to display the skeleton lines on user map
	public bool displaySkeletonLines = false;
	
	// Public Floats to specify the width and height of the depth and color maps as % of the camera width and height
	// if percents are zero, they are calculated based on actual Kinect image´s width and height
	private float MapsPercentWidth = 0f;
	private float MapsPercentHeight = 0f;
	
	// Minimum user distance in order to process skeleton data
	public float minUserDistance = 0.5f;
	
	// Public Bool to determine whether to detect only the closest user or not
	public bool detectClosestUser = true;
	
	// Public Bool to determine whether to use only the tracked joints (and ignore the inferred ones)
	public bool ignoreInferredJoints = true;
	
	// Selection of smoothing parameters
	public enum Smoothing : int { None, Default, Medium, Aggressive }
	public Smoothing smoothing = Smoothing.Default;
	
	// Public Bool to determine the use of additional filters
	public bool useBoneOrientationConstraints = true;
	//public bool useBoneOrientationsFilter = false;

	// Lists of AvatarController-objects that will be controlled by Kinect users
	public List<AvatarController> avatarControllers;
	
	// Calibration pose for each player, if needed
	public KinectGestures.Gestures playerCalibrationPose;
	
	// List of Gestures to be detected for each player
	public List<KinectGestures.Gestures> playerCommonGestures;

	// Minimum time between gesture detections
	public float minTimeBetweenGestures = 0.7f;
	
	// List of Gesture Listeners. They must implement KinectGestures.GestureListenerInterface
	public List<MonoBehaviour> gestureListeners;
	
	// GUI Text to show messages.
	public GUIText calibrationText;
	
	// GUI Texture to display the hand cursor for Player1
	//public GUITexture handCursor;
	
	// Bool to specify whether Left/Right-hand-cursor and the Click-gesture control the mouse cursor and click
	//public bool controlMouseCursor = false;
	

	// Bool to keep track of whether Kinect has been initialized
	private bool kinectInitialized = false; 
	
	// The singleton instance of KinectManager
	private static KinectManager instance = null;

//	// KinectServer instance
//	private KinectServer kinectServer = null;

	// SensorData structure
	private KinectInterop.SensorData sensorData = null;

	// Depth and user maps
//	private KinectInterop.DepthBuffer depthImage;
//	private KinectInterop.BodyIndexBuffer bodyIndexImage;
//	private KinectInterop.UserHistogramBuffer userHistogramImage;
	private Color32[] usersHistogramImage;
	private ushort[] usersPrevState;
	private float[] usersHistogramMap;

	private Texture2D usersLblTex;
	private Rect usersMapRect;
	private int usersMapSize;
//	private int minDepth;
//	private int maxDepth;
	
	// Color map
	//private KinectInterop.ColorBuffer colorImage;
	private Texture2D usersClrTex;
	private Rect usersClrRect;
	private int usersClrSize;
	
	// Kinect body frame data
	private KinectInterop.BodyFrameData bodyFrame;
	//private Int64 lastBodyFrameTime = 0;
	
	// List of all users
	private List<Int64> alUserIds;
	private Dictionary<Int64, int> dictUserIdToIndex;
	
	// Primary (first or closest) user ID
	private Int64 liPrimaryUserId = 0;
	
	// Kinect to world matrix
	private Matrix4x4 kinectToWorld = Matrix4x4.zero;
	private Matrix4x4 mOrient = Matrix4x4.zero;

	// Calibration gesture data for each player
	private Dictionary<Int64, KinectGestures.GestureData> playerCalibrationData = new Dictionary<Int64, KinectGestures.GestureData>();
	
	// gestures data and parameters
	private Dictionary<Int64, List<KinectGestures.GestureData>> playerGesturesData = new Dictionary<Int64, List<KinectGestures.GestureData>>();
	private Dictionary<Int64, float> gesturesTrackingAtTime = new Dictionary<Int64, float>();
	
	// List of Gesture Listeners. They must implement KinectGestures.GestureListenerInterface
	public List<KinectGestures.GestureListenerInterface> gestureListenerInts;
	
	// Body filter instances
	private JointPositionsFilter jointPositionFilter = null;
	private BoneOrientationsConstraint boneConstraintsFilter = null;
	//private BoneOrientationsFilter boneOrientationFilter = null;

	// returns the single KinectManager instance
    public static KinectManager Instance
    {
        get
        {
            return instance;
        }
    }

	// checks if Kinect is initialized and ready to use. If not, there was an error during Kinect-sensor initialization
	public static bool IsKinectInitialized()
	{
		return instance != null ? instance.kinectInitialized : false;
	}
	
	// checks if Kinect is initialized and ready to use. If not, there was an error during Kinect-sensor initialization
	public bool IsInitialized()
	{
		return kinectInitialized;
	}

//	// returns the internal sensor data structure (for debugging purposes only)
//	// do not modify the structure contents!
//	public KinectInterop.SensorData GetSensorData()
//	{
//		return sensorData;
//	}

	// returns the raw depth/user data,if ComputeUserMap is true
	public ushort[] GetRawDepthMap()
	{
		return sensorData != null ? sensorData.depthImage : null;
	}

	// returns the raw infrared data,if ComputeInfraredMap is true
	public ushort[] GetRawInfraredMap()
	{
		return sensorData != null ? sensorData.infraredImage : null;
	}
	
	// returns the depth image/users histogram texture,if ComputeUserMap is true
    public Texture2D GetUsersLblTex()
    { 
		return usersLblTex;
	}
	
	// returns the color image texture,if ComputeColorMap is true
	public Texture2D GetUsersClrTex()
	{ 
		return usersClrTex;
	}
	
	// returns true if at least one user is currently detected by the sensor
	public bool IsUserDetected()
	{
		return kinectInitialized && (alUserIds.Count > 0);
	}
	
	// returns true if the User is calibrated and ready to use
	public bool IsUserCalibrated(Int64 userId)
	{
		return dictUserIdToIndex.ContainsKey(userId);
	}
	
	// returns the number of currently detected users
	public int GetUsersCount()
	{
		return alUserIds.Count;
	}
	
	// returns the UserID by the given index
	public Int64 GetUserIdByIndex(int i)
	{
		if(i >= 0 && i < alUserIds.Count)
		{
			return alUserIds[i];
		}
		
		return 0;
	}
	
	// returns the UserID of the primary user (the first or the closest one), if there is any
	public Int64 GetPrimaryUserID()
	{
		return liPrimaryUserId;
	}
	
	// returns the User body data, for debug purposes only
	// do not change the data in the structure directly
	public KinectInterop.BodyData GetUserBodyData(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount)
			{
				return bodyFrame.bodyData[index];
			}
		}
		
		return new KinectInterop.BodyData();
	}
	
	// returns the User position, relative to the Kinect-sensor, in meters
	public Vector3 GetUserPosition(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].position;
			}
		}
		
		return Vector3.zero;
	}
	
	// returns the User rotation, relative to the Kinect-sensor
	public Quaternion GetUserOrientation(Int64 userId, bool flip)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
			   bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(flip)
					return bodyFrame.bodyData[index].normalRotation;
				else
					return bodyFrame.bodyData[index].mirroredRotation;
			}
		}
		
		return Quaternion.identity;
	}
	
	// returns the raw tracking state of the given joint of the specified user
	public TrackingState GetJointTrackingState(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < KinectInterop.Constants.JointCount)
				{
					return  bodyFrame.bodyData[index].joint[joint].trackingState;
				}
			}
		}
		
		return TrackingState.NotTracked;
	}
	
	// returns true if the given joint of the specified user is being tracked
	public bool IsJointTracked(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < KinectInterop.Constants.JointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					
					return ignoreInferredJoints ? (jointData.trackingState == TrackingState.Tracked) : 
						(jointData.trackingState != TrackingState.NotTracked);
				}
			}
		}
		
		return false;
	}
	
	// returns the joint position of the specified user, relative to the Kinect-sensor, in meters
	public Vector3 GetJointPosition(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < KinectInterop.Constants.JointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					return jointData.position;
				}
			}
		}
		
		return Vector3.zero;
	}
	
	// returns the joint direction of the specified user, relative to the parent joint
	public Vector3 GetJointDirection(Int64 userId, int joint, bool flipX, bool flipZ)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < KinectInterop.Constants.JointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					Vector3 jointDir = jointData.direction;

					if(flipX)
						jointDir.x = -jointDir.x;
					
					if(flipZ)
						jointDir.z = -jointDir.z;
					
					return jointDir;
				}
			}
		}
		
		return Vector3.zero;
	}
	
	// returns the direction between firstJoint and secondJoint for the specified user
	public Vector3 GetDirectionBetweenJoints(Int64 userId, int firstJoint, int secondJoint, bool flipX, bool flipZ)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				
				if(firstJoint >= 0 && firstJoint < KinectInterop.Constants.JointCount &&
					secondJoint >= 0 && secondJoint < KinectInterop.Constants.JointCount)
				{
					Vector3 firstJointPos = bodyData.joint[firstJoint].position;
					Vector3 secondJointPos = bodyData.joint[secondJoint].position;
					Vector3 jointDir = secondJointPos - firstJointPos;

					if(flipX)
						jointDir.x = -jointDir.x;
					
					if(flipZ)
						jointDir.z = -jointDir.z;
					
					return jointDir;
				}
			}
		}
		
		return Vector3.zero;
	}
	
	// returns the joint rotation of the specified user, relative to the Kinect-sensor
	public Quaternion GetJointOrientation(Int64 userId, int joint, bool flip)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
			   bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(flip)
					return bodyFrame.bodyData[index].joint[joint].normalRotation;
				else
					return bodyFrame.bodyData[index].joint[joint].mirroredRotation;
//				if((joint >= (int)JointType.SpineBase && joint <= (int)JointType.Neck) ||
//					(joint == (int)JointType.SpineShoulder))
//				{
//					// hip center is special case - use the sdk orientation
//					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
//					return ConvertMatrixToQuat(jointData.orientation, joint, flip);
//				}
//				else
//				{
//					int nextJoint = (int)KinectInterop.GetNextJoint((JointType)joint);
//
//					if(nextJoint != joint && nextJoint >= 0 && nextJoint < KinectInterop.Constants.JointCount &&
//					   IsJointTracked(userId, joint) && IsJointTracked(userId, nextJoint))
//					{
//						//KinectInterop.Joint jointData = bodyFrame.bodyData[index].joint[joint];
//						//return ConvertMatrixToQuat(jointData.orientation, joint, flip);
//						KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[nextJoint];
//
//						Vector3 jointDir = jointData.direction;
//						jointDir.z = -jointDir.z;
//	
//						if(!flip)
//						{
//							jointDir.x = -jointDir.x;
//							jointDir.y = -jointDir.y;
//						}
//						
//						Vector3 baseDir = KinectInterop.JointBaseDir[nextJoint];
//						if(!flip)
//						{
//							baseDir.x = -baseDir.x;
//							baseDir.y = -baseDir.y;
//						}
//						
//						Quaternion rotation = Quaternion.FromToRotation(baseDir, jointDir);
//						
//						return rotation;
//					}
//				}
			}
		}
		
		return Quaternion.identity;
	}
	
	// checks if the left hand confidence for a user is high
	// returns true if the confidence is high, false if it is low or user is not found
	public bool IsLeftHandConfidenceHigh(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return (bodyFrame.bodyData[index].leftHandConfidence == TrackingConfidence.High);
			}
		}
		
		return false;
	}
	
	// checks if the right hand confidence for a user is high
	// returns true if the confidence is high, false if it is low or user is not found
	public bool IsRightHandConfidenceHigh(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return (bodyFrame.bodyData[index].rightHandConfidence == TrackingConfidence.High);
			}
		}
		
		return false;
	}
	
	// returns the left hand state for a user
	public HandState GetLeftHandState(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].leftHandState;
			}
		}
		
		return HandState.NotTracked;
	}
	
	// returns the right hand state for a user
	public HandState GetRightHandState(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].rightHandState;
			}
		}
		
		return HandState.NotTracked;
	}
	
	// returns the interaction box for the left hand of the specified user, in meters
	public bool GetLeftHandInteractionBox(Int64 userId, ref Vector3 leftBotBack, ref Vector3 rightTopFront, bool bValidBox)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				bool bResult = true;
				
				if(bodyData.joint[(int)JointType.ShoulderRight].trackingState == TrackingState.Tracked &&
					bodyData.joint[(int)JointType.HipLeft].trackingState == TrackingState.Tracked)
				{
					rightTopFront.x = bodyData.joint[(int)JointType.ShoulderRight].position.x;
					leftBotBack.x = rightTopFront.x - 2 * (rightTopFront.x - bodyData.joint[(int)JointType.HipLeft].position.x);
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)JointType.HipRight].trackingState == TrackingState.Tracked &&
					bodyData.joint[(int)JointType.ShoulderRight].trackingState == TrackingState.Tracked)
				{
					leftBotBack.y = bodyData.joint[(int)JointType.HipRight].position.y;
					rightTopFront.y = bodyData.joint[(int)JointType.ShoulderRight].position.y;
					
					float fDelta = (rightTopFront.y - leftBotBack.y) * 2 / 3;
					leftBotBack.y += fDelta;
					rightTopFront.y += fDelta;
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)JointType.SpineBase].trackingState == TrackingState.Tracked)
				{
					leftBotBack.z = bodyData.joint[(int)JointType.SpineBase].position.z;
					rightTopFront.z = leftBotBack.z - 0.5f;
				}
				else
				{
					bResult = bValidBox;
				}
				
				return bResult;
			}
		}
		
		return false;
	}
	
	// returns the interaction box for the right hand of the specified user, in meters
	public bool GetRightHandInteractionBox(Int64 userId, ref Vector3 leftBotBack, ref Vector3 rightTopFront, bool bValidBox)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < KinectInterop.Constants.BodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				bool bResult = true;
				
				if(bodyData.joint[(int)JointType.ShoulderLeft].trackingState == TrackingState.Tracked &&
					bodyData.joint[(int)JointType.HipRight].trackingState == TrackingState.Tracked)
				{
					leftBotBack.x = bodyData.joint[(int)JointType.ShoulderLeft].position.x;
					rightTopFront.x = leftBotBack.x + 2 * (bodyData.joint[(int)JointType.HipRight].position.x - leftBotBack.x);
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)JointType.HipLeft].trackingState == TrackingState.Tracked &&
					bodyData.joint[(int)JointType.ShoulderLeft].trackingState == TrackingState.Tracked)
				{
					leftBotBack.y = bodyData.joint[(int)JointType.HipLeft].position.y;
					rightTopFront.y = bodyData.joint[(int)JointType.ShoulderLeft].position.y;
					
					float fDelta = (rightTopFront.y - leftBotBack.y) * 2 / 3;
					leftBotBack.y += fDelta;
					rightTopFront.y += fDelta;
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)JointType.SpineBase].trackingState == TrackingState.Tracked)
				{
					leftBotBack.z = bodyData.joint[(int)JointType.SpineBase].position.z;
					rightTopFront.z = leftBotBack.z - 0.5f;
				}
				else
				{
					bResult = bValidBox;
				}
				
				return bResult;
			}
		}
		
		return false;
	}
	
	// removes the currently detected kinect users, allowing a new detection/calibration process to start
	public void ClearKinectUsers()
	{
		if(!kinectInitialized)
			return;

		// remove current users
		for(int i = alUserIds.Count - 1; i >= 0; i--)
		{
			Int64 userId = alUserIds[i];
			RemoveUser(userId);
		}
		
		ResetFilters();
	}

	// resets data filters
	public void ResetFilters()
	{
		if(jointPositionFilter != null)
		{
			jointPositionFilter.Reset();
		}
	}
	
	// adds a gesture to the list of detected gestures for the specified user
	public void DetectGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : new List<KinectGestures.GestureData>();
		int index = GetGestureIndex(gesture, ref gesturesData);

		if(index >= 0)
		{
			DeleteGesture(UserId, gesture);
		}
		
		KinectGestures.GestureData gestureData = new KinectGestures.GestureData();
		
		gestureData.userId = UserId;
		gestureData.gesture = gesture;
		gestureData.state = 0;
		gestureData.joint = 0;
		gestureData.progress = 0f;
		gestureData.complete = false;
		gestureData.cancelled = false;
		
		gestureData.checkForGestures = new List<KinectGestures.Gestures>();
		switch(gesture)
		{
		case KinectGestures.Gestures.ZoomIn:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomOut);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.Wheel);			
			break;
			
		case KinectGestures.Gestures.ZoomOut:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomIn);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.Wheel);			
			break;
			
		case KinectGestures.Gestures.Wheel:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomIn);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomOut);			
			break;
		}

		gesturesData.Add(gestureData);
		playerGesturesData[UserId] = gesturesData;
		
		if(!gesturesTrackingAtTime.ContainsKey(UserId))
		{
			gesturesTrackingAtTime[UserId] = 0f;
		}
	}
	
	// resets the gesture-data state for the given gesture of the specified user
	public bool ResetGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;
		if(index < 0)
			return false;
		
		KinectGestures.GestureData gestureData = gesturesData[index];
		
		gestureData.state = 0;
		gestureData.joint = 0;
		gestureData.progress = 0f;
		gestureData.complete = false;
		gestureData.cancelled = false;
		gestureData.startTrackingAtTime = Time.realtimeSinceStartup + KinectInterop.Constants.MinTimeBetweenSameGestures;

		gesturesData[index] = gestureData;
		playerGesturesData[UserId] = gesturesData;

		return true;
	}
	
	// resets the gesture-data states for all detected gestures of the specified user
	public void ResetPlayerGestures(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			int listSize = gesturesData.Count;
			
			for(int i = 0; i < listSize; i++)
			{
				ResetGesture(UserId, gesturesData[i].gesture);
			}
		}
	}
	
	// deletes the given gesture from the list of detected gestures for the specified user
	public bool DeleteGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;
		if(index < 0)
			return false;
		
		gesturesData.RemoveAt(index);
		playerGesturesData[UserId] = gesturesData;

		return true;
	}
	
	// clears detected gestures list for the specified user
	public void ClearGestures(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			gesturesData.Clear();
			playerGesturesData[UserId] = gesturesData;
		}
	}
	
	// returns the count of detected gestures in the list of detected gestures for the specified user
	public int GetGesturesCount(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			return gesturesData.Count;
		}

		return 0;
	}
	
	// returns the list of detected gestures for the specified user
	public List<KinectGestures.Gestures> GetGesturesList(Int64 UserId)
	{
		List<KinectGestures.Gestures> list = new List<KinectGestures.Gestures>();
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			foreach(KinectGestures.GestureData data in gesturesData)
				list.Add(data.gesture);
		}

		return list;
	}
	
	// returns true, if the given gesture is in the list of detected gestures for the specified user
	public bool IsGestureDetected(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		return index >= 0;
	}
	
	// returns true, if the given gesture for the specified user is complete
	public bool IsGestureComplete(Int64 UserId, KinectGestures.Gestures gesture, bool bResetOnComplete)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			
			if(bResetOnComplete && gestureData.complete)
			{
				ResetPlayerGestures(UserId);
				return true;
			}
			
			return gestureData.complete;
		}
		
		return false;
	}
	
	// returns true, if the given gesture for the specified user is cancelled
	public bool IsGestureCancelled(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.cancelled;
		}
		
		return false;
	}
	
	// returns the progress in range [0, 1] of the given gesture for the specified user
	public float GetGestureProgress(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.progress;
		}
		
		return 0f;
	}
	
	// returns the current "screen position" of the given gesture for the specified user
	public Vector3 GetGestureScreenPos(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.screenPos;
		}
		
		return Vector3.zero;
	}
	
	// KinectManager's Internal Methods
	
	void Awake()
	{
		try
		{
			if(KinectInterop.EnsureKinectWrapperPresence())
			{
				// reload the same level
				Application.LoadLevel(Application.loadedLevel);
			}
		} 
		catch (Exception ex) 
		{
			Debug.LogError(ex.ToString());
			
			if(calibrationText != null)
			{
				calibrationText.guiText.text = ex.Message;
			}
		}
	}

	void Start() 
	{
//		int hr = 0;
		
		try
		{
//			if(startKinectServer)
//			{
//				// start the Kinect-server app
//				hr = StartKinectServer();
//	            if (hr != 0)
//				{
//	            	throw new Exception("Kinect2Server not started");
//				}
//			}
			
			// try to initialize the default Kinect2 sensor
			KinectInterop.FrameSource dwFlags = KinectInterop.FrameSource.TypeBody;
			if(computeUserMap)
				dwFlags |= KinectInterop.FrameSource.TypeDepth | KinectInterop.FrameSource.TypeBodyIndex;
			if(computeColorMap)
				dwFlags |= KinectInterop.FrameSource.TypeColor;
			if(computeInfraredMap)
				dwFlags |= KinectInterop.FrameSource.TypeInfrared;

//			hr = KinectInterop.InitDefaultKinectSensor(dwFlags, KinectInterop.Constants.ColorImageWidth, KinectInterop.Constants.ColorImageHeight);
//            if (hr != 0)
//			{
//            	throw new Exception("InitDefaultKinectSensor failed");
//			}

			// open the default kinect sensor
			sensorData = KinectInterop.OpenDefaultKinectSensor(dwFlags);
			if (sensorData == null)
			{
				throw new Exception("OpenDefaultKinectSensor failed");
			}

			// transform matrix - kinect to world
			kinectToWorld.SetTRS(new Vector3(0.0f, sensorHeight, 0.0f), Quaternion.identity, Vector3.one);
		}
		catch(DllNotFoundException ex)
		{
			string message = ex.Message + " cannot be loaded. Please check the Kinect SDK installation.";
			
			Debug.LogError(message);
			Debug.LogException(ex);
			
			if(calibrationText != null)
			{
				calibrationText.guiText.text = message;
			}
			
			return;
		}
		catch(Exception ex)
		{
			string message = ex.Message;

			Debug.LogError(message);
			Debug.LogException(ex);
			
			if(calibrationText != null)
			{
				calibrationText.guiText.text = message;
			}
			
			return;
		}
		
		// init skeleton structures
		bodyFrame = new KinectInterop.BodyFrameData(true);

		KinectInterop.SmoothParameters smoothParameters = new KinectInterop.SmoothParameters();
		
		switch(smoothing)
		{
			case Smoothing.Default:
				smoothParameters.smoothing = 0.5f;
				smoothParameters.correction = 0.5f;
				smoothParameters.prediction = 0.5f;
				smoothParameters.jitterRadius = 0.05f;
				smoothParameters.maxDeviationRadius = 0.04f;
				break;
			case Smoothing.Medium:
				smoothParameters.smoothing = 0.5f;
				smoothParameters.correction = 0.1f;
				smoothParameters.prediction = 0.5f;
				smoothParameters.jitterRadius = 0.1f;
				smoothParameters.maxDeviationRadius = 0.1f;
				break;
			case Smoothing.Aggressive:
				smoothParameters.smoothing = 0.7f;
				smoothParameters.correction = 0.3f;
				smoothParameters.prediction = 1.0f;
				smoothParameters.jitterRadius = 1.0f;
				smoothParameters.maxDeviationRadius = 1.0f;
				break;
		}
		
		// init data filters
		jointPositionFilter = new JointPositionsFilter();
		jointPositionFilter.Init(smoothParameters);
		
		// init the bone orientation constraints
		if(useBoneOrientationConstraints)
		{
			boneConstraintsFilter = new BoneOrientationsConstraint();
			boneConstraintsFilter.AddDefaultConstraints();
			boneConstraintsFilter.SetDebugText(calibrationText);
		}

		// get the main camera rectangle
		Rect cameraRect = Camera.main.pixelRect;
		
		// calculate map width and height in percent, if needed
		if(MapsPercentWidth == 0f)
			MapsPercentWidth = (sensorData.depthImageWidth / 2) / cameraRect.width;
		if(MapsPercentHeight == 0f)
			MapsPercentHeight = (sensorData.depthImageHeight / 2) / cameraRect.height;
		
		if(computeUserMap)
		{
			// init user-depth structures
			//depthImage = new KinectInterop.DepthBuffer(true);
			//bodyIndexImage = new KinectInterop.BodyIndexBuffer(true);
			
	        // Initialize depth & label map related stuff
	        usersLblTex = new Texture2D(sensorData.depthImageWidth, sensorData.depthImageHeight);
			usersMapRect = new Rect(cameraRect.width - cameraRect.width * MapsPercentWidth, cameraRect.height, cameraRect.width * MapsPercentWidth, -cameraRect.height * MapsPercentHeight);

			usersMapSize = sensorData.depthImageWidth * sensorData.depthImageHeight;
			usersHistogramImage = new Color32[usersMapSize];
			usersPrevState = new ushort[usersMapSize];
	        usersHistogramMap = new float[5001];
		}
		
		if(computeColorMap)
		{
			// init color image structures
			//colorImage = new KinectInterop.ColorBuffer(true);
			
			// Initialize color map related stuff
			usersClrTex = new Texture2D(sensorData.colorImageWidth, sensorData.colorImageHeight, TextureFormat.RGBA32, false);
	        usersClrRect = new Rect(cameraRect.width - cameraRect.width * MapsPercentWidth, cameraRect.height, cameraRect.width * MapsPercentWidth, -cameraRect.height * MapsPercentHeight);
			usersClrSize = sensorData.colorImageWidth * sensorData.colorImageHeight;
			
			if(computeUserMap && displayColorMap)
			{
				usersMapRect.x -= cameraRect.width * MapsPercentWidth; //usersClrTex.width / 2;
			}
		}

		// try to automatically find the available avatar controllers at the scene
		if(avatarControllers.Count == 0)
		{
			AvatarController[] avatars = FindObjectsOfType(typeof(AvatarController)) as AvatarController[];
			
			foreach(AvatarController avatar in avatars)
			{
				avatarControllers.Add(avatar);
			}
		}
		
        // Initialize user list to contain all users.
        alUserIds = new List<Int64>();
        dictUserIdToIndex = new Dictionary<Int64, int>();
	
		kinectInitialized = true;
		instance = this;
		
		DontDestroyOnLoad(gameObject);
		
		// GUI Text.
		if(calibrationText != null)
		{
			calibrationText.guiText.text = "WAITING FOR USERS";
		}
		
		Debug.Log("Waiting for users.");
	}
	
//	// Start the Kinect2Server app
//	private int StartKinectServer()
//	{
//		// start Kinect Server
//		kinectServer = new KinectServer();
//		kinectServer.RunKinectServer();
//		
//		float fTimeToWait = Time.realtimeSinceStartup + 10f; // allow 10 seconds time-out
//		int iPing = 0;
//		
//		while(Time.realtimeSinceStartup < fTimeToWait)
//		{
//			iPing = KinectInterop.PingKinect2Server();
//			if(iPing == 12345678)
//				break;
//		}
//		
//		if(iPing == 12345678)
//		{
//			iPing = 0;
//		}
//		
//		return iPing;
//	}
	
	void OnApplicationQuit()
	{
		// shut down the Kinect on quitting.
		if(kinectInitialized)
		{
			KinectInterop.CloseKinectSensor(sensorData);
			
//			KinectInterop.ShutdownKinectSensor();

			instance = null;
		}
		
//		// shut down the server, if any
//		if(kinectServer != null)
//		{
//			kinectServer.ShutdownKinectServer();
//		}
	}
	
    void OnGUI()
    {
		if(kinectInitialized)
		{
	        if(computeUserMap && displayUserMap)
	        {
	            GUI.DrawTexture(usersMapRect, usersLblTex);
	        }

			if(computeColorMap && displayColorMap)
			{
				GUI.DrawTexture(usersClrRect, usersClrTex);
			}
		}
    }
	
	void Update() 
	{
		if(kinectInitialized)
		{
			if(computeColorMap)
			{
				if(KinectInterop.PollColorFrame(sensorData))
				{
					UpdateColorMap();
				}
			}
			
			if(computeUserMap)
			{
				if(KinectInterop.PollDepthFrame(sensorData))
				{
					UpdateUserMap();
				}
			}
			
			if(computeInfraredMap)
			{
				if(KinectInterop.PollInfraredFrame(sensorData))
				{
					UpdateInfraredMap();
				}
			}
			
			if(KinectInterop.PollBodyData(sensorData, ref bodyFrame))
			{
				//lastFrameTime = bodyFrame.liRelativeTime;

				// filter the tracked joint positions
				if(smoothing != Smoothing.None)
				{
					jointPositionFilter.UpdateFilter(ref bodyFrame);
				}

				ProcessBodyFrameData();
			}

			foreach (AvatarController controller in avatarControllers)
			{
				int userIndex = controller.playerIndex;

				if((userIndex >= 0) && (userIndex < alUserIds.Count))
				{
					Int64 userId = alUserIds[userIndex];
					controller.UpdateAvatar(userId);
				}
			}
				
			
			foreach(Int64 userId in alUserIds)
			{
				if(!playerGesturesData.ContainsKey(userId))
					continue;

				// Check for player 1's gestures
				CheckForGestures(userId);
				
				// Check for complete gestures
				List<KinectGestures.GestureData> gesturesData = playerGesturesData[userId];
				
				foreach(KinectGestures.GestureData gestureData in gesturesData)
				{
					if(gestureData.complete)
					{
//						if(gestureData.gesture == KinectGestures.Gestures.Click)
//						{
//							if(controlMouseCursor)
//							{
//								MouseControl.MouseClick();
//							}
//						}
				
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							if(listener.GestureCompleted(userId, 0, gestureData.gesture, (JointType)gestureData.joint, gestureData.screenPos))
							{
								ResetPlayerGestures(userId);
							}
						}
					}
					else if(gestureData.cancelled)
					{
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							if(listener.GestureCancelled(userId, 0, gestureData.gesture, (JointType)gestureData.joint))
							{
								ResetGesture(userId, gestureData.gesture);
							}
						}
					}
					else if(gestureData.progress >= 0.1f)
					{
//						if((gestureData.gesture == KinectGestures.Gestures.RightHandCursor || 
//						    gestureData.gesture == KinectGestures.Gestures.LeftHandCursor) && 
//						   gestureData.progress >= 0.5f)
//						{
//							if(handCursor != null)
//							{
//								handCursor.transform.position = Vector3.Lerp(handCursor.transform.position, gestureData.screenPos, 3 * Time.deltaTime);
//							}
//							
//							if(controlMouseCursor)
//							{
//								MouseControl.MouseMove(gestureData.screenPos);
//							}
//						}
						
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							listener.GestureInProgress(userId, 0, gestureData.gesture, gestureData.progress, 
							                           (JointType)gestureData.joint, gestureData.screenPos);
						}
					}
				}
			}
			
		}
	}
	
	// Update the color image
	void UpdateColorMap()
	{
		//usersClrTex.SetPixels32(colorImage.pixels);
		usersClrTex.LoadRawTextureData(sensorData.colorImage);
		usersClrTex.Apply();
	}
	
	// Update the user histogram
    void UpdateUserMap()
    {
		//if(KinectInterop.PollUserHistogramFrame(ref userHistogramImage, computeColorMap))
		{
			// draw user histogram
			//usersLblTex.SetPixels32(userHistogramImage.pixels);

			UpdateUserHistogramImage();
			usersLblTex.SetPixels32(usersHistogramImage);

			// draw skeleton lines
			if(displaySkeletonLines)
			{
				for(int i = 0; i < alUserIds.Count; i++)
				{
					Int64 liUserId = alUserIds[i];
					int index = dictUserIdToIndex[liUserId];
					
					if(index >= 0 && index < KinectInterop.Constants.BodyCount)
					{
						DrawSkeleton(usersLblTex, ref bodyFrame.bodyData[index]);
					}
				}
			}

			usersLblTex.Apply();
		}
    }

	// Update the user infrared map
	void UpdateInfraredMap()
	{
		// does nothing at the moment
	}
	
	// Update the user histogram map
	void UpdateUserHistogramImage()
	{
		int numOfPoints = 0;
		Array.Clear(usersHistogramMap, 0, usersHistogramMap.Length);
		
		// Calculate cumulative histogram for depth
		for (int i = 0; i < usersMapSize; i++)
		{
			// Only calculate for depth that contains users
			if (sensorData.bodyIndexImage[i] != 255)
			{
				ushort depth = sensorData.depthImage[i];
				if(depth > 5000)
					depth = 5000;

				usersHistogramMap[depth]++;
				numOfPoints++;
			}
		}
		
		if (numOfPoints > 0)
		{
			for (int i = 1; i < usersHistogramMap.Length; i++)
			{   
				usersHistogramMap[i] += usersHistogramMap[i - 1];
			}
			
			for (int i = 0; i < usersHistogramMap.Length; i++)
			{
				usersHistogramMap[i] = 1.0f - (usersHistogramMap[i] / numOfPoints);
			}
		}

		ColorSpacePoint[] colorCoords = null;
		if(sensorData.colorImage != null)
		{
			colorCoords = new ColorSpacePoint[sensorData.depthImageWidth * sensorData.depthImageHeight];
			sensorData.coordMapper.MapDepthFrameToColorSpace(sensorData.depthImage, colorCoords);
		}
		
		// Create the actual users texture based on label map and depth histogram
		Color32 clrClear = Color.clear;
		for (int i = 0; i < usersMapSize; i++)
		{
			ushort userMap = sensorData.bodyIndexImage[i];
			ushort userDepth = sensorData.depthImage[i];
			
			ushort nowUserPixel = userMap != 255 ? (ushort)((userMap << 13) | userDepth) : userDepth;
			ushort wasUserPixel = usersPrevState[i];
			
			// draw only the changed pixels
			if(nowUserPixel != wasUserPixel)
			{
				usersPrevState[i] = nowUserPixel;
				
				if (userMap == 255)
				{
					usersHistogramImage[i] = clrClear;
				}
				else
				{
					if(sensorData.colorImage != null)
					{
						int cx = (int)(colorCoords[i].X);
						int cy = (int)(colorCoords[i].Y);
						int colorIndex = cx + cy * sensorData.colorImageWidth;

						if(colorIndex >= 0 && colorIndex < usersClrSize)
						{
							int ci = colorIndex << 2;
							Color32 colorPixel = new Color32(sensorData.colorImage[ci], sensorData.colorImage[ci + 1], sensorData.colorImage[ci + 2], 230);

							usersHistogramImage[i] = colorPixel;
						}
					}
					else
					{
						// Create a blending color based on the depth histogram
						float histDepth = usersHistogramMap[userDepth];
						Color c = new Color(histDepth, histDepth, histDepth, 0.9f);
						
						switch(userMap % 4)
						{
						case 0:
							usersHistogramImage[i] = Color.red * c;
							break;
						case 1:
							usersHistogramImage[i] = Color.green * c;
							break;
						case 2:
							usersHistogramImage[i] = Color.blue * c;
							break;
						case 3:
							usersHistogramImage[i] = Color.magenta * c;
							break;
						}
					}
				}
				
			}
		}
		
	}
	
	// Processes body frame data
	private void ProcessBodyFrameData()
	{
		List<Int64> addedUsers = new List<Int64>();
		List<int> addedIndexes = new List<int>();

		List<Int64> lostUsers = new List<Int64>();
		lostUsers.AddRange(alUserIds);
		
		for(int i = 0; i < KinectInterop.Constants.BodyCount; i++)
		{
			KinectInterop.BodyData bodyData = bodyFrame.bodyData[i];
			Int64 userId = bodyData.liTrackingID;
			
			if(bodyData.bIsTracked != 0 && Mathf.Abs(kinectToWorld.MultiplyPoint3x4(bodyData.position).z) >= minUserDistance)
			{
				// get the body position
				Vector3 bodyPos = kinectToWorld.MultiplyPoint3x4(bodyData.position);
				
				if(liPrimaryUserId == 0)
				{
					// check if this is the closest user
					bool bClosestUser = true;
					int iClosestUserIndex = i;
					
					if(detectClosestUser)
					{
						for(int j = 0; j < KinectInterop.Constants.BodyCount; j++)
						{
							if(j != i)
							{
								KinectInterop.BodyData bodyDataOther = bodyFrame.bodyData[j];
								
								if((bodyDataOther.bIsTracked != 0) && 
									(Mathf.Abs(kinectToWorld.MultiplyPoint3x4(bodyDataOther.position).z) < Mathf.Abs(bodyPos.z)))
								{
									bClosestUser = false;
									iClosestUserIndex = j;
									break;
								}
							}
						}
					}
					
					if(bClosestUser)
					{
						// add the first or closest userId to the list of new users
						if(!addedUsers.Contains(userId))
						{
							addedUsers.Add(userId);
							addedIndexes.Add(iClosestUserIndex);
						}
						
					}
				}
				
				// add userId to the list of new users
				if(!addedUsers.Contains(userId))
				{
					addedUsers.Add(userId);
					addedIndexes.Add(i);
				}

				// convert Kinect positions to world positions
				bodyFrame.bodyData[i].position = bodyPos;
				string debugText = String.Empty;

				int jointsCount = bodyData.joint.Length;
				for (int j = 0; j < jointsCount; j++)
				{
					bodyData.joint[j].position = kinectToWorld.MultiplyPoint3x4(bodyData.joint[j].position);
				
					if((bodyData.liTrackingID == liPrimaryUserId) && (j == (int)JointType.Head) &&
						bodyData.joint[j].trackingState != TrackingState.NotTracked)
					{
						//Vector3 jointPos = bodyData.joint[j].kinectPos;
						//debugText = string.Format("{0}: {1}", j, jointPos);

						if(calibrationText && debugText != String.Empty)
						{
							calibrationText.guiText.text = debugText;
						}
					}
				}

				// process special cases
				if(bodyData.joint[(int)JointType.HipLeft].trackingState != TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.SpineBase].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.HipRight].trackingState == TrackingState.Tracked)
				{
					bodyData.joint[(int)JointType.HipLeft].trackingState = TrackingState.Inferred;

					bodyData.joint[(int)JointType.HipLeft].kinectPos = bodyData.joint[(int)JointType.SpineBase].kinectPos +
						(bodyData.joint[(int)JointType.SpineBase].kinectPos - bodyData.joint[(int)JointType.HipRight].kinectPos);
					bodyData.joint[(int)JointType.HipLeft].position = bodyData.joint[(int)JointType.SpineBase].position +
						(bodyData.joint[(int)JointType.SpineBase].position - bodyData.joint[(int)JointType.HipRight].position);
					bodyData.joint[(int)JointType.HipLeft].direction = bodyData.joint[(int)JointType.HipLeft].position -
						bodyData.joint[(int)JointType.SpineBase].position;
				}

				if(bodyData.joint[(int)JointType.HipRight].trackingState != TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.SpineBase].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.HipLeft].trackingState == TrackingState.Tracked)
				{
					bodyData.joint[(int)JointType.HipRight].trackingState = TrackingState.Inferred;
					
					bodyData.joint[(int)JointType.HipRight].kinectPos = bodyData.joint[(int)JointType.SpineBase].kinectPos +
						(bodyData.joint[(int)JointType.SpineBase].kinectPos - bodyData.joint[(int)JointType.HipLeft].kinectPos);
					bodyData.joint[(int)JointType.HipRight].position = bodyData.joint[(int)JointType.SpineBase].position +
						(bodyData.joint[(int)JointType.SpineBase].position - bodyData.joint[(int)JointType.HipLeft].position);
					bodyData.joint[(int)JointType.HipRight].direction = bodyData.joint[(int)JointType.HipRight].position -
						bodyData.joint[(int)JointType.SpineBase].position;
				}
				
				if(bodyData.joint[(int)JointType.ShoulderLeft].trackingState != TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.SpineShoulder].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.ShoulderRight].trackingState == TrackingState.Tracked)
				{
					bodyData.joint[(int)JointType.ShoulderLeft].trackingState = TrackingState.Inferred;
					
					bodyData.joint[(int)JointType.ShoulderLeft].kinectPos = bodyData.joint[(int)JointType.SpineShoulder].kinectPos +
						(bodyData.joint[(int)JointType.SpineShoulder].kinectPos - bodyData.joint[(int)JointType.ShoulderRight].kinectPos);
					bodyData.joint[(int)JointType.ShoulderLeft].position = bodyData.joint[(int)JointType.SpineShoulder].position +
						(bodyData.joint[(int)JointType.SpineShoulder].position - bodyData.joint[(int)JointType.ShoulderRight].position);
					bodyData.joint[(int)JointType.ShoulderLeft].direction = bodyData.joint[(int)JointType.ShoulderLeft].position -
						bodyData.joint[(int)JointType.SpineShoulder].position;
				}
				
				if(bodyData.joint[(int)JointType.ShoulderRight].trackingState != TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.SpineShoulder].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.ShoulderLeft].trackingState == TrackingState.Tracked)
				{
					bodyData.joint[(int)JointType.ShoulderRight].trackingState = TrackingState.Inferred;
					
					bodyData.joint[(int)JointType.ShoulderRight].kinectPos = bodyData.joint[(int)JointType.SpineShoulder].kinectPos +
						(bodyData.joint[(int)JointType.SpineShoulder].kinectPos - bodyData.joint[(int)JointType.ShoulderLeft].kinectPos);
					bodyData.joint[(int)JointType.ShoulderRight].position = bodyData.joint[(int)JointType.SpineShoulder].position +
						(bodyData.joint[(int)JointType.SpineShoulder].position - bodyData.joint[(int)JointType.ShoulderLeft].position);
					bodyData.joint[(int)JointType.ShoulderRight].direction = bodyData.joint[(int)JointType.ShoulderRight].position -
						bodyData.joint[(int)JointType.SpineShoulder].position;
				}
				
				// calculate special directions
				if(bodyData.joint[(int)JointType.HipLeft].trackingState != TrackingState.NotTracked &&
				   bodyData.joint[(int)JointType.HipRight].trackingState != TrackingState.NotTracked)
				{
					Vector3 posRHip = bodyData.joint[(int)JointType.HipRight].position;
					Vector3 posLHip = bodyData.joint[(int)JointType.HipLeft].position;
					
					bodyData.hipsDirection = posRHip - posLHip;
					bodyData.hipsDirection -= Vector3.Project(bodyData.hipsDirection, Vector3.up);
				}

				if(bodyData.joint[(int)JointType.ShoulderLeft].trackingState != TrackingState.NotTracked &&
				   bodyData.joint[(int)JointType.ShoulderRight].trackingState != TrackingState.NotTracked)
				{
					Vector3 posRShoulder = bodyData.joint[(int)JointType.ShoulderRight].position;
					Vector3 posLShoulder = bodyData.joint[(int)JointType.ShoulderLeft].position;
					
					bodyData.shouldersDirection = posRShoulder - posLShoulder;
					bodyData.shouldersDirection -= Vector3.Project(bodyData.shouldersDirection, Vector3.up);
				}

				if(bodyData.joint[(int)JointType.WristLeft].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.HandLeft].trackingState == TrackingState.Tracked)
				{
					Vector3 pos1 = bodyData.joint[(int)JointType.WristLeft].position;
					Vector3 pos2 = bodyData.joint[(int)JointType.HandLeft].position;
					
					bodyData.leftHandDirection = pos2 - pos1;
				}
				
				if(bodyData.leftHandDirection != Vector3.zero &&
				   bodyData.joint[(int)JointType.HandLeft].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.ThumbLeft].trackingState == TrackingState.Tracked)
				{
					Vector3 pos1 = bodyData.joint[(int)JointType.HandLeft].position;
					Vector3 pos2 = bodyData.joint[(int)JointType.ThumbLeft].position;
					
					bodyData.leftThumbDirection = pos2 - pos1;
					bodyData.leftThumbDirection -= Vector3.Project(bodyData.leftThumbDirection, bodyData.leftHandDirection);

					Vector3 shouldersDir = bodyData.shouldersDirection;
					Vector3 thumbFwdDir = Vector3.forward;
					Vector3.OrthoNormalize(ref shouldersDir, ref thumbFwdDir);

					bodyData.leftThumbForward = thumbFwdDir;
					bodyData.leftThumbForward -= Vector3.Project(bodyData.leftThumbForward, bodyData.leftHandDirection);
					
					bodyData.leftThumbAngle = Vector3.Angle(bodyData.leftThumbForward, bodyData.leftThumbDirection);
				}

				if(bodyData.joint[(int)JointType.WristRight].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.HandRight].trackingState == TrackingState.Tracked)
				{
					Vector3 pos1 = bodyData.joint[(int)JointType.WristRight].position;
					Vector3 pos2 = bodyData.joint[(int)JointType.HandRight].position;
					
					bodyData.rightHandDirection = pos2 - pos1;
				}
				
				if(bodyData.rightHandDirection != Vector3.zero &&
				   bodyData.joint[(int)JointType.HandRight].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.ThumbRight].trackingState == TrackingState.Tracked)
				{
					Vector3 pos1 = bodyData.joint[(int)JointType.HandRight].position;
					Vector3 pos2 = bodyData.joint[(int)JointType.ThumbRight].position;
					
					bodyData.rightThumbDirection = pos2 - pos1;
					bodyData.rightThumbDirection -= Vector3.Project(bodyData.rightThumbDirection, bodyData.rightHandDirection);
					
					Vector3 shouldersDir = bodyData.shouldersDirection;
					Vector3 thumbFwdDir = Vector3.forward;
					Vector3.OrthoNormalize(ref shouldersDir, ref thumbFwdDir);
					
					bodyData.rightThumbForward = thumbFwdDir;
					bodyData.rightThumbForward -= Vector3.Project(bodyData.rightThumbForward, bodyData.rightHandDirection);

					bodyData.rightThumbAngle = Vector3.Angle(bodyData.rightThumbDirection, bodyData.rightThumbDirection);
				}

				if(bodyData.joint[(int)JointType.KneeLeft].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.AnkleLeft].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.FootLeft].trackingState == TrackingState.Tracked)
				{
					bodyData.leftLegDirection = bodyData.joint[(int)JointType.AnkleLeft].direction;
					bodyData.leftLegDirection.z = -bodyData.leftLegDirection.z;

					bodyData.leftFootDirection = bodyData.joint[(int)JointType.FootLeft].direction;
					bodyData.leftFootDirection.z = -bodyData.leftFootDirection.z;
				}
				
				if(bodyData.joint[(int)JointType.KneeRight].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.AnkleRight].trackingState == TrackingState.Tracked &&
				   bodyData.joint[(int)JointType.FootRight].trackingState == TrackingState.Tracked)
				{
					bodyData.rightLegDirection = bodyData.joint[(int)JointType.AnkleRight].direction;
					bodyData.rightLegDirection.z = -bodyData.rightLegDirection.z;
					
					bodyData.rightFootDirection = bodyData.joint[(int)JointType.FootRight].direction;
					bodyData.rightFootDirection.z = -bodyData.rightFootDirection.z;
				}
				
				// calculate world rotations of the body joints
				SetJointOrientations(ref bodyData);

				// filter orientation constraints
				if(useBoneOrientationConstraints && boneConstraintsFilter != null)
				{
					boneConstraintsFilter.Constrain(ref bodyData);
				}
				
				lostUsers.Remove(userId);
				bodyFrame.bodyData[i] = bodyData;
			}
		}
		
		// remove the lost users if any
		if(lostUsers.Count > 0)
		{
			foreach(Int64 userId in lostUsers)
			{
				RemoveUser(userId);
			}
			
			lostUsers.Clear();
		}

		// calibrate newly detected users
		if(addedUsers.Count > 0)
		{
			for(int i = 0; i < addedUsers.Count; i++)
			{
				Int64 userId = addedUsers[i];
				int userIndex = addedIndexes[i];

				CalibrateUser(userId, userIndex);
			}
			
			addedUsers.Clear();
			addedIndexes.Clear();
		}
	}
	
	// Adds UserId to the list of users
    void CalibrateUser(Int64 userId, int bodyIndex)
    {
		if(!alUserIds.Contains(userId))
		{
			if(CheckForCalibrationPose(userId, bodyIndex, playerCalibrationPose))
			{
				int uidIndex = alUserIds.Count;
				Debug.Log("Adding user " + uidIndex + ", ID: " + userId + ", Index: " + bodyIndex);
				
				alUserIds.Add(userId);
				dictUserIdToIndex[userId] = bodyIndex;
				
				if(liPrimaryUserId == 0)
				{
					liPrimaryUserId = userId;
					
					if(liPrimaryUserId != 0)
					{
						if(calibrationText != null && calibrationText.guiText.text != "")
						{
							calibrationText.guiText.text = "";
						}
					}
				}
				
				for(int i = 0; i < avatarControllers.Count; i++)
				{
					AvatarController avatar = avatarControllers[i];
					
					if(avatar && avatar.playerIndex == uidIndex)
					{
						avatar.SuccessfulCalibration(userId);
					}
				}
				
				// add the gestures to detect, if any
				foreach(KinectGestures.Gestures gesture in playerCommonGestures)
				{
					DetectGesture(userId, gesture);
				}
				
				// notify the gesture listeners about the new user
				foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
				{
					listener.UserDetected(userId, 0);
				}
				
				ResetFilters();
			}
		}
    }
	
	// Remove a lost UserId
	void RemoveUser(Int64 userId)
	{
		int uidIndex = alUserIds.IndexOf(userId);
		Debug.Log("Removing user " + uidIndex + ", ID: " + userId);
		
		for(int i = 0; i < avatarControllers.Count; i++)
		{
			AvatarController avatar = avatarControllers[i];
			
			if(avatar && avatar.playerIndex >= uidIndex && avatar.playerIndex < alUserIds.Count)
			{
				avatar.ResetToInitialPosition();
			}
		}

		// notify the gesture listeners about the user loss
		foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
		{
			listener.UserLost(userId, 0);
		}

		// clear gestures list for this user
		ClearGestures(userId);

		// clear calibration data for this user
		if(playerCalibrationData.ContainsKey(userId))
		{
			playerCalibrationData.Remove(userId);
		}

		// clean up the outdated calibration data in the data dictionary
		List<Int64> alCalDataKeys = new List<Int64>(playerCalibrationData.Keys);

		foreach(Int64 calUserID in alCalDataKeys)
		{
			KinectGestures.GestureData gestureData = playerCalibrationData[calUserID];

			if((gestureData.timestamp + 60f) < Time.realtimeSinceStartup)
			{
				playerCalibrationData.Remove(calUserID);
			}
		}

		alCalDataKeys.Clear();
		
		// remove from global users list
        alUserIds.Remove(userId);
		dictUserIdToIndex.Remove(userId);
		
		if(liPrimaryUserId == userId)
		{
			if(alUserIds.Count > 0)
			{
				liPrimaryUserId = alUserIds[0];
			}
			else
			{
				liPrimaryUserId = 0;
			}
		}
		
		for(int i = 0; i < avatarControllers.Count; i++)
		{
			AvatarController avatar = avatarControllers[i];
			
			if(avatar && avatar.playerIndex >= uidIndex && avatar.playerIndex < alUserIds.Count)
			{
				avatar.SuccessfulCalibration(alUserIds[avatar.playerIndex]);
			}
		}
		
		if(liPrimaryUserId == 0)
		{
			Debug.Log("Waiting for users.");
			
			if(calibrationText != null)
			{
				calibrationText.guiText.text = "WAITING FOR USERS";
			}
		}
	}
	
	// draws the skeleton in the given texture
	private void DrawSkeleton(Texture2D aTexture, ref KinectInterop.BodyData bodyData)
	{
		int jointsCount = KinectInterop.Constants.JointCount;
		
		for(int i = 0; i < jointsCount; i++)
		{
			int parent = (int)KinectInterop.GetParentJoint((JointType)i);
			
			if(bodyData.joint[i].trackingState != TrackingState.NotTracked && 
			   bodyData.joint[parent].trackingState != TrackingState.NotTracked)
			{
				Vector2 posParent = KinectInterop.GetKinectPointDepthCoords(sensorData, bodyData.joint[parent].kinectPos);
				Vector2 posJoint = KinectInterop.GetKinectPointDepthCoords(sensorData, bodyData.joint[i].kinectPos);
				
//				posParent.y = KinectInterop.Constants.ImageHeight - posParent.y - 1;
//				posJoint.y = KinectInterop.Constants.ImageHeight - posJoint.y - 1;
//				posParent.x = KinectInterop.Constants.ImageWidth - posParent.x - 1;
//				posJoint.x = KinectInterop.Constants.ImageWidth - posJoint.x - 1;

				if(posParent != Vector2.zero && posJoint != Vector2.zero)
				{
					//Color lineColor = playerJointsTracked[i] && playerJointsTracked[parent] ? Color.red : Color.yellow;
					DrawLine(aTexture, (int)posParent.x, (int)posParent.y, (int)posJoint.x, (int)posJoint.y, Color.yellow);
				}
			}
		}
		
		//aTexture.Apply();
	}
	
	// draws a line in a texture
	private void DrawLine(Texture2D a_Texture, int x1, int y1, int x2, int y2, Color a_Color)
	{
		int width = KinectInterop.Constants.DepthImageWidth;
		int height = KinectInterop.Constants.DepthImageHeight;
		
		int dy = y2 - y1;
		int dx = x2 - x1;
	 
		int stepy = 1;
		if (dy < 0) 
		{
			dy = -dy; 
			stepy = -1;
		}
		
		int stepx = 1;
		if (dx < 0) 
		{
			dx = -dx; 
			stepx = -1;
		}
		
		dy <<= 1;
		dx <<= 1;
	 
		if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
			for(int x = -1; x <= 1; x++)
				for(int y = -1; y <= 1; y++)
					a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
		
		if (dx > dy) 
		{
			int fraction = dy - (dx >> 1);
			
			while (x1 != x2) 
			{
				if (fraction >= 0) 
				{
					y1 += stepy;
					fraction -= dx;
				}
				
				x1 += stepx;
				fraction += dy;
				
				if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
					for(int x = -1; x <= 1; x++)
						for(int y = -1; y <= 1; y++)
							a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
			}
		}
		else 
		{
			int fraction = dx - (dy >> 1);
			
			while (y1 != y2) 
			{
				if (fraction >= 0) 
				{
					x1 += stepx;
					fraction -= dy;
				}
				
				y1 += stepy;
				fraction += dx;
				
				if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
					for(int x = -1; x <= 1; x++)
						for(int y = -1; y <= 1; y++)
							a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
			}
		}
		
	}

	// convert the matrix to quaternion, taking care of the mirroring
	private Quaternion ConvertMatrixToQuat(Quaternion rotation, int joint, bool flip)
	{
		if(rotation.x == 0f && rotation.y == 0f && rotation.z == 0f && rotation.w == 0f)
			return Quaternion.identity;
		
		mOrient.SetTRS(Vector3.zero, rotation, Vector3.one);
		
		UnityEngine.Vector4 vZ = mOrient.GetColumn(2);
		UnityEngine.Vector4 vY = mOrient.GetColumn(1);
		
		vZ.z = -vZ.z;

		if(!flip)
		{
			vZ.x = -vZ.x;
			vZ.y = -vZ.y;
		}
		
		if(vZ.x != 0.0f || vZ.y != 0.0f || vZ.z != 0.0f)
			return Quaternion.LookRotation(vZ, vY);
		else
			return Quaternion.identity;
	}

	// calculates joint orientations for the given body
	private void SetJointOrientations(ref KinectInterop.BodyData bodyData)
	{
		int jointCount = bodyData.joint.Length;

		for(int j = 0; j < jointCount; j++)
		{
			int joint = j;

//			bool bTurnedAway = false;
//			if(bodyData.joint[(int)JointType.Head].trackingState != TrackingState.NotTracked)
//				if(bodyData.joint[(int)JointType.Head].kinectPos.z > bodyData.joint[(int)JointType.Neck].kinectPos.z)
//					bTurnedAway = true;
//			
//			if(bodyData.joint[(int)JointType.FootLeft].trackingState != TrackingState.NotTracked)
//				if(bodyData.joint[(int)JointType.FootLeft].kinectPos.z > bodyData.joint[(int)JointType.AnkleLeft].kinectPos.z)
//					bTurnedAway = true;
//
//			if(bodyData.joint[(int)JointType.FootRight].trackingState != TrackingState.NotTracked)
//				if(bodyData.joint[(int)JointType.FootRight].kinectPos.z > bodyData.joint[(int)JointType.AnkleRight].kinectPos.z)
//					bTurnedAway = true;
//
//			if(bTurnedAway)
//			{
//				//joint = (int)KinectInterop.GetMirrorJoint((JointType)j);
//			}
			
			KinectInterop.JointData jointData = bodyData.joint[joint];
			if(jointData.trackingState != TrackingState.Tracked)
				continue;

			int nextJoint = (int)KinectInterop.GetNextJoint((JointType)joint);
			if(nextJoint != joint && nextJoint >= 0 && nextJoint < KinectInterop.Constants.JointCount)
			{
				KinectInterop.JointData nextJointData = bodyData.joint[nextJoint];

				if(nextJointData.trackingState == TrackingState.Tracked)
				{
					Vector3 baseDir = KinectInterop.JointBaseDir[nextJoint];
					Vector3 jointDir = nextJointData.direction;
					jointDir.z = -jointDir.z;

					jointData.normalRotation = Quaternion.FromToRotation(baseDir, jointDir);
					
					if((joint == (int)JointType.AnkleLeft) && (bodyData.leftLegDirection != Vector3.zero) &&
					   (bodyData.leftFootDirection != Vector3.zero))
					{
//						Vector3 rightDir = Vector3.Cross(bodyData.leftLegDirection, bodyData.leftFootDirection);
//						jointData.normalRotation *= Quaternion.AngleAxis(30f, rightDir);
					}
					
					if((joint == (int)JointType.AnkleRight) && (bodyData.rightLegDirection != Vector3.zero) &&
					   (bodyData.rightFootDirection != Vector3.zero))
					{
//						Vector3 rightDir = Vector3.Cross(bodyData.rightLegDirection, bodyData.rightFootDirection);
//						jointData.normalRotation *= Quaternion.AngleAxis(30f, rightDir);
					}
					
//						jointDir.x = -jointDir.x;
//						//jointDir.y = -jointDir.y;
//						
//						baseDir.x = -baseDir.x;
//						//baseDir.y = -baseDir.y;
//						
//						jointData.mirroredRotation = Quaternion.FromToRotation(baseDir, jointDir);

					Vector3 mirroredAngles = jointData.normalRotation.eulerAngles;
					mirroredAngles.y = -mirroredAngles.y;
					mirroredAngles.z = -mirroredAngles.z;

					jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);

					if((joint == (int)JointType.SpineBase) || (joint == (int)JointType.SpineMid) ||
					   (joint == (int)JointType.HipLeft) || (joint == (int)JointType.HipRight) ||
					   (joint == (int)JointType.KneeLeft) || (joint == (int)JointType.KneeRight) ||
					   (joint == (int)JointType.FootLeft) || (joint == (int)JointType.FootRight))
					{
						baseDir = Vector3.right;
						jointDir = bodyData.hipsDirection;
						jointDir.z = -jointDir.z;
						
						jointData.normalRotation *= Quaternion.FromToRotation(baseDir, jointDir);
						
//						jointDir.x = -jointDir.x;
//						//jointDir.y = -jointDir.y;
//						
//						baseDir.x = -baseDir.x;
//						//baseDir.y = -baseDir.y;
//						
//						jointData.mirroredRotation *= Quaternion.FromToRotation(baseDir, jointDir);
						
						mirroredAngles = jointData.normalRotation.eulerAngles;
						mirroredAngles.y = -mirroredAngles.y;
						mirroredAngles.z = -mirroredAngles.z;
						
						jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
						
						if(joint == (int)JointType.SpineBase)
						{
							bodyData.normalRotation = jointData.normalRotation;
							bodyData.mirroredRotation = jointData.mirroredRotation;
							//Debug.Log("TurnedAway: " + bTurnedAway);
						}
					}
					
					if((joint == (int)JointType.SpineShoulder) || (joint == (int)JointType.Neck))
					{
						baseDir = Vector3.right;
						jointDir = bodyData.shouldersDirection;
						jointDir.z = -jointDir.z;
						
						jointData.normalRotation *= Quaternion.FromToRotation(baseDir, jointDir);
						
//						jointDir.x = -jointDir.x;
//						//jointDir.y = -jointDir.y;
//						
//						baseDir.x = -baseDir.x;
//						//baseDir.y = -baseDir.y;
//						
//						jointData.mirroredRotation *= Quaternion.FromToRotation(baseDir, jointDir);
						
						mirroredAngles = jointData.normalRotation.eulerAngles;
						mirroredAngles.y = -mirroredAngles.y;
						mirroredAngles.z = -mirroredAngles.z;
						
						jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
					}
					
					if((joint == (int)JointType.WristLeft))
					{
						if(nextJointData.trackingState == TrackingState.Tracked)
						{
//							jointData.normalRotation *= Quaternion.AngleAxis(bodyData.leftThumbAngle, nextJointData.direction);
//							
//							mirroredAngles = jointData.normalRotation.eulerAngles;
//							mirroredAngles.y = -mirroredAngles.y;
//							mirroredAngles.z = -mirroredAngles.z;
//							
//							jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
						}
					}
					
					if((joint == (int)JointType.WristRight))
					{
						if(nextJointData.trackingState == TrackingState.Tracked)
						{
//							jointData.normalRotation *= Quaternion.AngleAxis(bodyData.rightThumbAngle, nextJointData.direction);
//							
//							mirroredAngles = jointData.normalRotation.eulerAngles;
//							mirroredAngles.y = -mirroredAngles.y;
//							mirroredAngles.z = -mirroredAngles.z;
//							
//							jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
						}
					}
				}

			}
			else
			{
				jointData.normalRotation = Quaternion.identity;
				jointData.mirroredRotation = Quaternion.identity;
			}

			bodyData.joint[joint] = jointData;
		}
	}

	// Estimates the current state of the defined gestures
	private void CheckForGestures(Int64 UserId)
	{
		if(!playerGesturesData.ContainsKey(UserId) || !gesturesTrackingAtTime.ContainsKey(UserId))
			return;
		
		// check for gestures
		if(Time.realtimeSinceStartup >= gesturesTrackingAtTime[UserId])
		{
			// get joint positions and tracking
			int iAllJointsCount = KinectInterop.Constants.JointCount;
			bool[] playerJointsTracked = new bool[iAllJointsCount];
			Vector3[] playerJointsPos = new Vector3[iAllJointsCount];
			
			int[] aiNeededJointIndexes = KinectGestures.GetNeededJointIndexes();
			int iNeededJointsCount = aiNeededJointIndexes.Length;
			
			for(int i = 0; i < iNeededJointsCount; i++)
			{
				int joint = aiNeededJointIndexes[i];
				
				if(joint >= 0 && IsJointTracked(UserId, joint))
				{
					playerJointsTracked[joint] = true;
					playerJointsPos[joint] = GetJointPosition(UserId, joint);
				}
			}
			
			// check for gestures
			List<KinectGestures.GestureData> gesturesData = playerGesturesData[UserId];
			
			int listGestureSize = gesturesData.Count;
			float timestampNow = Time.realtimeSinceStartup;
			
			for(int g = 0; g < listGestureSize; g++)
			{
				KinectGestures.GestureData gestureData = gesturesData[g];
				
				if((timestampNow >= gestureData.startTrackingAtTime) && 
					!IsConflictingGestureInProgress(gestureData, ref gesturesData))
				{
					KinectGestures.CheckForGesture(UserId, ref gestureData, Time.realtimeSinceStartup, 
						ref playerJointsPos, ref playerJointsTracked);
					gesturesData[g] = gestureData;

					if(gestureData.complete)
					{
						gesturesTrackingAtTime[UserId] = timestampNow + minTimeBetweenGestures;
					}
				}
			}
			
			playerGesturesData[UserId] = gesturesData;
		}
	}
	
	private bool IsConflictingGestureInProgress(KinectGestures.GestureData gestureData, ref List<KinectGestures.GestureData> gesturesData)
	{
		foreach(KinectGestures.Gestures gesture in gestureData.checkForGestures)
		{
			int index = GetGestureIndex(gesture, ref gesturesData);
			
			if(index >= 0)
			{
				if(gesturesData[index].progress > 0f)
					return true;
			}
		}
		
		return false;
	}
	
	// return the index of gesture in the list, or -1 if not found
	private int GetGestureIndex(KinectGestures.Gestures gesture, ref List<KinectGestures.GestureData> gesturesData)
	{
		int listSize = gesturesData.Count;
	
		for(int i = 0; i < listSize; i++)
		{
			if(gesturesData[i].gesture == gesture)
				return i;
		}
		
		return -1;
	}
	
	// check if the calibration pose is complete for given user
	private bool CheckForCalibrationPose(Int64 UserId, int bodyIndex, KinectGestures.Gestures calibrationGesture)
	{
		if(calibrationGesture == KinectGestures.Gestures.None)
			return true;

		KinectGestures.GestureData gestureData = playerCalibrationData.ContainsKey(UserId) ? 
			playerCalibrationData[UserId] : new KinectGestures.GestureData();
		
		// init gesture data if needed
		if(gestureData.userId != UserId)
		{
			gestureData.userId = UserId;
			gestureData.gesture = calibrationGesture;
			gestureData.state = 0;
			gestureData.timestamp = Time.realtimeSinceStartup;
			gestureData.joint = 0;
			gestureData.progress = 0f;
			gestureData.complete = false;
			gestureData.cancelled = false;
		}
		
		// get joint positions and tracking
		int iAllJointsCount = KinectInterop.Constants.JointCount;
		bool[] playerJointsTracked = new bool[iAllJointsCount];
		Vector3[] playerJointsPos = new Vector3[iAllJointsCount];
		
		int[] aiNeededJointIndexes = KinectGestures.GetNeededJointIndexes();
		int iNeededJointsCount = aiNeededJointIndexes.Length;
		
		for(int i = 0; i < iNeededJointsCount; i++)
		{
			int joint = aiNeededJointIndexes[i];
			
			if(joint >= 0)
			{
				KinectInterop.JointData jointData = bodyFrame.bodyData[bodyIndex].joint[joint];
				
				playerJointsTracked[joint] = jointData.trackingState != TrackingState.NotTracked;
				playerJointsPos[joint] = kinectToWorld.MultiplyPoint3x4(jointData.kinectPos);
			}
		}
		
		// estimate the gesture progess
		KinectGestures.CheckForGesture(UserId, ref gestureData, Time.realtimeSinceStartup, 
			ref playerJointsPos, ref playerJointsTracked);
		playerCalibrationData[UserId] = gestureData;

		// check if gesture is complete
		if(gestureData.complete)
		{
			gestureData.userId = 0;
			playerCalibrationData[UserId] = gestureData;

			return true;
		}

		return false;
	}
	
}