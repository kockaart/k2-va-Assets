using UnityEngine;
using Windows.Kinect;

using System.Collections;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System;
using System.IO;


public class KinectInterop
{
	// constants
	public static class Constants
	{
		public const int BodyCount = 6;
		public const int JointCount = 25;

		public const int ColorImageWidth = 1920; // 480; // 960; // 1920;
		public const int ColorImageHeight = 1080; // 270; // 540; // 1080;
		public const int DepthImageWidth = 512;
		public const int DepthImageHeight = 424;
		
		public const float MinTimeBetweenSameGestures = 0.0f;
		public const float PoseCompleteDuration = 1.0f;
		public const float ClickMaxDistance = 0.05f;
		public const float ClickStayDuration = 2.5f;
	}
	
	/// Data structures for interfacing C# with the native wrapper

    [Flags]
    public enum FrameSource : uint
    {
        TypeColor = 0x1,
        TypeInfrared = 0x2,
        TypeDepth = 0x8,
        TypeBodyIndex = 0x10,
        TypeBody = 0x20,
        TypeAudio = 0x40
    }
	
	public enum NuiErrorCodes : uint
	{
		FrameNoData = 0x83010001,
		StreamNotEnabled = 0x83010002,
		ImageStreamInUse = 0x83010003,
		FrameLimitExceeded = 0x83010004,
		FeatureNotInitialized = 0x83010005,
		DeviceNotGenuine = 0x83010006,
		InsufficientBandwidth = 0x83010007,
		DeviceNotSupported = 0x83010008,
		DeviceInUse = 0x83010009,
		
		DatabaseNotFound = 0x8301000D,
		DatabaseVersionMismatch = 0x8301000E,
		HardwareFeatureUnavailable = 0x8301000F,
		
		DeviceNotConnected = 0x83010014,
		DeviceNotReady = 0x83010015,
		SkeletalEngineBusy = 0x830100AA,
		DeviceNotPowered = 0x8301027F,
	}

//    public enum JointType : int
//    {
//        HipCenter = 0,
//        Spine = 1,
//        Neck = 2,
//        Head = 3,
//        ShoulderLeft = 4,
//        ElbowLeft = 5,
//        WristLeft = 6,
//        HandLeft = 7,
//        ShoulderRight = 8,
//        ElbowRight = 9,
//        WristRight = 10,
//        HandRight = 11,
//        HipLeft = 12,
//        KneeLeft = 13,
//        AnkleLeft = 14,
//        FootLeft = 15,
//        HipRight = 16,
//        KneeRight = 17,
//        AnkleRight = 18,
//        FootRight = 19,
//        SpineShoulder = 20,
//        HandTipLeft = 21,
//        ThumbLeft = 22,
//        HandTipRight = 23,
//        ThumbRight = 24
//		//Count = 25
//    }

    public static readonly Vector3[] JointBaseDir =
    {
        Vector3.zero,
        Vector3.up,
        Vector3.up,
        Vector3.up,
        Vector3.left,
        Vector3.left,
        Vector3.left,
        Vector3.left,
        Vector3.right,
        Vector3.right,
        Vector3.right,
        Vector3.right,
        Vector3.down,
        Vector3.down,
        Vector3.down,
        Vector3.forward,
        Vector3.down,
        Vector3.down,
        Vector3.down,
        Vector3.forward,
        Vector3.up,
        Vector3.left,
        Vector3.forward,
        Vector3.right,
        Vector3.forward
    };

//    public enum TrackingState
//    {
//        NotTracked = 0,
//        Inferred = 1,
//        Tracked = 2
//    }
//
//	public enum HandState
//    {
//        Unknown = 0,
//        NotTracked = 1,
//        Open = 2,
//        Closed = 3,
//        Lasso = 4
//    }
//	
//	public enum TrackingConfidence
//    {
//        Low = 0,
//        High = 1
//    }
//
//    [Flags]
//    public enum ClippedEdges
//    {
//        None = 0,
//        Right = 1,
//        Left = 2,
//        Top = 4,
//        Bottom = 8
//    }


	public class SensorData
	{
		public KinectSensor kinectSensor;
		public CoordinateMapper coordMapper;

		public BodyFrameReader bodyFrameReader;
		public BodyIndexFrameReader bodyIndexFrameReader;
		public ColorFrameReader colorFrameReader;
		public DepthFrameReader depthFrameReader;
		public InfraredFrameReader infraredFrameReader;

		public int bodyCount;

		public Body[] bodyData;
		public Int64 lastBodyFrameTime = 0;

		public int colorImageWidth;
		public int colorImageHeight;
		public int colorImagePixels;

		public byte[] colorImage;
		public Int64 lastColorFrameTime = 0;

		public int depthImageWidth;
		public int depthImageHeight;
		public int depthImagePixels;

		public ushort[] depthImage;
		public Int64 lastDepthFrameTime = 0;

		public ushort[] infraredImage;
		public Int64 lastInfraredFrameTime = 0;

		public byte[] bodyIndexImage;
		public Int64 lastBodyIndexFrameTime = 0;
	}

	public struct SmoothParameters
	{
		public float smoothing;
		public float correction;
		public float prediction;
		public float jitterRadius;
		public float maxDeviationRadius;
	}
	
	[StructLayout(LayoutKind.Sequential)]
	public struct JointData
    {
    	public JointType jointType;
    	public TrackingState trackingState;
    	public Vector3 kinectPos;
    	public Vector3 position;
		public Vector3 direction;
		public Quaternion orientation;  // deprecated
		public Quaternion normalRotation;
		public Quaternion mirroredRotation;
    }
	
	[StructLayout(LayoutKind.Sequential)]
	public struct BodyData
    {
        public Int64 liTrackingID;
        public Vector3 position;
		public Quaternion orientation;  // deprecated
		public Quaternion normalRotation;
		public Quaternion mirroredRotation;

        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 25, ArraySubType = UnmanagedType.Struct)]
        public JointData[] joint;

		public Vector3 hipsDirection;
		public Vector3 shouldersDirection;

		public Vector3 leftThumbForward;
		public Vector3 leftThumbDirection;
		public Vector3 leftHandDirection;
		public float leftThumbAngle;

		public Vector3 rightThumbForward;
		public Vector3 rightThumbDirection;
		public Vector3 rightHandDirection;
		public float rightThumbAngle;

		public Vector3 leftLegDirection;
		public Vector3 leftFootDirection;
		public Vector3 rightLegDirection;
		public Vector3 rightFootDirection;

		public HandState leftHandState;
		public TrackingConfidence leftHandConfidence;
		public HandState rightHandState;
		public TrackingConfidence rightHandConfidence;
		
        public uint dwClippedEdges;
        public short bIsTracked;
		public short bIsRestricted;
    }
	
	[StructLayout(LayoutKind.Sequential)]
    public struct BodyFrameData
    {
        public Int64 liRelativeTime;
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 6, ArraySubType = UnmanagedType.Struct)]
        public BodyData[] bodyData;
        public UnityEngine.Vector4 floorClipPlane;
		
		public BodyFrameData(bool bInit)
		{
			liRelativeTime = 0;
			floorClipPlane = UnityEngine.Vector4.zero;

			bodyData = new BodyData[Constants.BodyCount];
			
			for(int i = 0; i < Constants.BodyCount; i++)
			{
				bodyData[i].joint = new JointData[Constants.JointCount];
			}
		}
    }
	
	public struct ColorBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.ColorImageWidth * Constants.ColorImageHeight, ArraySubType = UnmanagedType.U4)]
		public Color32[] pixels;
		
		public ColorBuffer(bool bInit)
		{
			pixels = new Color32[Constants.ColorImageWidth * Constants.ColorImageHeight];
		}
	}
	
	public struct DepthBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.DepthImageWidth * Constants.DepthImageHeight, ArraySubType = UnmanagedType.U2)]
		public ushort[] pixels;
		
		public DepthBuffer(bool bInit)
		{
			pixels = new ushort[Constants.DepthImageWidth * Constants.DepthImageHeight];
		}
	}
	
	public struct BodyIndexBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.DepthImageWidth * Constants.DepthImageHeight, ArraySubType = UnmanagedType.U1)]
		public byte[] pixels;
		
		public BodyIndexBuffer(bool bInit)
		{
			pixels = new byte[Constants.DepthImageWidth * Constants.DepthImageHeight];
		}
	}
	
	public struct UserHistogramBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.DepthImageWidth * Constants.DepthImageHeight, ArraySubType = UnmanagedType.U4)]
		public Color32[] pixels;
		
		public UserHistogramBuffer(bool bInit)
		{
			pixels = new Color32[Constants.DepthImageWidth * Constants.DepthImageHeight];
		}
	}
	

	// Wrapped native functions
	// the version, the sample is built upon:
	[DllImport("Kernel32.dll", SetLastError = true)]
	static extern uint FormatMessage( uint dwFlags, IntPtr lpSource, uint dwMessageId, uint dwLanguageId, ref IntPtr lpBuffer, uint nSize, IntPtr pArguments);

 	[DllImport("kernel32.dll", SetLastError = true)]
	static extern IntPtr LocalFree(IntPtr hMem);
	
//	// Pings the server
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int PingKinect2Server();
//
//	// Initializes the default Kinect sensor
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int InitDefaultKinectSensor(FrameSource dwFlags, int iColorWidth, int iColorHeight);
//	
//	// Shuts down the opened Kinect2 sensor
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern void ShutdownKinectSensor();
	
//	// Returns the maximum number of the bodies
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetBodyCount();
//	
//	// Returns the latest body frame data available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetBodyFrameData(ref BodyFrameData pBodyFrame, bool bGetOrientations, bool bGetHandStates);


	// opens the default Kinect sensor and needed readers
	public static SensorData OpenDefaultKinectSensor(FrameSource dwFlags)
	{
		SensorData sensorData = new SensorData();

		sensorData.kinectSensor = KinectSensor.GetDefault();
		if(sensorData.kinectSensor == null)
			return null;

		sensorData.coordMapper = sensorData.kinectSensor.CoordinateMapper;
		sensorData.bodyCount = sensorData.kinectSensor.BodyFrameSource.BodyCount;

		if((dwFlags & KinectInterop.FrameSource.TypeBody) != 0)
		{
			sensorData.bodyFrameReader = sensorData.kinectSensor.BodyFrameSource.OpenReader();
			sensorData.bodyData = new Body[sensorData.bodyCount];
		}

		var frameDesc = sensorData.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
		sensorData.colorImageWidth = frameDesc.Width;
		sensorData.colorImageHeight = frameDesc.Height;
		
		if((dwFlags & KinectInterop.FrameSource.TypeColor) != 0)
		{
			sensorData.colorFrameReader = sensorData.kinectSensor.ColorFrameSource.OpenReader();
			sensorData.colorImage = new byte[frameDesc.BytesPerPixel * frameDesc.LengthInPixels];
		}
		
		sensorData.depthImageWidth = sensorData.kinectSensor.DepthFrameSource.FrameDescription.Width;
		sensorData.depthImageHeight = sensorData.kinectSensor.DepthFrameSource.FrameDescription.Height;

		if((dwFlags & KinectInterop.FrameSource.TypeDepth) != 0)
		{
			sensorData.depthFrameReader = sensorData.kinectSensor.DepthFrameSource.OpenReader();
			sensorData.depthImage = new ushort[sensorData.kinectSensor.DepthFrameSource.FrameDescription.LengthInPixels];
		}
		
		if((dwFlags & KinectInterop.FrameSource.TypeBodyIndex) != 0)
		{
			sensorData.bodyIndexFrameReader = sensorData.kinectSensor.BodyIndexFrameSource.OpenReader();
			sensorData.bodyIndexImage = new byte[sensorData.kinectSensor.BodyIndexFrameSource.FrameDescription.LengthInPixels];
		}
		
		if((dwFlags & KinectInterop.FrameSource.TypeInfrared) != 0)
		{
			sensorData.infraredFrameReader = sensorData.kinectSensor.InfraredFrameSource.OpenReader();
			sensorData.infraredImage = new ushort[sensorData.kinectSensor.InfraredFrameSource.FrameDescription.LengthInPixels];
		}
		
		if(!sensorData.kinectSensor.IsOpen)
		{
			sensorData.kinectSensor.Open();
		}

		return sensorData;
	}

	// closes opened readers and closes the sensor
	public static void CloseKinectSensor(SensorData sensorData)
	{
		if(sensorData != null)
		{
			if(sensorData.coordMapper != null)
			{
				sensorData.coordMapper = null;
			}

			if(sensorData.bodyFrameReader != null)
			{
				sensorData.bodyFrameReader.Dispose();
				sensorData.bodyFrameReader = null;
			}

			if(sensorData.bodyIndexFrameReader != null)
			{
				sensorData.bodyIndexFrameReader.Dispose();
				sensorData.bodyIndexFrameReader = null;
			}

			if(sensorData.colorFrameReader != null)
			{
				sensorData.colorFrameReader.Dispose();
				sensorData.colorFrameReader = null;
			}

			if(sensorData.depthFrameReader != null)
			{
				sensorData.depthFrameReader.Dispose();
				sensorData.depthFrameReader = null;
			}

			if(sensorData.infraredFrameReader != null)
			{
				sensorData.infraredFrameReader.Dispose();
				sensorData.infraredFrameReader = null;
			}
			
			if(sensorData.kinectSensor != null)
			{
				if (sensorData.kinectSensor.IsOpen)
				{
					sensorData.kinectSensor.Close();
				}
				
				sensorData.kinectSensor = null;
			}
		}
	}
	
//	// Returns the latest color frame data, if available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetColorFrameData(int iWidth, int iHeight, uint iBufferLength, ref ColorBuffer pColorFrame, ref Int64 liRelativeTime);
//	
//	// Returns the latest depth frame data, if available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetDepthFrameData(ref DepthBuffer pDepthFrame, ref int iMinDepthDistance, ref int iMaxDepthDistance, ref Int64 liRelativeTime);
//	
//	// Returns the latest infrared frame data, if available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetInfraredFrameData(ref DepthBuffer pInfraredFrame, ref Int64 liRelativeTime);
//	
//	// Returns the latest body-index frame data, if available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetBodyIndexFrameData(ref BodyIndexBuffer pBodyIndexFrame, ref Int64 liRelativeTime);
	

//	// Polls frame data from all opened image streams
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int PollImageFrameData(FrameSource dwFlags);
//	
//	// Gets the new color frame, if one is available. Returns S_OK if a new frame is available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetNewColorFrameData(ref IntPtr pColorFrame, ref Int64 liFrameTime);
//	
//	// Gets the new depth frame, if one available. Returns S_OK if a new frame is available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetNewDepthFrameData(ref IntPtr pDepthFrame, ref Int64 liFrameTime, ref int iMinDepth, ref int iMaxDepth);
//	
//	// Gets the new infrared frame, if one available. Returns S_OK if a new frame is available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetNewInfraredFrameData(ref IntPtr pInfraredFrame, ref Int64 liFrameTime);
//	
//	// Gets the new body index frame, if one available. Returns S_OK if a new frame is available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetNewBodyIndexFrameData(ref IntPtr pBodyIndexFrame, ref Int64 liFrameTime);
	
//	// Gets the new users' histogram frame, if one available. Returns S_OK if a new frame is available
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetUserHistogramFrameData(ref IntPtr pUserHistogramFrame, ref Int64 liFrameTime, bool bUseColorData);
	
	
//	// Returns color frame coordinates of a 3d Kinect-point
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetKinectPointColorCoords(float x, float y, float z, out int cx, out int cy);
//	
//	// Returns depth frame coordinates of a 3d Kinect-point
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetKinectPointDepthCoords(float x, float y, float z, out int dx, out int dy);
//	
//	// Returns Kinect 3d-coordinates of a depth point
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetDepthPointKinectCoords(int dx, int dy, ushort depthValue, out float kx, out float ky, out float kz);
//	
//	// Returns color frame coordinates of a depth point
//	[DllImportAttribute(@"Kinect2UnityClient.dll")]
//	public static extern int GetDepthPointColorCoords(int dx, int dy, ushort depthValue, out int cx, out int cy);
	

	// Public unility functions
	
	// Returns the system error message
	public static string GetSystemErrorMessage(int hr)
	{
		string message = string.Empty;
		uint uhr = (uint)hr;
		
	    const uint FORMAT_MESSAGE_ALLOCATE_BUFFER = 0x00000100;
	    const uint FORMAT_MESSAGE_IGNORE_INSERTS  = 0x00000200;
	    const uint FORMAT_MESSAGE_FROM_SYSTEM    = 0x00001000;
	
	    IntPtr lpMsgBuf = IntPtr.Zero;
	
	    uint dwChars = FormatMessage(
	        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
	        IntPtr.Zero,
	        (uint)hr,
	        0, // Default language
	        ref lpMsgBuf,
	        0,
	        IntPtr.Zero);
		
	    if (dwChars > 0)
		{
		    message = Marshal.PtrToStringAnsi(lpMsgBuf).Trim();
		
		    // Free the buffer.
		    LocalFree(lpMsgBuf);
		}
		else
	    {
	        // Handle the error.
	        message = "hr=0x" + uhr.ToString("X");
	    }
	
		return message;
	}
	
	// returns the parent joint of the given joint
	public static JointType GetParentJoint(JointType joint)
	{
		switch(joint)
		{
			case JointType.SpineBase:
				return JointType.SpineBase;
		
			case JointType.Neck:
				return JointType.SpineShoulder;
		
			case JointType.SpineShoulder:
				return JointType.SpineMid;
		
			case JointType.ShoulderLeft:
			case JointType.ShoulderRight:
				return JointType.SpineShoulder;
		
			case JointType.HipLeft:
			case JointType.HipRight:
				return JointType.SpineBase;
		
			case JointType.HandTipLeft:
			case JointType.ThumbLeft:
				return JointType.HandLeft;
		
			case JointType.HandTipRight:
			case JointType.ThumbRight:
				return JointType.HandRight;
		}
	
		return (JointType)((int)joint - 1);
	}
	
	// returns the next joint in the hierarchy, as to the given joint
	public static JointType GetNextJoint(JointType joint)
	{
		switch(joint)
		{
			case JointType.SpineBase:
				return JointType.SpineMid;
			case JointType.SpineMid:
				return JointType.SpineShoulder;
			case JointType.SpineShoulder:
				return JointType.Neck;
			case JointType.Neck:
				return JointType.Head;
			
			case JointType.ShoulderLeft:
				return JointType.ElbowLeft;
	        case JointType.ElbowLeft:
				return JointType.WristLeft;
	        case JointType.WristLeft:
				return JointType.HandLeft;
	        case JointType.HandLeft:
				return JointType.HandTipLeft;

	        case JointType.ShoulderRight:
				return JointType.ElbowRight;
	        case JointType.ElbowRight:
				return JointType.WristRight;
	        case JointType.WristRight:
				return JointType.HandRight;
	        case JointType.HandRight:
				return JointType.HandTipRight;

	        case JointType.HipLeft:
				return JointType.KneeLeft;
	        case JointType.KneeLeft:
				return JointType.AnkleLeft;
	        case JointType.AnkleLeft:
				return JointType.FootLeft;

	        case JointType.HipRight:
				return JointType.KneeRight;
	        case JointType.KneeRight:
				return JointType.AnkleRight;
	        case JointType.AnkleRight:
				return JointType.FootRight;
		}
		
		return joint;  // in case of end joint - Head, HandTipLeft, HandTipLeft, FootLeft, FootRight
	}
	
	// returns the mirror joint of the given joint
	public static JointType GetMirrorJoint(JointType joint)
	{
		switch(joint)
		{
			case JointType.ShoulderLeft:
				return JointType.ShoulderRight;
	        case JointType.ElbowLeft:
				return JointType.ElbowRight;
	        case JointType.WristLeft:
				return JointType.WristRight;
	        case JointType.HandLeft:
				return JointType.HandRight;
					
	        case JointType.ShoulderRight:
				return JointType.ShoulderLeft;
	        case JointType.ElbowRight:
				return JointType.ElbowLeft;
	        case JointType.WristRight:
				return JointType.WristLeft;
	        case JointType.HandRight:
				return JointType.HandLeft;
					
	        case JointType.HipLeft:
				return JointType.HipRight;
	        case JointType.KneeLeft:
				return JointType.KneeRight;
	        case JointType.AnkleLeft:
				return JointType.AnkleRight;
	        case JointType.FootLeft:
				return JointType.FootRight;
					
	        case JointType.HipRight:
				return JointType.HipLeft;
	        case JointType.KneeRight:
				return JointType.KneeLeft;
	        case JointType.AnkleRight:
				return JointType.AnkleLeft;
	        case JointType.FootRight:
				return JointType.FootLeft;
					
	        case JointType.HandTipLeft:
				return JointType.HandTipRight;
	        case JointType.ThumbLeft:
				return JointType.ThumbRight;
			
	        case JointType.HandTipRight:
				return JointType.HandTipLeft;
	        case JointType.ThumbRight:
				return JointType.ThumbLeft;
		}
	
		return joint;
	}
	
	// Polls for new skeleton data
	public static bool PollBodyData(SensorData sensorData, ref BodyFrameData bodyFrame)
	{
		bool bNewFrame = false;

//		BodyFrameData tempBodyFrame = new BodyFrameData();
//		int hr = GetBodyFrameData(ref tempBodyFrame, true, true);
//		
////		if((hr == 0) && ((sensorData.colorImage == null) || (tempBodyFrame.liRelativeTime >= sensorData.lastColorFrameTime)) &&
////		   ((sensorData.depthImage == null) || (tempBodyFrame.liRelativeTime >= sensorData.lastDepthFrameTime)))
//		{
//			if(hr == 0 /**&& (bodyFrame.liRelativeTime > lastFrameTime)*/)
//			{
//				bodyFrame = tempBodyFrame;
//				sensorData.lastBodyFrameTime = bodyFrame.liRelativeTime;
//				bNewFrame = true;
//			}
//		}

		if(sensorData.bodyFrameReader != null)
		{
			var frame = sensorData.bodyFrameReader.AcquireLatestFrame();

			if(frame != null)
			{
				frame.GetAndRefreshBodyData(sensorData.bodyData);
				bodyFrame.liRelativeTime = frame.RelativeTime;
				
				frame.Dispose();
				frame = null;

				for(int i = 0; i < sensorData.bodyCount; i++)
				{
					Body body = sensorData.bodyData[i];

					if (body == null)
					{
						bodyFrame.bodyData[i].bIsTracked = 0;
						continue;
					}

					bodyFrame.bodyData[i].bIsTracked = (short)(body.IsTracked ? 1 : 0);
					if(body.IsTracked)
					{
						bodyFrame.bodyData[i].liTrackingID = (long)body.TrackingId;

						for(int j = 0; j < Constants.JointCount; j++)
						{
							Windows.Kinect.Joint joint = body.Joints[(JointType)j];
							JointData jointData = bodyFrame.bodyData[i].joint[j];

							jointData.jointType = (JointType)j;
							jointData.trackingState = joint.TrackingState;

							jointData.position = new Vector3(joint.Position.X, joint.Position.Y, joint.Position.Z);
							jointData.kinectPos = jointData.position;

							//Debug.Log(jointData.jointType + " - " + joint.TrackingState + " " + jointData.kinectPos);

							if(j == 0)
							{
								jointData.direction = Vector3.zero;
							}
							else
							{
								int jParent = (int)GetParentJoint(jointData.jointType);

								if(jointData.trackingState != TrackingState.NotTracked && bodyFrame.bodyData[i].joint[jParent].trackingState != TrackingState.NotTracked)
								{
									jointData.direction = jointData.kinectPos - bodyFrame.bodyData[i].joint[jParent].kinectPos;
								}
							}

							jointData.orientation = Quaternion.identity;
//							Windows.Kinect.Vector4 vQ = body.JointOrientations[jointData.jointType].Orientation;
//							Quaternion orientation = new Quaternion(vQ.X, vQ.Y, vQ.Z, vQ.W);
//
//							if(j != 0)
//							{
//								Quaternion parentOri = bodyFrame.bodyData[i].joint[jParent].orientation;
//								jointData.orientation = parentOri * orientation;
//							}
//							else
//							{
//								jointData.orientation = orientation;
//							}

							if(j == 0)
							{
								bodyFrame.bodyData[i].position = jointData.position;
								bodyFrame.bodyData[i].orientation = jointData.orientation;
							}

							bodyFrame.bodyData[i].joint[j] = jointData;
						}

						bodyFrame.bodyData[i].leftHandState = body.HandLeftState;
						bodyFrame.bodyData[i].leftHandConfidence = body.HandLeftConfidence;
						
						bodyFrame.bodyData[i].rightHandState = body.HandRightState;
						bodyFrame.bodyData[i].rightHandConfidence = body.HandRightConfidence;
					}
				}

				bNewFrame = true;
			}
		}

		return bNewFrame;
	}
	
	// Polls for new color frame data
	//public static bool PollColorFrame(ref ColorBuffer colorImage)
	public static bool PollColorFrame(SensorData sensorData)
	{
		bool bNewFrame = false;

//		IntPtr imagePtr = IntPtr.Zero;
//		Int64 liFrameTime = 0;
//	
//		int hr = PollImageFrameData(FrameSource.TypeColor);
//		if (hr == 0)
//		{
//			hr = GetNewColorFrameData(ref imagePtr, ref liFrameTime);
//			
//			if(hr == 0)
//			{
//				colorImage = (ColorBuffer)Marshal.PtrToStructure(imagePtr, typeof(ColorBuffer));
//				bNewFrame = true;
//			}
//		}

		if(sensorData.colorFrameReader != null) 
		{
			var colorFrame = sensorData.colorFrameReader.AcquireLatestFrame();
			
			if(colorFrame != null)
			{
				colorFrame.CopyConvertedFrameDataToArray(sensorData.colorImage, ColorImageFormat.Rgba);
				sensorData.lastColorFrameTime = colorFrame.RelativeTime;

				colorFrame.Dispose();
				colorFrame = null;

				bNewFrame = true;
			}
		}

		return bNewFrame;
	}
	
	// Polls for new depth frame data
	//public static bool PollDepthFrame(ref DepthBuffer depthImage, ref BodyIndexBuffer bodyIndexImage, ref int minDepth, ref int maxDepth)
	public static bool PollDepthFrame(SensorData sensorData)
	{
		bool bNewFrame = false;

//		IntPtr imagePtr = IntPtr.Zero;
//		Int64 liFrameTime = 0;
//	
//		int hr = PollImageFrameData(FrameSource.TypeDepth | FrameSource.TypeBodyIndex);
//		if (hr == 0)
//		{
//			hr = GetNewDepthFrameData(ref imagePtr, ref liFrameTime, ref minDepth, ref maxDepth);
//			
//			if(hr == 0)
//			{
//				depthImage = (DepthBuffer)Marshal.PtrToStructure(imagePtr, typeof(DepthBuffer));
//
//				hr = GetNewBodyIndexFrameData(ref imagePtr, ref liFrameTime);
//				
//				if(hr == 0)
//				{
//					bodyIndexImage = (BodyIndexBuffer)Marshal.PtrToStructure(imagePtr, typeof(BodyIndexBuffer));
//					bNewFrame = true;
//				}
//			}
//		}
		
		if(sensorData.depthFrameReader != null)
		{
			var depthFrame = sensorData.depthFrameReader.AcquireLatestFrame();

			if(depthFrame != null)
			{
				depthFrame.CopyFrameDataToArray(sensorData.depthImage);
				sensorData.lastDepthFrameTime = depthFrame.RelativeTime;

				depthFrame.Dispose();
				depthFrame = null;

				bNewFrame = true;
			}

			if(sensorData.depthFrameReader != null)
			{
				var bodyIndexFrame = sensorData.bodyIndexFrameReader.AcquireLatestFrame();
				
				if(bodyIndexFrame != null)
				{
					bodyIndexFrame.CopyFrameDataToArray(sensorData.bodyIndexImage);
					sensorData.lastBodyIndexFrameTime = bodyIndexFrame.RelativeTime;

					bodyIndexFrame.Dispose();
					bodyIndexFrame = null;
					
					bNewFrame = true;
				}
			}
		}

		return bNewFrame;
	}
	
	// Polls for new infrared frame data
	//public static bool PollInfraredFrame(ref DepthBuffer infraredImage)
	public static bool PollInfraredFrame(SensorData sensorData)
	{
		bool bNewFrame = false;

//		IntPtr imagePtr = IntPtr.Zero;
//		Int64 liFrameTime = 0;
//	
//		int hr = PollImageFrameData(FrameSource.TypeInfrared);
//		if (hr == 0)
//		{
//			hr = GetNewInfraredFrameData(ref imagePtr, ref liFrameTime);
//			
//			if(hr == 0)
//			{
//				infraredImage = (DepthBuffer)Marshal.PtrToStructure(imagePtr, typeof(DepthBuffer));
//				bNewFrame = true;
//			}
//		}
		
		if(sensorData.infraredFrameReader != null)
		{
			var infraredFrame = sensorData.infraredFrameReader.AcquireLatestFrame();
			
			if(infraredFrame != null)
			{
				infraredFrame.CopyFrameDataToArray(sensorData.infraredImage);
				sensorData.lastInfraredFrameTime = infraredFrame.RelativeTime;

				infraredFrame.Dispose();
				infraredFrame = null;
				
				bNewFrame = true;
			}
		}
		
		return bNewFrame;
	}
	
//	// Polls for new color histogram frame data
//	public static bool PollUserHistogramFrame(ref UserHistogramBuffer userHistImage, bool bUseColorData)
//	{
//		bool bNewFrame = false;
//
//		IntPtr imagePtr = IntPtr.Zero;
//		Int64 liFrameTime = 0;
//	
//		int hr = GetUserHistogramFrameData(ref imagePtr, ref liFrameTime, bUseColorData);
//		
//		if(hr == 0)
//		{
//			userHistImage = (UserHistogramBuffer)Marshal.PtrToStructure(imagePtr, typeof(UserHistogramBuffer));
//			bNewFrame = true;
//		}
//		
//		return bNewFrame;
//	}
	
	// returns depth frame coordinates for the given 3d Kinect-space point
	public static Vector2 GetKinectPointDepthCoords(SensorData sensorData, Vector3 kinectPos)
	{
		Vector2 vPoint = Vector2.zero;

//		int dx = 0, dy = 0;

//		int hr = GetKinectPointDepthCoords(kinectPos.x, kinectPos.y, kinectPos.z, out dx, out dy);
//		
//		if(hr == 0)
//		{
//			return new Vector2(dx, dy);
//		}
		
		if(sensorData.coordMapper != null)
		{
			CameraSpacePoint camPoint = new CameraSpacePoint();
			camPoint.X = kinectPos.x;
			camPoint.Y = kinectPos.y;
			camPoint.Z = kinectPos.z;

			CameraSpacePoint[] camPoints = new CameraSpacePoint[1];
			camPoints[0] = camPoint;

			DepthSpacePoint[] depthPoints = new DepthSpacePoint[1];
			sensorData.coordMapper.MapCameraPointsToDepthSpace(camPoints, depthPoints);

			DepthSpacePoint depthPoint = depthPoints[0];

			if(depthPoint.X >= 0 && depthPoint.X < sensorData.depthImageWidth &&
			   depthPoint.Y >= 0 && depthPoint.Y < sensorData.depthImageHeight)
			{
				vPoint.x = depthPoint.X;
				vPoint.y = depthPoint.Y;
			}
		}

		return vPoint;
	}
	
	
	// Copy a resource file to the target
	private static bool CopyResourceFile(string targetFilePath, string resFileName, ref bool bOneCopied, ref bool bAllCopied)
	{
		TextAsset textRes = Resources.Load(resFileName, typeof(TextAsset)) as TextAsset;
		if(textRes == null)
		{
			bOneCopied = false;
			bAllCopied = false;
			
			return false;
		}
		
		FileInfo targetFile = new FileInfo(targetFilePath);
		if(!targetFile.Directory.Exists)
		{
			targetFile.Directory.Create();
		}
		
		if(!targetFile.Exists || targetFile.Length !=  textRes.bytes.Length)
		{
			if(textRes != null)
			{
				using (FileStream fileStream = new FileStream (targetFilePath, FileMode.Create, FileAccess.Write, FileShare.Read))
				{
					fileStream.Write(textRes.bytes, 0, textRes.bytes.Length);
				}
				
				bool bFileCopied = File.Exists(targetFilePath);
				
				bOneCopied = bOneCopied || bFileCopied;
				bAllCopied = bAllCopied && bFileCopied;
				
				return bFileCopied;
			}
		}
		
		return false;
	}
	
	// Copies the needed resources into the project directory
	public static bool EnsureKinectWrapperPresence()
	{
		bool bOneCopied = false, bAllCopied = true;
		
//		CopyResourceFile("KinectServer/Kinect2UnityServer.exe", "Kinect2UnityServer.exe", ref bOneCopied, ref bAllCopied);
//		CopyResourceFile("KinectServer/msvcp110d.dll", "msvcp110d.dll.x64", ref bOneCopied, ref bAllCopied);
//		CopyResourceFile("KinectServer/msvcr110d.dll", "msvcr110d.dll.x64", ref bOneCopied, ref bAllCopied);
//		
//		CopyResourceFile("Kinect2UnityClient.dll", "Kinect2UnityClient.dll", ref bOneCopied, ref bAllCopied);
//		CopyResourceFile("msvcp110d.dll", "msvcp110d.dll.i386", ref bOneCopied, ref bAllCopied);
//		CopyResourceFile("msvcr110d.dll", "msvcr110d.dll.i386", ref bOneCopied, ref bAllCopied);

		CopyResourceFile("KinectForUnity.dll", "KinectForUnity.dll", ref bOneCopied, ref bAllCopied);
		
		return bOneCopied && bAllCopied;
	}
	
}