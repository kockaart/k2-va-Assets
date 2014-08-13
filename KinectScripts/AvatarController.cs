using UnityEngine;
using Windows.Kinect;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using System.Text; 

public class AvatarController : MonoBehaviour
{	
	// The index of the player, whose movements the model represents. Default 0 (first player)
	public int playerIndex = 0;
	
	// Bool that has the characters (facing the player) actions become mirrored. Default false.
	public bool MirroredMovement = false;
	
	// Bool that determines whether the avatar is allowed to do vertical movement
	public bool VerticalMovement = false;
	
	// Rate at which avatar will move through the scene.
	private int MoveRate = 1;
	
	// Slerp smooth factor
	public float SmoothFactor = 10.0f;
	
	
	// Public variables that will get matched to bones. If empty, the Kinect will simply not track it.
	public Transform HipCenter;
	public Transform Spine;
	public Transform ShoulderCenter;
	public Transform Neck;
	public Transform Head;

	public Transform ClavicleLeft;
	public Transform ShoulderLeft;
	public Transform ElbowLeft; 
	public Transform HandLeft;
	public Transform FingersLeft;
	private Transform FingerTipsLeft = null;
	private Transform ThumbLeft = null;

	public Transform ClavicleRight;
	public Transform ShoulderRight;
	public Transform ElbowRight;
	public Transform HandRight;
	public Transform FingersRight;
	private Transform FingerTipsRight = null;
	private Transform ThumbRight = null;
	
	public Transform HipLeft;
	public Transform KneeLeft;
	public Transform FootLeft;
	private Transform ToesLeft = null;
	
	public Transform HipRight;
	public Transform KneeRight;
	public Transform FootRight;
	private Transform ToesRight = null;
	
	public Transform BodyRoot;

	// A required variable if you want to rotate the model in space.
	public GameObject offsetNode;
	
	// Variable to hold all them bones. It will initialize the same size as initialRotations.
	private Transform[] bones;
	
	// Rotations of the bones when the Kinect tracking starts.
    private Quaternion[] initialRotations;
	private Quaternion[] initialLocalRotations;
	
	// Rotations of the bones when the Kinect tracking starts.
    private Vector3[] initialDirections;

	// Initial position and rotation of the transform
	private Vector3 initialPosition;
	private Quaternion initialRotation;
	
	// Calibration Offset Variables for Character Position.
	private bool OffsetCalibrated = false;
	private float XOffset, YOffset, ZOffset;
	private Quaternion originalRotation;
	
	// private instance of the KinectManager
	private KinectManager kinectManager;

	private Vector3 hipsUp;  // up vector of the hips
	private Vector3 hipsRight;  // right vector of the hips
	private Vector3 chestRight;  // right vectory of the chest
	
	
    public void Start()
    {	
		// Holds our bones for later.
		bones = new Transform[25];
		
		// Initial rotations and directions of the bones.
		initialRotations = new Quaternion[bones.Length];
		initialLocalRotations = new Quaternion[bones.Length];
		initialDirections = new Vector3[bones.Length];
		
		// Map bones to the points the Kinect tracks
		MapBones();

		// Get initial bone directions
		GetInitialDirections();
		
		// Get initial bone rotations
		GetInitialRotations();
	}
	
	// Update the avatar each frame.
    public void UpdateAvatar(Int64 UserID)
    {	
		bool flipJoint = !MirroredMovement;
		
		// Get the KinectManager instance
		if(kinectManager == null)
		{
			kinectManager = KinectManager.Instance;
		}

		// move the avatar to its Kinect position
		MoveAvatar(UserID);

		// Update Head, Neck, Spine, and Hips
		TransformBone(UserID, JointType.SpineBase, 0, flipJoint);
		TransformBone(UserID, JointType.SpineMid, 1, flipJoint);
		TransformBone(UserID, JointType.SpineShoulder, 2, flipJoint);
		TransformBone(UserID, JointType.Neck, 3, flipJoint);
		TransformBone(UserID, JointType.Head, 4, flipJoint);
		
		// Beyond this, switch the arms and legs.
		
		// Left Arm --> Right Arm
		//TransformBoneDir(UserID, JointType.SpineShoulder, JointType.ShoulderLeft, !MirroredMovement ? 5 : 11, flipJoint);
		TransformBone(UserID, JointType.ShoulderLeft, !MirroredMovement ? 5 : 11, flipJoint);
		TransformBone(UserID, JointType.ElbowLeft, !MirroredMovement ? 6 : 12, flipJoint);
		TransformBone(UserID, JointType.WristLeft, !MirroredMovement ? 7 : 13, flipJoint);
		TransformBone(UserID, JointType.HandLeft, !MirroredMovement ? 8 : 14, flipJoint);
		TransformBone(UserID, JointType.HandTipLeft, !MirroredMovement ? 9 : 15, flipJoint);
		TransformBone(UserID, JointType.ThumbLeft, !MirroredMovement ? 10 : 16, flipJoint);

		// Right Arm --> Left Arm
		//TransformBoneDir(UserID, JointType.SpineShoulder, JointType.ShoulderRight, !MirroredMovement ? 11 : 5, flipJoint);
		TransformBone(UserID, JointType.ShoulderRight, !MirroredMovement ? 11 : 5, flipJoint);
		TransformBone(UserID, JointType.ElbowRight, !MirroredMovement ? 12 : 6, flipJoint);
		TransformBone(UserID, JointType.WristRight, !MirroredMovement ? 13 : 7, flipJoint);
		TransformBone(UserID, JointType.HandRight, !MirroredMovement ? 14 : 8, flipJoint);
		TransformBone(UserID, JointType.HandTipRight, !MirroredMovement ? 15 : 9, flipJoint);
		TransformBone(UserID, JointType.ThumbRight, !MirroredMovement ? 16 : 10, flipJoint);

		// Left Leg --> Right Leg
		TransformBone(UserID, JointType.HipLeft, !MirroredMovement ? 17 : 21, flipJoint);
		TransformBone(UserID, JointType.KneeLeft, !MirroredMovement ? 18 : 22, flipJoint);
		TransformBone(UserID, JointType.AnkleLeft, !MirroredMovement ? 19 : 23, flipJoint);
		TransformBone(UserID, JointType.FootLeft, !MirroredMovement ? 20 : 24, flipJoint);

		// Right Leg --> Left Leg
		TransformBone(UserID, JointType.HipRight, !MirroredMovement ? 21 : 17, flipJoint);
		TransformBone(UserID, JointType.KneeRight, !MirroredMovement ? 22 : 18, flipJoint);
		TransformBone(UserID, JointType.AnkleRight, !MirroredMovement ? 23 : 19, flipJoint);
		TransformBone(UserID, JointType.FootRight, !MirroredMovement ? 24 : 20, flipJoint);	
	}
	
	// Set bones to their initial positions and rotations.
	public void ResetToInitialPosition()
    {	
		if(bones == null)
			return;
		
		if(offsetNode != null)
		{
			// Set the offset's rotation to 0.
			offsetNode.transform.rotation = Quaternion.Euler(Vector3.zero);
		}

		transform.position = initialPosition;
		transform.rotation = initialRotation;

		// For each bone that was defined, reset to initial position.
		for (int i = 0; i < bones.Length; i++)
		{
			if (bones[i] != null)
			{
				bones[i].rotation = initialRotations[i];
			}
		}

		if(BodyRoot != null && BodyRoot.parent != null)
		{
			BodyRoot.parent.localPosition = Vector3.zero;
		}

		if(offsetNode != null)
		{
			// Restore the offset's rotation
			offsetNode.transform.rotation = originalRotation;
		}
    }
	
	// Invoked on the successful calibration of a player.
	public void SuccessfulCalibration(Int64 userId)
	{
		// reset the models position
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = originalRotation;
		}
		
		// re-calibrate the position offset
		OffsetCalibrated = false;
	}
	
	// Apply the rotations tracked by kinect to the joints.
    void TransformBone(Int64 userId, JointType joint, int boneIndex, bool flip)
    {
		Transform boneTransform = bones[boneIndex];
		if(boneTransform == null || kinectManager == null)
			return;
		
		int iJoint = (int)joint;
		if(iJoint < 0 || !kinectManager.IsJointTracked(userId, iJoint))
			return;
		
		// Get Kinect joint orientation
		Quaternion jointRotation = kinectManager.GetJointOrientation(userId, iJoint, flip);
		if(jointRotation == Quaternion.identity)
			return;

		// Smoothly transition to the new rotation
		Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

		if(SmoothFactor != 0f)
        	boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, SmoothFactor * Time.deltaTime);
		else
			boneTransform.rotation = newRotation;
	}
	
	// Moves the avatar in 3D space - pulls the tracked position of the spine and applies it to root.
	// Only pulls positional, not rotational.
	void MoveAvatar(Int64 UserID)
	{
		if(!kinectManager || !kinectManager.IsJointTracked(UserID, (int)JointType.SpineBase))
			return;
		
        // Get the position of the body and store it.
		Vector3 trans = kinectManager.GetUserPosition(UserID);
		
		// If this is the first time we're moving the avatar, set the offset. Otherwise ignore it.
		if (!OffsetCalibrated)
		{
			OffsetCalibrated = true;
			
			XOffset = !MirroredMovement ? trans.x * MoveRate : -trans.x * MoveRate;
			YOffset = trans.y * MoveRate;
			ZOffset = -trans.z * MoveRate;
		}
	
		// Smoothly transition to the new position
		Vector3 targetPos = Kinect2AvatarPos(trans, VerticalMovement);

		if(BodyRoot != null && BodyRoot.parent != null)
		{
			BodyRoot.parent.localPosition = SmoothFactor != 0f ? 
				Vector3.Lerp(BodyRoot.parent.localPosition, targetPos, SmoothFactor * Time.deltaTime) : targetPos;
		}
		else
		{
			transform.localPosition = SmoothFactor != 0f ? 
				Vector3.Lerp(transform.localPosition, targetPos, SmoothFactor * Time.deltaTime) : targetPos;
		}
	}
	
	// If the bones to be mapped have been declared, map that bone to the model.
	void MapBones()
	{
		bones[0] = HipCenter;
		bones[1] = Spine;
		bones[2] = ShoulderCenter;
		bones[3] = Neck;
		bones[4] = Head;
	
		bones[5] = ShoulderLeft;
		bones[6] = ElbowLeft;
		bones[7] = HandLeft;
		bones[8] = FingersLeft;
		bones[9] = FingerTipsLeft;
		bones[10] = ThumbLeft;
	
		bones[11] = ShoulderRight;
		bones[12] = ElbowRight;
		bones[13] = HandRight;
		bones[14] = FingersRight;
		bones[15] = FingerTipsRight;
		bones[16] = ThumbRight;
	
		bones[17] = HipLeft;
		bones[18] = KneeLeft;
		bones[19] = FootLeft;
		bones[20] = ToesLeft;
	
		bones[21] = HipRight;
		bones[22] = KneeRight;
		bones[23] = FootRight;
		bones[24] = ToesRight;
	}
	
	// Capture the initial directions of the bones
	void GetInitialDirections()
	{
		int[] intBone = { 1, 2, 3, 5, 6, 7, 11, 12, 13, 17, 18, 19, 21, 22, 23 };
		int[] endBone = { 4, 8, 14, 20, 24 };
		
		for (int i = 0; i < bones.Length; i++)
		{
			if(Array.IndexOf(intBone, i) >= 0)
			{
				// intermediary joint
				if(bones[i] && bones[i + 1])
				{
					initialDirections[i] = bones[i + 1].position - bones[i].position;
					initialDirections[i] = bones[i].InverseTransformDirection(initialDirections[i]);
				}
				else
				{
					initialDirections[i] = Vector3.zero;
				}
			}
			else if(Array.IndexOf(endBone, i) >= 0)
			{
				initialDirections[i] = initialDirections[i - 1];
			}
			else
			{
				// end joint
				initialDirections[i] = Vector3.zero;
			}
		}
		
		// special directions
		if(HipCenter && HipLeft && HipRight)
		{
			hipsUp = ((HipRight.position + HipLeft.position) / 2.0f) - HipCenter.position;
			hipsUp = HipCenter.InverseTransformDirection(hipsUp);
			
			hipsRight = HipRight.position - HipLeft.position;
			hipsRight = HipCenter.InverseTransformDirection(hipsRight);
			
			// make hipRight orthogonal to the direction of the hips
			Vector3.OrthoNormalize(ref hipsUp, ref hipsRight);
		}
		
		if(ShoulderCenter && ShoulderLeft && ShoulderRight)
		{
			chestRight = ShoulderRight.position - ShoulderLeft.position;
			chestRight = ShoulderCenter.InverseTransformDirection(chestRight);
			
			// make chestRight orthogonal to the direction of the spine
			chestRight -= Vector3.Project(chestRight, initialDirections[2]);
		}
	}

	// Capture the initial rotations of the bones
	void GetInitialRotations()
	{
		if(offsetNode != null)
		{
			// Store the original offset's rotation.
			originalRotation = offsetNode.transform.rotation;
			// Set the offset's rotation to 0.
			offsetNode.transform.rotation = Quaternion.Euler(Vector3.zero);
		}

		initialPosition = transform.position;
		initialRotation = transform.rotation;
		
		for (int i = 0; i < bones.Length; i++)
		{
			if (bones[i] != null)
			{
				initialRotations[i] = bones[i].rotation; // * Quaternion.Inverse(initialRotation);
				initialLocalRotations[i] = bones[i].localRotation;
			}
		}

		if(offsetNode != null)
		{
			// Restore the offset's rotation
			offsetNode.transform.rotation = originalRotation;
		}
	}
	
	// Converts kinect joint rotation to avatar joint rotation, depending on joint initial rotation and offset rotation
	Quaternion Kinect2AvatarRot(Quaternion jointRotation, int boneIndex)
	{
		Quaternion newRotation = jointRotation * initialRotations[boneIndex];

		if (offsetNode != null)
		{
			Vector3 totalRotation = newRotation.eulerAngles + offsetNode.transform.rotation.eulerAngles;
			newRotation = Quaternion.Euler(totalRotation);
		}
		
		return newRotation;
	}
	
	// Converts Kinect position to avatar skeleton position, depending on initial position, mirroring and move rate
	Vector3 Kinect2AvatarPos(Vector3 jointPosition, bool bMoveVertically)
	{
		float xPos;

		if(!MirroredMovement)
			xPos = jointPosition.x * MoveRate - XOffset;
		else
			xPos = -jointPosition.x * MoveRate - XOffset;
		
		float yPos = jointPosition.y * MoveRate - YOffset;
		float zPos = -jointPosition.z * MoveRate - ZOffset;
		
		Vector3 newPosition = new Vector3(xPos, bMoveVertically ? yPos : 0f, zPos);

		return newPosition;
	}
	
}

