using UnityEngine;

using System;
using System.Collections;
using System.Collections.Generic;


/// <summary>
/// Filter to correct the joint locations and joint orientations to constraint to range of viable human motion.
/// </summary>
using Windows.Kinect;


public class BoneOrientationsConstraint
{
	public enum ConstraintAxis { X = 0, Y = 1, Z = 2 }
	
    // The Joint Constraints.  
    private readonly List<BoneOrientationConstraint> jointConstraints = new List<BoneOrientationConstraint>();

	private GUIText debugText;
	

    /// Initializes a new instance of the BoneOrientationConstraints class.
    public BoneOrientationsConstraint()
    {
    }

	public void SetDebugText(GUIText debugText)
	{
		this.debugText = debugText;
	}
	
	// find the bone constraint structure for given joint
	// returns the structure index in the list, or -1 if the bone structure is not found
	private int FindBoneOrientationConstraint(int thisJoint)
	{
		for(int i = 0; i < jointConstraints.Count; i++)
		{
			if(jointConstraints[i].thisJoint == thisJoint)
				return i;
		}
		
		// not found
		return -1;
	}

    // AddJointConstraint - Adds a joint constraint to the system.  
    public void AddBoneOrientationConstraint(int thisJoint, ConstraintAxis axis, float angleMin, float angleMax)
    {
		int index = FindBoneOrientationConstraint(thisJoint);
		
		BoneOrientationConstraint jc = index >= 0 ? jointConstraints[index] : new BoneOrientationConstraint(thisJoint);
		
		if(index < 0)
		{
			index = jointConstraints.Count;
			jointConstraints.Add(jc);
		}
		
		AxisOrientationConstraint constraint = new AxisOrientationConstraint(axis, angleMin, angleMax);
		jc.axisConstrainrs.Add(constraint);
		
		jointConstraints[index] = jc;
     }

    // AddDefaultConstraints - Adds a set of default joint constraints for normal human poses.  
    public void AddDefaultConstraints()
    {
		AddBoneOrientationConstraint((int)JointType.Neck, ConstraintAxis.X, -10f, 30f);
		AddBoneOrientationConstraint((int)JointType.Neck, ConstraintAxis.Y, -10f, 10f);
		AddBoneOrientationConstraint((int)JointType.Neck, ConstraintAxis.Z, -30f, 30f);
	}

    // ApplyBoneOrientationConstraints and constrain rotations.
	public void Constrain(ref KinectInterop.BodyData bodyData)
    {
		KinectManager manager = KinectManager.Instance;

        for (int i = 0; i < this.jointConstraints.Count; i++)
        {
            BoneOrientationConstraint jc = this.jointConstraints[i];

			if (jc.thisJoint == (int)JointType.SpineBase || bodyData.joint[jc.thisJoint].normalRotation == Quaternion.identity)
                continue;
			if (bodyData.joint[jc.thisJoint].trackingState == TrackingState.NotTracked)
				continue;

			int prevJoint = (int)KinectInterop.GetParentJoint((JointType)jc.thisJoint);
			if (bodyData.joint[prevJoint].trackingState == TrackingState.NotTracked) 
				continue;

			Quaternion rotJointN = bodyData.joint[jc.thisJoint].normalRotation;
			Quaternion rotParentN = bodyData.joint[prevJoint].normalRotation;

			Quaternion rotLocalN = Quaternion.Inverse(rotParentN) * rotJointN;
			Vector3 eulerAnglesN = rotLocalN.eulerAngles;
			
			Quaternion rotJointM = bodyData.joint[jc.thisJoint].mirroredRotation;
			Quaternion rotParentM = bodyData.joint[prevJoint].mirroredRotation;
			
			Quaternion rotLocalM = Quaternion.Inverse(rotParentM) * rotJointM;
			Vector3 eulerAnglesM = rotLocalM.eulerAngles;
			
			bool isConstrained = false;

			for(int a = 0; a < jc.axisConstrainrs.Count; a++)
			{
				AxisOrientationConstraint ac = jc.axisConstrainrs[a];
				
				Quaternion axisRotation = Quaternion.AngleAxis(eulerAnglesN[ac.axis], ac.rotateAround);
				float angleFromMin = Quaternion.Angle(axisRotation, ac.minQuaternion);
				float angleFromMax = Quaternion.Angle(axisRotation, ac.maxQuaternion);
				 
				if(!(angleFromMin <= ac.angleRange && angleFromMax <= ac.angleRange))
				{
					// correct the axis that has fallen out of range.
					if(angleFromMin > angleFromMax)
					{
						eulerAnglesN[ac.axis] = ac.angleMax;
					}
					else
					{
						eulerAnglesN[ac.axis] = ac.angleMin;
					}

					// fix mirrored rotation as well
					if(ac.axis == 0)
					{
						eulerAnglesM[ac.axis] = eulerAnglesN[ac.axis];
					}
					else
					{
						eulerAnglesM[ac.axis] = -eulerAnglesN[ac.axis];
					}
					
					isConstrained = true;
				}
			}
			
			if(isConstrained)
			{
				rotLocalN = Quaternion.Euler(eulerAnglesN);
				rotJointN = rotParentN * rotLocalN;

				rotLocalM = Quaternion.Euler(eulerAnglesM);
				rotJointM = rotParentM * rotLocalM;
				
				// Put it back into the bone directions
				bodyData.joint[jc.thisJoint].normalRotation = rotJointN;
				bodyData.joint[jc.thisJoint].mirroredRotation = rotJointM;
//				dirJoint = constrainedRotation * dirParent;
//				bodyData.joint[jc.thisJoint].direction = dirJoint;
			}
			
        }
    }


    private struct BoneOrientationConstraint
    {
		public int thisJoint;
		public List<AxisOrientationConstraint> axisConstrainrs;
		
		
		public BoneOrientationConstraint(int thisJoint)
        {
			this.thisJoint = thisJoint;
			axisConstrainrs = new List<AxisOrientationConstraint>();
        }
    }
	
	
	private struct AxisOrientationConstraint
	{
		// the axis to rotate around
		public int axis;
		public Vector3 rotateAround;
				
		// min, max and range of allowed angle
		public float angleMin;
		public float angleMax;
		
		public Quaternion minQuaternion;
		public Quaternion maxQuaternion;
		public float angleRange;
				
		
		public AxisOrientationConstraint(ConstraintAxis axis, float angleMin, float angleMax)
		{
			// Set the axis that we will rotate around
			this.axis = (int)axis;
			
			switch(axis)
			{
				case ConstraintAxis.X:
					this.rotateAround = Vector3.right;
					break;
				 
				case ConstraintAxis.Y:
					this.rotateAround = Vector3.up;
					break;
				 
				case ConstraintAxis.Z:
					this.rotateAround = Vector3.forward;
					break;
			
				default:
					this.rotateAround = Vector3.zero;
					break;
			}
			
			// Set the min and max rotations in degrees
			this.angleMin = angleMin;
			this.angleMax = angleMax;
			
				 
			// Set the min and max rotations in quaternion space
			this.minQuaternion = Quaternion.AngleAxis(angleMin, this.rotateAround);
			this.maxQuaternion = Quaternion.AngleAxis(angleMax, this.rotateAround);
			this.angleRange = angleMax - angleMin;
		}
	}
	
}