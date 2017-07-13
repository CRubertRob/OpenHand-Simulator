#ifndef HAND_H
#define HAND_H

#include <rave/rave.h>

using namespace std;
using namespace OpenRAVE;

class Segment
{
public:
	Segment(string segmentName){
		name = segmentName;
		length = 0.0;
		thickness = 0.0;
		CS_Location[0] = 0.0; CS_Location[1] = 0.0; CS_Location[2] = 0.0;
		CS_RotationAngles[0] = 0.0; CS_RotationAngles[1] = 0.0; CS_RotationAngles[2] = 0.0;
	}
	void SetLength(float l){length = l;}
	void SetThickness(float t){thickness = t;}
	void SetCSLocation(float x, float y, float z)
	{
		CS_Location[0]=x;
		CS_Location[1]=y;
		CS_Location[2]=z;
	}
	void SetCSRotationAngles(float alphaX, float alphaY, float alphaZ)
	{
		CS_RotationAngles[0]=alphaX;
		CS_RotationAngles[1]=alphaY;
		CS_RotationAngles[2]=alphaZ;
	}

	float GetLength(){return length;}
	float GetThickness(){return thickness;}

	float GetCSLocationX(){return CS_Location[0];}
	float GetCSLocationY(){return CS_Location[1];}
	float GetCSLocationZ(){return CS_Location[2];}

	float GetCSRotationAngleX(){return CS_RotationAngles[0];}
	float GetCSRotationAngleY(){return CS_RotationAngles[1];}
	float GetCSRotationAngleZ(){return CS_RotationAngles[2];}


	string GetName(){return name;}

private:
	string name;
	float 	length;
	float 	thickness;
	float 	CS_Location[3];
	float 	CS_RotationAngles[3];
};

class Finger
{
public:
	Finger(string fingerName){
		name = fingerName;
		Metacarpal = new Segment("Metacarpal");
		Proximal = new Segment("Proximal");
		Medial = new Segment("Medial");
		Distal = new Segment("Distal");
	};
	Segment* GetMetacarpal(){return Metacarpal;}
	Segment* GetProximal(){return Proximal;}
	Segment* GetMedial(){return Medial;}
	Segment* GetDistal(){return Distal;}
	string GetName(){return name;}
	void SetKinBody(KinBodyPtr k){kinbody = k;}
	KinBodyPtr GetKinBody(){return kinbody;}
	void SetCMCLimits(float EF_lower,float EF_Upper,float AA_lower,float AA_upper){
		CMCLimits = new float[4];
		CMCLimits[0]= EF_lower;
		CMCLimits[1]= EF_Upper;
		CMCLimits[2]= AA_lower;
		CMCLimits[3]= AA_upper;
	}
	float* GetCMCLimits(){
		return CMCLimits;
	}
	void SetMCPLimits(float EF_lower,float EF_Upper,float AA_lower,float AA_upper){
		MCPLimits = new float[4];
		MCPLimits[0]= EF_lower;
		MCPLimits[1]= EF_Upper;
		MCPLimits[2]= AA_lower;
		MCPLimits[3]= AA_upper;
	}

	float* GetMCPLimits(){
		return MCPLimits;
	}

	void SetPIPLimits(float EF_lower,float EF_Upper){
		PIPLimits = new float[2];
		PIPLimits[0]= EF_lower;
		PIPLimits[1]= EF_Upper;
	}
	float* GetPIPLimits(){
		return PIPLimits;
	}

	void SetDIPLimits(float EF_lower,float EF_Upper)
	{
		DIPLimits = new float[2];
		DIPLimits[0]= EF_lower;
		DIPLimits[1]= EF_Upper;
	}
	float* GetDIPLimits(){
		return DIPLimits;
	}

	void SetIPLimits(float EF_lower,float EF_Upper)
	{
		IPLimits = new float[2];
		IPLimits[0]= EF_lower;
		IPLimits[1]= EF_Upper;
	}
	float* GetIPLimits(){
		return IPLimits;
	}

private:
	string name;
	Segment* Metacarpal;
	Segment* Proximal;
	Segment* Medial;
	Segment* Distal;
	KinBodyPtr kinbody;

	float* CMCLimits;
	float* MCPLimits;
	float* PIPLimits;
	float* DIPLimits;
	float* IPLimits;


};

class Hand
{
public:
	Hand(){
		Thumb = new Finger("Thumb");
		Index = new Finger("Index");
		Middle = new Finger("Middle");
		Ring = new Finger("Ring");
		Small = new Finger("Small");
	};
	Finger* GetThumb(){return Thumb;}
	Finger* GetIndex(){return Index;}
	Finger* GetMiddle(){return Middle;}
	Finger* GetRing(){return Ring;}
	Finger* GetSmall(){return Small;}

	void SetHB(float hb){HB = hb;}
	void SetHL(float hl){HL = hl;}

	float GetHB(){return HB;}
	float GetHL(){return HL;}
	void SetKinBodyRobot(RobotBasePtr k){robotKinBody = k;}
	RobotBasePtr GetKinBodyRobot(){return robotKinBody;}

private:
	Finger* Thumb;
	Finger* Index;
	Finger* Middle;
	Finger* Ring;
	Finger* Small;

    float HB;
    float HL;
    RobotBasePtr robotKinBody;
};

class Arm
{
public:
	Arm()
	{
		name = "Arm";
		foreArm = new Segment("foreArm");
		arm = new Segment("arm");
		wrist = new Segment("wrist");

	};
	Segment* GetWrist(){return wrist;}
	Segment* GetForeArm(){return foreArm;}
	Segment* GetArm(){return arm;}
	string GetName() {return name;}

	void SetKinBodyRobot(RobotBasePtr k){robotKinBody = k;}
	RobotBasePtr GetKinBodyRobot(){return robotKinBody;}

	void SetWirstLimits(float EF_lower,float EF_Upper,float AA_lower,float AA_upper)
	{
		wristLimits = new float[4];
		wristLimits[0]= EF_lower;
		wristLimits[1]= EF_Upper;
		wristLimits[2]= AA_lower;
		wristLimits[3]= AA_upper;
	}
	float* GetWirstLimits()
	{
		return wristLimits;
	}

	void SetShoulderLimits(float fi_lower,float fi_Upper,float theta_lower,float theta_upper, float psi_lower,float psi_upper)
	{
		shoulderLimits = new float[6];
		shoulderLimits[0]= fi_lower;
		shoulderLimits[1]= fi_Upper;
		shoulderLimits[2]= theta_lower;
		shoulderLimits[3]= theta_upper;
		shoulderLimits[4]= psi_lower;
		shoulderLimits[5]= psi_upper;
	}

	float* GetShoulderLimits()
	{
		return shoulderLimits;
	}

	void SetElbowLimits(float EF_lower,float EF_Upper,float PS_lower,float PS_upper)
	{
		elbowLimits = new float[4];
		elbowLimits[0]= EF_lower;
		elbowLimits[1]= EF_Upper;
		elbowLimits[2]= PS_lower;
		elbowLimits[3]= PS_upper;
	}
	float* GetElbowLimits()
	{
		return elbowLimits;
	}

private:
	string name;
	Segment* foreArm;
	Segment* arm;
	Segment* wrist;

	RobotBasePtr robotKinBody;

	float* wristLimits;
	float* elbowLimits;
	float* shoulderLimits;
};

class HandKinematicModel
{
public:
	void SetHand(Hand* handPtr){
		hand = handPtr;
	}
	virtual void SetBonesLength()=0;
	virtual void SetBonesThickness()=0;
	virtual void SetCS()=0;
	virtual void SetFingerLimits()=0;


protected:
	Hand* hand;

};

class ArmKinematicModel
{
public:
	void SetArm(Arm* armPtr)
	{
		arm = armPtr;
	}

	virtual void SetBonesLength()=0;
	virtual void SetBonesThickness()=0;
	virtual void SetCS()=0;
	virtual void SetSegmentLimits()=0;

protected:
	Arm * arm;
};

#endif
