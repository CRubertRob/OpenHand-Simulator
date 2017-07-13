#ifndef HANDKINEMATICS_H
#define HANDKINEMATICS_H

#include "hand.h"
#include "SanchoHandKinematicModelNewCS.h"
#include "SanchoArmKinematicModelNewCS.h"

using namespace std;
using namespace OpenRAVE;

class ArmHandKinematics
{

public:
	ArmHandKinematics()
    {
        hand = new Hand();
        handKinematicModel= new SanchoHandKinematicModel();
        handKinematicModel->SetHand(hand);

        arm = new Arm();
        armKinematicModel = new SanchoArmKinematicModel();
        armKinematicModel->SetArm(arm);
    }

    bool SetHandParameters(float HB, float HL)
    {
    	//RAVELOG_INFO("setting HandParameters \n");
    	hand->SetHB(HB);
		hand->SetHL(HL);

    	handKinematicModel->SetBonesLength();
    	handKinematicModel->SetBonesThickness();
    	handKinematicModel->SetCS();
    	handKinematicModel->SetFingerLimits();

    	return true;
    }

    bool SetArmParameters()
    {
    	//RAVELOG_INFO("setting ArmParameters \n");
    	armKinematicModel->SetBonesLength();
    	armKinematicModel->SetBonesThickness();
    	armKinematicModel->SetCS();
    	armKinematicModel->SetSegmentLimits();

    	return true;
    }


    string CreateStrFromAngles(float *cmc, float *mcp, float *pip, float *dip)
    {
    	string angulos = "";
    	//CMC - 4 items
        for(int i = 0;i < 4;i++){
            angulos = angulos + FloatToStr(cmc[i]) + " ";
        }
        //MCP - 4 items
        for(int i = 0;i < 4;i++){
            angulos = angulos + FloatToStr(mcp[i]) + " ";
        }
        //PIP - 2 items
        for(int i = 0;i < 2;i++){
            angulos = angulos + FloatToStr(pip[i]) + " ";
        }
        //DIP - 2 items
        for(int i = 0;i < 2;i++){
            angulos = angulos + FloatToStr(dip[i]) + " ";
        }
        return angulos;
    }

    //#define length(x) (sizeof(x) / sizeof(*(x)))

    string GetLimits(string finger)
    {
        string angulos = "";
        if(finger == "Thumb"){
            float *cmc = hand->GetThumb()->GetCMCLimits();
            float *mcp = hand->GetThumb()->GetMCPLimits();
            float *ip = hand->GetThumb()->GetIPLimits();
            //crear el string
            for(int i = 0;i < 4;i++){
                angulos = angulos + FloatToStr(cmc[i]) + " ";
            }
            for(int i = 0;i < 4;i++){
                angulos = angulos + FloatToStr(mcp[i]) + " ";
            }
            for(int i = 0;i < 2;i++){
                angulos = angulos + FloatToStr(ip[i]) + " ";
            }
            //Añado dos ceros al final para que tenga los mismos elementos que el
            //resto de dedos
            angulos = angulos + FloatToStr(0.0) + " " + FloatToStr(0.0) + " ";
        }
        else if(finger == "Index")
        {
			float *cmc = hand->GetIndex()->GetCMCLimits();
			float *mcp = hand->GetIndex()->GetMCPLimits();
			float *pip = hand->GetIndex()->GetPIPLimits();
			float *dip = hand->GetIndex()->GetDIPLimits();
			//RAVELOG_INFO("%d - %d - %d - %d", length(cmc), length(mcp), length(pip), length(dip));
            angulos = CreateStrFromAngles(cmc, mcp, pip, dip);
        }
        else if(finger == "Middle")
        {
        	float *cmc = hand->GetMiddle()->GetCMCLimits();
			float *mcp = hand->GetMiddle()->GetMCPLimits();
			float *pip = hand->GetMiddle()->GetPIPLimits();
			float *dip = hand->GetMiddle()->GetDIPLimits();
			//RAVELOG_INFO("%d - %d - %d - %d", length(cmc), length(mcp), length(pip), length(dip));
			angulos = CreateStrFromAngles(cmc, mcp, pip, dip);
		}
        else if(finger == "Ring")
		{
        	float *cmc = hand->GetRing()->GetCMCLimits();
			float *mcp = hand->GetRing()->GetMCPLimits();
			float *pip = hand->GetRing()->GetPIPLimits();
			float *dip = hand->GetRing()->GetDIPLimits();
			//RAVELOG_INFO("%d - %d - %d - %d", length(cmc), length(mcp), length(pip), length(dip));
			angulos = CreateStrFromAngles(cmc, mcp, pip, dip);
		}
        else if(finger == "Small")
		{
        	float *cmc = hand->GetSmall()->GetCMCLimits();
			float *mcp = hand->GetSmall()->GetMCPLimits();
			float *pip = hand->GetSmall()->GetPIPLimits();
			float *dip = hand->GetSmall()->GetDIPLimits();
			//RAVELOG_INFO("%d - %d - %d - %d", length(cmc), length(mcp), length(pip), length(dip));
			angulos = CreateStrFromAngles(cmc, mcp, pip, dip);
		}

        return angulos;
    }

    float GetLenghtSegment(string segment)
    {
        float length = 0.0;
        if(segment.find("Thumb") != string::npos){
            if(segment.find("Distal") != string::npos){
                length = hand->GetThumb()->GetDistal()->GetLength();
            }
            if(segment.find("Proximal") != string::npos){
                length = hand->GetThumb()->GetProximal()->GetLength();
            }
            if(segment.find("Metacarpal") != string::npos){
                length = hand->GetThumb()->GetMetacarpal()->GetLength();
            }
        }

        if(segment.find("Index") != string::npos){
            if(segment.find("Distal") != string::npos){
                length = hand->GetIndex()->GetDistal()->GetLength();
            }
            if(segment.find("Medial") != string::npos){
                length = hand->GetIndex()->GetMedial()->GetLength();
            }
            if(segment.find("Proximal") != string::npos){
                length = hand->GetIndex()->GetProximal()->GetLength();
            }
            if(segment.find("Metacarpal") != string::npos){
                length = hand->GetIndex()->GetMetacarpal()->GetLength();
            }
        }
        else
            if(segment.find("Middle") != string::npos){
                if(segment.find("Distal") != string::npos){
                    length = hand->GetMiddle()->GetDistal()->GetLength();
                }
                if(segment.find("Medial") != string::npos){
                    length = hand->GetMiddle()->GetMedial()->GetLength();
                }
                if(segment.find("Proximal") != string::npos){
                    length = hand->GetMiddle()->GetProximal()->GetLength();
                }
                if(segment.find("Metacarpal") != string::npos){
                    length = hand->GetMiddle()->GetMetacarpal()->GetLength();
                }
            }
            else
                if(segment.find("Ring") != string::npos){
                    if(segment.find("Distal") != string::npos){
                        length = hand->GetRing()->GetDistal()->GetLength();
                    }
                    if(segment.find("Medial") != string::npos){
                        length = hand->GetRing()->GetMedial()->GetLength();
                    }
                    if(segment.find("Proximal") != string::npos){
                        length = hand->GetRing()->GetProximal()->GetLength();
                    }
                    if(segment.find("Metacarpal") != string::npos){
                        length = hand->GetRing()->GetMetacarpal()->GetLength();
                    }
                }
                else
                    if(segment.find("Small") != string::npos){
                        if(segment.find("Distal") != string::npos){
                            length = hand->GetSmall()->GetDistal()->GetLength();
                        }
                        if(segment.find("Medial") != string::npos){
                            length = hand->GetSmall()->GetMedial()->GetLength();
                        }
                        if(segment.find("Proximal") != string::npos){
                            length = hand->GetSmall()->GetProximal()->GetLength();
                        }
                        if(segment.find("Metacarpal") != string::npos){
                            length = hand->GetSmall()->GetMetacarpal()->GetLength();
                        }
                    }
        return length;
    }

    float GetThicknessSegment(string segment)
    {
        float thickness = 0.0;
        if(segment.find("Thumb") != string::npos){
            if(segment.find("Distal") != string::npos){
                thickness = hand->GetThumb()->GetDistal()->GetThickness();
            }
            if(segment.find("Proximal") != string::npos){
                thickness = hand->GetThumb()->GetProximal()->GetThickness();
            }
            if(segment.find("Metacarpal") != string::npos){
                thickness = hand->GetThumb()->GetMetacarpal()->GetThickness();
            }
        }

        if(segment.find("Index") != string::npos){
            if(segment.find("Distal") != string::npos){
                thickness = hand->GetIndex()->GetDistal()->GetThickness();
            }
            if(segment.find("Medial") != string::npos){
                thickness = hand->GetIndex()->GetMedial()->GetThickness();
            }
            if(segment.find("Proximal") != string::npos){
                thickness = hand->GetIndex()->GetProximal()->GetThickness();
            }
            if(segment.find("Metacarpal") != string::npos){
                thickness = hand->GetIndex()->GetMetacarpal()->GetThickness();
            }
        }
        else
            if(segment.find("Middle") != string::npos){
                if(segment.find("Distal") != string::npos){
                    thickness = hand->GetMiddle()->GetDistal()->GetThickness();
                }
                if(segment.find("Medial") != string::npos){
                    thickness = hand->GetMiddle()->GetMedial()->GetThickness();
                }
                if(segment.find("Proximal") != string::npos){
                    thickness = hand->GetMiddle()->GetProximal()->GetThickness();
                }
                if(segment.find("Metacarpal") != string::npos){
                    thickness = hand->GetMiddle()->GetMetacarpal()->GetThickness();
                }
            }
            else
                if(segment.find("Ring") != string::npos){
                    if(segment.find("Distal") != string::npos){
                        thickness = hand->GetRing()->GetDistal()->GetThickness();
                    }
                    if(segment.find("Medial") != string::npos){
                        thickness = hand->GetRing()->GetMedial()->GetThickness();
                    }
                    if(segment.find("Proximal") != string::npos){
                        thickness = hand->GetRing()->GetProximal()->GetThickness();
                    }
                    if(segment.find("Metacarpal") != string::npos){
                        thickness = hand->GetRing()->GetMetacarpal()->GetThickness();
                    }
                }
                else
                    if(segment.find("Small") != string::npos){
                        if(segment.find("Distal") != string::npos){
                            thickness = hand->GetSmall()->GetDistal()->GetThickness();
                        }
                        if(segment.find("Medial") != string::npos){
                            thickness = hand->GetSmall()->GetMedial()->GetThickness();
                        }
                        if(segment.find("Proximal") != string::npos){
                            thickness = hand->GetSmall()->GetProximal()->GetThickness();
                        }
                        if(segment.find("Metacarpal") != string::npos){
                            thickness = hand->GetSmall()->GetMetacarpal()->GetThickness();
                        }
                    }
        return thickness;
    }

    RobotBasePtr GetHandKinbody()
    {
        return hand->GetKinBodyRobot();
    }

    void SetHandKinbody(RobotBasePtr k)
    {
        hand->SetKinBodyRobot(k);
    }

    string GetHandXMLString()
    {
    	int drawMesh = 1;
        string robotXmlData;
        robotXmlData = "<Robot name='HumanHand'><!--translation>0 0 0.25</translation-->";

        robotXmlData += DrawTorso();

        robotXmlData += DrawShoulder(drawMesh);

        robotXmlData += DrawArm(arm,drawMesh);

        Finger *finger;
        finger = hand->GetThumb();
        robotXmlData += DrawFinger(finger,drawMesh);
        //RAVELOG_INFO("robotXmlData:%s \n",robotXmlData.c_str());


        finger = hand->GetIndex();
        string IndexXmlData;
        IndexXmlData = DrawFinger(finger,drawMesh);
        //RAVELOG_INFO("IndexXmlData:%s \n",IndexXmlData.c_str());
        robotXmlData += IndexXmlData;



        finger = hand->GetMiddle();
        robotXmlData += DrawFinger(finger,drawMesh);
        finger = hand->GetRing();
        robotXmlData += DrawFinger(finger,drawMesh);
        finger = hand->GetSmall();
        robotXmlData += DrawFinger(finger,drawMesh);

        robotXmlData += "<Manipulator name='armHand'> <base>wrist_wrist</base> <effector>wrist_wrist</effector> <Translation>0.04  -0.05  0.0</Translation> <joints>Thumb_MCP_Abduction Thumb_MCP_Flexion Thumb_IP_Flexion Index_MCP_Flexion Index_PIP Index_DIP Middle_MCP_Flexion Middle_PIP Middle_DIP Ring_MCP_Flexion Ring_PIP Ring_DIP Small_MCP_Flexion Small_PIP Small_DIP</joints> <closingdirection>1 1 1 1 1 1 1 1 1 1 1 1 1 1 1</closingdirection> <direction>1 0 0</direction> </Manipulator>";
        //robotXmlData += "<Manipulator name='armHand'> <base>shoulder</base> <effector>shoulder</effector> <Translation>0.0  0.0  0.0</Translation> <joints>Thumb_IP_Flexion Thumb_MCP_Flexion Thumb_MCP_Abduction Thumb_CMC_Flexion Thumb_CMC_Abduction Index_DIP Index_PIP Index_MCP_Flexion Index_MCP_Abduction Index_CMC_Flexion Middle_DIP Middle_PIP Middle_MCP_Flexion Middle_MCP_Abduction Middle_CMC_Flexion Ring_DIP Ring_PIP Ring_MCP_Flexion Ring_MCP_Abduction Ring_CMC_Flexion Small_DIP  Small_PIP Small_MCP_Flexion Small_MCP_Abduction Small_CMC_Flexion shoulder_arm_rotation shoulder_arm_Flexion shoulder_arm_Abduccion arm_foreArm_Flexion arm_foreArm_Pronation wrist_foreArm_Flexion wrist_foreArm_Abduction</joints> <closingdirection>1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1</closingdirection> <direction>-1 0 0</direction> </Manipulator>";
        //robotXmlData += "<Manipulator name='armHand'> <base>wrist</base> <effector>wrist</effector> <Translation>0.0  0.0  0.0</Translation> <joints> Thumb_IP_Flexion Thumb_MCP_Flexion Thumb_MCP_Abduction Thumb_CMC_Flexion Thumb_CMC_Abduction Index_DIP Index_PIP Index_MCP_Flexion Index_MCP_Abduction Index_CMC_Flexion Middle_DIP Middle_PIP Middle_MCP_Flexion Middle_MCP_Abduction Middle_CMC_Flexion Ring_DIP Ring_PIP Ring_MCP_Flexion Ring_MCP_Abduction Ring_CMC_Flexion Small_DIP  Small_PIP Small_MCP_Flexion Small_MCP_Abduction Small_CMC_Flexion</joints> <closingdirection>1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1</closingdirection> <direction>-1 0 0</direction> </Manipulator>";

        robotXmlData += "</Robot>";
        return robotXmlData;
        //hand->SetKinBodyRobot(GetEnv()->ReadRobotXMLData(RobotBasePtr(), robotXmlData, std::list<std::pair<std::string,std::string> >()));
        //GetEnv()->AddRobot(hand->GetKinBodyRobot());
    }

    string DrawShoulder(int drawMesh)
    {
    	string xmlData;
    	//"<Geom type='cylinder'><translation>0.0 0.0 0.0</translation><Radius>0.01</Radius><Height>0.01</Height><diffusecolor>0.9608 0.9608 0.8627</diffusecolor></Geom>"
    	/*string xmlData = "<KinBody name='Shoulder'><translation>0.0 0.0 0.25</translation>"
    					 "<Body name='shoulder' type='dynamic'>"
    					 "<Geom type='trimesh'>"
				 	 	 "<translation>0.01150 -0.02777 0.02332</translation>"
				 	 	 "<quat>0.99778 0.06234 0.00147 -0.02347</quat>"
    					 "<Data>geometries/Skin/arm.iv </Data>"
    					 "<Render>geometries/Skin/arm.iv </Render>"
    					 "</Geom>"
    					 "</Body></KinBody>";*/
    	if (drawMesh == 1)
    	{
			float HB_mesh = 0.070;
			float scale_x = hand->GetHB()/HB_mesh;
			float scale_y = 1.0;
			//float scale_z = hand->GetHB()/HB_mesh;
			float scale_z = 1.0;

			xmlData = "<KinBody name='Shoulder'><translation>0.0 0.0 0.25</translation>"
									 "<Body name='shoulder' type='dynamic'>"
									 "<Geom type='trimesh'>";
			xmlData	+= "<Render>showlder.iv  "+ FloatToStr(scale_x) + " " + FloatToStr(scale_y) + " " + FloatToStr(scale_z)+"</Render>";
			xmlData	+= "<data>showlder.iv "   + FloatToStr(scale_x) + " " + FloatToStr(scale_y) + " " + FloatToStr(scale_z)+"</data>";
			xmlData	+= "</Geom>"
					   "</Body></KinBody>";
    	}
		else
		{
			xmlData = "<KinBody name='Shoulder'><translation>0.0 0.0 0.25</translation><Body name='shoulder' type='dynamic'>"
					"<Geom type='cylinder'><translation>0.0 0.0 0.0</translation><Radius>0.01</Radius><Height>0.01</Height><diffusecolor>0.9608 0.9608 0.8627</diffusecolor></Geom>"
					"</Body></KinBody>";
		}


    	return xmlData;
    }
    string DrawTorso()
    {
    	string xmlData = "<KinBody name='Torso'><Body name='torso' type='dynamic'><Geom type='cylinder'><translation>0.0 0.0 0.0</translation><Radius>0.01</Radius><Height>0.01</Height><diffusecolor>0.9608 0.9608 0.8627</diffusecolor><transparency>1.0</transparency></Geom></Body></KinBody>";
    	return xmlData;
    }

    string DrawVirtualShoulder()
    {
    	string xmlData = "<Body name='shoulder_Virtual' type='dynamic'>"
    						"<offsetfrom> shoulder </offsetfrom>"
							"<rotationaxis> 0 1 0 0 </rotationaxis>"
							"<rotationaxis> 0 0 1 0 </rotationaxis>"
							"<rotationaxis> 1 0 0 0 </rotationaxis>"
							"<translation>0 0 0</translation>"
							"<Geom type='cylinder'>"
								"<translation>0 0 0</translation>"
								"<Radius>0.02</Radius>"
								"<Height>0.001</Height>"
								"<diffusecolor>0.9608 0.9608 0.8627</diffusecolor>"
    			 	 	 	 	"<transparency>1.0</transparency>"
							"</Geom>"
						  "</Body>";
    	return xmlData;
    }

    string DrawArm(Arm* arm, int drawMesh)
    {
    	string kinbodyXmlData = "<KinBody name='CompletArm'>";

    	kinbodyXmlData += "<Joint name='torso_shoulder' type='Hinge' enable='false'><Body>torso</Body><Body>shoulder</Body><axis>0 0 -1</axis><limitsdeg>0 0</limitsdeg></Joint>";

    	//GetFlexionJointString("torso_shoulder", "torso", "shoulder", "", 0.0, 0.0);

    	//Virtual shoulder
    	kinbodyXmlData += DrawVirtualShoulder();

    	//Arm
		Segment* segment = arm->GetArm();
		string segmentName = segment->GetName();
		if (drawMesh == 1)
			kinbodyXmlData += GetMeshString(segmentName, segment, "shoulder");
		else
			kinbodyXmlData += GetCylinderString(segmentName, segment, "shoulder");
    	kinbodyXmlData += GetCylinderVirtualString(segmentName, segment, "shoulder");

    	//ForeArm
    	segment = arm->GetForeArm();
    	segmentName = segment->GetName();
    	if (drawMesh == 1)
    		kinbodyXmlData += GetMeshString(segmentName, segment, "arm_arm");
    	else
    		kinbodyXmlData += GetCylinderString(segmentName, segment, "arm_arm");
    	kinbodyXmlData += GetCylinderVirtualString(segmentName, segment, "arm_arm");

    	//wrist
    	segment = arm->GetWrist();
		segmentName = segment->GetName();
        if (drawMesh == 1)
        	kinbodyXmlData += GetMeshString(segmentName, segment, "foreArm_foreArm");
        else
        	kinbodyXmlData += GetCylinderString(segmentName, segment, "foreArm_foreArm");
		kinbodyXmlData += GetCylinderVirtualString(segmentName, segment, "foreArm_foreArm");

		float* shoulderLimits =  arm->GetShoulderLimits();
		//joint shoulder_arm_rotation (ESTA ES LA ROTACION EN X)
		string jointName = "shoulder_arm_rotation";
		string bodyName2 = "shoulder";
		string bodyName1 = "shoulder_Virtual";
		kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, shoulderLimits[0],shoulderLimits[1]);

    	//joint shoulder_arm_Flexion (ESTA ES LA ROTACION EN Z)
    	jointName = "shoulder_arm_Flexion";
		bodyName2 = "shoulder_Virtual";
    	bodyName1 = "arm_arm_Virtual";
    	kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, shoulderLimits[2], shoulderLimits[3]);

    	//joint shoulder_arm_Abduction (ESTA ES LA ROTACION EN Y)
		jointName = "shoulder_arm_Abduccion";
    	bodyName2 = "arm_arm_Virtual";
		bodyName1 = "arm_arm";
		kinbodyXmlData += GetPronationJointString(jointName, bodyName1, bodyName2, bodyName1, shoulderLimits[4], shoulderLimits[5]);

		float* elbowLimits =  arm->GetElbowLimits();
    	//joint arm_foreArm_Pronation
    	jointName = "arm_foreArm_Flexion";
    	bodyName1 = "foreArm_foreArm_Virtual";
		bodyName2 = "arm_arm";
		kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, elbowLimits[0], elbowLimits[1]);

		//joint arm_foreArm_Flexion
    	jointName = "arm_foreArm_Pronation";
    	bodyName1 = "foreArm_foreArm";
		bodyName2 = "foreArm_foreArm_Virtual";
		kinbodyXmlData += GetPronationJointString(jointName, bodyName1, bodyName2, bodyName1, elbowLimits[1], elbowLimits[2]);

		/* Original
		float* wristLimits =  arm->GetWirstLimits();
		//joint wrist_foreArm_abduction
		jointName = "wrist_foreArm_Abduction";
		bodyName1 = "wrist_wrist_Virtual";
		bodyName2 = "foreArm_foreArm";
		kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, wristLimits[0], wristLimits[1]);

    	//joint wrist_foreArm_flexion
    	jointName = "wrist_foreArm_Flexion";
    	bodyName1 = "wrist_wrist";
		bodyName2 = "wrist_wrist_Virtual";
    	kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, wristLimits[2], wristLimits[3]);
    	*/

    	// Modified
		float* wristLimits =  arm->GetWirstLimits();

    	//joint wrist_foreArm_flexion
    	jointName = "wrist_foreArm_Flexion";
    	bodyName1 = "wrist_wrist_Virtual";
    	bodyName2 = "foreArm_foreArm";
		kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, wristLimits[2], wristLimits[3]);

		//joint wrist_foreArm_abduction
		jointName = "wrist_foreArm_Abduction";
		bodyName1 = "wrist_wrist_Virtual";
		bodyName2 = "wrist_wrist";
		kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, wristLimits[0], wristLimits[1]);

    	kinbodyXmlData += "</KinBody>";
    	return kinbodyXmlData;
    }

    string DrawFinger(Finger *finger, int drawMesh)
    {
        string fingerName = finger->GetName();
        string kinbodyXmlData = "<KinBody name='" + fingerName + "'>";
        Segment *segment;
        string previousSegmentName;


        //Metacarpal
        segment = finger->GetMetacarpal();
        if(fingerName.compare("Thumb") != 0){
        	kinbodyXmlData += GetCylinderString(fingerName, segment, "wrist_wrist");
        }
        else {
        	if (drawMesh == 1)
        		kinbodyXmlData += GetMeshString(fingerName, segment, "wrist_wrist");
        	else
        		kinbodyXmlData += GetCylinderString(fingerName, segment, "wrist_wrist");
            kinbodyXmlData += GetCylinderVirtualString(fingerName, segment, "wrist_wrist");
        }

        //Proximal
        previousSegmentName = fingerName + "_" + segment->GetName();
        segment = finger->GetProximal();
        if (drawMesh == 1)
        	kinbodyXmlData += GetMeshString(fingerName, segment, previousSegmentName);
        else
        	kinbodyXmlData += GetCylinderString(fingerName, segment, previousSegmentName);
        kinbodyXmlData += GetCylinderVirtualString(fingerName, segment, previousSegmentName);

        //Medial
        previousSegmentName = fingerName + "_" + segment->GetName();
        if(fingerName.compare("Thumb") != 0){
            segment = finger->GetMedial();
            if (drawMesh == 1)
            	kinbodyXmlData += GetMeshString(fingerName, segment, previousSegmentName);
            else
            	kinbodyXmlData += GetCylinderString(fingerName, segment, previousSegmentName);
            previousSegmentName = fingerName + "_" + segment->GetName();
        }

        //Distal
        segment = finger->GetDistal();
        if (drawMesh == 1)
        	kinbodyXmlData += GetMeshString(fingerName, segment, previousSegmentName);
        else
        	kinbodyXmlData += GetCylinderString(fingerName, segment, previousSegmentName);

        string jointName;
        string bodyName1;
        string bodyName2;
        if(fingerName.compare("Thumb") == 0){
            float *IPLimits;
            IPLimits = finger->GetIPLimits();
            jointName = fingerName + "_IP_Flexion";
            bodyName1 = fingerName + "_" + finger->GetDistal()->GetName();
            bodyName2 = fingerName + "_" + finger->GetProximal()->GetName();
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, IPLimits[0], IPLimits[1]);
        }else{
            float *DIPLimits;
            DIPLimits = finger->GetDIPLimits();
            jointName = fingerName + "_DIP";
            bodyName1 = fingerName + "_" + finger->GetDistal()->GetName();
            bodyName2 = fingerName + "_" + finger->GetMedial()->GetName();
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, DIPLimits[0], DIPLimits[1]);
            float *PIPLimits;
            PIPLimits = finger->GetPIPLimits();
            jointName = fingerName + "_PIP";
            bodyName1 = fingerName + "_" + finger->GetMedial()->GetName();
            bodyName2 = fingerName + "_" + finger->GetProximal()->GetName();
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, PIPLimits[0], PIPLimits[1]);
        }
        float *MCPLimits;
        MCPLimits = finger->GetMCPLimits();
        jointName = fingerName + "_MCP_Flexion";
        bodyName1 = fingerName + "_" + finger->GetProximal()->GetName() + "_Virtual";
        bodyName2 = fingerName + "_" + finger->GetMetacarpal()->GetName();
        kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, MCPLimits[0], MCPLimits[1]);
        jointName = fingerName + "_MCP_Abduction";
        bodyName1 = fingerName + "_" + finger->GetProximal()->GetName() + "_Virtual";
        bodyName2 = fingerName + "_" + finger->GetProximal()->GetName();
        kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, MCPLimits[2], MCPLimits[3]);
        float *CMCLimits;
        if(fingerName.compare("Thumb") != 0){
            CMCLimits = finger->GetCMCLimits();
            jointName = fingerName + "_CMC_Flexion";
            bodyName1 = fingerName + "_" + finger->GetMetacarpal()->GetName();
            //bodyName2 = "wrist";
            bodyName2 = "wrist_wrist";
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[0], CMCLimits[1]);

        }else{
            //The thumb is the only that has abduction on CMC
            CMCLimits = finger->GetCMCLimits();
            jointName = fingerName + "_CMC_Flexion";
            bodyName1 = fingerName + "_" + finger->GetMetacarpal()->GetName() + "_Virtual";
            //bodyName2 = "wrist";
            bodyName2 = "wrist_wrist";
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[0], CMCLimits[1]);
            bodyName1 = fingerName + "_" + finger->GetMetacarpal()->GetName() + "_Virtual";
            bodyName2 = fingerName + "_" + finger->GetMetacarpal()->GetName();
            jointName = fingerName + "_CMC_Abduction";
            kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[2], CMCLimits[3]);
        }
        kinbodyXmlData += "</KinBody>";
        //RAVELOG_INFO("kinbodyXmlData:%s \n",kinbodyXmlData.c_str());
        return kinbodyXmlData;
    }

    string GetMeshString(string name, Segment *segment, string offsetFrom)
    {
        string segmentName = segment->GetName();
        string bodyName = name + "_" + segmentName;
		string ivName;
		ivName = bodyName + ".iv";

        string type = "dynamic";
        string xmlData = "<Body name='" + bodyName + "' type='" + type + "'>";
        if(offsetFrom.compare("") != 0){
            xmlData += "<offsetfrom>" + offsetFrom + "</offsetfrom>";
        }

    	float HL_mesh = 0.17581666;
    	float HB_mesh = 0.070;
		float scale_x, scale_y, scale_z;

		if (segmentName.compare("arm") == 0){
			scale_x = hand->GetHB()/HB_mesh;
			scale_y = 1.0;
			scale_z = 1.0;
		}
		else if (segmentName.compare("foreArm") == 0){
			scale_x = 1.0;
			scale_y = 1.0;
			scale_z = hand->GetHB()/HB_mesh;
		}
		else
		{
			scale_x = 1.0;
			scale_y = hand->GetHL()/HL_mesh;
			scale_z = hand->GetHB()/HB_mesh;
		}
		//Mesh
		xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>";
		xmlData	+= "<Geom type='trimesh'>";
		xmlData	+= "<Render>"+ ivName + " " + FloatToStr(scale_x) + " " + FloatToStr(scale_y) + " " + FloatToStr(scale_z)+"</Render>";
		xmlData	+= "<data>"  + ivName + " "   + FloatToStr(scale_x) + " " + FloatToStr(scale_y) + " " + FloatToStr(scale_z)+"</data></Geom>";
		xmlData	+= "</Body>";
		//RAVELOG_INFO("kinbodyXmlData:%s \n",xmlData.c_str());

		return xmlData;
    }

    string GetCylinderString(string name, Segment *segment, string offsetFrom)
    {
        string segmentName = segment->GetName();
        string bodyName = name + "_" + segmentName;
        string type = "dynamic";
        string xmlData = "<Body name='" + bodyName + "' type='" + type + "'>";
        if(offsetFrom.compare("") != 0){
            xmlData += "<offsetfrom>" + offsetFrom + "</offsetfrom>";
        }

        string color = "<diffusecolor>0.95 0.84 0.80</diffusecolor><transparency>1.0</transparency>";

        //RAVELOG_INFO("segmentName:%s \n",segmentName.c_str());
        if(segmentName.compare("arm") == 0 || segmentName.compare("foreArm") == 0 || segmentName.compare("wrist") == 0)
		{
 			color = "<diffusecolor>0.95 0.84 0.80</diffusecolor><transparency>1.0</transparency>";
 		}

		xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>" + "<Geom type='cylinder'><translation>0 " + FloatToStr(-segment->GetLength() / 2) + " 0</translation>" + "<Radius>" + FloatToStr(segment->GetThickness() / 2) + "</Radius><Height>" + FloatToStr(segment->GetLength()) + "</Height>" + color + "</Geom></Body>";

		return xmlData;
    }

    string GetCylinderVirtualString(string name, Segment *segment, string offsetFrom)
    {
        string segmentName = segment->GetName();
        string bodyName = name + "_" + segmentName + "_Virtual";
        string xmlData = "<Body name='" + bodyName + "' type='dynamic'>";
        if(offsetFrom.compare("") != 0){
            xmlData += "<offsetfrom>" + offsetFrom + "</offsetfrom>";
        }
        double Virtuallength = 0.001;

		string color = "<diffusecolor>0.95 0.84 0.80</diffusecolor><transparency>1.0</transparency>";

        xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>" + "<Geom type='cylinder'> <translation>0 " + FloatToStr(-Virtuallength / 2) + " 0</translation>" + "<Radius>" + FloatToStr(Virtuallength) + "</Radius><Height>" + FloatToStr(Virtuallength) + "</Height>" + color + "</Geom></Body>";
        return xmlData;
    }

    string GetFlexionJointString(string jointName, string body1, string body2, string offsetfrom, float lowlimit, float upperlimit)
    {
        string xmlData = "<Joint name='" + jointName + "' type='Hinge'> <Body>" + body1 + "</Body><Body>" + body2 + "</Body><offsetfrom>" + offsetfrom + "</offsetfrom><axis>0 0 -1</axis><limitsdeg>" + FloatToStr(lowlimit) + " " + FloatToStr(upperlimit) + "</limitsdeg></Joint>";
        return xmlData;
    }

    string GetAbductionJointString(string jointName, string body1, string body2, string offsetfrom, float lowlimit, float upperlimit)
    {
        string xmlData = "<Joint name='" + jointName + "' type='Hinge'> <Body>" + body1 + "</Body><Body>" + body2 + "</Body><offsetfrom>" + offsetfrom + "</offsetfrom><axis>-1 0 0</axis><limitsdeg>" + FloatToStr(lowlimit) + " " + FloatToStr(upperlimit) + "</limitsdeg></Joint>";
    	return xmlData;
    }

    string GetPronationJointString(string jointName, string body1, string body2, string offsetfrom, float lowlimit, float upperlimit)
	{
		string xmlData = "<Joint name='" + jointName + "' type='Hinge'> <Body>" + body1 + "</Body><Body>" + body2 + "</Body><offsetfrom>" + offsetfrom + "</offsetfrom><axis>0 -1 0</axis><limitsdeg>" + FloatToStr(lowlimit) + " " + FloatToStr(upperlimit) + "</limitsdeg></Joint>";
		return xmlData;
	}

    string FloatToStr(float num)
    {
         std::ostringstream stream;
         stream << num;
         return stream.str();
    }

private:
	Hand* hand;
	Arm* arm;
	HandKinematicModel* handKinematicModel;
	ArmKinematicModel* armKinematicModel;
};

#endif
