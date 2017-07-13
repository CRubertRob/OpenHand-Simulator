#ifndef HANDKINEMATICS_H
#define HANDKINEMATICS_H

#include "hand.h"
#include "SanchoHandKinematicModelNewCS.h"

using namespace std;
using namespace OpenRAVE;

class HandKinematics
{

public:
	HandKinematics()
    {
        hand = new Hand();
        handKinematicModel= new SanchoHandKinematicModel();
        handKinematicModel->SetHand(hand);
    }

    bool SetHandParameters(float HB, float HL)
    {
    	RAVELOG_INFO("setting HandParameters \n");
    	hand->SetHB(HB);
		hand->SetHL(HL);

    	handKinematicModel->SetBonesLength();
    	handKinematicModel->SetBonesThickness();
    	handKinematicModel->SetCS();
    	handKinematicModel->SetFingerLimits();

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
        string robotXmlData;
        robotXmlData = "<Robot name='HumanHand'>";
        robotXmlData += DrawWrist();
        Finger *finger;
        finger = hand->GetThumb();
        robotXmlData += DrawFinger(finger);
        //RAVELOG_INFO("robotXmlData:%s \n",robotXmlData.c_str());
        finger = hand->GetIndex();
        robotXmlData += DrawFinger(finger);
        finger = hand->GetMiddle();
        robotXmlData += DrawFinger(finger);
        finger = hand->GetRing();
        robotXmlData += DrawFinger(finger);
        finger = hand->GetSmall();
        robotXmlData += DrawFinger(finger);
        robotXmlData += "<Manipulator name='armHand'> <base>wrist</base> <effector>wrist</effector> <Translation>0.0  0.0  0.0</Translation> <joints> Thumb_IP_Flexion Thumb_MCP_Flexion Thumb_MCP_Abduction Thumb_CMC_Flexion Thumb_CMC_Abduction Index_DIP Index_PIP Index_MCP_Flexion Index_MCP_Abduction Index_CMC_Flexion Middle_DIP Middle_PIP Middle_MCP_Flexion Middle_MCP_Abduction Middle_CMC_Flexion Ring_DIP Ring_PIP Ring_MCP_Flexion Ring_MCP_Abduction Ring_CMC_Flexion Small_DIP  Small_PIP Small_MCP_Flexion Small_MCP_Abduction Small_CMC_Flexion</joints> <closingdirection>1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1</closingdirection> <direction>-1 0 0</direction> </Manipulator>";
        robotXmlData += "</Robot>";
        return robotXmlData;
        //hand->SetKinBodyRobot(GetEnv()->ReadRobotXMLData(RobotBasePtr(), robotXmlData, std::list<std::pair<std::string,std::string> >()));
        //GetEnv()->AddRobot(hand->GetKinBodyRobot());
    }

    string DrawWrist()
    {
        string xmlData = "	<KinBody name='Wrist'><rotationaxis> 1 0 0 0 </rotationaxis><translation>0.0 0.0 0</translation><Body name='wrist' type='dynamic'><Geom type='cylinder'><translation>0.0 0.0 0.0</translation><Radius>0.01</Radius><Height>0.01</Height><diffusecolor>1 1 0</diffusecolor></Geom></Body></KinBody>";
        //hand->SetKinBodyWrist(GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >()));
        //GetEnv()->AddKinBody(hand->GetKinBodyWrist());
        return xmlData;
    }

    string DrawFinger(Finger *finger)
    {
        string fingerName = finger->GetName();
        string kinbodyXmlData = "<KinBody name='" + fingerName + "'>";
        Segment *segment;
        string previousSegmentName;
        segment = finger->GetMetacarpal();
        kinbodyXmlData += GetCylinderString(fingerName, segment, "");
        if(fingerName.compare("Thumb") == 0){
            kinbodyXmlData += GetCylinderVirtualString(fingerName, segment, "");
        }
        previousSegmentName = fingerName + "_" + segment->GetName();
        segment = finger->GetProximal();
        kinbodyXmlData += GetCylinderString(fingerName, segment, previousSegmentName);
        kinbodyXmlData += GetCylinderVirtualString(fingerName, segment, previousSegmentName);
        previousSegmentName = fingerName + "_" + segment->GetName();
        if(fingerName.compare("Thumb") != 0){
            segment = finger->GetMedial();
            kinbodyXmlData += GetCylinderString(fingerName, segment, previousSegmentName);
            previousSegmentName = fingerName + "_" + segment->GetName();
        }
        segment = finger->GetDistal();
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
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, DIPLimits[0], DIPLimits[1]);
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
            bodyName2 = "wrist";
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[0], CMCLimits[1]);
        }else{
            //The thumb is the only that has abduction on CMC
            CMCLimits = finger->GetCMCLimits();
            jointName = fingerName + "_CMC_Flexion";
            bodyName1 = fingerName + "_" + finger->GetMetacarpal()->GetName() + "_Virtual";
            bodyName2 = "wrist";
            kinbodyXmlData += GetFlexionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[0], CMCLimits[1]);
            bodyName1 = fingerName + "_" + finger->GetMetacarpal()->GetName() + "_Virtual";
            bodyName2 = fingerName + "_" + finger->GetMetacarpal()->GetName();
            jointName = fingerName + "_CMC_Abduction";
            kinbodyXmlData += GetAbductionJointString(jointName, bodyName1, bodyName2, bodyName1, CMCLimits[2], CMCLimits[3]);
        }
        kinbodyXmlData += "</KinBody>";
        //RAVELOG_INFO("kinbodyXmlData:%s \n",kinbodyXmlData.c_str());
        //finger->SetKinBody(GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), kinbodyXmlData, std::list<std::pair<std::string,std::string> >()));
        //GetEnv()->AddKinBody(finger->GetKinBody());
        return kinbodyXmlData;
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
        //Cylinders
        xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>" + "<Geom type='cylinder'><translation>0 " + FloatToStr(-segment->GetLength() / 2) + " 0</translation>" + "<Radius>" + FloatToStr(segment->GetThickness() / 2) + "</Radius><Height>" + FloatToStr(segment->GetLength()) + "</Height><diffusecolor>1 0 0</diffusecolor></Geom></Body>";
        //Spheres
        //       	xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) +" </rotationaxis>"+
        //        			   "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) +" </rotationaxis>"+
        //        			   "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) +" </rotationaxis>"+
        //        			   "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>"+
        //        			   "<Geom type='sphere'> <translation>0 "+ FloatToStr(segment->GetLength()/2) +" 0</translation>"+
        //        			   "<Radius>" +  FloatToStr(segment->GetThickness()/2) +
        //        			   "</Radius><diffusecolor>1 0 0</diffusecolor></Geom></Body>";
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
        xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>" + "<Geom type='cylinder'> <translation>0 " + FloatToStr(-Virtuallength / 2) + " 0</translation>" + "<Radius>" + FloatToStr(segment->GetThickness() / 2) + "</Radius><Height>" + FloatToStr(Virtuallength) + "</Height><diffusecolor>1 1 0</diffusecolor></Geom></Body>";
        return xmlData;
    }

    string GetMeshString(string name, Segment *segment, string offsetFrom, string ivFile)
    {
        string segmentName = segment->GetName();
        string bodyName = name + "_" + segmentName;
        string xmlData = "<Body name='" + bodyName + "' type='dynamic'>";
        if(offsetFrom.compare("") != 0){
            xmlData += "<offsetfrom>" + offsetFrom + "</offsetfrom>";
        }
        xmlData += "<rotationaxis> 0 1 0 " + FloatToStr(segment->GetCSRotationAngleY()) + " </rotationaxis>" + "<rotationaxis> 0 0 1 " + FloatToStr(segment->GetCSRotationAngleZ()) + " </rotationaxis>" + "<rotationaxis> 1 0 0 " + FloatToStr(segment->GetCSRotationAngleX()) + " </rotationaxis>" + "<translation>" + FloatToStr(segment->GetCSLocationX()) + " " + FloatToStr(segment->GetCSLocationY()) + " " + FloatToStr(segment->GetCSLocationZ()) + "</translation>" + "<Geom type='cylinder'> <translation>0 " + FloatToStr(segment->GetLength() / 2) + " 0</translation>" + "<Radius>" + FloatToStr(segment->GetThickness() / 2) + "</Radius><Height>" + FloatToStr(segment->GetLength()) + "</Height><diffusecolor>1 0 0</diffusecolor></Geom></Body>";
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

    string FloatToStr(float num)
    {
         std::ostringstream stream;
         stream << num;
         return stream.str();
    }

private:
	Hand* hand;
	HandKinematicModel* handKinematicModel;

};

#endif
