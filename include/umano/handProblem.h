#ifndef HANDPROBLEM_H
#define HANDPROBLEM_H

#include "armhandKinematicsNewCS.h"

#include <iostream>
#include <sstream>
#include <fstream>


using namespace std;
using namespace OpenRAVE;

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

class HandProblem : public ModuleBase
{

public:
	HandProblem(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv)
    {
        __description = "Human Hand Model problem";

        RegisterCommand("sethandparameters",boost::bind(&HandProblem::SetHandParameters, this,_1,_2),"Set hand parameters");
        //RegisterCommand("drawhandposture",boost::bind(&HandKinematics::DrawHandPosture, this,_1,_2),"Draw Hand Posture");
        RegisterCommand("setposture",boost::bind(&HandProblem::SetPosture, this,_1,_2),"Draw Hand Posture");
        RegisterCommand("getposture",boost::bind(&HandProblem::GetPosture, this,_1,_2),"Get Hand Posture");
		RegisterCommand("loadcylinder",boost::bind(&HandProblem::LoadCylinder, this,_1,_2),"Draw Cylinder");
        RegisterCommand("loadsphere",boost::bind(&HandProblem::LoadSphere, this,_1,_2),"Draw Sphere");
        RegisterCommand("returnpointsjointsfinger", boost::bind(&HandProblem::ReturnPointsJointsFinger, this,_1,_2),"Input parameter -> name of finger, Output parameter -> x,y,z and radius of the two joints of the finger");
        RegisterCommand("loadobject", boost::bind(&HandProblem::LoadObject, this,_1,_2),"Load Object");
        RegisterCommand("gethandlimits", boost::bind(&HandProblem::GetHandLimits, this,_1,_2),"Get Joint Limits");
        RegisterCommand("calculatejacobianpart", boost::bind(&HandProblem::CalculateJacobianPart, this,_1,_2),"Get part of jacobian matrix");
        RegisterCommand("getlinkindex", boost::bind(&HandProblem::GetLinkIndex, this,_1,_2),"Get link index");
        RegisterCommand("getjointindexbyname", boost::bind(&HandProblem::GetJointIndexByName, this,_1,_2),"Get joint index");
        RegisterCommand("setjointlimitsbyname", boost::bind(&HandProblem::SetJointLimitsByName, this,_1,_2),"SetJointLimitsByName");
        RegisterCommand("getjointindexbyrobotandname", boost::bind(&HandProblem::GetJointIndexByRobotAndName, this,_1,_2),"GetJointIndexByRobotAndName");
        RegisterCommand("getjointvaluebyrobotandname", boost::bind(&HandProblem::GetJointValueByRobotAndName, this,_1,_2),"GetJointValueByRobotAndName");
        RegisterCommand("getlinkindexbyrobotandname", boost::bind(&HandProblem::GetLinkIndexByRobotAndName, this,_1,_2),"GetLinkIndexByRobotAndName");
        RegisterCommand("getlinkindexchainbyrobotandnames", boost::bind(&HandProblem::GetLinkIndexChainByRobotAndNames, this,_1,_2),"GetLinkIndexChainByRobotAndNames");
        RegisterCommand("getjointlimitsbyname", boost::bind(&HandProblem::GetJointLimitsByName, this,_1,_2),"GetJointLimitsByName");

        handKinematics = new ArmHandKinematics();

        //Set Names in the matlab file order
        _humanJointNames.push_back("Thumb_CMC_Flexion");    //0
        _humanJointNames.push_back("Thumb_CMC_Abduction");  //1
        _humanJointNames.push_back("Thumb_MCP_Flexion");    //2
        _humanJointNames.push_back("Thumb_MCP_Abduction");  //3
        _humanJointNames.push_back("Thumb_IP_Flexion");     //4
        _humanJointNames.push_back("");                     //5
        _humanJointNames.push_back("Index_CMC_Flexion");    //6
        _humanJointNames.push_back("");                     //7
        _humanJointNames.push_back("Index_MCP_Flexion");    //8
        _humanJointNames.push_back("Index_MCP_Abduction");  //9
        _humanJointNames.push_back("Index_PIP");            //10
        _humanJointNames.push_back("Index_DIP");            //11
        _humanJointNames.push_back("Middle_CMC_Flexion");   //12
        _humanJointNames.push_back("");                     //13
        _humanJointNames.push_back("Middle_MCP_Flexion");   //14
        _humanJointNames.push_back("Middle_MCP_Abduction"); //15
        _humanJointNames.push_back("Middle_PIP");           //16
        _humanJointNames.push_back("Middle_DIP");           //17
        _humanJointNames.push_back("Ring_CMC_Flexion");     //18
        _humanJointNames.push_back("");                     //19
        _humanJointNames.push_back("Ring_MCP_Flexion");     //20
        _humanJointNames.push_back("Ring_MCP_Abduction");   //21
        _humanJointNames.push_back("Ring_PIP");             //22
        _humanJointNames.push_back("Ring_DIP");             //23
        _humanJointNames.push_back("Small_CMC_Flexion");    //24
        _humanJointNames.push_back("");                     //25
        _humanJointNames.push_back("Small_MCP_Flexion");    //26
        _humanJointNames.push_back("Small_MCP_Abduction");  //27
        _humanJointNames.push_back("Small_PIP");            //28
        _humanJointNames.push_back("Small_DIP");            //29

        _humanJointNames.push_back("shoulder_arm_rotation");  //30
        _humanJointNames.push_back("shoulder_arm_Abduccion");  //31
        _humanJointNames.push_back("shoulder_arm_Flexion");    //32

        _humanJointNames.push_back("arm_foreArm_Flexion");   //34
        _humanJointNames.push_back("arm_foreArm_Pronation");	   //33

        _humanJointNames.push_back("");    					   //35
        _humanJointNames.push_back("wrist_foreArm_Flexion");   //36
        _humanJointNames.push_back("wrist_foreArm_Abduction"); //37
        _humanJointNames.push_back("");    					   //38

    }

	bool GetJointIndexByRobotAndName(ostream& sout, istream& sinput)
	{
		string robot_name;
		string joint_name;
		sinput >> robot_name;
		sinput >> joint_name;

		KinBodyPtr robot = GetEnv()->GetKinBody(robot_name);
		KinBody::JointPtr joint = robot->GetJoint(joint_name);
		int index = joint->GetJointIndex();

		//Preparamos el indice, sumandole uno, para cuando se lea en matlab
		index = index+1;
		sout<<index;

		return true;
	}

	bool GetLinkIndexChainByRobotAndNames(ostream& sout, istream& sinput)
	{
		string robot_name;
		string link1_name;
		string link2_name;

		sinput >> robot_name;
		sinput >> link1_name;
		sinput >> link2_name;

		KinBodyPtr robot = GetEnv()->GetKinBody(robot_name);

		KinBody::LinkPtr link1 = robot->GetLink(link1_name);
		int index1 =link1->GetIndex();

		KinBody::LinkPtr link2 = robot->GetLink(link2_name);
		int index2 =link2->GetIndex();

		std::vector<KinBody::JointPtr> vjoints;
		robot->GetChain( index1, index2, vjoints);

		string salida = "";

		for(unsigned int i=0; i<vjoints.size();i++){
			if(vjoints[i]->GetJointIndex()!=-1){
				salida += FloatToStr(1+vjoints[i]->GetJointIndex()) + " ";
			}
		}




		//Preparamos el indice, sumandole uno, para cuando se lea en matlab
		sout << salida << endl;

		return true;
	}


	bool GetLinkIndexByRobotAndName(ostream& sout, istream& sinput)
	{
		string robot_name;
		string link_name;
		sinput >> robot_name;
		sinput >> link_name;

		KinBodyPtr robot = GetEnv()->GetKinBody(robot_name);
		KinBody::LinkPtr link = robot->GetLink(link_name);
		int index =link->GetIndex();

		//Preparamos el indice, sumandole uno, para cuando se lea en matlab
		sout << index << endl;

		return true;
	}

	bool GetJointValueByRobotAndName(ostream& sout, istream& sinput)
	{
		string robot_name;
		string joint_name;
		sinput >> robot_name;
		sinput >> joint_name;

		KinBodyPtr robot = GetEnv()->GetKinBody(robot_name);
		KinBody::JointPtr joint = robot->GetJoint(joint_name);

		dReal angle =  joint->GetValue(0);

		sout<<angle;

		return true;
	}

	bool GetJointIndexByName(ostream& sout, istream& sinput)
	{
		string name;
		sinput >> name;

		//RAVELOG_INFO(name);

		int index = _handKinbody->GetJointIndex(name);

		//Preparamos el indice, sumandole uno, para cuando se lea en matlab
		index = index+1;
		sout<<index;


		return true;
	}

	bool SetJointLimitsByName(ostream& sout, istream& sinput)
		{
			dReal value_low;
			dReal value_up;
			string robot_name;
			string joint_name;

			sinput >> robot_name;
			sinput >> joint_name;

			sinput >>value_low ;
			sinput >>value_up ;

			RobotBasePtr robot = GetEnv()->GetRobot(robot_name);
			KinBody::JointPtr joint = robot->GetJoint(joint_name);

			std::vector<dReal> lower,upper;
			lower.push_back(value_low);
			upper.push_back(value_up);

			joint->SetLimits(lower, upper);

			return true;
		}

	bool GetJointLimitsByName(ostream& sout, istream& sinput)
		{
			string robot_name;
			string joint_name;

			sinput >> robot_name;
			sinput >> joint_name;

			RobotBasePtr robot = GetEnv()->GetRobot(robot_name);
			KinBody::JointPtr joint = robot->GetJoint(joint_name);

			std::pair<dReal, dReal> res = joint->GetLimit(0);

			string salida = "";
			salida += FloatToStr(res.first);
			salida +=" ";
			salida += FloatToStr(res.second);
			sout << salida << endl;

			return true;
		}



	bool GetLinkIndex (ostream& sout, istream& sinput)
	{
		string name;
		sinput >> name;

		KinBody::LinkPtr link = _handKinbody->GetLink(name);
		int index = link->GetIndex();
		sout << index << endl;

		return true;
	}

    bool SetHandParameters(ostream& sout, istream& sinput)
    {
    	//RAVELOG_INFO("setting hand parameters...\n");
    	float HB, HL;
    	sinput >> HB;
    	sinput >> HL;

    	handKinematics->SetHandParameters(HB,HL);
    	handKinematics->SetArmParameters();

    	string handXMLString = handKinematics->GetHandXMLString();
    	//RAVELOG_INFO(handXMLString);
    	handKinematics->SetHandKinbody(GetEnv()->ReadRobotXMLData(RobotBasePtr(), handXMLString, std::list<std::pair<std::string,std::string> >()));
    	_handKinbody = handKinematics->GetHandKinbody();
    	GetEnv()->Add(_handKinbody);

    	return true;
    }

//    bool LoadBox(ostream& sout, istream& sinput)
//    {
//    	RAVELOG_INFO("Drawing the box... \n");
//
//    	string name, width, height, length,translationX, translationY, translationZ, orientationX, orientationY, orientationZ, orientationDeg;
//    	float widthF, heightF, lengthF;
//
//    	sinput >> name;
//
//    	sinput >> widthF;
//    	width = FloatToStr(widthF/2);
//    	sinput >> heightF;
//    	height = FloatToStr(heightF/2);
//    	sinput >> lengthF;
//    	length = FloatToStr(lengthF/2);
//
//    	sinput >> translationX;
//		sinput >> translationY;
//		sinput >> translationZ;
//		sinput >> orientationX;
//		sinput >> orientationY;
//		sinput >> orientationZ;
//		sinput >> orientationDeg;
//
//		string xmlData = "<KinBody name='"+name+"'><Body name='"+name+"' type='dynamic'><Geom type='box'><extents>"+width+" "+height+" "+length+"</extents><translation>"+translationX+" "+translationY+" "+translationZ+"</translation><diffusecolor>0.95 0.84 0.80</diffusecolor><RotationAxis>"+orientationX+" "+orientationY+" "+orientationZ+" "+orientationDeg+"</RotationAxis>" + "</Geom></Body></KinBody>";
//
//		RAVELOG_INFO(xmlData);
//
//		KinBodyPtr box = GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >());
//		GetEnv()-> Add(box);
//		GetEnv()-> UpdatePublishedBodies();
//
//		sleep(0.1);
//
//		return true;
//    }

    bool CalculateJacobianPart (ostream& sout, istream& sinput)
    {
    	//_handKinbody = GetEnv()-> GetRobot("Hand");

    	//recibe el linkid y el punto de contacto
    	int linkid;
    	float x, y, z;
    	string name;

    	//RAVELOG_INFO("#Calculating part D of jacobian...\n");
    	sinput >> name;
    	sinput >> linkid;
    	sinput >> x;
    	sinput >> y;
    	sinput >> z;

    	Transform point;
    	point.trans.x = x;
    	point.trans.y = y;
    	point.trans.z = z;

		std::vector<dReal> jacobianoD, jacobianoL;
		RobotBasePtr robot;

		RAVELOG_INFO("name:%s\n",name.c_str());
		robot = GetEnv()->GetRobot(name);

		robot -> CalculateActiveJacobian(linkid, point.trans, jacobianoD);
		//RAVELOG_INFO("P3\n");
		//RAVELOG_INFO("TAMAÑO: %d\n", jacobianoD.size());

/*
		RAVELOG_INFO("%f %f %f\n",jacobianoD[0], jacobianoD[1],jacobianoD[2]);
		RAVELOG_INFO("%f %f %f\n",jacobianoD[3], jacobianoD[4],jacobianoD[5]);
		RAVELOG_INFO("%f %f %f %f %f %f\n",jacobianoD[6], jacobianoD[7],jacobianoD[8], jacobianoD[9], jacobianoD[10],jacobianoD[11] );
*/
		robot -> CalculateActiveAngularVelocityJacobian(linkid, jacobianoL);
	/*	RAVELOG_INFO("%f %f %f\n",jacobianoL[0], jacobianoL[1],jacobianoL[2]);
		RAVELOG_INFO("%f %f %f\n",jacobianoL[3], jacobianoL[4],jacobianoL[5]);
		RAVELOG_INFO("%f %f %f\n",jacobianoL[6], jacobianoL[7],jacobianoL[8]);
*/

		string salida = "";


		for ( unsigned int pos = 0; pos < jacobianoD.size(); pos++ )
		{
			salida += FloatToStr(jacobianoD[pos]) + " ";
		}

		for ( unsigned int pos = 0; pos < jacobianoL.size(); pos++ )
		{
			salida += FloatToStr(jacobianoL[pos]) + " ";
		}


		sout << salida;

		/*
		sout << FloatToStr(jacobianoD[0]) + " " + FloatToStr(jacobianoD[1]) + " " + FloatToStr(jacobianoD[2]) + " " +
    			FloatToStr(jacobianoD[3]) + " " + FloatToStr(jacobianoD[4]) + " " + FloatToStr(jacobianoD[5]) + " " +
    			FloatToStr(jacobianoD[6]) + " " + FloatToStr(jacobianoD[7]) + " " + FloatToStr(jacobianoD[8]) + " " +
    			FloatToStr(jacobianoL[0]) + " " + FloatToStr(jacobianoL[1]) + " " + FloatToStr(jacobianoL[2]) + " " +
				FloatToStr(jacobianoL[3]) + " " + FloatToStr(jacobianoL[4]) + " " + FloatToStr(jacobianoL[5]) + " " +
				FloatToStr(jacobianoL[6]) + " " + FloatToStr(jacobianoL[7]) + " " + FloatToStr(jacobianoL[8]) + " ";
		 */

		/*
		sout << FloatToStr(-jacobianoD[0]) + " " + FloatToStr(-jacobianoD[1]) + " " + FloatToStr(-jacobianoD[2]) + " " +
				FloatToStr(-jacobianoD[3]) + " " + FloatToStr(-jacobianoD[4]) + " " + FloatToStr(-jacobianoD[5]) + " " +
				FloatToStr(-jacobianoD[6]) + " " + FloatToStr(-jacobianoD[7]) + " " + FloatToStr(-jacobianoD[8]) + " " +
				FloatToStr(jacobianoL[0]) + " " + FloatToStr(jacobianoL[1]) + " " + FloatToStr(jacobianoL[2]) + " " +
				FloatToStr(jacobianoL[3]) + " " + FloatToStr(jacobianoL[4]) + " " + FloatToStr(jacobianoL[5]) + " " +
				FloatToStr(jacobianoL[6]) + " " + FloatToStr(jacobianoL[7]) + " " + FloatToStr(jacobianoL[8]) + " ";
		 */

    	return true;
    }

    bool LoadObject(ostream& sout, istream& sinput)
	{
		RAVELOG_INFO("Drawing the object... \n");

		string name, translationX, translationY, translationZ, orientationX, orientationY, orientationZ, orientationDeg;

		sinput >> name;
		sinput >> translationX;
		sinput >> translationY;
		sinput >> translationZ;
		sinput >> orientationX;
		sinput >> orientationY;
		sinput >> orientationZ;
		sinput >> orientationDeg;


		string xmlData = "<KinBody name='"+name+"'><translation>"+translationX+" "+translationY+" "+translationZ+"</translation><rotationaxis>"+orientationX+" "+orientationY+" "+orientationZ+" "+orientationDeg+"</rotationaxis><Body name='"+name+"' type='dynamic'><Geom type='trimesh'><Data>"+name+".wrl 0.00050</Data><Render>"+name+".wrl 0.00050</Render></Geom></Body></KinBody>";
		KinBodyPtr object = GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >());
		GetEnv()-> Add(object);
		GetEnv()-> UpdatePublishedBodies();

		sleep(0.1);

		return	true;
	}

    bool LoadSphere(ostream& sout, istream& sinput)
    {
    	RAVELOG_INFO("Drawing a sphere... \n");

    	string name, peso, radius, type, translationX, translationY, translationZ;
    	int typeBin;

    	sinput >> name;
    	sinput >> peso;
    	sinput >> radius;

    	//dynamic -> 1 static -> 0
		sinput >> typeBin;
		if (typeBin == 1 ) {
			type = "dynamic";
		} else {
			type = "static";
		}

		sinput >> translationX;
		sinput >> translationY;
		sinput >> translationZ;

		string xmlData = "<KinBody name='"+name+"'>"+"<translation>"+translationX+" "+translationY+" "+translationZ+"</translation>"+"<Body type='"+type+"'>"+ "<Mass type='custom'><total>"+peso+"</total></Mass>"+"<Geom type='sphere'>"+"<Radius>"+radius+"</Radius>"+"<diffusecolor>0.95 0.84 0.80</diffusecolor>"+"</Geom></Body></KinBody>";

		KinBodyPtr sphere = GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >());
		GetEnv()-> Add(sphere);
		GetEnv()-> UpdatePublishedBodies();

		sleep(0.1);

    	return	true;
    }

    bool LoadCylinder(ostream& sout, istream& sinput)
        {
        	RAVELOG_INFO("Drawing a cylinder... \n");

        	string name, type;
        	sinput >> name;

        	int typeBin;
        	string peso, mu, radius, height, translationX, translationY, translationZ, orientationX, orientationY, orientationZ, orientationDeg;
        	sinput >> peso;
        	sinput >> mu;
        	sinput >> radius;
        	sinput >> height;

        	//dynamic -> 1 static -> 0
        	sinput >> typeBin;
        	if (typeBin == 1 ) {
        		type = "dynamic";
			} else {
				type = "static";
			}

        	sinput >> translationX;
        	sinput >> translationY;
        	sinput >> translationZ;
        	sinput >> orientationX;
        	sinput >> orientationY;
        	sinput >> orientationZ;
        	sinput >> orientationDeg;
        	string xmlData;

        	xmlData = "<KinBody name='"+name+"'>"+"<translation>"+translationX+" "+translationY+" "+translationZ+"</translation><rotationaxis>"+orientationX+" "+orientationY+" "+orientationZ+" "+orientationDeg+"</rotationaxis>" +"<Body type='"+type+"'>"+ "<Mass type='custom'><total>"+peso+"</total></Mass>"+"<Geom type='cylinder'>"+"<Radius>"+radius+"</Radius><Height>"+height+"</Height><diffusecolor>0.95 0.84 0.80</diffusecolor>"+"</Geom></Body></KinBody>";

        	KinBodyPtr cylinder = GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >());
        	GetEnv()-> Add(cylinder);
        	GetEnv()-> UpdatePublishedBodies();

        	sleep(0.1);

        	return true;
        }

    bool SetPosture(ostream& sout, istream& sinput)
    {
    	//RAVELOG_INFO("setting natural posture \n");
    	double value;
    	vector<dReal> posture;
    	for(int i=0;i<39;i++)
    	{
    		sinput >> value;
    		posture.push_back(value);
    	}

    	vector<dReal> jointValues;
        _handKinbody->GetDOFValues(jointValues);
        for(int i=0;i<_handKinbody->GetDOF();i++)
        	jointValues[i]=0.0;

        //printPosture();

    	for(std::vector<KinBody::JointPtr>::const_iterator itjoint = _handKinbody->GetJoints().begin(); itjoint != _handKinbody->GetJoints().end(); ++itjoint)
    	{
    		jointValues[(*itjoint)->GetDOFIndex()]=posture[getMatlabIndex((*itjoint)->GetName())];
    	}


    	_handKinbody->SetDOFValues(jointValues,false);
    	//printPosture();

    	return true;
    }

    bool GetPosture(ostream& sout, istream& sinput)
	{
		//RAVELOG_INFO("getting hand posture \n");
		vector<dReal> jointValues;
		_handKinbody->GetDOFValues(jointValues);

		vector<dReal> postureInMatlab;
		for (int i =0;i<39;i++)
			postureInMatlab.push_back(0);

		for(std::vector<KinBody::JointPtr>::const_iterator itjoint = _handKinbody->GetJoints().begin(); itjoint != _handKinbody->GetJoints().end(); ++itjoint)
		{
			postureInMatlab[getMatlabIndex((*itjoint)->GetName())]= degrees((*itjoint)->GetValue(0));
		}

		for (int j = 0; j < 30; j++ )
		{
			sout << postureInMatlab[j];
			sout << " ";
		}
		sout << ";";
		for (int j = 30; j < 39; j++ )
		{
			sout << postureInMatlab[j];
			sout << " ";
		}

		return true;
	}

    bool ReturnPointsJointsFinger(ostream& sout, istream& sinput)
    {
    	string separator = "||";

    	//RAVELOG_INFO("Finding the points of the joints\n");

    	string finger_segment_name;
    	sinput >> finger_segment_name;

    	vector<KinBody::JointPtr> vbodyjoints = _handKinbody->GetJoints();
    	FOREACHC(itjoint, vbodyjoints)
    	{
    		KinBody::LinkPtr body0 = (*itjoint)->GetFirstAttached();
    		KinBody::LinkPtr body1 = (*itjoint)->GetSecondAttached();

    		float thickness;

    		if (finger_segment_name == body0->GetName()
				||
				finger_segment_name == body1->GetName())
    		{
				RaveVector<dReal> anchor = (*itjoint)->GetAnchor();

				string type = "dynamic";

				/*
				thickness = handKinematics->GetThicknessSegment(body0->GetName());
				DrawSphere(body0->GetName(), peso, thickness/2, type, anchor.x, anchor.y, anchor.z);
				 */

				//Para cuando se dibujan todos los segmentos de la mano
				if (finger_segment_name == body1->GetName())
				{
					thickness = handKinematics->GetThicknessSegment(body0->GetName());
					//DrawSphere(body0->GetName() + FloatToStr(rand()%100000), peso, thickness/2, type, anchor.x, anchor.y, anchor.z);
					sout << body0->GetName() + separator + FloatToStr(thickness/2) + separator + FloatToStr(anchor.x) + separator + FloatToStr(anchor.y) + separator + FloatToStr(anchor.z) + separator;
				}
				//Cuando se generan esferas para toda la mano hay algunas que se
				//dibujaran dos veces (para evitarlo quitar el else y 'finger_segment_name == body1->GetName()'
				else
				{
					thickness = handKinematics->GetThicknessSegment(body1->GetName());
					//DrawSphere(body0->GetName() + FloatToStr(rand()%100000), peso, thickness/2, type, anchor.x, anchor.y, anchor.z);
					sout << body1->GetName() + separator + FloatToStr(thickness/2) + separator + FloatToStr(anchor.x) + separator + FloatToStr(anchor.y) + separator + FloatToStr(anchor.z) + separator;
				}

				//Cuando el segmento de la mano es Distal hay que dibujar dos esferas con el mismo radio
				//una al principio y otra al final del segmento
				if (finger_segment_name.find("Distal")!=string::npos)
				{
					thickness = handKinematics->GetThicknessSegment(body1->GetName());
					float length = handKinematics->GetLenghtSegment(finger_segment_name);

					Transform lastLink = Transform();

					lastLink.trans.x = 0;
					lastLink.trans.y = length;
					lastLink.trans.z = 0;

					Transform t = body1->GetTransform();
					lastLink = t * lastLink;

					sout << body1->GetName() + separator + FloatToStr(thickness/2) + separator + FloatToStr(lastLink.trans.x) + separator + FloatToStr(lastLink.trans.y) + separator + FloatToStr(lastLink.trans.z) + separator;
				}
    		}
    	}
    	return true;
    }

    void DrawSphere(string name, float peso, float radius, string type, float translationX, float translationY, float translationZ)
    {
    	string pesoStr, radiusStr, translationXStr, translationYStr, translationZStr;

		//Convert float to string
		pesoStr = FloatToStr(peso);
		radiusStr = FloatToStr(radius);
		translationXStr = FloatToStr(translationX);
		translationYStr = FloatToStr(translationY);
		translationZStr = FloatToStr(translationZ);

		string xmlData = "<KinBody name='"+name+"'><translation>"+translationXStr+" "+translationYStr+" "+translationZStr+"</translation><Body type='"+type+"'><Mass type='custom'><total>"+pesoStr+"</total></Mass><Geom type='sphere'><Radius>"+radiusStr+"</Radius><diffusecolor>0 0 1</diffusecolor></Geom></Body></KinBody>";

		KinBodyPtr sphere = GetEnv()->ReadKinBodyXMLData(KinBodyPtr(), xmlData, std::list<std::pair<std::string,std::string> >());
		GetEnv()-> Add(sphere);
		GetEnv()-> UpdatePublishedBodies();

		sleep(0.1);
    }

    void printPosture()
    {
		vector<dReal> jointValues;
		_handKinbody->GetDOFValues(jointValues);

		vector<dReal> postureInMatlab;
		for (int i =0;i<39;i++)
			postureInMatlab.push_back(0);

		for(std::vector<KinBody::JointPtr>::const_iterator itjoint = _handKinbody->GetJoints().begin(); itjoint != _handKinbody->GetJoints().end(); ++itjoint)
		{
			postureInMatlab[getMatlabIndex((*itjoint)->GetName())]= degrees((*itjoint)->GetValue(0));
		}


		RAVELOG_INFO(" Hand Posture:\n %f %f %f %f %f %f\n %f %f %f %f %f %f\n %f %f %f %f %f %f\n %f %f %f %f %f %f\n %f %f %f %f %f %f\n",
				postureInMatlab[0],postureInMatlab[1],postureInMatlab[2],postureInMatlab[3],postureInMatlab[4],postureInMatlab[5],
				postureInMatlab[6],postureInMatlab[7],postureInMatlab[8],postureInMatlab[9],postureInMatlab[10],postureInMatlab[11],
				postureInMatlab[12],postureInMatlab[13],postureInMatlab[14],postureInMatlab[15],postureInMatlab[16],postureInMatlab[17],
				postureInMatlab[18],postureInMatlab[19],postureInMatlab[20],postureInMatlab[21],postureInMatlab[22],postureInMatlab[23],
				postureInMatlab[24],postureInMatlab[25],postureInMatlab[26],postureInMatlab[27],postureInMatlab[28],postureInMatlab[29]);

		RAVELOG_INFO(" Arm Posture:\n %f %f %f\n %f %f %f\n %f %f %f \n",
						postureInMatlab[30],postureInMatlab[31],postureInMatlab[32],
						postureInMatlab[33],postureInMatlab[34],postureInMatlab[35],
						postureInMatlab[36],postureInMatlab[37],postureInMatlab[38]);
    }

	int getMatlabIndex(string name)
	{
		for (int i=0; _humanJointNames.size();i++)
		{
			if (name == _humanJointNames[i])
			{
				return i;
			}
		}
		return 0;
	}

	dReal degrees(dReal rad)
	{
		return rad*180/3.1416;
	}

	string FloatToStr(float num)
	{
		 std::ostringstream stream;
		 stream << num;
		 return stream.str();
	}

	bool GetHandLimits(ostream& sout, istream& sinput)
	{
		string finger;
		sinput >> finger;
		sout << handKinematics->GetLimits(finger);
		return true;
	}

private:
	vector<string> _humanJointNames;
	ArmHandKinematics* handKinematics;
	RobotBasePtr _handKinbody;
};

#endif
