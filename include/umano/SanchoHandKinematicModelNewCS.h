#ifndef SANCHOHANDKINEMATICMODEL_H
#define SANCHOHANDKINEMATICMODEL_H

#include "hand.h"

class SanchoHandKinematicModel: public HandKinematicModel
{
public:

	void SetBonesLength(){
    	// Length of the bones
		// Thumb
    	hand->GetThumb()->GetDistal()->SetLength(0.158*hand->GetHL()); //Distal phalanx (Buchholz et al, 1992)
    	hand->GetThumb()->GetProximal()->SetLength(0.196*hand->GetHL()); //Proximal phalanx (Buchholz et al, 1992)
    	hand->GetThumb()->GetMetacarpal()->SetLength(0.251*hand->GetHL()); //Metacarpal (Buchholz et al, 1992)

		// Index
    	hand->GetIndex()->GetDistal()->SetLength(0.097*hand->GetHL()); //Distal phalanx  (Buchholz et al, 1992)
    	hand->GetIndex()->GetMedial()->SetLength(0.143*hand->GetHL()); //Medial phalanx (Buchholz et al, 1992)
    	hand->GetIndex()->GetProximal()->SetLength((1.94+0.45)*hand->GetIndex()->GetMedial()->GetLength()/1.29); //Proximal phalanx (An et al, 1979)
    	hand->GetIndex()->GetMetacarpal()->SetLength(sqrt(pow((0.374*hand->GetHL()),2)+pow((0.126*hand->GetHB()),2))); //Metacarpal. Estimated from (Buchholz et al, 1992)

		// Middle
    	hand->GetMiddle()->GetDistal()->SetLength(0.108*hand->GetHL()); //Distal phalanx   (Buchholz et al, 1992)
    	hand->GetMiddle()->GetMedial()->SetLength(0.170*hand->GetHL()); //Medial phalanx  (Buchholz et al, 1992)
    	hand->GetMiddle()->GetProximal()->SetLength((1.62+0.37)*hand->GetMiddle()->GetMedial()->GetLength()/1.22); //Proximal phalanx  (An et al, 1979)
    	hand->GetMiddle()->GetMetacarpal()->SetLength((0.446-0.073)*hand->GetHL()); //Metacarpal. Estimated from (Buchholz et al, 1992)

		// Ring
    	hand->GetRing()->GetDistal()->SetLength(0.107*hand->GetHL()); //Distal phalanx   (Buchholz et al, 1992)
    	hand->GetRing()->GetMedial()->SetLength(0.165*hand->GetHL()); //Medial phalanx  (Buchholz et al, 1992)
    	hand->GetRing()->GetProximal()->SetLength((1.56+0.34)*hand->GetRing()->GetMedial()->GetLength()/1.21); //Proximal phalanx  (An et al, 1979)
    	hand->GetRing()->GetMetacarpal()->SetLength(sqrt(pow((0.336*hand->GetHL()),2)+pow((0.077*hand->GetHB()),2))); //Metacarpal. Estimated from (Buchholz et al, 1992)

		// Little
    	hand->GetSmall()->GetDistal()->SetLength(0.093*hand->GetHL()); //Distal phalanx   (Buchholz et al, 1992)
    	hand->GetSmall()->GetMedial()->SetLength(0.117*hand->GetHL()); //Medial phalanx  (Buchholz et al, 1992)
    	hand->GetSmall()->GetProximal()->SetLength((1.78+0.49)*hand->GetSmall()->GetMedial()->GetLength()/1.29); //Proximal phalanx  (An et al, 1979)
    	hand->GetSmall()->GetMetacarpal()->SetLength(sqrt(pow((0.295*hand->GetHL()),2)+pow((0.179*hand->GetHB()),2))); //Metacarpal. Estimated from (Buchholz et al, 1992)
	}
	void SetBonesThickness(){
    	// Thicknesses of the bones estimated from own measurement

    	// Thumb
    	hand->GetThumb()->GetMetacarpal()->SetThickness(0.438*hand->GetHB());
    	hand->GetThumb()->GetProximal()->SetThickness(0.5*(0.231+0.227)*hand->GetHB());
    	hand->GetThumb()->GetDistal()->SetThickness(0.5*(0.239+0.185)*hand->GetHB());

    	// Index
    	hand->GetIndex()->GetMetacarpal()->SetThickness(1.2*0.5*(0.231+0.227)*hand->GetHB());
    	hand->GetIndex()->GetProximal()->SetThickness(0.5*(0.231+0.227)*hand->GetHB());
    	hand->GetIndex()->GetMedial()->SetThickness(0.5*(0.210+0.186)*hand->GetHB());
    	hand->GetIndex()->GetDistal()->SetThickness(0.5*(0.198+0.153)*hand->GetHB());

    	// Medial
    	hand->GetMiddle()->GetMetacarpal()->SetThickness((0.212+0.220)*0.5*hand->GetHB()*1.2);
    	hand->GetMiddle()->GetProximal()->SetThickness((0.212+0.220)*0.5*hand->GetHB());
    	hand->GetMiddle()->GetMedial()->SetThickness((0.206+0.187)*0.5*hand->GetHB());
    	hand->GetMiddle()->GetDistal()->SetThickness((0.200+0.159)*0.5*hand->GetHB());

    	// Ring
    	hand->GetRing()->GetMetacarpal()->SetThickness((0.202+0.204)*0.5*hand->GetHB()*1.2);
    	hand->GetRing()->GetProximal()->SetThickness((0.202+0.204)*0.5*hand->GetHB());
    	hand->GetRing()->GetMedial()->SetThickness((0.192+0.172)*0.5*hand->GetHB());
    	hand->GetRing()->GetDistal()->SetThickness((0.187+0.148)*0.5*hand->GetHB());

    	// Little
    	hand->GetSmall()->GetMetacarpal()->SetThickness((0.186+0.180)*0.5*hand->GetHB()*1.2);
    	hand->GetSmall()->GetProximal()->SetThickness((0.186+0.180)*0.5*hand->GetHB());
    	hand->GetSmall()->GetMedial()->SetThickness((0.172+0.157)*0.5*hand->GetHB());
    	hand->GetSmall()->GetDistal()->SetThickness((0.165+0.133)*0.5*hand->GetHB());
	}
	void SetCS(){
    	// CS Location
    	hand->GetThumb()->GetMetacarpal()->SetCSLocation(0.0,-0.073*hand->GetHL(), 0.196*hand->GetHB());
    	hand->GetThumb()->GetProximal()->SetCSLocation(0.0, -hand->GetThumb()->GetMetacarpal()->GetLength(),0.0);
    	hand->GetThumb()->GetDistal()->SetCSLocation(0.0, -hand->GetThumb()->GetProximal()->GetLength(),0.0);

    	hand->GetIndex()->GetMetacarpal()->SetCSLocation(0.0,-0.073*hand->GetHL(), 0.126*hand->GetHB());
    	hand->GetIndex()->GetProximal()->SetCSLocation(0.0, -hand->GetIndex()->GetMetacarpal()->GetLength(),0.0);
    	hand->GetIndex()->GetMedial()->SetCSLocation(0.0, -hand->GetIndex()->GetProximal()->GetLength(),0.0);
    	hand->GetIndex()->GetDistal()->SetCSLocation(0.0, -hand->GetIndex()->GetMedial()->GetLength(),0.0);

    	hand->GetMiddle()->GetMetacarpal()->SetCSLocation(0.0, -0.073*hand->GetHL(), 0.0);
    	hand->GetMiddle()->GetProximal()->SetCSLocation(0.0, -hand->GetMiddle()->GetMetacarpal()->GetLength(),0.0);
    	hand->GetMiddle()->GetMedial()->SetCSLocation(0.0, -hand->GetMiddle()->GetProximal()->GetLength(),0.0);
    	hand->GetMiddle()->GetDistal()->SetCSLocation(0.0, -hand->GetMiddle()->GetMedial()->GetLength(),0.0);

    	hand->GetRing()->GetMetacarpal()->SetCSLocation(0.0, -0.073*hand->GetHL(), -0.129*hand->GetHB());
    	hand->GetRing()->GetProximal()->SetCSLocation(0.0, -hand->GetRing()->GetMetacarpal()->GetLength(),0.0);
    	hand->GetRing()->GetMedial()->SetCSLocation(0.0, -hand->GetRing()->GetProximal()->GetLength(),0.0);
    	hand->GetRing()->GetDistal()->SetCSLocation(0.0, -hand->GetRing()->GetMedial()->GetLength(),0.0);

    	hand->GetSmall()->GetMetacarpal()->SetCSLocation(0.0, -0.073*hand->GetHL(), -0.223*hand->GetHB());
    	hand->GetSmall()->GetProximal()->SetCSLocation(0.0,-hand->GetSmall()->GetMetacarpal()->GetLength(),0.0);
    	hand->GetSmall()->GetMedial()->SetCSLocation(0.0,-hand->GetSmall()->GetProximal()->GetLength(),0.0);
    	hand->GetSmall()->GetDistal()->SetCSLocation(0.0,-hand->GetSmall()->GetMedial()->GetLength(),0.0);

    	// CS Rotation Angles

    	//Thumb
    	// Measured with digital camera
    	float ang_z=23.17;
    	float ang_x=32.58;
    	float ang_y=-87.98;
    	ang_x=-ang_x;
    	ang_y=-ang_y;

    	hand->GetThumb()->GetMetacarpal()->SetCSRotationAngles(ang_x, ang_y, ang_z);
    	hand->GetThumb()->GetProximal()->SetCSRotationAngles(0.0, 0.0, 0.0);
    	hand->GetThumb()->GetDistal()->SetCSRotationAngles(0.0, 0.0, 0.0);

    	float alpha1;
    	float alpha2;

    	//Index
    	alpha1=-15; //An & Cooney (1991)
    	alpha2=atan(0.126*hand->GetHB()/(0.374*hand->GetHL()))*180/PI; // From the estimation of the rotation center of CMC joint
    	alpha1 = -alpha1;
    	alpha2 = -alpha2;

    	hand->GetIndex()->GetMetacarpal()->SetCSRotationAngles(alpha2,0.0,0.0);
    	hand->GetIndex()->GetProximal()->SetCSRotationAngles(alpha1,0.0,0.0);
    	hand->GetIndex()->GetMedial()->SetCSRotationAngles(0.0, 0.0, 0.0);
    	hand->GetIndex()->GetDistal()->SetCSRotationAngles(0.0, 0.0, 0.0);

    	//Middle
    	alpha1=-6; //An & Cooney (1991)
    	alpha2=0.0*180/PI;
		alpha1 = -alpha1;
		alpha2 = -alpha2;

    	hand->GetMiddle()->GetMetacarpal()->SetCSRotationAngles(alpha2,0.0,0.0);
    	hand->GetMiddle()->GetProximal()->SetCSRotationAngles(alpha1,0.0,0.0);
    	hand->GetMiddle()->GetMedial()->SetCSRotationAngles(0.0, 0.0, 0.0);
    	hand->GetMiddle()->GetDistal()->SetCSRotationAngles(0.0, 0.0, 0.0);

    	//Ring
    	alpha1=0; //An & Cooney (1991)
    	alpha2 = atan((-0.077*hand->GetHB())/(0.336*hand->GetHL()))*180/PI;
		alpha1 = -alpha1;
		alpha2 = -alpha2;

    	hand->GetRing()->GetMetacarpal()->SetCSRotationAngles(alpha2,0.0,0.0);
    	hand->GetRing()->GetProximal()->SetCSRotationAngles(alpha1,0.0,0.0);
    	hand->GetRing()->GetMedial()->SetCSRotationAngles(0.0, 0.0, 0.0);
    	hand->GetRing()->GetDistal()->SetCSRotationAngles(0.0, 0.0, 0.0);

    	//Small
    	alpha1=7; //An & Cooney (1991)
    	alpha2=atan((-0.170*hand->GetHB())/(0.295*hand->GetHL()))*180/PI;
		alpha1 = -alpha1;
		alpha2 = -alpha2;

    	hand->GetSmall()->GetMetacarpal()->SetCSRotationAngles(alpha2,0.0,0.0);
    	hand->GetSmall()->GetProximal()->SetCSRotationAngles(alpha1,0.0,0.0);
    	hand->GetSmall()->GetMedial()->SetCSRotationAngles(0.0, 0.0, 0.0);
    	hand->GetSmall()->GetDistal()->SetCSRotationAngles(0.0, 0.0, 0.0);
	}
	void SetFingerLimits()
	{
		//CMCLimits (Extension,Flexion, Adduccion, Abduccion)
		hand->GetThumb()->SetCMCLimits(-25.0, 35.0, -30.0, 60.0); //I think it should have limit -
		//hand->GetThumb()->SetCMCLimits(-25.0, 35.0, 0.0, 60.0); //Original
		hand->GetThumb()->SetMCPLimits(-10.0, 80.0, -30.0, 60.0);
		//hand->GetThumb()->SetMCPLimits(-10.0, 55.0, 0.0, 60.0);//Original
		hand->GetThumb()->SetIPLimits(-15.0, 80.0);

		hand->GetIndex()->SetCMCLimits(0.0, 0.0, 0.0, 0.0); 	//No Movement in CMC
		//hand->GetIndex()->SetMCPLimits(0.0, 80.0, -13.0, 42.0);
		hand->GetIndex()->SetMCPLimits(0.0, 90.0, -15.0, 42.0);
		hand->GetIndex()->SetPIPLimits(0.0, 100.0);
		hand->GetIndex()->SetDIPLimits(-10.0, 90.0);

		hand->GetMiddle()->SetCMCLimits(0.0, 0.0, 0.0, 0.0); 	//No Movement in CMC
		hand->GetMiddle()->SetMCPLimits(0.0, 90.0, -8.0, 35.0);
		hand->GetMiddle()->SetPIPLimits(0.0, 100.0);
		hand->GetMiddle()->SetDIPLimits(-10.0, 90.0);

		//hand->GetRing()->SetCMCLimits(0.0, 10.0, 0.0, 10.0); original peña
		hand->GetRing()->SetCMCLimits(0.0, 15.0, 0.0, 0.0);
		hand->GetRing()->SetMCPLimits(0.0, 90.0, -14.0, 20.0);
		hand->GetRing()->SetPIPLimits(0.0, 100.0);
		hand->GetRing()->SetDIPLimits(-20.0, 90.0);

		//hand->GetSmall()->SetCMCLimits(0.0, 20.0, 0.0, 20.0);
		hand->GetSmall()->SetCMCLimits(0.0, 30.0, 0.0, 0.0);
		//hand->GetSmall()->SetMCPLimits(0.0, 80.0, -33.0, 19.0); original peña
		hand->GetSmall()->SetMCPLimits(0.0, 90.0, -40.0, 30.0); //Modificado para ampliar
		hand->GetSmall()->SetPIPLimits(0.0, 100.0);
		hand->GetSmall()->SetDIPLimits(-30.0, 90.0);


//		//CMCLimits (Extension,Flexion, Adduccion, Abduccion)
//		hand->GetThumb()->SetCMCLimits(-25.0, 35.0, -30.0, 60.0); //I think it should have limit -
//		hand->GetThumb()->SetMCPLimits(46.1, 46.1, 0.0, 0.0);
//		hand->GetThumb()->SetIPLimits(8.5, 8.5);
//
//		hand->GetIndex()->SetCMCLimits(0.0, 0.0, 0.0, 0.0); 	//No Movement in CMC
//		hand->GetIndex()->SetMCPLimits(0.0, 90.0, 0.0, 0.0);
//		hand->GetIndex()->SetPIPLimits(25.5, 25.5);
//		hand->GetIndex()->SetDIPLimits(13.1, 13.1);
//
//		hand->GetMiddle()->SetCMCLimits(0.0, 0.0, 0.0, 0.0); 	//No Movement in CMC
//		hand->GetMiddle()->SetMCPLimits(0.0, 90.0, 0.0, 0.0);
//		hand->GetMiddle()->SetPIPLimits(30.1, 30.1);
//		hand->GetMiddle()->SetDIPLimits(12.7, 12.7);
//
//		hand->GetRing()->SetCMCLimits(2.0, 2.0, 0.0, 0.0);
//		hand->GetRing()->SetMCPLimits(0.0, 90.0, 0.0, 0.0);
//		hand->GetRing()->SetPIPLimits(34.5, 34.5);
//		hand->GetRing()->SetDIPLimits(11.7, 11.7);
//
//
//		hand->GetSmall()->SetCMCLimits(5.0, 5.0, 0.0, 0.0);
//		hand->GetSmall()->SetMCPLimits(0.0, 90.0, 0.0, 0.0); //Modificado para ampliar
//		hand->GetSmall()->SetPIPLimits(32.1, 32.1);
//		hand->GetSmall()->SetDIPLimits(15.6, 15.6);

	}

};

#endif
