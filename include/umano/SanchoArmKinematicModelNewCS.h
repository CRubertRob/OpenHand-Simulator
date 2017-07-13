#ifndef SANCHOARMKINEMATICMODEL_H
#define SANCHOARMKINEMATICMODEL_H

#include "hand.h"

class SanchoArmKinematicModel: public ArmKinematicModel
{
public:

	void SetBonesLength()
	{
		//Ajustadas a la mesh
		arm->GetArm()->SetLength(0.27);
		arm->GetForeArm()->SetLength(0.248);

		arm->GetWrist()->SetLength(0.01);
	}
	void SetBonesThickness()
	{
		//Diameter
		arm->GetArm()->SetThickness(0.12);
		//arm->GetForeArm()->SetThickness(0.08);
		arm->GetWrist()->SetThickness(0.01);

		//Para mujeres de experimento
		arm->GetForeArm()->SetThickness(0.04);
	}

	void SetCS()
	{
		//CS Location
		arm->GetArm()->SetCSLocation(0.0, 0.0, 0.0);
		arm->GetForeArm()->SetCSLocation(0.0, -arm->GetArm()->GetLength(), 0.0);
		arm->GetWrist()->SetCSLocation(0.0, -arm->GetForeArm()->GetLength(), 0.0);

		// CS Rotation Angles
		arm->GetArm()->SetCSRotationAngles(0.0, 0.0, 0.0);
		arm->GetForeArm()->SetCSRotationAngles(0.0, 0.0, 0.0);
		arm->GetWrist()->SetCSRotationAngles(0.0, 0.0, 0.0);

	}

	void SetSegmentLimits()
	{
		arm->SetElbowLimits(0.0, 145.0, 0.0, 175.0);
		//arm->SetWirstLimits(-15.0, 45.0, -85.0, 85.0);
		//Ampliados a ojo: abduccion, flexion
		arm->SetWirstLimits(-30.0, 90.0, -90.0, 90.0);
		arm->SetShoulderLimits(-180.0, 50.0, -30.0, 180.0, -180.0, 180.0);
	}
};

#endif
