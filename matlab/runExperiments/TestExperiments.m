function [QM] = TestExperiments(p, objectName, HB, HL, posture_0, posture_1, arm_posture, performGrasp, calculateMesures, testMatlabPosture, randomValues, graspFingers)

showPosture1 = 1;
showPosture0 = 0;
drawHandMarkers = 1; 
drawObjectMarkers = 1;
drawWrist = 0;

contactsVector = [];

posture_0 = posture_0*pi/180;
posture_1 = posture_1*pi/180;
arm_posture = arm_posture*pi/180;

if( ~exist('graspFingers','var') )
    graspFingers= [1 1 1 1 1];
end

if( ~exist('calculateMesures','var') )
	calculateMesures = 1;
end

if( ~exist('testMatlabPosture','var') )
	testMatlabPosture = 0;
end
if( ~exist('performGrasp','var') )
	performGrasp = 1;
end

if( exist('scene','var') )
    orEnvLoadScene(scene,1);
    scene
end


if (size(p,1) > 0)
    [wristAxes, wristPoint, objectT, handles] = PreparePostureWithMarkers(p, drawHandMarkers, drawObjectMarkers, drawWrist);
end

if (size(p,1) > 0)
	[umano_id, idLinkObjeto, TObjectRelHand, gVector] = PrepareOR(HB, HL, posture_0, posture_1, arm_posture, showPosture1, showPosture0, objectName, wristAxes, wristPoint, objectT);
else
	[umano_id, idLinkObjeto, TObjectRelHand, gVector] = PrepareOR(HB, HL, posture_0, posture_1, arm_posture, showPosture1, showPosture0, objectName);
end
TObjectRelHand
gVector 

if( randomValues == 1 )
	 display('Generating random posture')
	 [posture_1] = GetRandomPosture(umano_id, posture_1);
end

if (performGrasp == 1)
    [contactsVector, finalGraspPosture,fingers] = ClosureAlgoritm(umano_id,posture_0,posture_1,arm_posture, graspFingers);
end

if size(contactsVector,1) > 0 
	if calculateMesures == 1
		[medidas_norm] = CalculateMeasures(umano_id, idLinkObjeto, contactsVector);
	else
		medidas_norm = [];
	end
    
	if testMatlabPosture == 1	
        [finalGraspPostureMatlab] = TestMatlabPosture(HB, HL, arm_posture,posture_1,posture_0,TObjectRelHand, gVector, graspFingers);
    	Diff = finalGraspPostureMatlab-finalGraspPosture;
		DiferenciaMaxima = max(max(abs(Diff)))
	end
	
	QM.measures = medidas_norm;
	QM.posture = posture_1*180/pi;
	QM.finalGraspPosture = finalGraspPosture;
else
	QM.measures = [];
	QM.posture = [];
	QM.finalGraspPosture = [];
end

str_postura_actual = orProblemSendCommand('getposture', umano_id);
finalGraspPosture = Convert_str2mat(str_postura_actual, 5, 6);
%orEnvClose();
