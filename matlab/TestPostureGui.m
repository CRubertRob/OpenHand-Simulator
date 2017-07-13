%function [QM] = TestPostureGui(posturePath, markers, postureFileI, postureFileE, ifgrasp, ifqm, HB, HL, j)
function [QM] = TestPostureGui()
% postureFile='Posture28'
% posturePath='/home/bea2/workspace/umano/env/postures/Humano/Grasp'
% ifgrasp=0
% ifqm=0

%orEnvClose()
objectName = 'object';
QM=[];
% addpath(genpath(posturePath))

HB=78;
HL=172;
% run(postureFileE);
disp('posture 0')

disp ('end')

disp('no initial posture')
posture_0(1,:)=[	-17.0000	4.0000	0.0000	32.9999	0.0000	0.0000	];        
posture_0(2,:)=[	0.0000	0.0000	0.0000	0.0000	0.0000	0.0000	]*pi/180;
posture_0(3,:)=[	0.0000	0.0000	0.0000	0.0000	0.0000	0.0000	]*pi/180;
posture_0(4,:)=[	0.0000	0.0000	0.0000	0.0000	0.0000	0.0000	]*pi/180;
posture_0(5,:)=[	0.0000	0.0000	0.0000	0.0000	0.0000	0.0000	]*pi/180;

arm_posture(1,:) = [	0.0000	0.0000	0.0000	];
arm_posture(2,:) = [	0.0000	0.0000	0.0000	];
arm_posture(3,:) = [	0.0000	0.0000	0.0000	];

graspFingers = [1 1 1 1 1 ];

disp('no mark file')   
p =[];

posture_1=posture_0;    

j=-1;    

performGrasp = 1;

calculateMesures = 0;

testMatlabPosture = 0;
randomValues = 0;
posture_0

[QM] = TestExperiments(p, objectName, HB, HL, posture_0, posture_1, arm_posture, performGrasp, calculateMesures, testMatlabPosture, randomValues, graspFingers);   
