function [finalGraspPostureMatlab] = TestMatlabPosture(HB, HL, arm_posture,posture_1,posture_0,TObjectRelHand, gVector, graspFingers)

close all;
clear M_s M_e M_w M1n M2n M3n M4n
clear normales puntos_contacto cdg weight
clear q1 q2 q3 q4 q5
clear q1p q2p q3p q4p q5p
clear q1pp q2pp q3pp q4pp q5pp
clear nF1 nF2 nF3 nF4 nF5 P1 P2 P3 P4 P5

%Object
global mu
radius=0.025
height=0.20;
mass = 0.401;
mu=0.8;

 [finalGraspPostureMatlab] = CalculoBiomecanico(HB, HL, arm_posture, posture_0, posture_1, radius, height, mass, mu, TObjectRelHand, gVector,graspFingers)