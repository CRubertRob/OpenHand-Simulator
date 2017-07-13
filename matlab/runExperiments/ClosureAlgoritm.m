function [contactsVector, finalGraspPosture, fingers] = ClosureAlgoritm(umano_id,posture_0,posture_1,arm_posture,graspFingers)
%graspFingers = [1 1 1 0 0]


arm_postureStr = mat2str(reshape(transpose(arm_posture),1,numel(arm_posture)));

% CMC, MCP (Extension, Flexion, Adduccion, Abduccion) 
% IP, PIP, DIP (Extension, Flexion)
% ORDEN DE LOS DATOS: CMC MCP ( IP || PIP DIP )
thumb_limits = orProblemSendCommand(['gethandlimits Thumb'], umano_id);
index_limits = orProblemSendCommand(['gethandlimits Index'], umano_id);
middle_limits = orProblemSendCommand(['gethandlimits Middle'], umano_id);
ring_limits = orProblemSendCommand(['gethandlimits Ring'], umano_id);
small_limits = orProblemSendCommand(['gethandlimits Small'], umano_id);

limites(1,:) = Convert_str2mat(thumb_limits, 1, 12);
limites(2,:) = Convert_str2mat(index_limits, 1, 12);
limites(3,:) = Convert_str2mat(middle_limits, 1, 12);
limites(4,:) = Convert_str2mat(ring_limits, 1, 12);
limites(5,:) = Convert_str2mat(small_limits, 1, 12);

limites_inf = [ limites(:,1) limites(:,3) limites(:,5) limites(:,7) limites(:,9) limites(:,11) ]*pi/180;
limites_sup = [ limites(:,2) limites(:,4) limites(:,6) limites(:,8) limites(:,10) limites(:,12) ]*pi/180;


matrix_normals_contactPointsIN = [];
robots = orEnvGetRobots();
robot = robots{1};

posture_inicial = posture_0;

N = 50;
velocity = (posture_1 - posture_0)/N;
numGraspFingers =size(find(graspFingers==1),2);
i = 0;
fingersIN = [1 1 1 1 1];
flag=numel(find(fingersIN == 0));

while flag < numGraspFingers
    i = i + 1;
    if i>100, break, end 	%to break when thumb takes too long to contact
    if i>(N-5) %When it gets close to the values of the fingers that do not contact, we make the velocity really small
        velocity (find(graspFingers==0),:)= (posture_1(find(graspFingers==0),:)- posture_0(find(graspFingers==0),:))/N;
    end
   
    posture_anterior = posture_0;  
    [posture_0, fingersOUT, matrix_normals_contactPointsOUT] = CloseHandVelocity(1, posture_anterior, fingersIN, umano_id, arm_postureStr, posture_inicial, velocity, fingersIN, limites_inf, limites_sup, matrix_normals_contactPointsIN);
    fingersIN = fingersOUT;
    matrix_normals_contactPointsIN = matrix_normals_contactPointsOUT;
    [fingersOUT,matrix_normals_contactPointsOUT] = checkCollisionODE(matrix_normals_contactPointsIN, robot, fingersIN, umano_id);
    flag=numel(find(fingersOUT == 0));
    fingersIN = fingersOUT;
    matrix_normals_contactPointsIN = matrix_normals_contactPointsOUT;    
end

%Remove fingers that did not contact
fingers = [1 2 3 4 5];
fingers(find(fingersIN == 1)) =[];

    
    
%codigo cierre completo FALTA ADAPTARLO
% fingers = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
% joint_movements(1,:) = [1 1 1];
% joint_movements(2,:) = [1 1 1];
% joint_movements(3,:) = [1 1 1];
% joint_movements(4,:) = [1 1 1];
% joint_movements(5,:) = [1 1 1];
% while flag < 5
%     i = i + 1; 
%     posture_anterior = posture;
%     [posture] = CloseHandFull (i, posture_anterior, umano_id, joint_movements, arm_postureStr, posture_inicial);
%     %pause(0.01);
%     [joint_movements, fingersOUT, matrix_normals_contactPointsOUT] = checkCollisionODEFull(robot, joint_movements, fingersIN, matrix_normals_contactPointsIN);
%     flag=numel(find(fingersOUT == 0));   
%     fingersIN = fingersOUT;
%     matrix_normals_contactPointsIN = matrix_normals_contactPointsOUT;     
% end

contactsVector = [];
finalGraspPosture = [];

if flag > 2 		%Minimun number of contacts
	for i = fingers
	    for j = 1:3 
	        if size(matrix_normals_contactPointsIN.point{i},1) == 0
	            contactsVector(j,i) = 0.0;
	            contactsVector(j + 3,i) = 0.0;            
	        else
	            contactsVector(j,i) = matrix_normals_contactPointsIN.point{i}(j);
	            contactsVector(j + 3,i) = matrix_normals_contactPointsIN.normal{i}(j);
	        end        
	    end       
	end 
	%Quitar las columnas que tengan 0 ya que no nos interesan y almaceno el
	%vector con 0 para algun uso futuro
	
	columnas = [];
	for i = 1:size(contactsVector,2)
	    if contactsVector(1,i) == 0
	        columnas = [columnas i];
	    end
	end
    contactsVector(:,columnas) = [];
	
	%obtener la postura final
	str_postura_actual = orProblemSendCommand('getposture', umano_id);
	finalGraspPosture = Convert_str2mat(str_postura_actual, 5, 6);
end



