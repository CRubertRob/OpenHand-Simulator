function [umano_id, idLinkObjeto, TObjectRelHand, gVector, hand_id] = PrepareOR(HB, HL, posture_0, posture_1, arm_posture, showPosture1, showPosture0, objectName, wristAxes, wristPoint, objectT)

TObjectRelHand =[];
gVector = [];

%Create the problem
%Check if problem already created?
umano_id = orEnvCreateProblem('HandProblem', '',1);

robots = orEnvGetRobots();
if size(robots,1) > 0    
	if (strcmp(robots{1}.name, 'HumanHand'))
		orBodyDestroy(robots{1}.id)
	end
end

%Draw the hand
% Hand size. Parameters in meters
HB=(HB)/1000;
HL=(HL)/1000;
s = orProblemSendCommand(['sethandparameters ' sprintf('%f ',HB) ' ' sprintf('%f ',HL)],umano_id);

if( exist('wristAxes','var') )    
	hand_id = orEnvGetBody('HumanHand');    
	%set correct rotation
	Trans = [0 0 0];
	Thand = [transpose(wristAxes) transpose(Trans)];
	orBodySetTransform(hand_id, Thand);

	%read wrist position
	wristT = GetArmHandTrans('wrist_wrist_Virtual',umano_id);
	Trans = -transpose(wristT(1:3,4))+wristPoint;
	Thand = [transpose(wristAxes) transpose(Trans)];
	orBodySetTransform(hand_id, Thand);
end 

if( exist('objectT','var') )
	%Transform object
	idLinkObjeto = orEnvGetBody(objectName);
	orBodySetTransform(idLinkObjeto, objectT);

	%Calcular la transformacion del objeto relativo a la mano
	Tobj = reshape(orBodyGetTransform(idLinkObjeto), [3 4]);
	Thand = reshape(orBodyGetTransform(hand_id), [3 4]);
	Tobj(4,:) = [ 0.0 0.0 0.0 1.0];
	Thand(4,:) = [ 0.0 0.0 0.0 1.0]; 
    TObjectRelHand = inv(Thand)*Tobj; %Transform to Ximo

	%Transladar por si no converge
	%distancia = 0.01
	%cdg = transpose(TObjectRelHand(1:3,4));
	%yGlobal = [1 0 0];
	%yVector = transpose(TObjectRelHand(1:3,2))
	%TObjectRelHand(1:3,4)=transpose(cdg+distancia*yVector)
    %orEnvPlot([cdg; cdg+distancia*yVector],'color',[0 1 0],'size',3,'line');
	%orEnvPlot([cdg],'color',[1 0 0],'size',3);

	%Calcular el vector de gravedad en relativas
	gGlobal = [0 0 -1];
	gVector = transpose(inv(Thand(1:3,1:3))* transpose(gGlobal)); %g to Ximo
    cdg = transpose(TObjectRelHand(1:3,4));
    orEnvPlot([cdg; cdg+0.05*gVector],'color',[1 0 0],'size',3,'line');
    
	%Mostrar mano y objeto en relativas
    %Thand = eye(4)
    %orBodySetTransform(hand_id, Thand(1:3,1:4));
    %orBodySetTransform(idLinkObjeto, TObjectRelHand(1:3,1:4));
end

arm_postureStr = mat2str(reshape(transpose(arm_posture),1,numel(arm_posture)));

if showPosture0==1
	postureNew = reshape(transpose(posture_0),1,30);
	postureStr = mat2str(postureNew);
	s = orProblemSendCommand(['setposture ' postureStr(2:end-1) ' ' arm_postureStr(2:end-1)],umano_id);
	disp('Approach Posture. Press a key to continue');
	pause();
end

if showPosture1==1
	postureNew = reshape(transpose(posture_1),1,30);
	postureStr = mat2str(postureNew);
	s = orProblemSendCommand(['setposture ' postureStr(2:end-1) ' ' arm_postureStr(2:end-1)],umano_id);
	%disp('Grasp Posture. Press a key to continue');
	%pause();
end

%centro del objeto botella
idLinkObjeto = orEnvGetBody(objectName);


