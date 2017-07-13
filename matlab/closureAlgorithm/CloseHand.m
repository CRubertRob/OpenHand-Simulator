function [posture] = CloseHand(i, posture, fingers, umano_id, arm_postureStr, posture_inicial )
% i       -> Grado que se va incrementando el movimiento de cada dedo de la mano
% posture -> Postura actual de la mano que se va a modificar dependiendo de
% los dedos que haya que mover (solo qhabra que mover los que me lleguen con valor 1)
% fingers -> vector de 5 enteros [thumb index medial ring little], cuando
% el valor viene a 1 significa que se puede mover
%umano_id-> Id del Handproblem 


% Posture of the hand 
% [flex CMC, abd CMC, flex MCP, abd MCP, flex PIP, flex DIP]. Angles in radians
% if fingers(1)==1
%     posture(1,:)=[i 50-i 5+i 0 5+i 0]*pi/180; % Thumb. Posture(1,6) has no use
% end
% if fingers(2)==1
%     posture(2,:)=[0 0 20+i 0 20+i 10+i]*pi/180; % Index. Posture(2,1:2) have no use
% end
% if fingers(3)==1
%     posture(3,:)=[0 0 20+i 0 20+i 10+i]*pi/180; % Medial. Posture(3,1:2) have no use
% end
% if fingers(4)==1
%     posture(4,:)=[4 0 15+i 0 20+i 10+i]*pi/180; % Ring. Posture(4,2) has no use
% end
% if fingers(5)==1
%     posture(5,:)=[8 0 10+i 0 10+i 10+i]*pi/180; % Litle. Posture(5,2) has no use
% end

if fingers(1)==1
    posture(1,:) = posture_inicial(1,:) + [i -i i 0 i 0]*pi/180; % Thumb. Posture(1,6) has no use
end
if fingers(2)==1
    posture(2,:) = posture_inicial(2,:) + [0 0 i 0 i i]*pi/180; % Index. Posture(2,1:2) have no use
end
if fingers(3)==1
    posture(3,:) = posture_inicial(3,:) + [0 0 i 0 i i]*pi/180; % Medial. Posture(3,1:2) have no use
end
if fingers(4)==1
    posture(4,:) = posture_inicial(4,:) + [0 0 i 0 i i]*pi/180; % Ring. Posture(4,2) has no use
end
if fingers(5)==1
    posture(5,:) = posture_inicial(5,:) + [0 0 i 0 i i]*pi/180; % Litle. Posture(5,2) has no use
end

postureNew = reshape(transpose(posture),1,30);
postureStr = mat2str(postureNew);
s = orProblemSendCommand(['setposture ' postureStr(2:end-1) ' ' arm_postureStr(2:end-1)],umano_id);



