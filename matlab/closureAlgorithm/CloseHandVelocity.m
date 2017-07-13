function [posture, fingersOUT, matrix_normals_contactPointsOUT] = CloseHandVelocity(i, posture, fingers, umano_id, arm_postureStr, posture_inicial, velocity, fingersIN, limites_inf, limites_sup, matrix_normals_contactPointsIN )

% i       -> Grado que se va incrementando el movimiento de cada dedo de la mano
% posture -> Postura actual de la mano que se va a modificar dependiendo de
% los dedos que haya que mover (solo qhabra que mover los que me lleguen con valor 1)
% fingers -> vector de 5 enteros [thumb index medial ring little], cuando
% el valor viene a 1 significa que se puede mover
%umano_id-> Id del Handproblem 

fingersOUT = fingersIN;
posture_anterior = posture;
matrix_normals_contactPointsOUT = matrix_normals_contactPointsIN;

posture_inicial = posture;
posture = GetNewPosture(i, posture, fingers, posture_inicial, velocity );

for i=1:size(limites_sup,1)
   for j=1:size(limites_sup,2)
        if posture(i,j) < limites_inf(i,j) 
            posture(i,j) = limites_inf(i,j);
        elseif posture(i,j) > limites_sup(i,j)
            posture(i,j) = limites_sup(i,j);                        
        end
   end
end

% format
% actu = posture*180/pi
% superior = limites_inf*180/pi
% inferior = limites_sup*180/pi

anterior = posture_anterior;
actual = posture;
for dedo=1:size(fingers,2)
    if (fingers(dedo)==1)
        if (posture_anterior(dedo,:)==posture(dedo,:))
            matrix_normals_contactPointsOUT.normal{dedo} = [0 0 0];
            matrix_normals_contactPointsOUT.point{dedo} = [0 0 0];
            fingersOUT(dedo) = 0;    
        end
    end
end

postureNew = reshape(transpose(posture),1,30);
postureStr = mat2str(postureNew);
s = orProblemSendCommand(['setposture ' postureStr(2:end-1) ' ' arm_postureStr(2:end-1)],umano_id);
