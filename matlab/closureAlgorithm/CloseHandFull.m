function [posture] = CloseHandFull(i, posture, umano_id, joint_movements, arm_postureStr, posture_inicial )
% i       -> Grado que se va incrementando el movimiento de cada dedo de la mano
% posture -> Postura actual de la mano que se va a modificar dependiendo de
% los dedos que haya que mover (solo qhabra que mover los que me lleguen con valor 1)
% fingers -> vector de 5 enteros [thumb index medial ring little], cuando
% el valor viene a 1 significa que se puede mover
%umano_id-> Id del Handproblem 
%joint_movements -> Es una matriz de 5x3. Cada una de las columnas se
%corresponde con una joint de un dedo. Si el valor de la columna es 1,
%significa que lo piedo seguir aumentando. El cierre funciona del siguiente
%modo: Cuando la falange distal no se puede mover, se considera que ninguna
%de los otros segmentos sigue en movimiento, si el segmento medio ya no
%puede moverse, el proximal tampoco podra, pero si que seguira su
%movimiento el distal, si el proximal no puede moverse el resto de los
%dedos si que podra.


% Posture of the hand 
% [flex CMC, abd CMC, flex MCP, abd MCP, flex PIP, flex DIP]. Angles in radians
% if fingers(1)==1
%     posture(1,:)=posture(1,:)+[i -i i 0 i 0]*pi/180; % Thumb. Posture(1,6) has no use
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

%[CMC, MCP, PIP, DIP]
%thumb
if joint_movements(1,3) == 1
    posture(1,5) = posture_inicial(1,5) + (i)*pi/180; 
    if joint_movements(1,2) == 1
        posture(1,3) = posture_inicial(1,3) + (i)*pi/180;               
        if joint_movements(1,1) == 1
            %Sobre la misma joint se produce una flexion y una abduccion a
            %la vez
           posture(1,2) = posture_inicial(1,2) + (-i)*pi/180;
           posture(1,1) = posture_inicial(1,1) + (i)*pi/180;
        end
    end  
end

% index y medial
for f=[2 3]
    if joint_movements(f,3) == 1
        posture(f,6) = posture_inicial(f,6) + (i)*pi/180; 
        if joint_movements(f,2) == 1
            posture(f,5) = posture_inicial(f,5) + (i)*pi/180;               
            if joint_movements(f,1) == 1
               posture(f,3) = posture_inicial(f,3) + (i)*pi/180;
            end
        end  
    end
end   

%ring
if joint_movements(4,3) == 1
    posture(4,6) = posture_inicial(4,6) + (i)*pi/180; 
    if joint_movements(4,2) == 1
        posture(4,5) = posture_inicial(4,5) + (i)*pi/180;               
        if joint_movements(4,1) == 1
           posture(4,3) = posture_inicial(4,3) + (i)*pi/180;
        end
    end  
end

%small
if joint_movements(5,3) == 1
    posture(5,6) = posture_inicial(5,6) + (i)*pi/180; 
    if joint_movements(5,2) == 1
        posture(5,5) = posture_inicial(5,5) + (i)*pi/180;               
        if joint_movements(5,1) == 1
           posture(5,3) = posture_inicial(5,3) + (i)*pi/180;
        end
    end  
end

postureNew = reshape(transpose(posture),1,30);
postureStr = mat2str(postureNew);
s = orProblemSendCommand(['setposture ' postureStr(2:end-1) ' ' arm_postureStr(2:end-1)],umano_id);
