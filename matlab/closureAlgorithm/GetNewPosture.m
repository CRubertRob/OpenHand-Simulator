function [posture] = GetNewPosture(i, posture, fingers, posture_inicial, velocity )

% i       -> Grado que se va incrementando el movimiento de cada dedo de la mano
% posture -> Postura actual de la mano que se va a modificar dependiendo de
% los dedos que haya que mover (solo qhabra que mover los que me lleguen con valor 1)
% fingers -> vector de 5 enteros [thumb index medial ring little], cuando
% el valor viene a 1 significa que se puede mover

if fingers(1)==1
    posture(1,:) = posture_inicial(1,:) + velocity(1,:)*i; % Thumb. Posture(1,6) has no use
end
if fingers(2)==1
    posture(2,:) = posture_inicial(2,:) + velocity(2,:)*i; % Index. Posture(2,1:2) have no use
end
if fingers(3)==1
    posture(3,:) = posture_inicial(3,:) + velocity(3,:)*i; % Medial. Posture(3,1:2) have no use
end
if fingers(4)==1
    posture(4,:) = posture_inicial(4,:) + velocity(4,:)*i; % Ring. Posture(4,2) has no use
end
if fingers(5)==1
    posture(5,:) = posture_inicial(5,:) + velocity(5,:)*i; % Litle. Posture(5,2) has no use
end
