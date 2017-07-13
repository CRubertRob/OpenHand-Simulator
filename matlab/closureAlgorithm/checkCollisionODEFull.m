function [joint_movements, fingersOUT, matrix_normals_contactPointsOUT] = checkCollisionODEFull ( robot, joint_movements, fingersIN, matrix_normals_contactPointsIN)

% fingers -> vector de 5 enteros con los dedos en los que ha habido colision [thumb index middle ring small]
% si se pone a 0 es porque ha habido una colision
% matrix_normals_contactPoints -> Es una matriz de 5x2 en la que se
% almacenan los siguientes valores:
    % normal_thumb   point_thumb
    % normal_index   point_index
    % normal_middle  point_middle
    % normal_ring    point_ring
    % normal_small   point_small
    
% %linkid de los dedos sin brazo
% thumbDistal = 5;
% indexDistal = 10;
% middleDistal = 15;
% ringDistal = 20;
% littleDistal = 25;

% %linkid de los dedos CON brazo
thumbDistal = 12;
thumbProximal = 10;
thumbMetacarpal = 8;
indexDistal = 17;
indexMedial = 16;
indexProximal = 14;
middleDistal = 22;
middleMedial = 21;
middleProximal = 19;
ringDistal = 27;
ringMedial = 26;
ringProximal = 24;
littleDistal = 32;
littleMedial = 31;
littleProximal = 29;

% [collision,colbodyid,contacts]=orEnvCheckCollision(2,[],1,15)
% collision = 1 --> Existe colision, collision = 0 --> No hay colision
% colbodyid -> id del cuerpo con el que colisiona
% Filas de la matriz contacts
% x
% y
% z
% normalX
% normalY
% normalZ
% profundidad
x = 1;
y = 2;
z = 3;
normalX = 4;
normalY = 5;
normalZ = 6;
depth = 7;

matrix_normals_contactPointsOUT = matrix_normals_contactPointsIN;
fingersOUT = fingersIN;

friccion = 0.4;

minium_contact = 0.003;

flag=length(find(fingersIN == 0));
flag_thumb=fingersIN(1);
flag_index=fingersIN(2);
flag_medial=fingersIN(3);
flag_ring=fingersIN(4);
flag_little=fingersIN(5);

thumb_distal_flex = joint_movements(1,3);
thumb_proximal_flex = joint_movements(1,2);
thumb_metacarpal_flex_abd = joint_movements(1,1);

index_distal_flex = joint_movements(2,3);
index_medial_flex = joint_movements(2,2);
index_proximal_flex = joint_movements(2,1);

middle_distal_flex = joint_movements(3,3);
middle_medial_flex = joint_movements(3,2);
middle_proximal_flex = joint_movements(3,1);

ring_distal_flex = joint_movements(4,3);
ring_medial_flex = joint_movements(4,2);
ring_proximal_flex = joint_movements(4,1);

little_distal_flex = joint_movements(5,3);
little_medial_flex = joint_movements(5,2);
little_proximal_flex = joint_movements(5,1);

%
%
%
%para que saque todos los contactos hay que cambiar los indices
%matrix_normals_contactPointsOUT.normal{2} como antes
%
%
%
%

if flag < 5
    %Thumb
    if flag_thumb == 1
       if thumb_distal_flex == 1
            [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,thumbDistal);
            if collision == 1 %&& contacts(depth) > minium_contact
                joint_movements(1,3) = 0;
                joint_movements(1,2) = 0;
                joint_movements(1,1) = 0;
                fingersOUT(1)=0;
                matrix_normals_contactPointsOUT.normal{1} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                matrix_normals_contactPointsOUT.point{1} = [contacts(x) contacts(y) contacts(z)];
                DrawContacts(transpose([matrix_normals_contactPointsOUT.point{1} -matrix_normals_contactPointsOUT.normal{1}]),friccion);
            elseif thumb_proximal_flex == 1
                [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,thumbProximal);
                if collision == 1 %&& contacts(depth) < -minium_contact
                    joint_movements(1,2) = 0;
                    joint_movements(1,1) = 0;
                     %matrix_normals_contactPointsOUT.normal{2} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                     %matrix_normals_contactPointsOUT.point{2} = [contacts(x) contacts(y) contacts(z)];
                     %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{2} -matrix_normals_contactPointsOUT.normal{2}]),friccion);
                elseif thumb_metacarpal_flex_abd == 1
                    [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,thumbMetacarpal);
                    if collision == 1 %&& contacts(depth) < -minium_contact
                        joint_movements(1,1) = 0;
                         %matrix_normals_contactPointsOUT.normal{3} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                         %matrix_normals_contactPointsOUT.point{3} = [contacts(x) contacts(y) contacts(z)];
                         %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{3} -matrix_normals_contactPointsOUT.normal{3}]),friccion);
                    end
                end
            end
       end   
    end

    %Index
    if flag_index==1
        if index_distal_flex == 1
            [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,indexDistal);
            if collision == 1 %&& contacts(depth) < -minium_contact
                joint_movements(2,3) = 0;
                joint_movements(2,2) = 0;
                joint_movements(2,1) = 0;
                fingersOUT(2)=0;
                matrix_normals_contactPointsOUT.normal{2} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                matrix_normals_contactPointsOUT.point{2} = [contacts(x) contacts(y) contacts(z)];
                DrawContacts(transpose([matrix_normals_contactPointsOUT.point{2} -matrix_normals_contactPointsOUT.normal{2}]),friccion);
            elseif index_medial_flex == 1
                [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,indexMedial);
                if collision == 1 %&& contacts(depth) < -minium_contact
                    joint_movements(2,2) = 0;
                    joint_movements(2,1) = 0;
                     %matrix_normals_contactPointsOUT.normal{5} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                     %matrix_normals_contactPointsOUT.point{5} = [contacts(x) contacts(y) contacts(z)];
                     %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{5} -matrix_normals_contactPointsOUT.normal{5}]),friccion);
                elseif index_proximal_flex == 1
                    [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,indexProximal);
                    if collision == 1 %&& contacts(depth) < -minium_contact
                        joint_movements(2,1) = 0;
                         %matrix_normals_contactPointsOUT.normal{6} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                         %matrix_normals_contactPointsOUT.point{6} = [contacts(x) contacts(y) contacts(z)];
                         %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{6} -matrix_normals_contactPointsOUT.normal{6}]),friccion);
                    end
                end
            end
        end
    end

    %Middle
    if flag_medial==1
        if middle_distal_flex == 1
            [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,middleDistal);
            if collision == 1 %&& contacts(depth) < -minium_contact
                joint_movements(3,3) = 0;
                joint_movements(3,2) = 0;
                joint_movements(3,1) = 0;
                fingersOUT(3)=0;
                matrix_normals_contactPointsOUT.normal{3} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                matrix_normals_contactPointsOUT.point{3} = [contacts(x) contacts(y) contacts(z)];
                DrawContacts(transpose([matrix_normals_contactPointsOUT.point{3} -matrix_normals_contactPointsOUT.normal{3}]),friccion);
            elseif middle_medial_flex == 1
                [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,middleMedial);
                if collision == 1 %&& contacts(depth) < -minium_contact
                    joint_movements(3,1) = 0;
                    joint_movements(3,1) = 0; 
                     %matrix_normals_contactPointsOUT.normal{8} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                     %matrix_normals_contactPointsOUT.point{8} = [contacts(x) contacts(y) contacts(z)];
                     %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{8} -matrix_normals_contactPointsOUT.normal{8}]),friccion);
                elseif middle_proximal_flex == 1
                    [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,middleProximal);
                    if collision == 1 %&& contacts(depth) < -minium_contact
                        joint_movements(3,1) = 0; 
                         %matrix_normals_contactPointsOUT.normal{9} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                         %matrix_normals_contactPointsOUT.point{9} = [contacts(x) contacts(y) contacts(z)];
                         %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{9} -matrix_normals_contactPointsOUT.normal{9}]),friccion);
                    end                    
                end
            end
        end
    end

    %Ring
    if flag_ring==1
        if ring_distal_flex == 1
            [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,ringDistal);
            if collision == 1 %&& contacts(depth) < -minium_contact
                joint_movements(4,3) = 0;
                joint_movements(4,2) = 0;
                joint_movements(4,1) = 0;
                fingersOUT(4)=0;
                matrix_normals_contactPointsOUT.normal{4} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                matrix_normals_contactPointsOUT.point{4} = [contacts(x) contacts(y) contacts(z)];
                DrawContacts(transpose([matrix_normals_contactPointsOUT.point{4} -matrix_normals_contactPointsOUT.normal{4}]),friccion);
            elseif ring_medial_flex == 1
                [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,ringMedial);
                if collision == 1 %&& contacts(depth) < -minium_contact
                    joint_movements(4,2) = 0;
                    joint_movements(4,1) = 0; 
                     %matrix_normals_contactPointsOUT.normal{11} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                     %matrix_normals_contactPointsOUT.point{11} = [contacts(x) contacts(y) contacts(z)];
                     %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{11} -matrix_normals_contactPointsOUT.normal{11}]),friccion);
                elseif ring_proximal_flex == 1
                    [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,ringProximal);
                    if collision == 1 %&& contacts(depth) < -minium_contact
                        joint_movements(4,1) = 0; 
                         %matrix_normals_contactPointsOUT.normal{12} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                         %matrix_normals_contactPointsOUT.point{12} = [contacts(x) contacts(y) contacts(z)];
                         %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{12} -matrix_normals_contactPointsOUT.normal{12}]),friccion);
                    end
                end
            end
        end
    end

    %Little
    if flag_little==1
        if little_distal_flex == 1
            [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,littleDistal);
            if collision == 1 %&& contacts(depth) < -minium_contact
                joint_movements(5,3) = 0;
                joint_movements(5,2) = 0;
                joint_movements(5,1) = 0;
                fingersOUT(5)=0;
                matrix_normals_contactPointsOUT.normal{5} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                matrix_normals_contactPointsOUT.point{5} = [contacts(x) contacts(y) contacts(z)]; 
                DrawContacts(transpose([matrix_normals_contactPointsOUT.point{5} -matrix_normals_contactPointsOUT.normal{5}]),friccion);          
            elseif little_medial_flex == 1
                [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,littleMedial);
                if collision == 1 %&& contacts(depth) < -minium_contact
                    joint_movements(5,2) = 0;
                    joint_movements(5,1) = 0;
                     %matrix_normals_contactPointsOUT.normal{14} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                     %matrix_normals_contactPointsOUT.point{14} = [contacts(x) contacts(y) contacts(z)];
                     %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{14} -matrix_normals_contactPointsOUT.normal{14}]),friccion);
                elseif little_proximal_flex == 1
                    [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,littleProximal);
                    if collision == 1 %&& contacts(depth) < -minium_contact
                        joint_movements(5,1) = 0;
                         %matrix_normals_contactPointsOUT.normal{15} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
                         %matrix_normals_contactPointsOUT.point{15} = [contacts(x) contacts(y) contacts(z)];
                         %DrawContacts(transpose([matrix_normals_contactPointsOUT.point{15} -matrix_normals_contactPointsOUT.normal{15}]),friccion);
                    end
                end
            end
        end
    end
end

% 
% if flag<5
%     
%     %bodyid = orEnvGetBody('HumanHand')     
%     
%     if flag_thumb==1
%         [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,thumbDistal);
%         if collision == 1 && contacts(depth) > minium_contact
%             matrix_normals_contactPointsOUT.normal{1} = [-contacts(normalX) -contacts(normalY) -contacts(normalZ)];
%             matrix_normals_contactPointsOUT.point{1} = [contacts(x) contacts(y) contacts(z)];
%             fingersOUT(1)=0;
%         end
%     end
%     if flag_index==1
%         [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,indexDistal);
%         if collision == 1 && contacts(depth) < -minium_contact
%             matrix_normals_contactPointsOUT.normal{2} = [contacts(normalX) contacts(normalY) contacts(normalZ)];
%             matrix_normals_contactPointsOUT.point{2} = [contacts(x) contacts(y) contacts(z)];
%             fingersOUT(2)=0;
%         end
%     end
%     if flag_medial==1
%         [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,middleDistal);
%         if collision == 1 && contacts(depth) < -minium_contact          
%             matrix_normals_contactPointsOUT.normal{3} = [contacts(normalX) contacts(normalY) contacts(normalZ)];
%             matrix_normals_contactPointsOUT.point{3} = [contacts(x) contacts(y) contacts(z)];
%             fingersOUT(3)=0;
%         end
%     end
%     if flag_ring==1
%         [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,ringDistal);        
%         if collision == 1 && contacts(depth) < -minium_contact
%             matrix_normals_contactPointsOUT.normal{4} = [contacts(normalX) contacts(normalY) contacts(normalZ)];
%             matrix_normals_contactPointsOUT.point{4} = [contacts(x) contacts(y) contacts(z)];
%             fingersOUT(4)=0;
%         end
%     end    
%     if flag_little==1
%         [collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,littleDistal);
%         if collision == 1 && contacts(depth) < -minium_contact
%             matrix_normals_contactPointsOUT.normal{5} = [contacts(normalX) contacts(normalY) contacts(normalZ)];
%             matrix_normals_contactPointsOUT.point{5} = [contacts(x) contacts(y) contacts(z)];
%             fingersOUT(5)=0;
%         end
%     end   
% end    

