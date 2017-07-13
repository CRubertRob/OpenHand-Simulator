function [fingersOUT, matrix_normals_contactPointsOUT] = checkCollisionODE ( matrix_normals_contactPointsIN, robot, fingersIN, umano_id)
 
% linkid de los dedos CON brazo
thumbDistal = str2num(orProblemSendCommand(['getlinkindex Thumb_Distal'],umano_id));
indexDistal = str2num(orProblemSendCommand(['getlinkindex Index_Distal'],umano_id));
middleDistal = str2num(orProblemSendCommand(['getlinkindex Middle_Distal'],umano_id));
ringDistal = str2num(orProblemSendCommand(['getlinkindex Ring_Distal'],umano_id));
littleDistal = str2num(orProblemSendCommand(['getlinkindex Small_Distal'],umano_id));

matrix_normals_contactPointsOUT = matrix_normals_contactPointsIN;
fingersOUT = fingersIN;

flag = length(find(fingersIN == 0));
flag_thumb=fingersIN(1);
flag_index=fingersIN(2);
flag_medial=fingersIN(3);
flag_ring=fingersIN(4);
flag_little=fingersIN(5);

if flag<5    
	if flag_thumb==1
		fingerID = thumbDistal;
		fingerNum = 1;
		[matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal (fingerID, fingerNum, robot, matrix_normals_contactPointsOUT, fingersOUT);
    end
    if flag_index==1
        fingerID = indexDistal;
        fingerNum = 2;
		[matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal (fingerID, fingerNum, robot, matrix_normals_contactPointsOUT, fingersOUT);
    end
	if flag_medial==1
		fingerID = middleDistal;
		fingerNum = 3;
		[matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal (fingerID, fingerNum, robot, matrix_normals_contactPointsOUT, fingersOUT);
    end
	if flag_ring==1
		fingerID = ringDistal;
		fingerNum = 4;
		[matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal (fingerID, fingerNum, robot, matrix_normals_contactPointsOUT, fingersOUT);
    end    
	if flag_little==1
		fingerID = littleDistal;
		fingerNum = 5;
		[matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal (fingerID, fingerNum, robot, matrix_normals_contactPointsOUT, fingersOUT);
	end   
end    

%---------------------------------------------------------------------------------
function [matrix_normals_contactPointsOUT, fingersOUT] = getPointNormal(fingerID, fingerNum, robot,matrix_normals_contactPointsOUT, fingersOUT)

[collision,colbodyid,contacts]=orEnvCheckCollision(robot.id,[],1,fingerID);

% minium_contact = 0.003;
minimum_contact = 0.0015; %Work best with ode
minimum_contact = 0.001; %Work best with ode and new hand mesh
friccion = 0.4; %Only to draw the cones
maxDepth = 0.0;
%handles = [];
if collision == 1 
    %Draw contacts with lines 
    %handle = orEnvPlot(transpose(contacts(1:3,:)),'color',[0 1 0],'size',3);
    %handles = [handles,handle];
    %num_contacts = size(contacts(depth,:),2)
    %for ncontacts=1:size(contacts(depth,:),2)
    %    point = transpose(contacts(1:3,ncontacts));
    %    normal = transpose(contacts(4:6,ncontacts)*-1.0);               
    %    handle = orEnvPlot([point; point+contacts(7,ncontacts)*normal],'color',[0 0 0],'size',2,'line');
    %    handles = [handles,handle];
	%end

    [maxDepth,maxDepth_i] = max(abs(contacts(7,:)));
	point = transpose(contacts(1:3,maxDepth_i));
	% La funcion orEnvCheckCollision de matlab me devuelve las normales hacia
	% fuera del punto de contacto y para nuestros experimentos nos interesa que
	% esten cambiadas de signo. Por eso se lo cambiamos.
    normal = transpose(contacts(4:6,maxDepth_i)*-1.0);
            
    %Draw contact with maxDepth
    %handle = orEnvPlot(transpose(contacts(1:3,maxDepth_i)),'color',[1 0 0],'size',3);
    %handles = [handles,handle];
	%handle = orEnvPlot([point; point-maxDepth*normal],'color',[1 0 0],'size',2,'line');	
	%handles = [handles,handle];
	%orEnvClose(handles);
            
	if maxDepth > minimum_contact
		%maxDepth
        matrix_normals_contactPointsOUT.normal{fingerNum} = normal;
        matrix_normals_contactPointsOUT.point{fingerNum} = point;
        DrawContacts(transpose([matrix_normals_contactPointsOUT.point{fingerNum} -matrix_normals_contactPointsOUT.normal{fingerNum}]),friccion);
        fingersOUT(fingerNum)=0;    
    end
    
end           
              
