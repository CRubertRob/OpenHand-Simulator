function [ TLink ] = GetArmHandTrans( name,umano_id )
hand_id = orEnvGetBody('HumanHand');
link_id = str2num(orProblemSendCommand(['getlinkindex ' name],umano_id));
T = orBodyGetLinks(hand_id);
TLink = T(:,link_id+1);
TLink = reshape(TLink, [3 4]);
%orEnvPlot(transpose(TLink(1:3,4)),'color',[1 0 0],'size',7);%D

