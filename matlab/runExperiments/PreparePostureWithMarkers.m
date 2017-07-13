function [wristAxes, wristPoint, objectT, handles] = PreparePostureWithMarkers(p, drawHandMarkers, drawObjectMarkers, drawWrist, wristOffsetFactor)

handles = [];

if( ~exist('drawHandMarkers','var') )
	drawHandMarkers = 1;
end

if( ~exist('drawObjectMarkers','var') )
	drawObjectMarkers = 1;
end

if( ~exist('drawWrist','var') )
	drawWrist = 1;
end

if( ~exist('wristOffsetFactor','var') )
	wristOffsetFactor = 0.5;
end

%Load posture
wrist2 = [p(11:13)]/1000;
wrist3 = [p(14:16)]/1000;
wrist1 = [p(17:19)]/1000;
wrist4 = [p(20:22)]/1000;
cylinder_markers = [p(2:10)]/1000;
finger_markers= [p(23:end)]/1000;
   
%Wrist position
wristOffset = (-p(12)+p(39))/1000;
wristOffset = wristOffset*wristOffsetFactor;

Y_muneca=(wrist1-wrist2)/norm((wrist1-wrist2));
Z_muneca_ini=(wrist3-wrist4)/norm((wrist3-wrist4));
X_muneca=cross(Y_muneca,Z_muneca_ini); 
X_muneca=X_muneca/norm(X_muneca);
Z_muneca=cross(X_muneca,Y_muneca);
wristAxes=[X_muneca;Y_muneca;Z_muneca];

rot_angle = acos(dot(Z_muneca_ini,Z_muneca))*180/pi;
wristPoint = (wrist4+wrist3)/2;
wristPoint = wristPoint+wristOffset*Y_muneca;

% Object Transformation
% http://en.wikipedia.org/wiki/Line-plane_intersection
po = (cylinder_markers(1:3)+cylinder_markers(4:6))/2;
lo = cylinder_markers(7:9);
n = cylinder_markers(4:6) - cylinder_markers(1:3);
l = n;
d = dot((po - lo),n)/dot(l,n);
p = d*l + lo;
objectT = [RotateY(n) transpose(p)];

%Draw wrist
if drawWrist == 1
	handle = orEnvPlot([wrist2;wrist3;wrist1;wrist4],'color',[0 0 1;0 1 0;1 0 0;0 1 1],'size',5);
	handles = [handles,handle];
	handle = orEnvPlot(wristPoint,'color',[1 0 0],'size',10);
	handles = [handles,handle];
end

%Draw Object Markers
if drawObjectMarkers == 1
	handle = orEnvPlot([cylinder_markers(1:3);cylinder_markers(4:6);cylinder_markers(7:9)],'color',[1 0 0;0 1 0;0 0 1],'size',3);
	handles = [handles,handle];
end

%Draw Hand Markers
if drawHandMarkers == 1
	finger_markers = reshape(finger_markers,3,size(finger_markers,2)/3);
	finger_markers = transpose(finger_markers);
	handle = orEnvPlot(finger_markers,'color',[0 0 0],'size',5);
	handles = [handles, handle];
end