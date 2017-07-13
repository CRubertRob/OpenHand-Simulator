function [posture_1] = GetRandomPosture(umano_id, posture_1)

randomValues = rand(5,6);

posture(1,:)=[	1.0000	1.0000	1.0000	1.0000	1.0000	0.0000	];
posture(2,:)=[	0.0000	0.0000	1.0000	1.0000	1.0000	1.0000	];
posture(3,:)=[	0.0000	0.0000	1.0000	1.0000	1.0000	1.0000	];
posture(4,:)=[	1.0000	0.0000	1.0000	1.0000	1.0000	1.0000	];
posture(5,:)=[	1.0000	0.0000	1.0000	1.0000	1.0000	1.0000	];

% Random posture with joint limits

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

	for fila=1:5
		for col=1:6
			lowerLimit = limites(fila,col+(col-1));
			upperLimit = limites(fila,col+col);
			range = (upperLimit-lowerLimit)*5/100;
			lower = range * -1;
			upper = range ;
			randomPosture(fila,col) = (upper - lower)*randomValues(fila,col)+lower;
		end
	end	


%range = 6;
%lower = range * -1;
%upper = range ;
%randomPosture = (upper - lower)*randomValues+lower;

randomPosture = posture.*randomPosture;
randomPosture =randomPosture*pi/180;

posture_1 = posture_1+randomPosture;
posture_1*180/pi;



