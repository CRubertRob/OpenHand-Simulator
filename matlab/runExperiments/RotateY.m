function [ RotMat ] = RotateY( normal )

target_dir = normal/norm(normal);
y_axis = transpose([0 1 0]);
rot_angle = acos(dotprod(target_dir,y_axis));

if abs(rot_angle) > 1e-8
   if(cross(target_dir, y_axis) == [0 0 0])
    rot_axis = [0 0 1];
   else
    rot_axis = cross(target_dir, y_axis)/norm(cross(target_dir, y_axis));
    %rot_axis = abs(rot_axis);
   end
   RotMat = Rotatef(rot_angle, rot_axis(1), rot_axis(2), rot_axis(3));
else
   RotMat = [1 0 0;0 1 0;0 0 1];
end




   
%% FUNCION ROTATEF
% Con esta funcion conseguimos la matriz de rotacion para un vector
function RotMatrix = Rotatef(angle, x, y, z)

    vector = [x y z];
    vector = vector/norm(vector);

    axis.x = vector(1);
    axis.y = vector(2);
    axis.z = vector(3);
    
    b = angle;

    c = cos(b);
    ac = 1 - c;
    s = sin(b);
   
    RotVector(1) = axis.x * axis.x * ac + c;
    RotVector(2) = axis.x * axis.y * ac - axis.z * s;
    RotVector(3) = axis.x * axis.z * ac + axis.y * s;
    
    RotVector(4) = axis.y * axis.x * ac + axis.z * s;
    RotVector(5) = axis.y * axis.y * ac + c;
    RotVector(6) = axis.y * axis.z * ac - axis.x * s;
    
    RotVector(7) = axis.z * axis.x * ac - axis.y * s;
    RotVector(8) = axis.z * axis.y * ac + axis.x * s;
    RotVector(9) = axis.z * axis.z * ac + c;   
    RotMatrix = reshape(RotVector,3,3);
    
    %RotMatrix = transpose(RotMatrix);
    
    
