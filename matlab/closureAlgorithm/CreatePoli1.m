function [Poli1] = CreatePoli1( s )
%CREATEVECTOR Summary of this function goes here
%   Detailed explanation goes here

parameters = findstr(s, '||');     
for pos = 1:length(parameters)-1
    aux{pos} = str2double(s(parameters(pos)+2:parameters(pos+1)-1));
end
% Dedo
Poli1.r{1}=aux{1};
Poli1.c{1}=[aux{2};aux{3};aux{4};];       
Poli1.r{2}=aux{6};
Poli1.c{2}=[aux{7};aux{8};aux{9};];              
Poli1.N=2;
