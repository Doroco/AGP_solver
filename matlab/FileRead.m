clc;
clear;
 A = importdata('240_Node.txt');
 node_set = [];
 
for i=1:size(A,1)
    node_set(i,:) = str2num( A{i,1});
end

save('potential_240.mat')