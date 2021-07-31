clc;
clear;

load('potential_172.mat')
agent(:,1) = node_set(:,2);
agent(:,2) = node_set(:,1);
Fov =172;
%% 이미지 받아오기
%binaryMap = imread('GVD.bmp');
binaryMap = rgb2gray(imread('map_outline.jpg'));
%% 바이너리 필터링
binaryMap(binaryMap<230)=0;
binaryMap(binaryMap>=230)=255;

%%  테두리 만들기
binaryMap(1,1:size(binaryMap,2)) = 0;
binaryMap(size(binaryMap,1),1:size(binaryMap,2)) = 0;
binaryMap(1:size(binaryMap,1),1) = 0;
binaryMap(1:size(binaryMap,1),size(binaryMap,2)) = 0;

[Map_Height,Map_Width] = size(binaryMap);
%%  distMap만들기
coverageMap = binaryMap;
coverageMap(coverageMap<230)=1;
coverageMap(coverageMap>=230)=0;
% state-of-art node Approach for cost 
[Dist_Map,IDX] = bwdist(coverageMap);
Dist_Map(Dist_Map==0)= 0.000001;


%% main loop (plot)

coverageMap = binaryMap;
[Map_Height,Map_Width] = size(binaryMap);

for i= 1:size(agent,1)
    coverageMap = sol_coloring(agent(i,:),coverageMap,Fov,Map_Height,Map_Width);
end
soultion_size = 10;
for n=1:size(agent,1)
    for nx = agent(n,1)-soultion_size:1:agent(n,1)+soultion_size
        for ny = agent(n,2)-soultion_size:1:agent(n,2)+soultion_size
          if(nx > Map_Width)
               continue;
           end
           if(ny > Map_Height)
               continue;
           end
           if(int32(ny)<= 0 || int32(nx)<= 0)
               continue;
           end
           if(coverageMap(int32(ny),int32(nx)) == 0)
               continue;
           end
           if(uint16(soultion_size^2) < uint16(nx^2)+uint16(ny^2))
                coverageMap(uint16(ny),uint16(nx)) = uint16(20);
           end
        end
    end
end
%%
imshow(coverageMap);
%%

coverageMap(coverageMap==20)=1;
coverageMap(coverageMap==0)=1;
coverageMap(coverageMap ~= 1)=0;

[Dist_Map,IDX] = bwdist(coverageMap);
Dist_Map(Dist_Map < 1) =30;
for i= 1: Map_Height
    for j=1:Map_Width
        Dist_Map(i,j) = Dist_Map(i,j);
    end
end

%%
[X,Y] = meshgrid(1:30:Map_Width,1:30:Map_Height);
Z = Dist_Map(1:30:Map_Height,1:30:Map_Width); 
surf(X,Y,Z,'FaceAlpha',0.5);