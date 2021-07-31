clc; clear;


%% 이미지 받아오기
%binaryMap = imread('GVD.bmp');
binaryMap = rgb2gray(imread('map_outline.jpg'));
binaryMap.size()
%% 바이너리 필터링
binaryMap(binaryMap<230)=0;
binaryMap(binaryMap>=230)=255;
binaryMap(binaryMap ~=134) = 255;
save('map.bmp',binaryMap)

%%  테두리 만들기
binaryMap(1,1:size(binaryMap,2)) = 0;
binaryMap(size(binaryMap,1),1:size(binaryMap,2)) = 0;
binaryMap(1:size(binaryMap,1),1) = 0;
binaryMap(1:size(binaryMap,1),size(binaryMap,2)) = 0;

%%  distMap만들기
coverageMap = binaryMap;
coverageMap(coverageMap<230)=1;
coverageMap(coverageMap>=230)=0;
% state-of-art node Approach for cost 
[Dist_Map,IDX] = bwdist(coverageMap);
Dist_Map(Dist_Map==0)= 0.000001;


%% parameter설정
%커버리지 맵 복구
coverageMap = binaryMap;

%Parameter 설정
[Map_Height,Map_Width] = size(binaryMap);

%agent imformation
dt = 1;
vel_resolution = 1;

Predict_time = 3.0;
% 장애물 Gain값
Fov =50;
K_node = Fov;
K_obs  = K_node;
max_vel = 14;
min_vel = -14;
heading_cost = 0.001;
F_x =0;
F_y = 0;

coverArea = 0;
for i = 1:Map_Height
    for j = 1:Map_Width
        if(coverageMap(i,j) == 255)
            coverArea = coverArea +1;
        end
    end
end
cost = [];

%% main Loop

% %calc Potential Field 
% for i = 1:Map_Height
%     for j = 1:Map_Width
%         Dist_Map(i,j) = K_obs *1/Dist_Map(i,j)^3;
%     end
% end
N = coverArea/(Fov^2*pi);
disp(N)
agent = [];

while(1)
    x_pos = (1+(Map_Width-1)*rand(1,1));
    y_pos = (1+(Map_Height-1)*rand(1,1));
    
    if(coverageMap(int32(y_pos),int32(x_pos)) ~= 255)
       continue;
    end
    
    for i= 1:size(agent,1)
        coverageMap(agent(i,2),agent(i,1))=20;
    end
    
    agent = [ agent ; int32(x_pos) int32(y_pos)];
    
    if( N < size(agent,1))
        break;
    end
end

% x_pos = 0;
% y_pos = 0;
%윗 라인 부터 원소 생성
while(true)
    x_pos = (1+(Map_Width-1)*rand(1,1));
    y_pos = (1+(Map_Height-1)*rand(1,1));

%     x_pos = x_pos + 1;
%     y_pos = y_pos + 1;
    
    if(coverageMap(int32(y_pos),int32(x_pos)) ~= 255)
       continue;
    end
    
     coverageMap = binaryMap;
     
    agent = [ agent ; int32(x_pos) int32(y_pos)];
    
    % 노드 집합들 (Cell형)
    NodeCell ={};
    ObsCell ={};
    
    %가해지는 힘(Accelation)
    Fx = [];
    Fy = [];
    
    %dynamic Window Velocity Candidate
%     v_min_x = [];
%     v_max_x = [];
%     v_min_y = [];
%     v_max_y = [];
    
    
    %할당된 Agent들을 표시
    for i= 1:size(agent,1)
        coverageMap(agent(i,2),agent(i,1))=20;
    end
    
%     %각 Agent들에 대한 계산 ㄱㄱ
%     parfor i= 1:size(agent,1)
%         [obs,node]= search_neighbor(agent(i,:),coverageMap,Fov,Map_Height,Map_Width);
%         [NodeCell{1,i},~,~] = unique(node,'rows');
%         [ObsCell{1,i},~,~] = unique(obs,'rows');
%     end
% 
%     %Calc Potential
%     parfor i= 1:size(agent,1)
%         [Fx(i) ,Fy(i)] = agent_potential(agent(i,:),K_obs,K_node,NodeCell{1,i},ObsCell{1,i},Fov)
%         %calc Dynamoc window
%         v_min_x(i) =max(min_vel,min_vel+Fx(i)*dt);
%         v_max_x(i) =min(max_vel,max_vel-Fx(i)*dt);
%         v_min_y(i) = max(min_vel,min_vel+Fy(i)*dt);
%         v_max_y(i) = min(max_vel,max_vel-Fy(i)*dt);
%     end
    %agent Moving
    min_iter = 1;
    for sequence=1:min_iter
        static = 0;
        parfor i= 1:size(agent,1)
            [agent(i,:),info,cost] = select_bestCandiadidate(agent(i,:),K_obs,K_node,min_vel,max_vel,min_vel,max_vel,vel_resolution,coverageMap,Fov,Map_Height,Map_Width,Dist_Map);
            static = static + info;
        end
        if(static == size(agent,1)-1)
            break;
        end
    end
    
    for i= 1:size(agent,1)
        coverageMap = sol_coloring(agent(i,:),coverageMap,Fov,Map_Height,Map_Width);
    end
    
    coveredArea = 0;
    parfor i = 1:Map_Height
        for k = 1:Map_Width
            if(coverageMap(i,k) == 150)
                coveredArea = coveredArea +1;
            end
        end
    end
    
    coverRate = coveredArea/coverArea * 100;
    disp(coverRate);
    if(coverRate > 99.8)
        break;
    end
    
    imshow(coverageMap);
    pause(0.001);
end

coverageMap = binaryMap;
for i= 1:size(agent,1)
    coverageMap = sol_coloring(agent(i,:),coverageMap,Fov,Map_Height,Map_Width);
end
soultion_size = 4;
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
imshow(coverageMap);