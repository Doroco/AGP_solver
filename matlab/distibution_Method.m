clc; clear;


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

%%  distMap만들기
coverageMap = binaryMap;
coverageMap(coverageMap<230)=1;
coverageMap(coverageMap>=230)=0;
% state-of-art node Approach for cost 
[Dist_Map,IDX] = bwdist(coverageMap);
Dist_Map(Dist_Map==0)= 0.000001;


%% parameter설정

safe_dist = 1;
agent = [];

%start Position
agent(1) = 20;
agent(2) = 20;

%커버리지 맵 복구
coverageMap = binaryMap;
safeMap = coverageMap;

%Parameter 설정
[Map_Height,Map_Width] = size(binaryMap);

%agent imformation

move_resolution = 1;
% 장애물 Gain값
Fov =240;

coverArea = 0;
for i = 1:Map_Height
    for j = 1:Map_Width
        if(coverageMap(i,j) == 255)
            coverArea = coverArea +1;
        end
        if(Dist_Map(i,j) <= safe_dist && safeMap(i,j) == 255)
            safeMap(i,j) = 75;
        end
    end
end



coverageMap = binaryMap;
n= 1;

%% main Loop

while(true)
    coverageMap = sol_coloring(agent(n,:),coverageMap,Fov,Map_Height,Map_Width);

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
                    coverageMap(uint16(ny),uint16(nx)) = uint16(30);
               end
            end
        end
    end
    imshow(coverageMap);

    n = n + 1;
    
%     if(n == 20)
%         a=3;
%     end

    agent(n,:) = Cost_Evaluation(agent(n-1,:),Fov,move_resolution,coverageMap,Map_Height,Map_Width,Dist_Map);
    while(1)
        maxSize = 0;
        R_Width = 1;
        R_Height = 1;
        row = [];
        col = [];
        BWMap =coverageMap;
        BWMap(BWMap==255)=255;
        BWMap(BWMap~=255)=0;
        L = bwlabel(BWMap,4);
        C = unique(L);
        if(C == 0)
            break;
        end
        if(max(C) >= 1)
            for i=1:max(C)
                [r, c] = find(L==i);
                if(size(r,1) > maxSize)
                    maxSize = size(r,1);
                    R_Width = size(r,1);
                    R_Height = size(c,1);
                    row = r;
                    col = c;
                end
            end
        end
        r_pos= [];
        r_pos(2) = row(int32(1+(R_Width-1)*rand(1,1)));
        r_pos(1) = col(int32(1+(R_Height-1)*rand(1,1)));
        
        if(coverageMap(r_pos(2),r_pos(1)) ~= 255)
           continue;
        end
   
        
        Mindist = 1e6;
        for i= 1:size(agent,1)-1
            dist = sqrt( (agent(n,1)-agent(i,1))^2 +(agent(n,2)-agent(i,2))^2 );
            if(dist < Mindist)
                Mindist = dist;
            end
        end

        if(Mindist > Fov/4)
            break;
        else
            agent(n,:) = Cost_Evaluation(r_pos,Fov,move_resolution,coverageMap,Map_Height,Map_Width,Dist_Map);
        end
    end
    
    coverRate = ( (nnz(coverageMap==150)+nnz(coverageMap==30) )/coverArea)*100;
    disp(coverRate);
    if(coverRate> 99.9)
        break;
    end
    
    
end

coverageMap = binaryMap;
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
imshow(coverageMap);