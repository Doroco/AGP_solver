clc; clear;

binaryMap = rgb2gray(imread('map_outline.jpg'));
binaryMap(binaryMap<230)=0;
binaryMap(binaryMap>=230)=255;

coverageMap = rgb2gray(imread('map_outline.jpg'));
coverageMap(coverageMap<230)=1;
coverageMap(coverageMap>=230)=0;

[Dist_Map,IDX] = bwdist(coverageMap);
Dist_Map(Dist_Map==0)= 0.000001;
coverageMap = binaryMap;

wall = edge(coverageMap,'canny');

%Initialize part
covrage_area = 0;
soultion_size = 15;
[Map_Height,Map_Width] = size(binaryMap);

for nx = 1:Map_Height
    for ny = 1:Map_Width
        if(binaryMap(nx,ny) == 255)
            covrage_area = covrage_area+1;
        end
    end
end

num= 0;
isEnd = false;
k=0.034;
k_obs = 0.165;
Fov = 240;

%화살표 크기

%전하의 Gain및 position 설정
while(isEnd ~= true)
    scale=5;
    ex=0; ey=0;
    Ex=0; Ey=0;
    v=0;
%     a = 0;
%     for nx = 1:Map_Height
%         for ny = 1:Map_Width
%             if(coverageMap(nx,ny) > uint32(150))
%                 x_pos = nx;
%                 y_pos = ny;
%                 a = 1;
%                 break;
%             end
%         end
%         if(a == 1)
%             break;
%         end
%     end
    x_pos = (1+(Map_Height-1)*rand(1,1));
    y_pos = (1+(Map_Width-1)*rand(1,1));
    if(coverageMap(uint16(x_pos),uint16(y_pos)) ~= 255)
        continue;
    end

    coverageMap = binaryMap;

    
    covraged_area = 0;
    num = num +1;
    q(num)= 10;
    mass(num) = 1;
    x(num)=x_pos; 
    y(num)=y_pos; 
    velocity_x(num) = 0;
    velocity_y(num) = 0;
    
    N = length(q);
    [xPos, yPos]=meshgrid(1:1:Map_Width,1:1:Map_Height);
    
    % -------------------------------------------------------- 전기장안에 힘 분석 끝
    iter = 0;
    
    
    while(true)
        v_totoal_x = 0;
        v_totoal_y = 0;
        for n=1:N
            %거리 r 구하기
            r=sqrt((x(n)-xPos).^2+(y(n)-yPos).^2);
            r_obs = Dist_Map(1:size(Dist_Map,1),1:size(Dist_Map,2));
            [dGx,dGy] = gradient(Dist_Map);

            % 쿨롱 Force 
            if(r > Fov) 
                ex = 0;
                ey = 0;
            else
                ex=-k*(q(n)./r.^2).*((x(n)-xPos)./r);
                ey=-k*(q(n)./r.^2).*((y(n)-yPos)./r);
            end

            % 벽에의한 Force 
            if(r_obs > Fov) 
                ex =ex+ 0;
                ey =ey+ 0;
            else
                ex=ex-k_obs*(q(n)./r_obs.^2).*(dGx./r_obs);
                ey=ey-k_obs*(q(n)./r_obs.^2).*(dGy./r_obs);
            end


            %Electric Field ---> Mat형태로 나와서 각 pos에 대해서 전기장을 계산 할 수 있게 된다.
            Ex=Ex+ex;
            Ey=Ey+ey;

            %전위 구하기  ---> Mat형태로 나오게 된다. 각 pos에 대해서 전위을 계산 할 수 있게 된다.
    %         v=v-ex.*(r.*((x(n)-xPos)./r));
    %         v=v-ey.*(r.*((y(n)-yPos)./r));



            %draw Circle line  %Degree
            for theta =0:0.05:360
                for radius = 0:1:Fov
                   pixel_x = x(n) - radius*sin(theta*(pi/180));
                   pixel_y = y(n) + radius*cos(theta*(pi/180));
                   if(int32(pixel_x)<= 0 || int32(pixel_y)<= 0)
                       continue;
                   end
                   if(int32(pixel_x) > Map_Height || int32(pixel_y) > Map_Width)
                       continue;
                   end
                   if(binaryMap(int32(pixel_x),int32(pixel_y)) == 0)
                       break;
                   end
                   coverageMap(int32(pixel_x),int32(pixel_y)) = uint16(150);
                end
            end
        end
        imshow(coverageMap);
        pause(0.001);
       for n=1:N
            %거리 r 구하기
            r=sqrt((x(n)-xPos).^2+(y(n)-yPos).^2);
            r_obs = Dist_Map(1:size(Dist_Map,1),1:size(Dist_Map,2));
            [dGx,dGy] = gradient(Dist_Map);

            % 쿨롱 Force 
            if(r > Fov) 
                ex = 0;
                ey = 0;
            else
                ex=-k*(q(n)./r.^2).*((x(n)-xPos)./r);
                ey=-k*(q(n)./r.^2).*((y(n)-yPos)./r);
            end

            % 벽에의한 Force 
            if(r_obs > Fov) 
                ex =ex + 0;
                ey =ex + 0;
            else
                ex=ex-k_obs*(q(n)./r_obs.^2).*(dGx./r_obs);
                ey=ey-k_obs*(q(n)./r_obs.^2).*(dGy./r_obs);
            end
%             disp('before'); 
%             x(n) 
%             y(n)
            x(n) = uint32(x(n));
            y(n) = uint32(y(n));
%             disp('afterr'); 
%             x(n) 
%             y(n)

            velocity_x(n) = velocity_x(n) +int32((ex(x(n),y(n)).*Ex(x(n),y(n)))./mass(n));
            velocity_y(n) = velocity_y(n) +int32((ey(x(n),y(n)).*Ey(x(n),y(n)))./mass(n));
            move_x(n) = velocity_x(n);
            move_y(n) = velocity_y(n);
            if(move_x(n) > 20)
                move_x(n) = 20;
            end

            if(move_y(n) > 20)
                move_y(n) = 20;
            end

            if(move_x(n) < -20)
                move_x(n) = -20;
            end

            if(move_y(n) < -20)
                move_y(n) = -20;
            end

            if(x(n) + int32(velocity_x(n)) < Map_Height && y(n) + int32(velocity_y(n)) < Map_Width && (x(n) + int32(velocity_x(n))) > 0 && (y(n) + int32(velocity_y(n))) > 0)
                if( binaryMap((x(n) + int32(velocity_x(n))),(y(n) + int32(velocity_y(n)))) ~= 0 )
                    x(n) = x(n) + int32(velocity_x(n));
                    y(n) = y(n) + int32(velocity_x(n));
                end
            end
            v_totoal_x = v_totoal_x + abs(velocity_x(n));
            v_totoal_y = v_totoal_y + abs(velocity_y(n));
       end
       if(v_totoal_x + v_totoal_y < 3 || iter > 3 )
           break;
       end
       iter = iter +1;
    end
    
    %calc Coverage
    for nx = 1:Map_Height
        for ny = 1:Map_Width
            if(coverageMap(nx,ny) == 150)
                covraged_area = covraged_area +1;
            end
        end
    end
    
    coverage_rate = covraged_area/covrage_area;
    disp('coverage_Rate = '); disp(coverage_rate*100); 
    imshow(coverageMap);
    pause(0.001);
    if(coverage_rate*100 > 99.5)
        for n=1:N
            for nx = x(n)-soultion_size:1:x(n)+soultion_size
                for ny = y(n)-soultion_size:1:y(n)+soultion_size
                   if(int32(ny)<= 0 || int32(ny)<= 0)
                       continue;
                   end
                   if(coverageMap(int32(nx),int32(ny)) == 0)
                       continue;
                   end
                   if(uint16(soultion_size^2) < uint16(nx^2)+uint16(ny^2))
                        coverageMap(uint16(nx),uint16(ny)) = uint16(20);
                   end
                end
            end
        end
        break;
    end
end
for nx = 1:Map_Height
    for ny =1:Map_Width
       if(binaryMap(int32(nx),int32(ny)) == 0)
           Ex(nx,ny) = 0;
           Ey(nx,ny) = 0;
       end
    end
end
imshow(coverageMap);
hold on
surf(y,x,Ex(x,y)+Ey(x,y));
hold on
imshow(coverageMap);
%contour(x,y,sqrt(Ex(x,y).^2+Ey(x,y).^2),20);
% hold on
% contour(yPos,xPos,v);
% hold on
% ima=quiver(yPos,xPos,Ex*scale,Ey*scale,'Color','bLACK');
