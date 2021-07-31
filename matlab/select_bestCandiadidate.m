function [posinfo,static,cost] = select_bestCandiadidate(pos,K_obs,K_node,v_min_x,v_max_x,v_min_y,v_max_y,vel_resolution,coverageMap,Fov,Map_Height,Map_Width,Dist_Map)
    min_cost = 1e15;
    poscd = [];
    cost = 0;
    best_x = 1111;
    best_y = 1111;
    for vx = v_min_x : vel_resolution : v_max_x
        for vy = v_min_y : vel_resolution : v_max_y
            poscd(1) = int32(pos(1) + vx);
            poscd(2) = int32(pos(2) + vy);
            if(poscd(1) <= 0 ||  poscd(2) <= 0)
                continue;
            end
           if(poscd(1) > Map_Width)
               break;
           end
           if( poscd(2) > Map_Height)
               break;
           end
            if(coverageMap(poscd(2),poscd(1)) == 20)
                continue;
            end
            if(coverageMap(poscd(2),poscd(1)) == 0)
                continue;
            end
            [obs,node]= search_neighbor(poscd,coverageMap,Fov,Map_Height,Map_Width);
            [node,~,~] = unique(node,'rows');
            [obs,~,~] = unique(obs,'rows');
            [Fx , Fy]  = agent_potential(poscd,K_obs,K_node,node,obs,Fov);
            differ = Calc_Heading(pos,K_obs,K_node,node,obs,Fov);
           % disp(sqrt(double(Fx^2+Fy^2)));
           % disp(K_obs/Dist_Map(poscd(2),poscd(1)));
            cost = K_node*sqrt(double(Fx^2+Fy^2))+K_obs/(Dist_Map(poscd(2),poscd(1)) )^2;%+(Fov*differ)^6;
            if(Dist_Map(poscd(2),poscd(1)) < 7)
                cost = 1e15;
            end
            if( min_cost >= cost)
                min_cost = cost;
                posinfo(1) = poscd(1);
                posinfo(2) = poscd(2);
                best_x = vx;
                best_y = vy;
            end
        end
    end
    if(min_cost == 1e15)
        posinfo(1) = pos(1);
        posinfo(2) = pos(2);
        static = 0;
    else
        static = 0;
    end
    if(abs(best_x)+abs(best_y) <= 3)
        static = 1;
    end
    cost = min_cost;
end