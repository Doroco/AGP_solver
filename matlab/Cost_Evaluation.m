function best_pos = Cost_Evaluation(pos,Fov,move_resolution,safeMap,Map_Height,Map_Width,Dist_Map)
    m_pos = [];
    best_pos = pos;
    max_cost = -1e7;
   for vx = -Fov:move_resolution:Fov
        for vy = -Fov:move_resolution:Fov
            m_pos(1) = pos(1)+vx;
            m_pos(2) = pos(2)+vy;
            if(m_pos(1) <= 0 ||  m_pos(2) <= 0)
                continue;
            end
            if(m_pos(1) >= Map_Width || m_pos(2) >= Map_Height)
                break;
            end
            if(safeMap(m_pos(2),m_pos(1)) < 75)
                continue;
            end
            if(Dist_Map(m_pos(2),m_pos(1)) < 20)
                continue;
            end
            sy = m_pos(2)-int32(Fov*0.7071);
            if(sy < 1)
                sy = 1;
            end
            ey = m_pos(2)+int32(Fov*0.7071);
            if(ey > Map_Height)
                ey = Map_Height;
            end
            sx = m_pos(1)-int32(Fov*0.7071);
            if(sx < 1)
                sx = 1;
            end
            ex = m_pos(1)+int32(Fov*0.7071);   
            if(ex > Map_Width)
                ex = Map_Width;
            end
            FOV_image = safeMap(sy:ey,sx:ex);
            if( nnz(FOV_image==255) < 3)
                continue;
            end
           
            FillAreaCost = nnz(FOV_image==255)-nnz(FOV_image==0)-nnz(FOV_image==30);
            Heading = mod( abs(atan2( double(pos(2)-m_pos(2)), double(pos(1)-m_pos(1)))),90);
            potential_Cost = (FillAreaCost)*Dist_Map(m_pos(2),m_pos(1));
            if(FillAreaCost < 0 )
                FillAreaCost = 0;
                potential_Cost =Dist_Map(m_pos(2),m_pos(1))-nnz(FOV_image==30);
                Heading =potential_Cost* Heading;
            end
            cost = FillAreaCost+(FillAreaCost+1)*Heading +potential_Cost;
            if(max_cost < cost)
%                 disp(FillAreaCost)
%                 disp((FillAreaCost+1)*Heading)
%                 disp(potential_Cost)
%                 disp(m_pos(1));
%                 disp(m_pos(2));
                max_cost = cost;
                best_pos(1) = m_pos(1);
                best_pos(2) = m_pos(2);
            end
        end
   end
end 