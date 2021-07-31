function [obs_set,node_set] = search_neighbor(pos,coverageMap,Fov,Map_Height,Map_Width)

    obs_set = [];
    node_set = [];
    for theta =0:0.05:360
        for radius = 0:1:Fov
           pixel_x = pos(1) + radius*cos(theta*(pi/180));
           pixel_y = pos(2) - radius*sin(theta*(pi/180));
           if(int32(pixel_x)<= 0)
               break;
           end
           if(int32(pixel_y)<= 0)
               break;
           end
           if(int32(pixel_x) > Map_Width)
               break;
           end
           if( int32(pixel_y) > Map_Height)
               break;
           end
           if(coverageMap(int32(pixel_y),int32(pixel_x)) == 0)
               obs_set = [obs_set ; int32(pixel_x) int32(pixel_y)];
               break;
           end
           if(coverageMap(int32(pixel_y),int32(pixel_x)) == 20)
               node_set = [node_set ; int32(pixel_x) int32(pixel_y)];
               break;
           end
        end
    end
    
end