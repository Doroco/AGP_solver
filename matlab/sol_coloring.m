function coverageMap =sol_coloring(pos,coverageMap,Fov,Map_Height,Map_Width)
    for theta =0:0.05:360
        for radius = 0:1:Fov
           pixel_x = pos(1) + radius*cos(theta*(pi/180));
           pixel_y = pos(2) + radius*sin(theta*(pi/180));
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
           if( coverageMap(int32(pixel_y),int32(pixel_x)) == 0)
               break;
           end
           coverageMap(int32(pixel_y),int32(pixel_x)) = 150;
        end
    end
end