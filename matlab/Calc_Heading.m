function distance = Calc_Heading(pos,K_obs,K_node,NodeCell,ObsCell,Fov)
    distance = 0;
    %calc Node Force
    for i=1:size(NodeCell,1)
        distance = distance + mod(atan2( double(pos(2)-NodeCell(i,2)), double(pos(1)-NodeCell(i,1))),90) ;
    end
    distance = distance/size(NodeCell,1);
end