function [Fx ,Fy] = agent_potential(pos,K_obs,K_node,NodeCell,ObsCell,Fov)
    Fx = 0;
    Fy = 0;
    
    %calc Node Force
    for i=1:size(NodeCell,1)
        distance = sqrt(double((pos(1)-NodeCell(i,1))^2 + (pos(2)-NodeCell(i,2))^2));
        if(distance <= 1)
            distance = 1;
        end
        if(distance < Fov)
            Fx = Fx + (pos(1)-NodeCell(i,1))/distance^2;
            Fy = Fy + (pos(2)-NodeCell(i,2))/distance^2;
        end
    end
%     Fx = Fx/size(NodeCell,1);
%     Fy = Fy/size(NodeCell,1);
    %calc Obs Force
%     for i=1:size(ObsCell,1)
%         distance = sqrt(double((pos(1)-ObsCell(i,1))^2 + (pos(2)-ObsCell(i,2))^2));
%         if(distance < Fov)
%             Fx = Fx + K_obs*(pos(1)-ObsCell(i,1))/distance^3;
%             Fy = Fy + K_obs*(pos(2)-ObsCell(i,2))/distance^3;
%         end
%     end
end