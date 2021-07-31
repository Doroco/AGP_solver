function [Ex,Ey] = Compute_Potential(Map,gain,)
    k=1;
    N=length(q);
    for n=1:N
        if q(n)>=0
            hold on
            plot(x(n),y(n),'og','Color','red')
        else
            hold on
           plot(x(n),y(n),'og','Color','blue')
        end
    end

    xmin=-3;
    xmax=3;
    ymin=-3;
    ymax=3;
    [xPos , yPos]=meshgrid(xmin:0.2:xmax,ymin:0.2:ymax);


    scale=10;
    ex=0; ey=0;
    Ex=0; Ey=0;
    v=0; v2=0;

    for n=1:N
        r=sqrt((x(n)-xPos).^2+(y(n)-yPos).^2);
        ex=-k*(q(n)./r.^2).*((x(n)-xPos)./r);
        Ex=Ex+ex;
        ey=-k*(q(n)./r.^2).*((y(n)-yPos)./r);
        Ey=Ey+ey;


        v=v-ex.*(r.*((x(n)-xPos)./r));
        v=v-ey.*(r.*((y(n)-yPos)./r));
    end
    image=figure(1);
    hold on
    contour(xPos,yPos,v);
    hold on
    ima=quiver(xPos,yPos,Ex*scale,Ey*scale,'Color','bLACK');

end