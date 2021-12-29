function [x,y,angle] = PlotPath(path,veh)
    rmin = veh.MIN_CIRCLE;
    type = path.type;
    x = [];
    y = [];
    angle=[];
    seg = [path.t,path.u,path.v,path.w,path.x];
    pvec = [0,0,0];
    for i = 1:5        
        if type(i) == 'S'
            theta = pvec(3);
            dl = rmin*seg(i);
            dvec = [dl*cos(theta), dl*sin(theta), 0];
            dx = pvec(1)+linspace(0,dvec(1));
            dy = pvec(2)+linspace(0,dvec(2));
            Routephi = theta*linspace(1,1);
            x = [x,dx];
            y = [y,dy];
            angle = [angle,Routephi];
            pvec = pvec+dvec;
        elseif type(i) == 'L'
            theta = pvec(3);
            dtheta = seg(i);
            cenx = pvec(1)-rmin*sin(theta);
            ceny = pvec(2)+rmin*cos(theta);
            t = theta-pi/2+linspace(0,dtheta);
            Routephi = theta + linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
            x = [x,dx];
            y = [y,dy];
            angle=[angle,Routephi];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = dtheta;
        elseif type(i) == 'R'
            theta = pvec(3);
            dtheta = -seg(i);
            cenx = pvec(1)+rmin*sin(theta);
            ceny = pvec(2)-rmin*cos(theta);
            t = theta+pi/2+linspace(0,dtheta);
            Routephi  = theta + linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
            x = [x,dx];
            y = [y,dy];
            angle=[angle,Routephi];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = -dtheta;
        else
            % do nothing
        end
        if dl > 0
            plot(dx,dy,'b','LineWidth',2);
        else
            plot(dx,dy,'r','LineWidth',2);
        end
        hold on
    end
%     axis ([-3 10 -1 4])
    plot(0,0,'kx','MarkerSize',10,'LineWidth',2)
    quiver(0,0,1,0,'r');
    plot(x(end),y(end),'ko','MarkerSize',10,'LineWidth',2)
    quiver(x(end),y(end),cos(pvec(end)),sin(pvec(end)),'r');