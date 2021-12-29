% 简单证明CCC有尖瓣cusps路径短
function [isok,path] = CCC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmL(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),-t,-u,-v,0,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi); % 旋转+Y正负变换
    yb = x*sin(phi)-y*cos(phi); %LpRpL
    [isok,t,u,v] = LpRmL(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),-v,-u,-t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),-v,-u,-t,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end