function [isok,path] = CSC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpSpLp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(15,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(15,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(16,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,-y,phi); % timeflp + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(16,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(13,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(13,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,-y,-phi); % reflect 
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(14,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(14,:),-t,-u,-v,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end