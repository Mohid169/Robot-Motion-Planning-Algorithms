function [closest_pt qk]=ClosestPointOnTriangleToPoint(vertices,pt)


x=pt(1);
y=pt(2);
v1=vertices(:,1); v2=vertices(:,2); v3=vertices(:,3);

tri=polyshape(vertices');

if isinterior(tri,x,y)
    closest_pt=[x y]'
    qk=[0;0];
end

edge1=proj1(v1,v2,[x y]');
edge2=proj1(v1,v3,[x y]');
edge3=proj1(v3,v2, [x y]');

dist=inf;
pts=[ v1 v2 v3 edge1 edge2 edge3];
N=length(pts);

for i=1:N
    d=norm(pt-pts(:,i));
    if d<dist
        dist=d;
        closest_pt=pts(:,i);
        index=i;
    end
end
if index<4
    qk=pts(:,index);
elseif closest_pt==edge1
    qk=pts(:,1:2);
elseif closest_pt==edge2
    qk=pts(:,2:3);
elseif closest_pt==edge3
    qk=[pts(:,3) pts(:,1)];
end
    
end
function [b q]=proj1(p1,p2,pt)
l=p2-p1;
m=pt-p1;  
p2e=dot(m,l);
norm_edge=dot(l,l);

conditional=p2e/norm_edge;

if conditional>=1
    b=p2;
    q=p2;
elseif conditional<=0
    b=p1;
    q=p1;
else 
    b=p1+conditional*l;
    q=[p1 p2];
end
end

    
    
    


