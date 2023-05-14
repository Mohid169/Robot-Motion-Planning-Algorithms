%% This function calculates the force that is then parsed to the Potenial Field algorith,

function F=findForce(pos,q_goal,O,eta)

nu=1000;
threshold=8;
%% Negative Force (repulsive force from obstacle)

Fb=[0;0];

for i=1:length(O)
    obs=O{i};
    
    Dists=[];
    n=size(obs,2)-1;
    for t=1:n
       tri=[obs(:,1) obs(:,t:t+1)];
       [pt ~]=ClosestPointOnTriangleToPoint(tri,pos);
       Dists=[Dists euclidean(pos,pt)];
    end
    Dist=min(Dists);
    
    if Fb<threshold
        Fb=Fb+nu*(1/Dist-1/threshold)*Dist^(-3)*(pos-pt);
    else
        Fb=Fb;
    end
    
end


    

%% Postiive Force (attactive force to goal)
Fa=-eta*(pos-q_goal) ;

%% Force 
F=Fa+Fb


end