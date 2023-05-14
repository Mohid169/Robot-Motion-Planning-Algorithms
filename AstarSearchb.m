function [path]=AstarSearch(G, n_init, n_goal)
O=[n_init]; %Open Set
C=[]; %Closed Set


[nodes columns]=size(G); %Obtain the number of nodes from the dimenisons of the graph


for n=1:nodes %intialize all the nodes
    prev(n)=NaN; %Initialize everything
    f(n)=inf; %Metric (past cost + heuristic cost)
    g(n)=inf; %Past Cost
end
g(n_init)=0; %Set the g as 0
f(n_init)=0; %Set the cost as 0

while (isempty(O)==0)
  
   [M I]=min(f(O)); %Initialize the distance at the starting node as 0

    n_best=O(I); %Assignning the n_best as one the best metric 
    O(I)=[]; %Remove N_best from the set of univisted nodes
    
    C=[C n_best]; %Add n_best to the set of visited nodes
   

    if (n_best==n_goal) %if we are at the goal, get out of the loop
        break
    end
    
    
    neighbors=find(G(n_best,:)>0 &(G(n_best,:)~=inf)); %Obtain the neighbors from the graph

    for x=1:length(neighbors) %Iterate across the length
        if (neighbors(x)<inf) %If the neighbor is less than inf (this is just a remanant from my old cost but it doesnt really have an impact)
            if ismember(neighbors(x),C)
               continue
            end

            g_temp=g(n_best)+G(n_best,neighbors(x)); %Calculate the past distance + the distance to the neighbor

            if ~ismember(neighbors(x),O) %if neighbors is not in the set, then add it to the set of unvisited nodes
                O=[O neighbors(x)];

            elseif g_temp>g(neighbors(x)) %Update value if greater than current
                continue
            end
            
            %Update the values
            prev(neighbors(x))=n_best; 
            g(neighbors(x))=g_temp;        
            f(neighbors(x))=g(neighbors(x)) + hb(G(neighbors(x),:));
            
        end 
    end
         
end

%Calculating the path
 path(1)=(n_goal) ;
 path(2)=prev(n_goal);
 i=2;
 while (path(i)~=n_init)
    i= i+1;
    path(i)=prev(path(i-1));
 end
    
    path=flip(path);
end 