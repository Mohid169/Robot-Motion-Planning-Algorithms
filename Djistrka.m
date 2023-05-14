function [ path]=Dijkstra(G, n_init, n_goal)
[m t]=size(G);


for n=1:t %For each node n in graph 
    dist(n)=inf; %Set the distance as infinity
    prev(n)=NaN; %Set the previous nodes as NUll
end

dist(n_init)=0; %Initialize the distance at the starting node as 0

for i=1:m %Create a set of all the notes
    U(i)=i; %The number of nodes is equal to the m, the number of nodes in the adjency matrix
end


while ismember(n_goal, U) %While the goal node is still in the set of unvisited nodes
    
    %This section of code is used to find the node with the smallest
    %distance
   
    [M I]=min(dist(U));
    
    C=U(I); %Assigning the node in U w/ the smallest distance as the current node
    U(I)=[]; %Remove C from U
    
    
    neighbors= find(G(C,:)>0 &(G(C,:)~=inf));
     %Obtain the neighbors from the adjaceny matrix
    for v=1:length(neighbors) %For all of the current nodes neighbors
        alt=dist(C)+G(C,neighbors(v)); %Compute the distance from initial node to the current node + the distance between the current node and this particular neighbor
        if alt<dist((neighbors(v))) %If the total distance is less than that current path to the neighbor
            dist(neighbors(v))=alt; %Replace it with the newly computed, shorter distance
            prev(neighbors(v))=C; %Set the current node as the node that precedes V
        end
    
    end

end 
    
    %Construct the shortest path
    distance=dist(n_goal);
    path(1)=(n_goal) ;
    path(2)=prev(n_goal);
    i=2;
 
    while (path(i)~=n_init)
        i= i+1;
        path(i)=prev(path(i-1));
    end
    path=flip(path);

end
    
    

