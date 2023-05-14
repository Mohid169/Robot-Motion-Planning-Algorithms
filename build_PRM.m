function [path, V, E] = build_PRM(qi, qg, NumNodes, step, O, x_max, y_max)
hold on;

    qI = qi;
    qG = qg;

    figure(1)
    axis([0 x_max 0 y_max])

    obs=cell2mat(O(1))
    
    %Create containers
    V =  [];
    E = [];
    
    while size(V,2)<NumNodes %generate n free random configurations
        c= 1;
        while c==1
            q_rand=[floor(rand(1)*x_max); floor(rand(1)*y_max)];
            for i=1:length(O) %if point does not collide with obstacle, proceede
                [in, on]=inpolygon(q_rand(1,1),q_rand(2,1),O{i}(1,:),O{i}(2,:));
            end
            if in==0 && on==0
                c=0;
                V=[V q_rand]; %add q_rand as part of graph
            end
        end
        
    end
    
    V=[V qI qG]; %add start and goal as vertix
    
    for i=1:size(V,2)
        node=V(:,i);
        
        %sort vertices based on distance
        distances=arrayfun(@(x) norm(V(:,x)-node), 1:size(V,2));
        [~, index]=sort(distances);
        
        index=index(2: 1+step);
        
        %see if the edge intersects with the obstacle
        for j=1:step
            flag=0;
            for edge_index=1:size(E,2)
                if E(1,edge_index)==j && E(2,edge_index)
                    flag=1;
                    break
                end
            end
            boolean=isintersect_linepolygon([V(:,index(1,j)) V(:,i)], O);
            if flag==0 && boolean==0
                E=[E [i; index(1,j)]];
            end
        end
    end
    
    %Create adjacency matrix
    total_nodes=size(V,2);
    G=zeros(total_nodes);
    for i=1:total_nodes
        for j=1:total_nodes
            G(i,j)=inf;
        end
    end
    
    for i=1:total_nodes
       for j=1:total_nodes
           if i==j
               G(i,j)=0;
           end
       end
    end
    
    
    %find lengths of edges
      
    %weight adjaceny matrix
    total_edges=size(E,2);
    for i=1:total_edges
        temp_1=V(:,E(1,i));
        temp_2=V(:,E(2,i));
        G(E(1,i), E(2,i))=euclidean(temp_1,temp_2);
        
    end
    total_nodes
    %apply Djistrka for analysis (could not get this part to work!!)
    path=Djistrka(G,total_nodes-1,total_nodes);
end
    
function boolean=isintersect_linepolygon(L,S)
S=cell2mat(S(1));
N = size(S,2);
Q = [S S(:,1)];
te=0;
tl=1;
boolean=0;

if L(:,1) == L(:,2)
    [in, on] = inpolygon(L(1,1), L(2,1), S(1,:), S(2,:));
    if in(1) || on(1)
        boolean = 1;
        return
    end
else
    te = 0;
    tl = 1;
    ds = L(:,2) - L(:,1);
    for i = 1:N
        n_i=[Q(2, i+1) - Q(2, i) -1*(Q(1, i+1) - Q(1, i))];
        N = -1*dot(L(:,1) - Q(:,i), n_i);
        D = dot(ds, n_i);
       
        if D == 0 
            if N < 0
                boolean = 0;
                return
            end
        end
        t=N/D;
        if D<0
            te = max([te t]);
            if te > tl
                boolean = 0;
                return
            end
        elseif D > 0
            tl = min([tl t]);
            if tl < te
                boolean = 0;
                return
            end
        end
    end
    if te <= tl
        boolean = 1;
        return
    else
        boolean = 0;
        return
    end
end
end



function dist = euclidean(q1,q2)
    dist = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
end
    