function [path, V, E] = build_RRT(qi, qg, NumNodes, step, O, x_max, y_max)


    qI = qi;
    qG = qg;

    figure(1)
    axis([0 x_max 0 y_max])
    for i=1:length(O)
        
        obs = cell2mat(O(i));
       
        x_obs = obs(1, :);
        y_obs = obs(2, :);
        polyin = polyshape({x_obs}, {y_obs});
        plot(polyin)
        hold on
    end

    V =  [];
    V = [V qI; -1];
    E = [];
    
    figure(1)
    axis([0 x_max 0 y_max])
    plot(polyin)
    hold on
   
    for i = 1:NumNodes
        % whether or not it is collision free
        q_rand = [floor(rand(1)*x_max); floor(rand(1)*y_max)];

        % Break if goal node happens to be reached
        for j = 1:size(V,2)
            if V(j) == qG
                break
            end
        end
        
        [q_near, ind]=Nearest_Vertex(q_rand,V);
    
        
        tmp = [0; 0];
        tmp(1) = q_near(1) + step/50*q_rand(1);
        tmp(2) = q_near(2) + step/50*q_rand(2);
        q_new = [tmp(1); tmp(2)];
        edge_k=[q_near q_new];

        boolean=isintersect_linepolygon(edge_k,O)

        if boolean~=1
            line([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'Color', 'k', 'LineWidth', .5);
            E = [E [ind; size(V,2)]];
            drawnow
            hold on
           
            V = [V [q_new; ind]];
          end
        
    end
    R = V;
    first_i = -1;
    a=4;
    while 3<a
        [q_nearest, q_index] = Nearest_Vertex(qG, R);  
        edge_new=[q_nearest,qG]
        if ~isintersect_linepolygon(edge_new,O)
            first_i = q_index;
             a=1;
        else
            R(:, q_index) = [];
        end
        
    
    end
    path = [qG];
    [~, index_new] = Nearest_Vertex(R(1:2, first_i), V);
    while (index_new ~= -1)
        q = V(1:2,index_new);
        path = [q path];

        %% plot
        index_new = V(3,index_new);
        plot(q(1), q(2), 'gs');
        plot(path(1, 1:2), path(2, 1:2), 'r');
        if index_new==-1
            break
        end
        pause(0.01)
    end
    V(3, :) = [];
    
    
    hold off
end

function [q_near, ind] = Nearest_Vertex(q, V)

    ind=-1;
    dist_qnear = inf;
    for j = 1:size(V,2)
           qt = V(1:2, j);
           temp = euclidean(qt, q);
           if temp < dist_qnear
                dist_qnear = temp;
                q_near = qt;
                ind=j;
           end
        end
end
function boolean=isintersect_linepolygon(L,S)
S=cell2mat(S(1));
N = size(S,2);
Q = [S S(:,1)];
te=0;
tl=1;
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

