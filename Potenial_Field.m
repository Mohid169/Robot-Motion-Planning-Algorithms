%Potential Field Method Script
clear
close all
figure;

CB1 = [0 50 50 0; 25 25 50 50];
CB2 = [80 80 70 70; 50 100 100 50];

C={CB1,CB2};

q_init = [0.5; 0.5];
q_goal = [95; 95];

path = [q_init];
pos= q_init;
eta=.5;
while euclidean(pos, q_goal) > 0.1
    if euclidean(pos,q_goal)<1
        eta=10; %change eta to increase attractive force since it was taking a longtime to reach the goal when it was a meter inrange
    end
    force = findForce(pos, q_goal, C,eta);
    gradient=.1*force/norm(force);
    pos = pos + gradient;
    path = [path pos];
    
end


%% Visualize
 figure;
 hold on

for i = 1:length(C)
    C_obs = polyshape(C{i}');
    plot(C_obs);
end
axis([0 100 0 100]);

%Iterate until goal is reached

plot(q_init(1), q_init(2), 'or');
plot(q_goal(1), q_goal(2), 'pg') ;
plot(path(1,:), path(2,:), 'blue','LineWidth', 2.0);

hold off;
