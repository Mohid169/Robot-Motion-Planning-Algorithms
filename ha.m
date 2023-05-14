function h = ha(G2,neighbor,n_goal)
%HA Summary of this function goes here

path=shortestpath(graph(G2),neighbor,n_goal);
h=length(path);

%This heuerisitic is good because it is always admissable (e.g) the shortest
%nodes to goal will always greater than or equal 1, which will always be
%less than the actual cost to get the goal (the minimum is weighting is
%one), therefore this heuristic is always admissable

end

