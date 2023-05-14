function h=hb(row)
idx=find(row==inf);
for i=1:length(idx)
    row(idx(i))=0;
end
h=sum(row);

end

%I don't think this heuristic is good because it is not admissable in all
%cases. There are multiple cases where this heuristic is inadmissable, for
%example the case the case where the path to the goal node is lower than
%the combined weights of the neighbors. Let's say the path to the goal node
%2 but the sum of the weights of the neighbors is 10. Therefore, this
%heuristic is inadmissable (aka, not good).