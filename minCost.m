clear all; close all;
load('C_matrix.mat')
[T, N] = size(C);
DP = zeros(T+1, N);
cost_matrix = C;
DP_path=zeros(T,N);

for r=T:-1:1
    for c = N:-1:1
        best_cost=inf;  
        best_prev_c='None';
        if c == 1
            %DP(r,c)=cost_matrix(r,c)+min([DP(r,c), DP(r,c+1)]);
            offset1=[0 1];
            for i=1:length(offset1)
                if DP(r+1,c+offset1(i))<best_cost
                    best_cost=DP(r+1,c+offset1(i));
                    best_prev_c=c+offset1(i);
                end
            end
            DP_path(r,c)=best_prev_c;
            DP(r,c)=best_cost+cost_matrix(r,c);
            
        elseif c == N
            %DP(r+1,c) = cost_matrix(r,c)+min([DP(r,c-1),DP(r,c)]);
            offset2=[-1 0];
            for i=1:length(offset2)
                if DP(r+1,c+offset2(i))<best_cost
                    best_cost=DP(r+1,c+offset2(i));
                    best_prev_c=c+offset2(i);
                end
            end
            DP_path(r,c)=best_prev_c;
            DP(r,c)=best_cost+cost_matrix(r,c);
        else
            offset3=[-1 0 1];
            for i=1:length(offset3)
                if DP(r+1,c+offset3(i))<best_cost
                    best_cost=DP(r+1,c+offset3(i));
                    best_prev_c=c+offset3(i);
                end
            end
            DP_path(r,c)=best_prev_c;
            DP(r,c)=best_cost+cost_matrix(r,c);

        end
      
    end
end
% flipped since the top starts at 
%DP_path(1)=[];
    
optimal_cost=min(DP(1,:));
disp(optimal_cost)

path=[];
c=argmin(DP(1,:));
cost=0;
path=[c];
costs=[];
for r=1:T-1
    c=DP_path(r,c);
    cost=cost+cost_matrix(r,path(r));
    costs=[costs cost_matrix(r,path(r))];
    path=[path c];
end

disp(path)
DP(T+1,:)=[];
costs
figure;
imagesc((flipud(DP)));
printcost(C,T,N)
function printcost(C, T, N)
   % close all;
   % fig = figure; fig.Position = [1 41 1920 963]; fig.Color = 'w';
    plot([zeros(1,T+1), 0:N; N+zeros(1,T+1), 0:N ], ....
         [0:T,zeros(1,N+1);0:T,T+zeros(1,N+1)], 'k');
    
    for j = 1:N
        for i = 1:T
            text(j-0.5, T+0.5-i, num2str(C(i,j)), ...
                "FontSize",20, "HorizontalAlignment","center", ...
                "VerticalAlignment","middle");
        end
    end
end
function [I,M]=argmin(varargin)
[M,I] = min(varargin{:});
end

