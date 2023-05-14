clear; close all; clc


O={};
obs =  [60 60 40 40; 
       20 40 40 20];
O{1}=obs
q_i = [0; 0]; q_g = [70; 70]; x_max = 100; y_max = 100; step = 1; NumNodes = 100;


[path, V, E] = build_RRT(q_i, q_g, NumNodes, step, O, x_max, y_max);
  %%
  obs =  [6 6 4 4 ; 
          2 4 4  2];
O{1}=obs;
[path, V, G] = build_PRM([0 0]', [10 10]', 100, 5, O, 10, 10);