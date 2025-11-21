%% Run Experiment - Input experiment trials and run from both positions
clc; clear; close all;

Rz = @(q)[cos(q), -sin(q); sin(q), cos(q)];

CoMs = [0        0; 
       0.02268 -0.01512; 
       0.0189   0.0189; 
      -0.01512  0.00756;
      -0.01134 -0.01701];

attachments = [-0.086   0;
            0       0.086];


final_vectors = [];
final_push = [];

for i=1:size(CoMs,1)
    CoM = CoMs(i,:)';
    for j=1:size(attachments,1)
        aPoints = attachments(j,:);

        Simulation
        
        final_push = [final_push, (Rz(-O.p(3))*A.desired_direc)];
        close all;
    end
    final_vectors = [final_vectors; final_push];
    final_push = [];
end

final_vectors = round(final_vectors,3);