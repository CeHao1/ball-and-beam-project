function [A, B] = integrator(rank)

A = zeros(rank);
for i=1:rank-1
    A(i, i+1)=1;
end

B = zeros(rank, 1);
B(end)=1;


end