clc;clear;
close all;

x = [-0.1, 0, 0, 0]';
Np = 100;
ref = [0.1*ones(1, Np+1);zeros(1, Np+1)];
dt = 0.02;

tic

[V_servo, theta_d] = linearized_MPC_01(x, ref, Np, dt);

toc