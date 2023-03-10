%% Clean environment
clear;
clc;
close all;

%% Get .CSV Filename
filename = "/home/tommaso/Documents/Tracciati/skidpad.csv"

%% Read data
Data=csvread(filename);
X = Data(:, 1);
Y = Data(:, 2);

%% Plot data
plot(X, Y)
