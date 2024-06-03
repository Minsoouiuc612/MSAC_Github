clear; clc;   % Clear The Previous Points
format long
Ns     = 50; % Set The Number of Sample Points
% RES    = 15;  % Set The DAC Resolution
OFFSET = 0;   % Set An Offset Value For The DAC Output
ARR = 3200;
PHASE_PERCENT = 0.30; %Percent phase offset based on ENTIRE PWM frequency (as opposed to just the HIGH part)
%------------[ Calculate The Sample Points ]-------------
T = 0:((2*pi/(Ns-1))):(2*pi);
Y = sin(T);
Y = Y + 1;    
% Y = Y*((2^RES-1)-2*OFFSET)/(2+OFFSET);
Y = Y*(ARR/2);
Y = round(Y);
figure(1)
scatter(T, Y);

Y2 = sin(T + (PHASE_PERCENT * 2 * pi));
Y2 = Y2 + 1;    
% Y = Y*((2^RES-1)-2*OFFSET)/(2+OFFSET);
Y2 = Y2*(ARR/2);
Y2 = round(Y2);   
figure(2)
scatter(T, Y2);
% plot(T, Y);
% grid
%--------------[ Set any 0 entries to 1 (can't have 0 for DMA) ]---------------
Y(find(Y==0)) = 1;
Y2(find(Y2==0)) = 1;

%--------------[ Print The Sample Points Y ]---------------
elements_per_line = 10;

for i = 1 : Ns
    if i ~= Ns
        fprintf(['%6.0f, '], Y(i))
    else
        fprintf(['%6.0f'], Y(i))
    end

    if mod(i, elements_per_line) == 0
        fprintf('\n');
    end
end
fprintf('\n\n');

%--------------[ Print The Sample Points Y2 ]---------------
elements_per_line = 10;

for i = 1 : Ns
    if i ~= Ns
        fprintf(['%6.0f, '], Y2(i))
    else
        fprintf(['%6.0f'], Y2(i))
    end

    if mod(i, elements_per_line) == 0
        fprintf('\n');
    end
end
fprintf('\n\n');
 
%-------------------[ Other Examples To Try ]-------------------
%  Y = diric(T, 13);     % Periodic Sinc
%  Y = sawtooth(T)       % Sawtooth
%  Y = sawtooth(T, 0.5); % Triangular