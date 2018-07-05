%%PRACTICA 2 RECEPCION
%% RECEPTOR
    clear all; clc; close all;
    Fs=48e3;
    mpbits=16;
    nChannels = 1;
    %%
    recObj = audiorecorder(Fs,mpbits,nChannels);

    %%
    disp('Grabando Señal');
    record(recObj);

    %%

    stop(recObj);
    disp('Fin de la grabacion');
    %% recuperacion de los datos
    %%play(recObj);
    SIGNAL_RECEIVED = getaudiodata(recObj);
    plot(SIGNAL_RECEIVED,'r');title('RC modulated DSB-sc ');grid on;
    %%
%%    filename = 'Practica2.wav';
%%    audiowrite(filename,SIGNAL_RECEIVED,fs);

%% PARTE 1
%% Paso 1
plot(SIGNAL_RECEIVED,'r');title('Señal impulso grabada ');grid on;
%% Paso 2
plot(SIGNAL_RECEIVED,'r');title('Señal impulso grabada ');grid on;
figure(2);pwelch(s1,[],[],[],fs,'power');
%% Paso 3
plot(SIGNAL_RECEIVED,'r');title('Señal impulso grabada ');grid on;
figure(2);pwelch(s1,[],[],[],fs,'power');
%%%%%%%%%%%%%%%
%%% PARTE 2
%% Eliminamos el silencio
Comienzo = min(find(SIGNAL_RECEIVED > 0.5))- 100;
%%Final = max(find(SIGNAL_RECEIVED > 0.5))+ 100;
SIGNAL = SIGNAL_RECEIVED(Comienzo:end);

plot(SIGNAL,'r');title('Señal impulso grabada ');grid on;
%%figure(2);pwelch(SIGNAL,[],[],[],fs,'power');

%% PARTE 3
%% MATCH FILTER
    B = 10e3; %señal en banda base
    fm = 5e3;
    beta = 0.5;
    D  = 10;
    Rb = 2*fm/(1+beta);%B = fm
    fs = 48e3;
    Ts = 1/fs;
    Tp = 1/Rb;
    mp = Tp/Ts;
    mp = ceil(mp);
    Rb = fs / mp;
    Tp = 1/Rb;
    energy = Tp; 
    type = 'rc';
    [Prc, t] = rcpulse(beta,D,Tp,Ts,type,energy);
%% DEMODULACION
Fs = 48e3;
Fc = 6000;
[num,den] = butter(10,Fc*2/Fs); % Lowpass filter (LPF) 
freqz(num,den);
s1 = amdemod(SIGNAL,Fc,Fs,0,0,num,den); % Demodulate. % Observe las siguientes gráficas 


figure(2);pwelch(s1,[],[],[],Fs,'power'); % OBSERVAMOS el espectro de frecuencia de la señal demodulada
%% Filtrada con match filter
FINAL_SIGNAL = conv(s1,Prc);                           %%Signal Pulse Train
FINAL_SIGNAL = s1;
eyediagram(FINAL_SIGNAL(89:89+100*mp),2*mp);
%% Diagrama de ojo
FINAL_SIGNAL = FINAL_SIGNAL(89:end)/max(abs(FINAL_SIGNAL(89:end)));
mp = 8;
ed = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',3*mp)
show(ed);
ed(FINAL_SIGNAL(1:Fs));                      %% La señal tiene que ser un vector columna
%% Continuación
%%% graficar las primeras 100, 500 y 1000 muestras
stem(FINAL_SIGNAL(1:1:100));
vint = round(abs(FINAL_SIGNAL(89:8:64*1089+88)));
%%vint(vint <= 0) = 0;
%%
MSignal = reshape(vint(1:end)',8,1089);
MSignal = MSignal';
%%
VMS = bi2de(MSignal,'left-msb');
%%
LenaRecSignal = reshape(VMS,33,33);
LenaRecSignal = cast(LenaRecSignal,'uint8');
imshow(LenaRecSignal)
title('Lena Reconstruida');





