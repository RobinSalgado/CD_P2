%% PRACTICA #3 Recepcion
close all; clc; clear all;      %% Limpiamos las variables, terminal y cerramos ventanas
%%
Fs=48e3;                        %% Frecuencia de muestreo
mpbits=16;                      %% Bits por muestra
nChannels = 1;                  %% Canal  
recObj = audiorecorder(Fs,mpbits,nChannels);    %% Creamos objeto de audio

%%      Grabamos la señal
disp('Grabando Señal');
record(recObj);

%%      Terminamos de grabar la señal

stop(recObj);
disp('Fin de la grabacion');
%% recuperacion de los datos
SIGNAL_RECEIVED = getaudiodata(recObj);
%%plot(SIGNAL_RECEIVED,'r');title('RC modulated DSB-sc ');grid on;

%% Eliminamos el silencio
Comienzo = min(find(SIGNAL_RECEIVED > 0.5))- 100;
SIGNAL = SIGNAL_RECEIVED(Comienzo:end);
%% FILTRAMOS LA SEÑAL
%% Filtrar senal con filtros pasabandas.

%%% Ancho de Banda
B_IMG  = 2400;          %% Ancho de banda de la imagen
B_SOUND = 7200;         %% Ancho de banda del sonido

% Frecuencia de las portadoras
Fc_S = 7500;
Fc_I = 17500;
% Orden del filtro.
O = 100;
% Crear filtro.
m_SOUND = [1 1 0 0];
f_SOUND = [0 (Fc_S+B_SOUND-1000)/(Fs/2) (Fc_S+B_SOUND-1000)/(Fs/2) 1];
%%
m_IMG = [0 0 1 1 ];
f_IMG = [0 (Fc_I-B_IMG+500)/(Fs/2) (Fc_I-B_IMG+500)/(Fs/2)  1];
Filtro_IMG = fir2(O,f_IMG,m_IMG);
freqz(Filtro_IMG,1)
%%
Filtro_AUD = fir2(O,f_SOUND,m_SOUND);
freqz(Filtro_AUD,1)
%%
Signal_IMG = conv(Filtro_IMG,SIGNAL);
Signal_AUD = conv(Filtro_AUD,SIGNAL);
pwelch(Signal_AUD,[],[],[],Fs,'power')
%% DEMODULAMOS
IMG_DEM = abs(hilbert(Signal_IMG));
AUD_DEM = abs(hilbert(Signal_AUD));
%% FIND END OF SIGNAL
IMG_END = max(find(abs(IMG_DEM) > 0.4));
AUD_END = max(find(abs(AUD_DEM) > 0.4));

%% FIX SIGNAL SIZE
IMG_DEM = IMG_DEM(100:IMG_END);
AUD_DEM = AUD_DEM(60:AUD_END);
%%
IMG_DEM = IMG_DEM - mean(IMG_DEM);
AUD_DEM = AUD_DEM - mean(AUD_DEM);
%%
IMG_DEM = (IMG_DEM/max(abs(IMG_DEM)));
AUD_DEM = (AUD_DEM/max(abs(AUD_DEM)));
%%
pwelch(AUD_DEM,[],[],[],Fs,'power');            %% Espectro de frecuencias.
%% MATCH FILTERS
%%% PULSO SRRC
% Frecuencia y periodo de muestreo.
Fs = 48000;             %% Frecuencia de muestreo
Ts  = 1/Fs;             %%intervalo de muestreo
D   = 10;               %% D = duración del pulso como número de intervalos de símbolo
r   = 0.2;              %% Beta del raised cosine
%%% Ancho de Banda
B_IMG  = 2400;          %% Ancho de banda de la imagen
B_SOUND = 7200;         %% Ancho de banda del sonido

% Tasa de bits
Rb_IMG =   2*B_IMG / (1+r);     %% Tasa de bits de la imagen
Rb_SOUND = 2*B_SOUND / (1+r);   %% Tasa de bits del sonido

% Intervalo de muestreo. o periodo de bit
Tp_IMG = 1/Rb_IMG;              %%2.5e-4     RB_IMG = 4000  B_IMG = 2400  
Tp_SOUND = 1/Rb_SOUND;          %%8.3333e-5  RB_SOUND = 12000 B_SOUND = 7200

% Numero de muestras por bit.
mp_IMG = ceil(Tp_IMG/Ts);         %% Muestras por bit de la señal de imagen
mp_SOUND = ceil(Tp_SOUND/Ts);     %% Muestras por bit de la señal de sonido

% ENERGIA DE LA SEÑAL
E_IMG   = 1/Rb_IMG;      %% Energia de la señal de imagen
E_SOUND = 1/Rb_SOUND;    %% Energia de la señal de sonido      
type = 'srrc';

[P_SOUND t_SOUND] = rcpulse(r,D,Tp_SOUND,Ts,type,E_SOUND);  %% Pulso formador SRRC
[P_IMG t_IMG] = rcpulse(r,D,Tp_IMG,Ts,type,E_IMG);  %% Pulso formador SRRC

%% APLICAMOS EL MATCH FILTER PARA CADA SEÑAL
IMG_MATCH = conv(IMG_DEM,P_IMG);
AUD_MATCH = conv(AUD_DEM,P_SOUND);
%% NORMALIZAMOS LAS SEÑALES
IMG_MATCH = (IMG_MATCH/max(abs(IMG_MATCH)));
AUD_MATCH = (AUD_MATCH/max(abs(AUD_MATCH)));

%% CLUSTERING IMG
%% Cuantificamos la señal
vint_IMG = [0 0];
Simbolos= 1000;
%%
for i =  1:  (ceil(numel(IMG_MATCH)/(Simbolos*mp_IMG)))
    i;
    %%% Creamos las matrices para los valores del tren de pulsos del tamaño correcto
    if (i*mp_IMG*Simbolos) < numel(IMG_MATCH)
        Matriz_IMG = reshape(abs(IMG_MATCH(i*mp_IMG*Simbolos-mp_IMG*Simbolos+1:i*mp_IMG*Simbolos)),[mp_IMG,Simbolos]);
    else
        fin = floor(numel(IMG_MATCH((i-1)*mp_IMG*Simbolos+1:end))/12);
        Matriz_IMG = reshape(abs(IMG_MATCH((i)*mp_IMG*Simbolos-mp_IMG*Simbolos+1:((i)*mp_IMG*Simbolos-mp_IMG*Simbolos+1)+(fin*12)-1)),[mp_IMG,fin]);
    end
    
    %   Creamos las matrices para los valores cuantizados.
    Matriz_Q_IMG = Matriz_IMG;

    Matriz_Q_IMG(Matriz_Q_IMG > 0.3 )= 1; 
    Matriz_Q_IMG(Matriz_Q_IMG <= 0.3 ) = 0; 

    % Calcular Cluster Variance.
    cv_IMG = var(Matriz_Q_IMG - Matriz_IMG, 0, 2);
    k = find(cv_IMG == (min(cv_IMG)));

    vint_IMG_SEC = Matriz_Q_IMG(k:mp_IMG:end);

    vint_IMG = [vint_IMG vint_IMG_SEC];
end
%% Aqui comienza el vector
k2 = strfind(vint_IMG,header);
vint_IMG = vint_IMG(k2:end);

%% CLUSTERING AUDIO
%% Cuantificamos la señal
vint_AUD = [0 0];
Simbolos= 1000;
%% para sacar la cantidad de veces que se repite el ciclo for se necesita dividir el numero total de simbolos entre el tamaño del bloque
for i =  1:  (ceil(numel(AUD_MATCH)/(Simbolos*mp_SOUND)))
    
    %% Creamos las matrices para los valores del tren de pulsos del tamaño correcto
    % checamos si el final del bloque no excede el tamaño de la señal de audio match
    if (i*mp_SOUND*Simbolos) < numel(AUD_MATCH)
        Matriz_AUD = reshape(abs(AUD_MATCH((i-1)*mp_SOUND*Simbolos+1:i*mp_SOUND*Simbolos)),[mp_SOUND,Simbolos]);
    else
        %%
        fin = floor(numel(AUD_MATCH((i-1)*mp_SOUND*Simbolos+1:end))/mp_SOUND);
        %%
        Matriz_AUD = reshape(abs(AUD_MATCH((i)*mp_SOUND*Simbolos-mp_SOUND*Simbolos+1:((i)*mp_SOUND*Simbolos-mp_SOUND*Simbolos+1)+(fin*mp_SOUND)-1)),[mp_SOUND,fin]);
        %%
    end
    
    %%   Creamos las matrices para los valores cuantizados.
    Matriz_Q_AUD = Matriz_AUD;

    Matriz_Q_AUD(Matriz_Q_AUD > 0.3 )= 1; 
    Matriz_Q_AUD(Matriz_Q_AUD <= 0.3 ) = 0; 

    % Calcular Cluster Variance.
    cv_aud = var(Matriz_Q_AUD - Matriz_AUD, 0, 2);
    k = find(cv_aud == (min(cv_aud)));

    vint_AUD_SEC = Matriz_Q_AUD(k:mp_SOUND:end);

    vint_AUD = [vint_AUD vint_AUD_SEC];
    %%
end
%% Aqui comienza el vector
k2 = min(strfind(vint_AUD,header));
%%
%% RECONSTRUIMOS LA SEÑAL DE AUDIO
Audio_HEADER = 8;
vint_AUD = vint_AUD(k2+Audio_HEADER:end)

%% GRABAMOS LA SEÑAL
fileID = fopen('AUDIO REC.opus','w');
fwrite(fileID,vint_AUD(1:871872)','ubit1');
fclose(fileID);


%% RECONSTRUIMOS LA SEÑAL DE IMAGEN
S_Header = 40;
vint_IMG = vint_IMG(S_Header+1:8*512*512+(S_Header));
%%
MSignal = reshape(vint_IMG',8,512*512);
MSignal = MSignal';
%%
VMS = bi2de(MSignal,'left-msb');
%%
LenaRecSignal = reshape(VMS,512,512);
LenaRecSignal = cast(LenaRecSignal,'uint8');
imshow(LenaRecSignal)
title('Lena Reconstruida');
%% ERRORES IMAGEN
lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
LenaMat = de2bi(lena,8,'left-msb');     %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
LenaMat = LenaMat';                     %% invertimos la matriz
Bits_Lena = LenaMat(:);                      %% La guardamos como un vector de bits
Bits_Lena = cast(Bits_Lena,'double');   

ERROR_IMG = sum(abs(vint_IMG-Bits_Lena'))/2;
ERROR_IMG_PORCENTUAL = ERROR_IMG/numel(Bits_Lena(:,1));
%% gsdfg
ERROR_AUD = sum(abs(vint_AUD-DATA_Sound(1:end-112)'));
ERROR_AUD_PORCENTUAL = ERROR_AUD/numel(DATA_Sound(:,1));