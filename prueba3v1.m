%% PRACTICA #3
%% Paso #1
close all; clc; clear all;      %% Limpiamos las variables, terminal y cerramos ventanas
%% Generando la señal
%% Cargamos el archivo de imagen.
lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
LenaMat = de2bi(lena,8,'left-msb');     %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
LenaMat = LenaMat';                     %% invertimos la matriz
Bits_Lena = LenaMat(:);                      %% La guardamos como un vector de bits
Bits_Lena = cast(Bits_Lena,'double');                %% transdormamos la matriz b en un vector de bits tipo double
%%% AGREGAMOS EL HEADER
[w,h]=size(lena);

header = [de2bi(85,8,'left-msb') de2bi(w,16,'left-msb') de2bi(h,16,'left-msb')];
Bits_Lena = [header Bits_Lena'];

%% Cargamos el archivo de audio
%fileID = fopen('SongOfStorms.wav');
Fs = 44100;
%%samples = [69*Fs,129*Fs];
fileID = fopen('Seven Nation Army.opus');
DATA_Sound = fread(fileID,'ubit1');

%%% AGREGAMOS EL HEADER de AUDIO
header = de2bi(85,8,'left-msb');
DATA_BIN = [header DATA_Sound(:)'];
%% PULSO SRRC
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
%% TREN DE PULSOS AMI
%%% Tren de pulsos para Sonido
DATA_BIN = cast(DATA_BIN,'double');                %% transdormamos la matriz b en un vector de bits tipo double
pulse = 1;                               %% valor para AMI comenzando en -1
for count = 1:numel(DATA_BIN)                  %% Pasa por todos los valores de b
    if DATA_BIN(count)==1                      %% Si el valor de b es == 1
        if pulse == -1                  %% checa el valor de la variable pulso
            pulse = 1;                  %% cambia el valor de pulse
            DATA_BIN(count)= -1;               %% Cambia el valor de b a -1
        else
            pulse = -1;                 %% cambia el valor de pulse a -1
            DATA_BIN(count)= 1;                %% Cambia el valor de b a 1
        end
    end
end
vector = numel(DATA_BIN);                          %%numel(b) value
s_AUD = zeros(1,vector*mp_SOUND);                     %% un vector de tamaño b*mp
s_AUD(1:mp_SOUND:end) = DATA_BIN;                    %% cada mp bits se agrega el valor de b
%% Tren de pulsos AMI para Imagen
Bits_Lena = cast(Bits_Lena,'double');                %% transdormamos la matriz b en un vector de bits tipo double
pulse = 1;                               %% valor para AMI comenzando en -1
for count = 1:numel(Bits_Lena)                  %% Pasa por todos los valores de b
    if Bits_Lena(count)==1                      %% Si el valor de b es == 1
        if pulse == -1                  %% checa el valor de la variable pulso
            pulse = 1;                  %% cambia el valor de pulse
            Bits_Lena(count)= -1;               %% Cambia el valor de b a -1
        else
            pulse = -1;                 %% cambia el valor de pulse a -1
            Bits_Lena(count)= 1;                %% Cambia el valor de b a 1
        end
    end
end
vector = numel(Bits_Lena);                          %%numel(b) value
s_IMG = zeros(1,vector*mp_IMG);                     %% un vector de tamaño b*mp
s_IMG(1:mp_IMG:end) = Bits_Lena;                    %% cada mp bits se agrega el valor de b
%% TREN DE PULSOS

Signal_SOUND = conv(P_SOUND,s_AUD); 
Signal_IMG = conv(P_IMG,s_IMG);

%% MODULACION DE LA SEÑAL CON LARGE CARRIER
%% MODULADA CON PORTADORA
Fc_S = 7500;
Fc_I = 17500;
CARRAMP_SOUND = max(abs(Signal_SOUND)); %Carrier Amplitude
SOUND_samLC = ammod(Signal_SOUND,Fc_S,Fs,0,CARRAMP_SOUND*.95); % AM-DSB-LC Se multiplica por .95 para no caer en sobremodulación

CARRAMP_IMG = max(abs(Signal_IMG)); %Carrier Amplitude
IMG_samLC = ammod(Signal_IMG,Fc_I,Fs,0,CARRAMP_IMG*.95); % AM-DSB-LC Se multiplica por .95 para no caer en sobremodulación
%%%%%% OFDM
%%
pwelch((IMG_samLC(1:10000)+SOUND_samLC(1:10000)),[],[],[],Fs,'power');            %% Espectro de frecuencias.
%% Transmición
%%Total_Signal = SOUND_samLC  +  [IMG_samLC  zeros(1,numel(SOUND_samLC) - numel(IMG_samLC))]; %% Sumamos las señales
Total_Signal = IMG_samLC  +  [SOUND_samLC  zeros(1,numel(IMG_samLC) - numel(SOUND_samLC))]; %% Sumamos las señales

%Total_Signal2 = SOUND_samLC;
%soundsc([zeros(1,Fs) Total_Signal zeros(1,Fs)],Fs);
