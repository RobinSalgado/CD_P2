%% Practica #1
%% Paso 1
%%% 1.	Conectar la salida de audio a la entrada de audio de otra computadora y transmitir una señal sinusoidal
%%%% utilizar una frecuencia de muestreo fs = 96KHz y 16 bits por muestra
%%%% genere una señal sinodal de amplitud 1 , frecuencia igual a 5,000 Hz y duración un segundo

close all; clc; clear all;      %% Borramos todo
fs = 96e3;                  %%Frecuencia de muestreo de 96KHz
mp = 16;                    %%Bits por muestra
Amp=1;                      %%Amplitud
f=5000;                     %%Frecuencia de la sinodal a 5kHz
T=1;                        %%Tiempo total de la señal
t=0:1/fs:T;                 %%Vector de tiempo
Senoidal=sin(2*pi*f*t);     %%Generamos la señal Senoidal
plot(t,Senoidal);           %%Graficamos la señal Senoidal 
soundsc(Senoidal,fs);       %%Reproducimos el audio de la señal

%% Paso 2	Identificar el canal, es decir, obtener su respuesta en frecuencia. Vamos a emplear tres técnicas diferentes. 
%% a) impulso conformado por un segundo de 0 1 0 
Pulse = zeros(1,2*fs+1)             %% Creamos dos segundos de silencio con un vector de ceros
Pulse(fs) = 1;                      %% A la mitad del vector ponemos un solo pulso
soundsc(Pulse,fs);                  %%Reproducimos con una frecuencia de 96kHz

%% c)	Como segunda técnica, utilizaremos una señal que tiene el mismo espectro que el impulso: el ruido gaussiano
Ruido = randn(1,5*fs);              %% Generamos un vector de ruido gausiano de 5 segundos
pwelch(Ruido,[],[],[],fs,'power');  %% Lo analisamos con pwelch
%%soundsc(Ruido,fs);                %% Lo reproducimos
%% Genere una señal “chirp” (-1 volt a 1 volt) de frecuencias 500:500:20000. 
%%Basándose en el siguiente ejemplo:        
t=0:1/fs:2;                         %% Vector de tiempo de dos segundos con separacion de 1/fs 
Signal_Chirp=chirp(t,500,2,20e3);              %% Generamos una señal chirp de dos segundos con frecuencia de 500 a 20KHz
pwelch(Signal_Chirp,[],[],[],fs,'power');      %% Analisis con pwelch

soundsc(Signal_Chirp,fs);                      %% Reproducimos

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parte #2 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% En esta parte de la práctica realizaremos transmisión en banda base. 
%%% Usaremos los diagramas de ojo para evaluar el funcionamiento del sistema. 
%%% Recuerde utilizar los niveles de volumen y sensibilidad encontrados en la Parte I.
%% Suponga D = 10 y fs = 8192 
clear all; clc; close all;          %% Cerramos ventanas y borramos variables junto con consola
Fs=8192;            %% Frecuencia de muestreo
D=10;               %% D = duración del pulso como número de intervalos de símbolo
fm = 5000;           %% Frecuencia maxima
B = 0.5;            %% Beta del raised cosine
mp = 12;            %% muestras por bit
Rb = 2*fm/(1+B);    %%bit Rate
Rb = (Fs/mp);       %%Bit Rate redondeado para que el raised cosine funcione
E=1/Rb;             %%Energia
Tp=1/Rb;            %%Periodo de bit
Ts=1/Fs;            %%intervalo de muestreo
type='srrc';          %% Tipo de pulso
[Prc t]=rcpulse(B,D,Tp,Ts,type,E);      %% Generamos el pulso Raised Cosine
stem(Prc)
%% b) BIPOLAR NRZ AMI
%%%1.- Utilizar la imagen de Lena recortada (vector de bits) concatenada con un “header” 
%%% (tamaño de la imagen: ancho y alto), y a partir de ellos, generar una señal AMI utilizando el pulso base.

lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
lenarec=lena(252:283,319:350);          %% Recortamos la imagen para que tenga 32x32 pixeles
b = de2bi(lenarec,8,'left-msb');        %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
b = b';                                 %% invertimos la matriz
bits = b(:);                            %% La guardamos como un vector de bits

%%% AGREGAMOS EL HEADER
[w h]=size(lenarec);                    %% Guardamos los variables de lo ancho y lo alto
header = [de2bi(85,8,'left-msb') de2bi(w,8,'left-msb') de2bi(h,8,'left-msb')];  %%Concatenamos en un header
header = cast(header,'uint8')           %% Cambiamos el tipo de variable a entero por seguridad
bits = [header, bits'];                 %% Concatenamos
%%
b = cast(bits,'double');                %% transdormamos la matriz b en un vector de bits tipo double
B = 0.5;                                %% Beta = 0.5
pulse=1;                               %% valor para AMI comenzando en 1

for count = 1:numel(b)                  %% Pasa por todos los valores de b
    if b(count)==1                      %% Si el valor de b es == 1
        if pulse == -1                  %% checa el valor de la variable pulso
            pulse = 1;                  %% cambia el valor de pulse
            b(count)= -1;               %% Cambia el valor de b a -1
        else
            pulse = -1;                 %% cambia el valor de pulse a -1
            b(count)= 1;                %% Cambia el valor de b a 1
        end
    end
end

    s = zeros(1,numel(b)*mp);           %% un vector de tamaño b*mp
    s(1:mp:end) = b;                    %% cada mp bits se agrega el valor de b

    LENA_AMI= conv(s,Prc);                           %%Signal Pulse Train
%% 3.-	Añada a la señal un segundo de silencio al inicio 
%%% y un encabezado de 8 bits con el dato hexadecimal igual a 81h. 
%%%LENA_AMI = [header, dec2bin(85),LENA_AMI]            Falta concatenar el
%%%header
%%%bits = [header bitslena]
%%%    = [85d headerlena bitslena]


%% 4.	Graficar el diagrama de ojo de la señal a transmitir, 
%%% así como su densidad espectral de potencia (ignorando el primer segundo de silencio). 

pwelch(LENA_AMI,[],[],[],Fs,'power');       %%Graficamos su densidad espectral de potencia
mp = 12;
ed = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',mp); %%Creamos el diagrama de ojo
show(ed);                           
ed(LENA_AMI');                      %% La señal tiene que ser un vector columna

%% 5.	Transmitir la señal y verificar la transmisión (tren de pulsos) en el osciloscopio (dominio del tiempo y frecuencia) 
fs = 48e3;
soundsc(LENA_AMI,fs);
%%6.	Capturar la señal en la PC receptora y eliminar, de la señal recibida, la parte que corresponde al primer segundo de silencio. 
%%7.	Graficar el diagrama de ojo de la señal recibida, y verificar que el ojo esté abierto. Graficar también su densidad espectral de potencia. 


%% 8.	Repita los puntos (1?7) pero utilice como pulso formador el Square Root Raised Cosine (SRRC) con B =  0.5 
%%SSRC
%%% Suponga D = 10 y fs = 8192 
clear all; clc; close all;          %% Cerramos ventanas y borramos variables junto con consola
Fs  = 8192;             %% Frecuencia de muestreo
D   = 10;               %% D = duración del pulso como número de intervalos de símbolo
fm  = 5000;              %% Frecuencia maxima
B   = 0.5;              %% Beta del raised cosine
mp  = 12;               %% muestras por bit
Rb  = 2*fm/(1+B);       %%bit Rate
Rb  = (Fs/mp);          %%Bit Rate redondeado para que el raised cosine funcione
E   = 1/Rb;             %%Energia
Tp  = 1/Rb;             %%Periodo de bit
Ts  = 1/Fs;             %%intervalo de muestreo
type = 'srrc';
[Psrrc t] = rcpulse(B,D,Tp,Ts,type,E);  %% Pulso formador SRRC

%% b) BIPOLAR NRZ AMI
%%%1.- Utilizar la imagen de Lena recortada (vector de bits) concatenada con un “header” 
%%% (tamaño de la imagen: ancho y alto), y a partir de ellos, generar una señal AMI utilizando el pulso base.

lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
lenarec =lena(252:283,319:350);          %% Recortamos la imagen para que tenga 32x32 pixeles
b = de2bi(lenarec,8,'left-msb');        %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
b = b';                                 %% invertimos la matriz
bits = b(:);                            %% La guardamos como un vector de bits
%%%%%%
%%% AGREGAMOS EL HEADER

[w h]=size(lenarec);
header = [de2bi(85,8,'left-msb') de2bi(w,8,'left-msb') de2bi(h,8,'left-msb')];
header = cast(header,'uint8')
bits = [header, bits'];

b = cast(bits,'double');                %% transdormamos la matriz b en un vector de bits tipo double
B = 0.5;                                %% Beta = 0.5
pulse=-1;                               %% valor para AMI comenzando en -1

for count = 1:numel(b)                  %% Pasa por todos los valores de b
    if b(count)==1                      %% Si el valor de b es == 1
        if pulse == -1                  %% checa el valor de la variable pulso
            pulse = 1;                  %% cambia el valor de pulse
            b(count)= -1;               %% Cambia el valor de b a -1
        else
            pulse = -1;                 %% cambia el valor de pulse a -1
            b(count)= 1;                %% Cambia el valor de b a 1
        end
    end
end

s = zeros(1,numel(b)*mp);           %% un vector de tamaño b*mp
s(1:mp:end) = b;                    %% cada mp bits se agrega el valor de b

LENA_AMI_SRRC= conv(s,Psrrc);                           %%Signal Pulse Train


%% 8.4.	Graficar el diagrama de ojo de la señal a transmitir, 
%%% así como su densidad espectral de potencia (ignorando el primer segundo de silencio). 

pwelch(LENA_AMI_SRRC,[],[],[],Fs,'power');
mp = 12;
ed = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',mp)
show(ed);
ed(LENA_AMI_SRRC');                      %% La señal tiene que ser un vector columna

%% 8.5.	Transmitir la señal y verificar la transmisión (tren de pulsos) en el osciloscopio (dominio del tiempo y frecuencia) 
fs = 96e3;
soundsc([zeros(1,fs),LENA_AMI_SRRC,zeros(1,fs)],fs);
%%6.	Capturar la señal en la PC receptora y eliminar, de la señal recibida, la parte que corresponde al primer segundo de silencio. 
%%7.	Graficar el diagrama de ojo de la señal recibida, y verificar que el ojo esté abierto. Graficar también su densidad espectral de potencia. 

%%%%%%%%%%%%%%%%%%%%%%%%%
%% PARTE #3
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% EJEMPLO DE LA PRACTICA%%
close all; clear all; clc;      %% Limpiamos las variables. la consola  y cerramos ventanas
t = .01;                        %% Periodo de la señal
Fc = 10000;                     %% Frecuencia centrada en 10kHZ
Fs = 80000;                     %% Frecuencia de muestreo de 8Khz
t = [0:1/Fs:0.01]';             %% Vector de tiempo
s = sin(2*pi*300*t)+2*sin(2*pi*600*t); % Original signal 
[num,den] = butter(10,Fc*2/Fs); % Lowpass filter (LPF) 
sam = ammod(s,Fc,Fs);           % Modulate. 
s1 = amdemod(sam,Fc,Fs,0,0,num,den); % Demodulate. % Observe las siguientes gráficas 
plot(t,s); hold on              %%Graficamos
plot(t,sam);  

%% 2.
%% a. Intentaremos transmitir en una banda de 10 kHz, entre 1 kHz y 11 kHz 
%%% (es decir, la frecuencia de la  portadora es 6 kHz). 
%%% Diseñe una señal con pulsos coseno alzado con  B = 0.5  y Fm =  5KHz 
%%% Utilizando AM tipo DSB?SC, module la señal para que quede centrada en 6 kHz. 
%%% Obtenga su espectro  y verifique que sea como se espera. Utilice Fs=48KHz. 

%% Generando la señal
%%% SEÑAL
close all; clear all; clc;      %% Limpiamos las variables. la consola  y cerramos ventanas
lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
lenarec=lena(252:284,319:351);          %% Recortamos la imagen para que tenga 32x32 pixeles
b = de2bi(lenarec,8,'left-msb');        %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
b = b';                                 %% invertimos la matriz
bits = b(:);                            %% La guardamos como un vector de bits
b = cast(bits,'double');                %% transdormamos la matriz b en un vector de bits tipo double

%%% PULSO RC
%%
 %debido a que mp dió 7.2 lo tuvimos que redondear hacia arriba y volver a 
 % calcular rb. obteniendo una nueva frecuencia máxima de 4500 hz con
 % wvtool(p)
 
B = 10e3; %señal en banda base
fm = 5e2;
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
type = 'srrc';
[Prc, t] = rcpulse(beta,D,Tp,Ts,type,energy);
 
%%% TREN DE PULSOS AMI
%%%%%%
%%% AGREGAMOS EL HEADER

[w h]=size(lenarec);
header = [de2bi(85,8,'left-msb') de2bi(w,8,'left-msb') de2bi(h,8,'left-msb')];
bits = [header bits'];

b = cast(bits,'double');                %% transdormamos la matriz b en un vector de bits tipo double
B = 0.5;                                %% Beta = 0.5
pulse = 1;                               %% valor para AMI comenzando en -1

for count = 1:numel(b)                  %% Pasa por todos los valores de b
    if b(count)==1                      %% Si el valor de b es == 1
        if pulse == -1                  %% checa el valor de la variable pulso
            pulse = 1;                  %% cambia el valor de pulse
            b(count)= -1;               %% Cambia el valor de b a -1
        else
            pulse = -1;                 %% cambia el valor de pulse a -1
            b(count)= 1;                %% Cambia el valor de b a 1
        end
    end
end

%%
vector = 8736;                      %%numel(b) value
s = zeros(1,vector*mp);             %% un vector de tamaño b*mp
s(1:mp:end) = b;                    %% cada mp bits se agrega el valor de b

Signal= conv(s,Prc);                           %%Signal Pulse Train

%% Modulando la señal
Fc = 6000;                                  %% Frecuencia centrada en 
fs = 48e3;
sam = ammod(Signal,Fc,fs); % Modulate.      %% Modulacion de la señal
pwelch(sam,[],[],[],fs,'power');            %% Espectro de frecuencias.

%% ENVIAR
fs = 48000;
soundsc([zeros(1,fs) sam zeros(1,fs)],fs);

%% MODULADA CON PORTADORA
CARRAMP = max(abs(Signal)); %Carrier Amplitude
samLC = ammod(Signal,Fc,Fs,0,CARRAMP); % AM-DSB-LC
%Indice de modulacion
m = (max(samLC)-CARRAMP)/CARRAMP;
soundsc([zeros(1,fs) samLC zeros(1,fs)],fs);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%
%% Eliminamos el silencio
SIGNAL_RECEIVED = [ zeros(1,fs), sam zeros(1,fs)];
Comienzo = min(find(abs(SIGNAL_RECEIVED) > 0.5)-10);
Final = max(find(SIGNAL_RECEIVED > 0.5))+ 50;
SIGNAL = SIGNAL_RECEIVED(Comienzo:Final);

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
%%FINAL_SIGNAL = s1;
eyediagram(FINAL_SIGNAL(89:89+100*mp),3*mp);
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




