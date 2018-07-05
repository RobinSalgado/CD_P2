%% Practica #1
%% Paso 1
%%% 1.	Conectar la salida de audio a la entrada de audio de otra computadora y transmitir una señal sinusoidal
%%%% utilizar una frecuencia de muestreo fs = 96KHz y 16 bits por muestra
%%%% genere una señal sinodal de amplitud 1 , frecuencia igual a 5,000 Hz y duración un segundo
close all; clc; clear all;      %% Borramos todo
fs = 96e3;
mp = 16;
Amp=1
f=5000;
T=1
t=0:1/fs:T;
y=sin(2*pi*f*t);
plot(t,y);
soundsc(y,fs);

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
y=chirp(t,500,2,20e3);              %% Generamos una señal chirp de dos segundos con frecuencia de 500 a 20KHz
pwelch(y,[],[],[],fs,'power');      %% Analisis con pwelch

soundsc(y,fs);                      %% Reproducimos

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
fm = 500;           %% Frecuencia maxima
B = 0.5;            %% Beta del raised cosine
mp = 12;            %% muestras por bit
Rb = 2*fm/(1+B);    %%bit Rate
Rb = (Fs/mp);       %%Bit Rate redondeado para que el raised cosine funcione
E=1/Rb;             %%Energia
Tp=1/Rb;            %%Periodo de bit
Ts=1/Fs;            %%intervalo de muestreo
type='rc';          %% Tipo de pulso
[Prc t]=rcpulse(B,D,Tp,Ts,type,E);      %% Generamos el pulso Raised Cosine
stem(Prc)
%% b) BIPOLAR NRZ AMI
%%%1.- Utilizar la imagen de Lena recortada (vector de bits) concatenada con un “header” 
%%% (tamaño de la imagen: ancho y alto), y a partir de ellos, generar una señal AMI utilizando el pulso base.
load lena512.mat
%lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
%lenarec=lena(252:283,319:350);          %% Recortamos la imagen para que tenga 32x32 pixeles
lenarec=lena512(252:284,319:351);  
b = de2bi(lenarec,8,'left-msb');        %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
b = b';                                 %% invertimos la matriz
bits = b(:);                            %% La guardamos como un vector de bits

%%%%%%
%%% AGREGAMOS EL HEADER

[w h]=size(lenarec);
header = [de2bi(85,8,'left-msb') de2bi(w,8,'left-msb') de2bi(h,8,'left-msb')];
%%
header = cast(header,'uint8')
bits = [header, bits'];
%%
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

    LENA_AMI= conv(s,Prc);                           %%Signal Pulse Train
%% 3.-	Añada a la señal un segundo de silencio al inicio 
%%% y un encabezado de 8 bits con el dato hexadecimal igual a 81h. 
%%%LENA_AMI = [header, dec2bin(85),LENA_AMI]            Falta concatenar el
%%%header
%%%bits = [header bitslena]
%%%    = [85d headerlena bitslena]


%% 4.	Graficar el diagrama de ojo de la señal a transmitir, 
%%% así como su densidad espectral de potencia (ignorando el primer segundo de silencio). 

pwelch(LENA_AMI,[],[],[],Fs,'power');
mp = 12;
ed = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',mp)
show(ed);
ed(LENA_AMI');                      %% La señal tiene que ser un vector columna

%% 5.	Transmitir la señal y verificar la transmisión (tren de pulsos) en el osciloscopio (dominio del tiempo y frecuencia) 
fs = 96e3;
soundsc([zeros(1,96000),LENA_AMI,zeros(1,96000)],fs);
%%6.	Capturar la señal en la PC receptora y eliminar, de la señal recibida, la parte que corresponde al primer segundo de silencio. 
%%7.	Graficar el diagrama de ojo de la señal recibida, y verificar que el ojo esté abierto. Graficar también su densidad espectral de potencia. 


%% 8.	Repita los puntos (1?7) pero utilice como pulso formador el Square Root Raised Cosine (SRRC) con B =  0.5 
%%SSRC
%%% Suponga D = 10 y fs = 8192 
Fs  = 8192;            %% Frecuencia de muestreo
D   = 10;               %% D = duración del pulso como número de intervalos de símbolo
fm  = 500;           %% Frecuencia maxima
B   = 0.5;            %% Beta del raised cosine
mp  = 12;            %% muestras por bit
Rb  = 2*fm/(1+B);    %%bit Rate
Rb  = (Fs/mp);       %%Bit Rate redondeado para que el raised cosine funcione
E   = 1/Rb;             %%Energia
Tp  = 1/Rb;            %%Periodo de bit
Ts  = 1/Fs;            %%intervalo de muestreo
type = 'srrc';
[Psrrc t] = rcpulse(B,D,Tp,Ts,type,E);  %% Pulso formador SRRC

%% b) BIPOLAR NRZ AMI
%%%1.- Utilizar la imagen de Lena recortada (vector de bits) concatenada con un “header” 
%%% (tamaño de la imagen: ancho y alto), y a partir de ellos, generar una señal AMI utilizando el pulso base.

%lena = imread('lena512.bmp');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
lenarec=lena512(252:284,319:351);  
%lenarec =lena(252:283,319:350);          %% Recortamos la imagen para que tenga 32x32 pixeles
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

LENA_AMI= conv(s,Psrrc);                           %%Signal Pulse Train


%% 8.4.	Graficar el diagrama de ojo de la señal a transmitir, 
%%% así como su densidad espectral de potencia (ignorando el primer segundo de silencio). 

pwelch(LENA_AMI,[],[],[],Fs,'power');
mp = 12;
ed = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',mp)
show(ed);
ed(LENA_AMI');                      %% La señal tiene que ser un vector columna

%% 8.5.	Transmitir la señal y verificar la transmisión (tren de pulsos) en el osciloscopio (dominio del tiempo y frecuencia) 
fs = 96e3;
soundsc([zeros(1,fs),LENA_AMI,zeros(1,fs)],fs);
%%6.	Capturar la señal en la PC receptora y eliminar, de la señal recibida, la parte que corresponde al primer segundo de silencio. 
%%7.	Graficar el diagrama de ojo de la señal recibida, y verificar que el ojo esté abierto. Graficar también su densidad espectral de potencia. 

%%%%%%%%%%%%%%%%%%%%%%%%%
%% PARTE #3
%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc;      %% Limpiamos las variables. la consola  y cerramos ventanas
t = .01; 
Fc = 10000; 
Fs = 80000; 
t = [0:1/Fs:0.01]'; 
s = sin(2*pi*300*t)+2*sin(2*pi*600*t); % Original signal 
[num,den] = butter(10,Fc*2/Fs); % Lowpass filter (LPF) 
sam = ammod(s,Fc,Fs); % Modulate. 
s1 = amdemod(sam,Fc,Fs,0,0,num,den); % Demodulate. % Observe las siguientes gráficas 
plot(t,s); hold on 
plot(t,sam);  

%% 2.
%% a. Intentaremos transmitir en una banda de 10 kHz, entre 1 kHz y 11 kHz 
%%% (es decir, la frecuencia de la  portadora es 6 kHz). 
%%% Diseñe una señal con pulsos coseno alzado con  B = 0.5  y Fm =  5KHz 
%%% Utilizando AM tipo DSB?SC, module la señal para que quede centrada en 6 kHz. 
%%% Obtenga su espectro  y verifique que sea como se espera. Utilice Fs=48KHz. 

%% Generando la señal
%%% SEÑAL
load lena512.mat
%lena = imread('lena512.mat');           %% Leemos el archido de la imagen lena y guardamos en la variable lena
lenarec=lena512(252:284,319:351);          %% Recortamos la imagen para que tenga 32x32 pixeles
b = de2bi(lenarec,8,'left-msb');        %% Transdormamos la imagen a una matriz de bits. cada valor se representa con 8 bits
b = b';                                 %% invertimos la matriz
bits = b(:);                            %% La guardamos como un vector de bits
b = cast(bits,'double');                %% transdormamos la matriz b en un vector de bits tipo double
%lenarec=lena512(252:284,318:350);

%%% PULSO RC
%%
 %debido a que mp dió 7.2 lo tuvimos que redondear hacia arriba y volver a 
 % calcular rb. obteniendo una nueva frecuencia máxima de 4500 hz con
 % wvtool(p)
 
B = 10e3; %señal en banda base
fm = 5e3; %frecuencia máxima
beta = 0.5; 
D  = 10;
Rb = 2*fm/(1+beta);% bit rate
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
 
%%% TREN DE PULSOS AMI
%%%%%%
%%% AGREGAMOS EL HEADER
%%
[w h]=size(lenarec);
header = [de2bi(85,8,'left-msb') de2bi(w,8,'left-msb') de2bi(h,8,'left-msb')];
bits = [header bits'];

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

%%
vector = 8736;                      %%numel(b) value
s = zeros(1,vector*mp);             %% un vector de tamaño b*mp
s(1:mp:end) = b;                    %% cada mp bits se agrega el valor de b

Signal= conv(s,Prc);                           %%Signal Pulse Train

%% Modulando la señal
Fc = 6000;                                  %% Frecuencia centrada en 
sam = ammod(Signal,Fc,fs); % Modulate.      %% Modulacion de la señal
pwelch(sam,[],[],[],fs,'power');            %% Espectro de frecuencias.

%% ENVIAR
%fs = 96000;
soundsc([zeros(1,fs) sam zeros(1,fs)],fs);
%% b. Transmita y reciba la señal. En el receptor, fíltrela con un filtro pasabanda en la banda de la señal. 
%% Rrecibir
%%
fs=48e3;
mpbits=16;
nChannels = 1;
recObj=audiorecorder(fs,mpbits,nChannels);
disp('Grabando');
record(recObj,3);

%utilizado para resp. en frecuencia del canal 
% figure(5);pwelch(mydata,[],[],[],fs,'power');

% stop(recObj);
% disp('Fin de la grabacion');
%% recuperacion de los datos
play(recObj);
mydata=getaudiodata(recObj);
figure(10);
plot(mydata,'r');title('RC modulated DSB-sc ');grid on;

%% demodulamos
Fs = 48e3;
[num,den] = butter(10,Fc*2/Fs); % Lowpass filter (LPF) 
s1 = amdemod(mydata,Fc,Fs,0,0,num,den); % Demodulate. % Observe las siguientes gráficas 

figure(2);pwelch(s1_matched,[],[],[],fs,'power'); % OBSERVAMOS el espectro de frecuencia de la señal demodulada

%%
ed_5k = comm.EyeDiagram('SampleRate',Fs,'SamplesPerSymbol',mp)
show(ed_5k)
ed_5k(s1)