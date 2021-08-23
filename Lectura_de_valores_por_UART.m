clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 
S = serialport("COM7",115200);
n = read(S,1,'char')
    if n == '1';
        write(S,1,'char');
        data=read(S,8,'char')
        n=0;
end

