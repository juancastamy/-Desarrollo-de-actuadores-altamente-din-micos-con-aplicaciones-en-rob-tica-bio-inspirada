clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 
S = serialport("COM7",115200);
while(1)
   
    n = read(S,1,'char')
        if n == '1'
            write(S,1,'char');
            data=read(S,4,'uint8')
            n=0;
            data_real=bitconcat(fi(data,0,8))
        end
end

