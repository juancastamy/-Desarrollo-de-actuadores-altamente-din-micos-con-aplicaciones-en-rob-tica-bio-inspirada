clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 
S = serialport('COM7', 115200);
count = 1;
s=read(S,1,'uint8');
tiempo = 5000;
p = tiempo-1;
while(1)
    write(S,1,'char');
    n = read(S,1,'char')
    if n == '1'   
        data(count,:)=read(S,8,'uint8');
        n=0;        
        count = count+1
    end
    
    if count==tiempo
        for i=1:p
            data_real(i,:)=data(i,1)*2^24+data(i,2)*2^16+data(i,3)*2^8+data(i,4);
            data_real2(i,:)=data(i,5)*2^24+data(i,6)*2^16+data(i,7)*2^8+data(i,8);
        end
        figure(1);clf;
        hold on;
        plot(data_real);

        plot(data_real2);
        legend({'referencia','motor'},'Location','northeast')
        break;
    end
end

