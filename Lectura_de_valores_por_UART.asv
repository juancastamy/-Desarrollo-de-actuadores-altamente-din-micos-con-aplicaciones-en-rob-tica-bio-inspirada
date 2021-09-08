clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 
S = serialport("COM7",115200);
count = 1;
while(1)
    write(S,0,'char');
    n = read(S,1,'char')
    if n == '1'
        
        write(S,1,'char');
        %write(S,180,'char');
        
        if count < 1000
            write(S,180,'char');
            
        elseif count>=1000 & count<2000
            write(S,270,'char');
        elseif count>=2000 & count<3000
            write(S,180,'char');
        end
        
        data(count,:)=read(S,4,'uint8');
        data2(count,:)=read(S,4,'uint8');
        n=0;
        data_real(count,:)=data(count,1)*2^24+data(count,2)*2^16+data(count,3)*2^8+data(count,4);
        data_real2(count,:)=data2(count,1)*2^24+data2(count,2)*2^16+data2(count,3)*2^8+data2(count,4);
        
        count = count+1;
    end
    
    if count==3000
        figure(1);clf;
        hold on;
        plot(data_real);

        plot(data_real2);
        legend({'referencia','motor'},'Location','northeast')
        break;
    end
end

