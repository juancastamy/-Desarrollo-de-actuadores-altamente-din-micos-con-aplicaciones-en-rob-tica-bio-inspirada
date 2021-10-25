clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 
S = serialport('COM8', 115200);
count = 1;
s=read(S,1,'uint8');
tiempo = 5000;
p = tiempo-1;
dt = 0.01;
t0 = 0;
tf = 48.98;
k=(tf-t0)/dt;
t = t0:0.01:48.96;
t1=t';
while(1)
    write(S,1,'char');
    n = read(S,1,'char')
    if n == '1'   
        data(count,:)=read(S,16,'uint8');
        n=0;        
        count = count+1
    end
    
    if count==tiempo
        for i=1:(k-1)
            data_real(i,:)=data(i,1)*2^24+data(i,2)*2^16+data(i,3)*2^8+data(i,4);
            data_real2(i,:)=data(i,5)*2^24+data(i,6)*2^16+data(i,7)*2^8+data(i,8);
            data_real3(i,:)=data(i,9)*2^24+data(i,10)*2^16+data(i,11)*2^8+data(i,12);
            data_real4(i,:)=data(i,13)*2^24+data(i,14)*2^16+data(i,15)*2^8+data(i,16);
        end
        figure(1);clf;
        hold on;
        plot(t1,data_real');

        plot(t1,data_real2')
        
        legend({'Ref posición','Posición motor'},'Location','northeast','FontSize',15);
        
        figure(2);clf;
        hold on;
        plot(t1,data_real3');

        plot(t1,data_real4')
        
        legend({'Ref velocidad','Velocidad motor'},'Location','northwest','FontSize',15);
        break;
    end
end