%clear all;
%% CONFIGURACION PARA HABILITAR LA COMUNICACION SERIAL 

proceso = 3;
if proceso == 1
    S = serialport('COM8', 115200);
    count = 1;
    s=read(S,1,'uint8');
    tiempo = 10000;
    p = tiempo-1;
    dt = 0.001;
    t0 = 0;
    tf = 9.99;
    k=9992;
    t = t0:0.001:tf;
    t1=t';
    while(1)
        write(S,1,'char');
        n = read(S,1,'char')
        if n == '1'   
            data(count,:)=read(S,20,'uint8');
            n=0;        
            count = count+1
        end
%{
        if count==tiempo
            for i=1:(k-1)
                pulso(i,:)=(data(i,1)*2^24+data(i,2)*2^16+data(i,3)*2^8+data(i,4))*pi/180;
                posicion(i,:)=(data(i,5)*2^24+data(i,6)*2^16+data(i,7)*2^8+data(i,8))*pi/180;
                pulso_velocidad(i,:)=data(i,9)*2^24+data(i,10)*2^16+data(i,11)*2^8+data(i,12);
                velocidad(i,:)=data(i,13)*2^24+data(i,14)*2^16+data(i,15)*2^8+data(i,16);
                corriente(i,:)=(data(i,17)*2^24+data(i,18)*2^16+data(i,19)*2^8+data(i,20));
            end
            %PULSO = pulso*12/4095;
            CORRIENTE = corriente/1000;
            figure(1);clf;
            hold on;
            plot(t1,pulso');

            plot(t1,posicion')

            legend({'Ref posición','Posición motor'},'Location','northeast','FontSize',15);

            figure(2);clf;
            hold on;
            plot(t1,velocidad');

            legend({'Velocidad motor'},'Location','northwest','FontSize',15);
            figure(3);clf;
            hold on;
            
            plot(t1,CORRIENTE');
            

            legend({'corriente medida'},'Location','northwest','FontSize',15);
            
            figure(4);clf;
            hold on;
            plot(t1,pulso');
            plot(t1,posicion');
            plot(t1,CORRIENTE');
          %  plot(t1,velocidad');

            legend({'pulso','Posición','Corriente'},'Location','northeast','FontSize',15);

            break;
        
        end
        %}
    end
    %save('prueba_LQI.mat')
elseif proceso == 2

    load('Valores_impulso_con_carga.mat')


    dt = 0.01;
    t0 = 0;
    tf = 10;
    k=(tf-t0)/dt;
    t = t0:0.01:tf;
    t1=t';
    p=1;
    for n = 1:length(t1)
        ref(:,n) = (pulso(n))/4095*12;
        cor(:,n) = corriente(n);
        vel(:,n) = velocidad(n);
        if (n == 1)
            pos(:,n) = posicion(n);
        else
            if(posicion(n-1) > posicion(n))
                p = n-1;
                pos(:,n) = posicion(n)+pos(p);
            else
                pos(:,n) = posicion(n)+pos(p);
            end
        end
    end
    save('graficas_para_LQI.mat','pos','vel','cor');
    figure(1);clf;
    hold on;
    plot(t1,cor);

    plot(t1,vel);
    plot(t1,pos);

    legend({'Ref posición','vel','Posición motor'},'Location','northeast','FontSize',15);
    save('Valores_para_optimizacion2.mat','pos','vel','cor','ref');
    
else 
  % load('data_prueba_LQI','data','k','t1');
%   k=16693
%   t0 = 0;
%   tf = 16.691;
%    t = t0:0.001:tf;
%     t1=t';
   for i=1:(k-1)
        pulso(i,:)=(data(i,1)*2^24+data(i,2)*2^16+data(i,3)*2^8+data(i,4))*pi/180;
        posicion(i,:)=(data(i,5)*2^24+data(i,6)*2^16+data(i,7)*2^8+data(i,8))*pi/180;
        pulso_velocidad(i,:)=data(i,9)*2^24+data(i,10)*2^16+data(i,11)*2^8+data(i,12);
        velocidad(i,:)=data(i,13)*2^24+data(i,14)*2^16+data(i,15)*2^8+data(i,16);
        corriente(i,:)=(data(i,17)*2^24+data(i,18)*2^16+data(i,19)*2^8+data(i,20));
   end
  
    PULSO = pulso*12/4095;
    CORRIENTE = corriente/1000;
    figure(1);clf;
    hold on;
    plot(t1,pulso');

    plot(t1,posicion')

    legend({'Ref posición','Posición motor'},'Location','northeast','FontSize',15);

    figure(2);clf;
    hold on;
    plot(t1,velocidad');

    legend({'Ref velocidad','Velocidad motor'},'Location','northwest','FontSize',15);
    figure(3);clf;
    hold on;
    plot(t1,CORRIENTE');
    legend({'corriente medida'},'Location','northwest','FontSize',15);

    figure(4);clf;
    hold on;
    plot(t1,PULSO');

    plot(t1,posicion');
    plot(t1,CORRIENTE');
    plot(t1,velocidad');

    legend({'pulso','Posición','Corriente','Velocidad'},'Location','northeast','FontSize',15);
end

