function magPlot(com,baud)
    if(nargin<2)
        baud=57600;
    end
    if(nargin<1)
        com='COM38';
    end
    
    try
        global done;
        done=0;
        %open serial port
        ser=serial(com,'BaudRate',baud);
        set(ser,'Timeout',30);
        %open port
        fopen(ser);

        %print ^C to stop simulation
        fprintf(ser,03);
        %set to machine readable opperation
        %fprintf(ser,'output machine');
        %start reading data
        fprintf(ser,'mag');
        %burn three lines
        fgetl(ser);
        fgetl(ser);
        fgetl(ser)

        B0=[];
        B1=[];
        B2=[];
        T=[];

        hBf=figure(1);
        set(hBf,'CloseRequestFcn',@fig_close);
        hB=plot([0],[0],'b',[0],[0],'r',[0],[0],'g');
        xlabel('Time [s]');
        ylabel('Magnetic Filed Intensity [Gauss]');
        legend('X','Y','Z','Location','EastOutside');
        
        %only show 90 seconds of data
        maxlen=90;

        set(ser,'Timeout',15);

        while ~done
           line=fgetl(ser);
           dat=sscanf(line,'%i %i');
           if(length(dat)~=2)
               fprintf('Could not parse \"%s\"\n',line(1:end-1));
               break;
           end
           B0=[B0,dat(1)];
           B1=[B1,dat(2)];
           B2=[B2,0];
           if isempty(T)
               T=[0];
           else
               T=[T,T(end)+10];
           end

           %Change Field plot data
           set(0,'CurrentFigure',hBf);
           set(hB(1),'XData',T,'YData',B0);
           set(hB(2),'XData',T,'YData',B1);
           set(hB(3),'XData',T,'YData',B2);
           axis('tight');
           drawnow;
        end
    catch err
        if exist('ser','var')
            fclose(ser);
            delete(ser);
        end
        if exist('hBf','var')
            delete(hBf);
        end
        rethrow(err);
    end
        if exist('ser','var')
            fclose(ser);
            delete(ser);
        end
        if exist('hBf','var')
            delete(hBf);
        end
    clear done;
end

%handle a closed figure
%function fig_close(src,evnt)
function fig_close(~,~)
    global done;
    done=1;
    clear done;
end
