% %  real time plot:only for ptt
cnt = 0;
errcnt = 0;
data = [];
ch1 = [];
ch2 = [];
ch3 = [];
ch4 = [];
deta = [];
datano = 1;
figure;
fid = fopen('testMring.txt','w');%打开test.txt,并向里写内容；
%Accept a connection from any machine on port 30001.
%端口，30001
echotcpip('off');%开启或者关闭TCP/IP服务；
t=tcpip('0.0.0.0', 30001, 'NetworkRole', 'server');%TCP/IP端口为30001；任何远端器件可以连接；
pause(1);%暂停1s；至少0.01s;
bytetorec = 244; %20个Byte一次；
%Open a connection. This will not return until a connection is received.
fopen(t);
datatmp = [];
%Read the waveform and confirm it visually by plotting it.
SPO2 = 1;EHR = 2;PTT=3;
TYPE = PTT;
topcheck_ptt = 96;
while 1
data=fread(t, bytetorec, 'char');%读二进制数据；这里是1个Byte读一次；
%data = Y(startp:startp+19);
%startp = startp+20;
if length(data) ~= 0 % not equal 读到1Byte;
    %data
    fwrite(fid,data);%写入；
    cnt = cnt+1;
    datatmp = [datatmp; data(:)];%记录数据；矩阵变成向量，一列；
    [sidptt] = find(datatmp==topcheck_ptt);%0x60;标记索引；

    if length(sidptt) == 0
        continue;
    end
	
	
    if (length(sidptt) ~= 0)
        datatmp = datatmp(sidptt(1):end);%（1：end）
        if(length(datatmp) >= 244)%从5E开始有9个数；
            dnum = datatmp(1:244);
            datatmp = datatmp(245:end);%下一批数据，一包20Byte;
            if (dnum(1) == topcheck_ptt && dnum(2) == 1)
                framecount = dnum(7);
                tmpbuf = dnum(8:(8+framecount*6-1));
                framcnt = 1;
                while(framcnt < framecount*6)
                    ch1(datano) = afe_convert_sensor_data(tmpbuf(framcnt), tmpbuf(framcnt+1), tmpbuf(framcnt+2));
                    ch2(datano) = afe_convert_sensor_data(tmpbuf(framcnt+3), tmpbuf(framcnt+4), tmpbuf(framcnt+5));
                    datano = datano + 1;
                    framcnt = framcnt + 6;
                    if mod(datano,5) == 0 %mod,取余；
                        pause(0.0001);
                        subplot(2,1,1);axis([1 200 0 4000]);plot(ch1(max(1,datano-200):end));
                        subplot(2,1,2);axis([1 200 0 4000]);plot(ch2(max(1,datano-200):end));
                    end

                end
            else
                errcnt = errcnt+1;
            end            
        end
    end
	
	
	
	
	


    if (cnt > 0 && mod(cnt,10) == 0)
        fprintf('cnt = %d\n',cnt);
    end
end

    if length(data) == 0 % not equal 读到1Byte;
        break;
    end 
end
plot(data);
fclose(fid);
fclose(t);
delete(t);
clear t;




