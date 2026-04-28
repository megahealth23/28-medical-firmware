% %  BP analyse:singe file input:ECG data analyse
clear;
clc;
close all;
a=[];
b=[];
c=[];
A = [];
pi_red = [];
pi_infra = [];
whichversion = 2;

xlableformaxstart =0;
data = 100000*100*20*60*100; %100Hz * 20 point * 60s *50min
t = fopen('D:\mdate\Megaring_BPV8\RawDataServer\Matlab_code\testMring_eegfingerandabdomen.txt','r');%
wave = fread(t,data);
len = length(wave);

fclose(t);
framcnt =1;datano=1;
fsample = 100;
errcnt=1;
% c)下标从0开始：raw_data[8] - raw_data[x]:有效数据的字段，其中x = 8 + (val(raw_data[6]) *val(raw_data[7])-1)
% 此处，：raw_data[1]=96/测试类型;raw_data[2]=1；raw_data[3-6]=包序号；raw_data[7]：包个数；
for i =1:(len-20) %
    if(wave(i)==96 && wave(i+1)==1)%96+1
        framecount = wave(i+6);
        tmpbuf = wave(i+7:(i+7+framecount*6-1));
        framcnt=1;
        while(framcnt < framecount*6)
%             ch1(datano) = tmpbuf(framcnt)*2^16+tmpbuf(framcnt+1)*2^8+tmpbuf(framcnt+2);
%             ch2(datano) = tmpbuf(framcnt+3)*2^16+ tmpbuf(framcnt+4)*2^8+tmpbuf(framcnt+5);
            ch1(datano) = afe_convert_sensor_data(tmpbuf(framcnt), tmpbuf(framcnt+1), tmpbuf(framcnt+2));
            ch2(datano) = afe_convert_sensor_data(tmpbuf(framcnt+3), tmpbuf(framcnt+4), tmpbuf(framcnt+5));
            datano = datano + 1;
            framcnt = framcnt + 6;
        end
        i = i + 1;%244 1包；
    else
        errcnt = errcnt+1;
    end
end


figure;subplot(211);plot(ch1,'r');subplot(212);plot(ch2,'r')
