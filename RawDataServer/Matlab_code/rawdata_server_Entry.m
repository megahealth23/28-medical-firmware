% @author: litianzheng
%%%%
HOST = '0.0.0.0';
PORT = 30001;

%%%%%%   %%%%%%%
RUNNING_MODE_NONE = 0;
RUNNING_MODE_FILE = 1;
RUNNING_MODE_SERVER = 2;
running_type = RUNNING_MODE_SERVER;

if( RUNNING_MODE_NONE == running_type )
    disp("未选择模式,退出....");
    return;
end

log_fid = 0;
if( RUNNING_MODE_FILE == running_type )
    [fileName,pathName] = uigetfile('*.dat','Please select an data file');%file selection 
    abs_filename = sprintf("%s%s",pathName, fileName);
    log_fid = fopen(abs_filename);
    disp(["打开文件", abs_filename]);
end

tcp_obj = 0;
raw_fid = 0;
rawdata_filename = ['.\dat\ble_rawdata-', datestr(now,'yyyy-mm-dd-HH-MM-SS'), '.dat'];
if( RUNNING_MODE_SERVER == running_type )
    [~, res_cmd___]=system('ipconfig');
    % res_cmd__=regexp(res_cmd___,'IP Address. . . . . . . . . . . . : \d{1,3}.\d{1,3}.\d{1,3}.\d{1,3}','match')
    res_cmd__=regexp(res_cmd___,'IPv4 地址 . . . . . . . . . . . . : \d{1,3}.\d{1,3}.\d{1,3}.\d{1,3}','match');
    [res_row, res_col] = size(res_cmd__);
    if (0 == res_row || 0 == res_col)
        disp("未连接网络,退出....");
        return;
    end
    res_cmd_=res_cmd__{1};
    res_cmd=regexp(res_cmd_,'\d{1,3}.\d{1,3}.\d{1,3}.\d{1,3}','match');
    IP_ADDRESS=res_cmd{1};
    
    raw_fid=fopen(rawdata_filename,'w');%写入文件路径
    fprintf("文件保存位置：%s", rawdata_filename);
    tcp_obj = tcpip(HOST, PORT, 'NetworkRole', 'server');%TCP/IP端口为30001；任何远端器件可以连接；
    fprintf("服务器(%s)启动，监听端口%d...\n服务器正在运行，请连接到同一局域网\n等待客户端连接...\n", IP_ADDRESS, PORT);
    fopen(tcp_obj);
end

%%%%  global variable %%%%
start_flag = 0; 
start_idx = 0; 
stop_idx = 0; 
p_index = 0;
pk_cnt = 0;
res_cnt = 0;
data = [];
data_size = 0; 
data_ble = [];   
remaining_stream = [];     

%%%%  RAWDATA_TYPE_PTT  %%%%
RAWDATA_TYPE_PTT = 96;
pttgreen_cnt = 1;
pttgreen_xlist = [];
pttgreen_ylist = [];
pttecg_cnt = 1;
pttecg_xlist = [];
pttecg_ylist = [];   
%{
%%%%%  RAWDATA_TYPE_SPO2  %%%%
RAWDATA_TYPE_SPO2 = 96;
accdata_cnt = 0;
accdata_xlist = [];
accdata_ylist = [];
reddata_cnt = 0;
reddata_xlist = [];
reddata_ylist = [];
infradata_cnt = 0;
infradata_xlist = [];
infradata_ylist = [];
%}
%{
%%%%%  RAWDATA_TYPE_EHR  %%%%
RAWDATA_TYPE_EHR = 96;
accdata_cnt = 0;
accdata_xlist = [];
accdata_ylist = [];
reddata_cnt = 0;
reddata_xlist = [];
reddata_ylist = [];
infradata_cnt = 0;
infradata_xlist = [];
infradata_ylist = [];
%}
rawdata_type = RAWDATA_TYPE_PTT;

while true
    x1=[];
    y1=[];
    x2=[];
    y2=[];
    x3=[];
    y3=[];
    
    if( RUNNING_MODE_FILE == running_type )
       if( feof(log_fid) )
            disp(" ~feof(log_fid)");
            break;
        end
        pause(0.1);
        [data, count]=fread(log_fid, 244, 'uchar');%读二进制数据；这里是1个Byte读一次；
    end
    
    if( RUNNING_MODE_SERVER == running_type )
        data=fread(tcp_obj, 244, 'uchar');%读二进制数据；这里是1个Byte读一次；
        data = data';
        count=fwrite(raw_fid, data, 'uchar');
    end
    if( isempty(data) )       
        if( RUNNING_MODE_FILE == running_type )
            fclose(log_fid);
            disp("None data，文件已经读完......");
        end
        if( RUNNING_MODE_SERVER == running_type )
           fclose(tcp_obj);
           disp("None data，客户端已经断开连接......");

           fseek(raw_fid, 0,'eof');
           filelength = ftell(raw_fid);
           fclose(raw_fid);           
           fprintf("起始序号：%d, 结束序号：%d, 本地文件长度：%d\n",start_idx, stop_idx, filelength);
           fprintf("Notify包数：%d, 单个Notify帧长：%d, 总共传输数据量：%d\n", (stop_idx+1)-start_idx, 244, 244*((stop_idx+1)-start_idx));
        end
        return;
    end
    if(RAWDATA_TYPE_PTT == rawdata_type)                      
        [elemt_cnt, ptt_frames, remaining_stream_, index] = get_ptt_frames_v1(remaining_stream, data);     
        remaining_stream = remaining_stream_;
        
        if(0 == start_flag) 
            start_flag = 1;
            start_idx = index(1)*2^24 +index(2)*2^16+ index(3)*2^8 + index(4);
            stop_idx = index(length(index)-3)*2^24 +index(length(index)-2)*2^16+ index(length(index)-1)*2^8 + index(length(index)-0)*2^0;
        else
            stop_idx = index(length(index)-3)*2^24 +index(length(index)-2)*2^16+ index(length(index)-1)*2^8 + index(length(index)-0)*2^0;
        end
        
        [greenled_arr, ecg_arr, result ]=extract_ptt_data(ptt_frames, elemt_cnt);
        res_cnt = res_cnt + elemt_cnt;
        x_append = pttgreen_cnt: 1: pttgreen_cnt + elemt_cnt-1;
        pttgreen_cnt = pttgreen_cnt + elemt_cnt;
        pttgreen_xlist = [pttgreen_xlist x_append]; 
        pttgreen_ylist = [pttgreen_ylist greenled_arr];
        x1_len = length(pttgreen_xlist);
        if( x1_len > 250)
           x1 = pttgreen_xlist(x1_len-250:x1_len);
           y1 = pttgreen_ylist(x1_len-250:x1_len);
        else
           x1 = [];
           y1 = [];
        end
        x_append = pttecg_cnt: 1 : pttecg_cnt + elemt_cnt - 1;
        pttecg_cnt = pttecg_cnt + elemt_cnt;
        pttecg_xlist = [pttecg_xlist x_append];
        pttecg_ylist = [pttecg_ylist ecg_arr];
        x2_len = length(pttgreen_xlist);
        if (x2_len > 250)
            x2 = pttecg_xlist(x2_len-250:x2_len);
            y2 = pttecg_ylist(x2_len-250:x2_len);
        else
            x2 = [];
            y2 = [];
        end
    end
 %{
    if(RAWDATA_TYPE_SPO2 == rawdata_type)                      
        [elemt_cnt, ptt_frames, remaining_stream_, index] = get_spo2_frames_v1(remaining_stream, data);     
        remaining_stream = remaining_stream_;
        [greenled_arr, ecg_arr, result ]=extract_spo2_data(ptt_frames, elemt_cnt);
        res_cnt = res_cnt + elemt_cnt;
        x_append = pttgreen_cnt: 1: pttgreen_cnt + elemt_cnt-1;
        pttgreen_cnt = pttgreen_cnt + elemt_cnt;
        pttgreen_xlist = [pttgreen_xlist x_append]; 
        pttgreen_ylist = [pttgreen_ylist greenled_arr];
        x1_len = length(pttgreen_xlist);
        if( x1_len > 250)
           x1 = pttgreen_xlist(x1_len-250:x1_len);
           y1 = pttgreen_ylist(x1_len-250:x1_len);
        else
           x1 = [];
           y1 = [];
        end
        x_append = pttecg_cnt: 1 : pttecg_cnt + elemt_cnt - 1;
        pttecg_cnt = pttecg_cnt + elemt_cnt;
        pttecg_xlist = [pttecg_xlist x_append];
        pttecg_ylist = [pttecg_ylist ecg_arr];
        x2_len = length(pttgreen_xlist);
        if (x2_len > 250)
            x2 = pttecg_xlist(x2_len-250:x2_len);
            y2 = pttecg_ylist(x2_len-250:x2_len);
        else
            x2 = [];
            y2 = [];
        end
    end
 %}
 %{
   if(RAWDATA_TYPE_EHR == rawdata_type)                      
        [elemt_cnt, ptt_frames, remaining_stream_, index] = get_ehr_frames_v1(remaining_stream, data);     
        remaining_stream = remaining_stream_;
        [greenled_arr, ecg_arr, result ]=extract_ehr_data(ptt_frames, elemt_cnt);
        res_cnt = res_cnt + elemt_cnt;
        x_append = pttgreen_cnt: 1: pttgreen_cnt + elemt_cnt-1;
        pttgreen_cnt = pttgreen_cnt + elemt_cnt;
        pttgreen_xlist = [pttgreen_xlist x_append]; 
        pttgreen_ylist = [pttgreen_ylist greenled_arr];
        x1_len = length(pttgreen_xlist);
        if( x1_len > 250)
           x1 = pttgreen_xlist(x1_len-250:x1_len);
           y1 = pttgreen_ylist(x1_len-250:x1_len);
        else
           x1 = [];
           y1 = [];
        end
        x_append = pttecg_cnt: 1 : pttecg_cnt + elemt_cnt - 1;
        pttecg_cnt = pttecg_cnt + elemt_cnt;
        pttecg_xlist = [pttecg_xlist x_append];
        pttecg_ylist = [pttecg_ylist ecg_arr];
        x2_len = length(pttgreen_xlist);
        if (x2_len > 250)
            x2 = pttecg_xlist(x2_len-250:x2_len);
            y2 = pttecg_ylist(x2_len-250:x2_len);
        else
            x2 = [];
            y2 = [];
        end
    end
 %}
    pause(0.0001);
    subplot(3,1,1);
    axis([1 200 0 4000]);
    plot(x1,y1);
    subplot(3,1,2);
    axis([1 200 0 4000]);
    plot(x2,y2);
    subplot(3,1,3);
    axis([1 200 0 4000]);
    plot(x3,y3);
end

if( RUNNING_MODE_FILE == running_type )
    fclose(log_fid);
end
