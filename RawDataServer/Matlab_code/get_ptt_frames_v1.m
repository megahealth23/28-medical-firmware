 %  v1表示固定长度的Notify帧(默认244)， v2表示变长Notify帧（最少6字节）
 %  20201030，这里是第一次测试版本，先加2，进行测试
 %  正式版本更改为0，因为林总认为加2不妥
 %  20201228，这里是第二次测试版本，加1，仅附加帧的长度，并进行测试
 %  正式版留意邮件附件
function [elemt_cnt, ptt_frames, remaining_stream, index] = get_ptt_frames_v1( old_tcp_stream, new_tcp_stream)
% local setting %
ELEMT_PTT_SIZE = 6;
ELEMT_PTT_CNT = 0;

PROTOCOL_V1_LENGTH  = 244; %   10 sample of ptt data  
RAWDATA_OFFSET_V1 = 2+4; % 2Byte:  rawType: 1Byte; Protocol:1Byte
                         % 4Byte:  the bytes of Notify Frame 
PTT_RAWDATA_OFFSET_V1 = RAWDATA_OFFSET_V1 + 1;  
RawDataTYPE_PTT = 96;
PROTOCOL_V1 = 1;   % send data[1] = 0x01

work_stream = [old_tcp_stream  new_tcp_stream];
ptt_frames = [];
remaining_stream = [];
index = [];
stream_length = length(work_stream);
if  PROTOCOL_V1_LENGTH > stream_length 
    remaining_stream = work_stream;
    elemt_cnt = 0;
    ptt_frames = [];
    return;
end

ite = 1;
elemt_cnt = 0;
while( (stream_length - PROTOCOL_V1_LENGTH) >= (ite-1) )     % send data[0] = 0x60 send data[1] = 0x01
    if (RawDataTYPE_PTT ==  work_stream(ite)) &&  (PROTOCOL_V1 ==  work_stream(ite+1))
        if (ite-1) == (stream_length-PROTOCOL_V1_LENGTH)   % the end frame of data
            ELEMT_PTT_CNT = work_stream(ite+RAWDATA_OFFSET_V1);
            elemt_cnt = elemt_cnt + ELEMT_PTT_CNT;

            ptt_frames = [ptt_frames work_stream(ite+PTT_RAWDATA_OFFSET_V1:ite+PTT_RAWDATA_OFFSET_V1+ELEMT_PTT_CNT*ELEMT_PTT_SIZE-1)];      
            remaining_stream = [];
            index = [index work_stream(ite+2:ite+5)];
            return;
        end
        
        if( RawDataTYPE_PTT == work_stream(ite+PROTOCOL_V1_LENGTH))
            ELEMT_PTT_CNT = work_stream(ite+RAWDATA_OFFSET_V1);
            elemt_cnt = elemt_cnt + ELEMT_PTT_CNT;
            ptt_frames = [ptt_frames work_stream(ite+PTT_RAWDATA_OFFSET_V1:ite+PTT_RAWDATA_OFFSET_V1+ELEMT_PTT_CNT*ELEMT_PTT_SIZE-1)];
            remaining_stream = work_stream(ite+PROTOCOL_V1_LENGTH:stream_length);
            index = [index work_stream(ite+2:ite+5)];
            ite = ite + PROTOCOL_V1_LENGTH;
        end
    else
            ite = ite + 1;
    end
end
    

