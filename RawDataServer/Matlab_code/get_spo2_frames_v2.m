%{
%
%
%
%
%
%
%}
function [elemt_cnt, ptt_frames, remaining_stream, index] = get_ptt_frames_v2( old_tcp_stream, new_tcp_stream)

RAWDATA_OFFSET_V2 = 2+4; % 2Byte:  rawType: 1Byte; Protocol:1Byte
                         % 4Byte:  the bytes of Notify Frame 
 %  v1表示固定长度的Notify帧， v2表示变长Notify帧
 %  20201030，这里是第一次测试版本，先加2，进行测试
 %  正式版本更改为0，因为林总认为加2不妥
 %  20201228，这里是第二次测试版本，加1，仅附加帧的长度，并进行测试
 %  正式版留意邮件附件
PTT_RAWDATA_OFFSET_V2 = RAWDATA_OFFSET_V2 + 1;  
RawDataTYPE_PTT = 96;
ELEMT_PTT_CNT = 0;
ELEMT_PTT_SIZE = 5;
PROTOCOL_V2_LENGTH  = 0; %   10 sample of ptt data  
PROTOCOL_V2 = 1;   % send data[1] = 0x01

work_stream = [old_tcp_stream  new_tcp_stream];
ptt_frames = [];
remaining_stream = [];
stream_length = length(work_stream);
index = [];
% lest 1 elemt %
if  ( PTT_RAWDATA_OFFSET_V2 + ELEMT_PTT_SIZE >= stream_length)
    remaining_stream = work_stream;
    elemt_cnt = 0;
    ptt_frames = [];
    return; 
end

ite = 1;
elemt_cnt = 0;
while  ((ite-1) <= (stream_length - PTT_RAWDATA_OFFSET_V2 - ELEMT_PTT_SIZE) )    % send data[0] = 0x60 send data[1] = 0x01
    if( (RawDataTYPE_PTT ==  work_stream(ite)) &&  ( PROTOCOL_V2 == work_stream(ite+1) ) )      
       ELEMT_PTT_CNT = work_stream(ite+RAWDATA_OFFSET_V2);
       PROTOCOL_V2_LENGTH = PTT_RAWDATA_OFFSET_V2 + ELEMT_PTT_CNT*ELEMT_PTT_SIZE;
       
      if  (ite-1) > (stream_length-PROTOCOL_V2_LENGTH)
          remaining_stream = work_stream(ite:stream_length);
          return;
      end       
       
      if ( (stream_length - PROTOCOL_V2_LENGTH) == (ite-1) )                  % the end frame of data is equal to PROTOCOL_V2_LENGTH
           ptt_frames = [ptt_frames work_stream(ite+PTT_RAWDATA_OFFSET_V2:stream_length)];
           elemt_cnt = elemt_cnt + ELEMT_PTT_CNT;
           remaining_stream = [];
           index = [index work_stream(ite+2:ite+5)];
           return;
      end
       
      if (RawDataTYPE_PTT == work_stream(ite + PROTOCOL_V2_LENGTH))
            ptt_frames = [ptt_frames work_stream(ite+PTT_RAWDATA_OFFSET_V2:ite+PTT_RAWDATA_OFFSET_V2 + ELEMT_PTT_CNT*ELEMT_PTT_SIZE-1)];
            elemt_cnt = elemt_cnt + ELEMT_PTT_CNT;
            remaining_stream = work_stream(ite+PROTOCOL_V2_LENGTH:stream_length);
            index = [index work_stream(ite+2:ite+5)];
            ite = ite + PROTOCOL_V2_LENGTH;
      else
            ite = ite + 1;
       end
    else
        ite = ite + 1;
    end
end



    

