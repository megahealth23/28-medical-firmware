# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 17:06:51 2020

@author: litianzheng
"""

__author__ = 'litianzheng'

from afe_object import AFE_Object as afe


class PTT_Object(afe):
    def __init__(self):
# 2Byte:  rawType: 1Byte; Protocol:1Byte
# 4Byte:  the bytes of Notify Frame 
        self.RAWDATA_OFFSET_V1 = 2+4
        self.RAWDATA_OFFSET_V2 = 2+4

        self.RawDataTYPE_PTT = 96

 #  20201030，这里是第一次测试版本，先加2，进行测试
 #  正式版本更改为0，因为林总认为加2不妥
 #  20201228，这里是第二次测试版本，加1，仅附加帧的长度，并进行测试
 #  正式版留意邮件附件
        self.PTT_RAWDATA_OFFSET_V1 = self.RAWDATA_OFFSET_V1 + 1
        self.PTT_RAWDATA_OFFSET_V2 = self.RAWDATA_OFFSET_V2 + 1

        self.ELEMT_PTT_CNT = 0
        self.ELEMT_PTT_SIZE = 6
############  V1:the fixed length(default: 244)   ##########
############  V2:the un-fixed length(greater than or equal: 6=2+4)   ##########
        self.PROTOCOL_V1_LENGTH  = 244  
        self.PROTOCOL_V2_LENGTH  = 0 # wait for next version #
        self.PROTOCOL_V1 = 1   # send data[1] = 0x01
        self.PROTOCOL_V2 = 1   # send data[1] = 0x01
        
    def get_ptt_frames_v1(self, old_stream, new_stream):
        work_stream = []
        if old_stream:
            work_stream = list(bytes(old_stream)) + list(bytes(new_stream))
        else:
            work_stream = list(bytes(new_stream))
        
        ptt_frames = []
        remaining_stream = []
        index = []
        stream_length = len(work_stream)
                
        if  (self.PROTOCOL_V1_LENGTH > stream_length):
            remaining_stream = work_stream
            elemt_cnt = 0
            ptt_frames = []
            return (elemt_cnt, ptt_frames, remaining_stream, index)
        
        ite = 0
        elemt_cnt = 0

        while  ( self.PROTOCOL_V1_LENGTH <= (stream_length - ite)  ) :   # send data[0] = 0x60 send data[1] = 0x01
            if( (self.RawDataTYPE_PTT ==  work_stream[ite]) and  (self.PROTOCOL_V1 == work_stream[ite+1])):
                
                self.ELEMT_PTT_CNT = work_stream[ite+self.RAWDATA_OFFSET_V1]>>0

                if ( self.PROTOCOL_V1_LENGTH == (stream_length-ite) )  :                  # the end frame of data is equal to PROTOCOL_V1_LENGTH
                    if ptt_frames:
                        ptt_frames = ptt_frames + work_stream[ite+self.PTT_RAWDATA_OFFSET_V1:ite+self.PTT_RAWDATA_OFFSET_V1 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE ]
                    else:
                        ptt_frames = work_stream[ite+self.PTT_RAWDATA_OFFSET_V1:ite+self.PTT_RAWDATA_OFFSET_V1 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE]

 
                    elemt_cnt = elemt_cnt + self.ELEMT_PTT_CNT
                    remaining_stream = [];
                    if index:
                        index = index + work_stream[ite+2:ite+6]
                    else:
                        index = work_stream[ite+2:ite+6]
                        
                    return (elemt_cnt, ptt_frames, remaining_stream,index)
                    
                if (self.RawDataTYPE_PTT == work_stream[ite + self.PROTOCOL_V1_LENGTH]):
                    if ptt_frames:                    
                        ptt_frames = ptt_frames + work_stream[ite+self.PTT_RAWDATA_OFFSET_V1:ite+self.PTT_RAWDATA_OFFSET_V1 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE ]
                    else:
                        ptt_frames =  work_stream[ite+self.PTT_RAWDATA_OFFSET_V1:ite+self.PTT_RAWDATA_OFFSET_V1 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE ]
                    elemt_cnt = elemt_cnt + self.ELEMT_PTT_CNT;
                    remaining_stream = work_stream[ite+self.PROTOCOL_V1_LENGTH:]
                    if index:
                        index = index + work_stream[ite+2:ite+6]
                    else:
                        index = work_stream[ite+2:ite+6]
                        
                    ite = ite + self.PROTOCOL_V1_LENGTH;
        
        return (elemt_cnt, ptt_frames, remaining_stream, index)
#***************************************************#    
    def get_ptt_frames_v2(self, old_stream, new_stream):
        work_stream = []
        
        if old_stream:
            work_stream = list(bytes(old_stream)) + list(bytes(new_stream))
        else:
            work_stream = list(bytes(new_stream))
        ptt_frames = []
        remaining_stream = []
        index = []

        stream_length = len(work_stream)
        if  (self.PTT_RAWDATA_OFFSET_V2 + self.ELEMT_PTT_SIZE >= stream_length):
            remaining_stream = work_stream
            elemt_cnt = 0
            ptt_frames = []
            return (elemt_cnt, ptt_frames, remaining_stream, index)

        ite = 0
        elemt_cnt = 0
        while  ( ite  < stream_length - self.PTT_RAWDATA_OFFSET_V2 - self.ELEMT_PTT_SIZE ) :   # send data[0] = 0x60 send data[1] = 0x01
            if( (self.RawDataTYPE_PTT ==  work_stream[ite]) and  (self.PROTOCOL_V2 == work_stream[ite+1])):      
                
                self.ELEMT_PTT_CNT = work_stream[ite+self.RAWDATA_OFFSET_V2]>>0
                self.PROTOCOL_V2_LENGTH = self.PTT_RAWDATA_OFFSET_V2 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE

                if  (self.PROTOCOL_V2_LENGTH > stream_length-ite):
                    remaining_stream = work_stream[ite:stream_length]
                    return (elemt_cnt, ptt_frames, remaining_stream, index)
                
                if ( self.PROTOCOL_V2_LENGTH == (stream_length-ite) )  :                  # the end frame of data is equal to PROTOCOL_V2_LENGTH
                    ptt_frames = ptt_frames + work_stream[ite+self.PTT_RAWDATA_OFFSET_V2:stream_length]
                    elemt_cnt = elemt_cnt + self.ELEMT_PTT_CNT
                    remaining_stream = []
                    if index:
                        index = index + work_stream[ite+2:ite+6]
                    else:
                        index = work_stream[ite+2:ite+6]
                    return (elemt_cnt, ptt_frames, remaining_stream, index)
                    
                if (self.RawDataTYPE_PTT == work_stream[ite + self.PROTOCOL_V2_LENGTH]):
                    ptt_frames = ptt_frames + work_stream[ite+self.PTT_RAWDATA_OFFSET_V2:ite+self.PTT_RAWDATA_OFFSET_V2 + self.ELEMT_PTT_CNT*self.ELEMT_PTT_SIZE ]
                    elemt_cnt = elemt_cnt + self.ELEMT_PTT_CNT
                    remaining_stream = work_stream[ite+self.PROTOCOL_V2_LENGTH:stream_length]
                    if index:
                        index = index + work_stream[ite+2:ite+6]
                    else:
                        index = work_stream[ite+2:ite+6]
                    ite = ite + self.PROTOCOL_V2_LENGTH
                else:
                    ite = ite + 1
            else:
                ite = ite + 1

        return (elemt_cnt, ptt_frames, remaining_stream, index)    
#***************************************************#
    def extract_ptt_data(self, ptt_frames, elemt_cnt):
        result = True
        ecg_arr = []
        greenled_arr=[]

        if ((self.ELEMT_PTT_SIZE*elemt_cnt) != len(ptt_frames)):
            result = False
            greenled_arr = []
            ecg_arr = []
            return (greenled_arr, ecg_arr, result)
        else:
            result = True

        greenled_arr = []
        ecg_arr = []
        for ite in range(elemt_cnt):
            frame_sidx = ite * self.ELEMT_PTT_SIZE;
            greenled_val = afe.afe_convert_sensor_data( (ptt_frames[frame_sidx]<<0 ), (ptt_frames[frame_sidx+1]<<0), (ptt_frames[frame_sidx+2]<<0))
            ecg_val = afe.afe_convert_sensor_data( (ptt_frames[frame_sidx+3]<<0), (ptt_frames[frame_sidx+4]<<0),  (ptt_frames[frame_sidx+5]<<0) )
   
            greenled_arr.append(greenled_val)
            ecg_arr.append( ecg_val)
        
        return (greenled_arr, ecg_arr, result)


