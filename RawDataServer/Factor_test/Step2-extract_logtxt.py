# -*- coding: utf-8 -*-
"""
Created on Wed Mar 31 18:31:59 2021

@author: litianzheng
"""
import matplotlib.pyplot as plt


def str_to_int(strhex_msb, strhex_midb, strhex_lsb):
    val_msb = int("0x"+strhex_msb,16)
    val_midb = int("0x"+strhex_midb,16)
    val_lsb = int("0x"+strhex_lsb,16)
    val = (val_msb<<16) + (val_midb<<8) + (val_lsb<<0)
    print("val:%d [%s:%d %s:%d %s:%d]"%(val, strhex_msb, val_msb, strhex_midb, val_midb, strhex_lsb, val_lsb))
    return (val_msb, val_midb, val_lsb, val)

def afe_convert_sensor_data(msb, midb, lsb):
    sensor_val = 0
        
    data = (msb<<16)+(midb<<8)+(lsb<<0)
    symbol = (data>>21)&0x07
        
    if(0x00 == symbol):
        sensor_val = data&0x1FFFFF
    elif(0x07 == symbol):
        sensor_val = (int)(data | 0xFF000000)
    elif(0x01 == symbol):
        sensor_val = 0x200000
    elif(0x06 == symbol):
        sensor_val = 0x200000
        sensor_val = - sensor_val
                
    return sensor_val

# initialize variables:
lines = []
idx_list = []
ecg_list = []

with open('./log.txt','r') as src_file:
    # iterate over each line:    
    for line in src_file.readlines():

        # append line to lines list:
        if "<<<<:" in line:
            start = line.index("<<<<:")+5
            lines.append(line[start:])

# print each line:
for line in lines:
    print(line, end=' ')
    val_msb, val_midb, val_lsb, frame_id = str_to_int('00', line[3:5], line[6:8])

    for idx in range(5):
        sector_start = 15 + idx * 9
        msb_idx_start = sector_start + 0
        msb_idx_end =  sector_start + 2
        midb_idx_start =  sector_start + 3
        midb_idx_end =  sector_start + 5
        lsb_idx_start =  sector_start + 6
        lsb_idx_end =  sector_start + 8
        
        val_msb, val_midb, val_lsb, val = str_to_int(line[msb_idx_start:msb_idx_end], line[midb_idx_start:midb_idx_end], line[lsb_idx_start:lsb_idx_end])
        ecg_val = afe_convert_sensor_data(val_msb, val_midb, val_lsb)
        ecg_list.append(ecg_val)
        idx_list.append( (frame_id-1)*5 + (idx+1) )

print(idx_list)
print(ecg_list)

plt.plot(idx_list,ecg_list,'r-')