# -*- coding: utf-8 -*-
"""
Created on Sun Jan  3 08:55:06 2021

@author: litianzheng
"""

class AFE_Object(object):
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