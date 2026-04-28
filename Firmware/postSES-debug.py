##################################################################
# CreateAt          : 2023-07-16 21:52:57
# Author            : Fan Xu
# FilePath          : \FactoryScriptsc:\workInZG\ZG29\RING\ZG29-svn-fanxu2\Firmware\postSES-debug.py
# LastEditTime      : 2023-07-16 21:53:29
# LastEditors       : Fan Xu2
# Description       : 
# Copyright         : Copyright (c) 2021 MegaHealth. All Rights Reserved.
# Website           : https://www.megahealth.cn
###################################################################
# -*- coding:utf-8 -*-
import os
import time
import codecs
import re

def get_app_svn():
    #f_rev = open(r"..\..\svn_rev.c", 'r')
    f_rev = open(r"../../svn_rev.c", 'r')
    
    while  True:
        # Get next line from file
        line  =  f_rev.readline()
        # If line is empty then end of file reached
        if  not  line  :
            break;
        
        if "Major" in line:
            Major = re.search(r"= (.*);", line)
            print("Major : "+Major.group(1))
        elif "Minor" in line:
            Minor = re.search(r"= (.*);", line)
            print("Minor : "+Minor.group(1))
        elif "Patch" in line:
            Patch = re.search(r"= (.*);", line)
            print("Patch : "+Patch.group(1))
        # Close Close   
        #  
    f_rev.close()
    ver = f"{Major.group(1)}.{Minor.group(1)}.{Patch.group(1)}"
    print(ver)
    return str(ver)


# 制作DFU文件
os.system(r'cd ../../../scripts/scripts && start app-dfu-zip-SES-debug.bat %s' % get_app_svn() )
time.sleep(4)
os.system(r'taskkill /f /im cmd.exe')

print(("\"RingV%s-debug.ZIP\" path: " % get_app_svn()))


