##################################################################
# CreateAt          : 2023-04-18 17:19:29
# Author            : Fan Xu
# FilePath          : \auto-sleep-24-9-4\Firmware\project\SES\postSES.py
# LastEditTime      : 2024-09-10 17:22:40
# LastEditors       : Fan Xu2
# Description       : 
# Copyright         : Copyright (c) 2021 MegaHealth. All Rights Reserved.
# Website           : https://www.megahealth.cn
###################################################################
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
os.system(r'cd ../../../scripts/scripts && start app-dfu-zip-SES.bat %s' % get_app_svn() )
time.sleep(4)
# os.system(r'taskkill /f /im cmd.exe')

print(("\"RingV%s.ZIP\" path: " % get_app_svn()))


