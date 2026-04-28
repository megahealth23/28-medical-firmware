# -*- coding:utf-8 -*-
import os
import time
import codecs

def get_app_svn():
    #f_rev = open(r"..\..\svn_rev.c", 'r')
    f_rev = open(r"../../svn_rev.c", 'r')
    line = f_rev.readline()
    rev = line[line.find("= ")+2 : line.find(";")]
    f_rev.close()
    #print ( str(rev) )
    return str(rev)


# 制作DFU文件
os.system(r'cd ../../../scripts/scripts && start app-dfu-zip.bat %s' % get_app_svn() )
time.sleep(4)
os.system(r'taskkill /f /im cmd.exe')

print(("\"RingV%s.ZIP\" path: " % get_app_svn()))


