# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 16:57:20 2020

@author: litianzheng
"""
from ptt_object import PTT_Object as rawdata_ptt
import socket 
import time
import struct
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np  
import time
import csv
import sys
import os
from tkinter import Tk
from tkinter.filedialog import askopenfilename

RUNNING_MODE_NONE = 0
RUNNING_MODE_FILE = 1
RUNNING_MODE_SERVER = 2
running_type = RUNNING_MODE_SERVER

HOST = '0.0.0.0'
PORT = 30001
BUFSIZ = 1024   
ADDRESS = (HOST, PORT)

#########  global variable ##############
start_flag = 0
start_idx = 0
stop_idx = 0
p_index = 0
data_list = [0]
plt.figure(1)
pk_cnt = 0
res_cnt = 0
data_size = 0 
data_ble = []   
remaining_stream = []      
# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display
    
#######  RAWDATA_TYPE_PTT  ###########
RAWDATA_TYPE_PTT = 96
pttgreen_cnt = 0
pttgreen_xlist = []
pttgreen_ylist = []
pttecg_cnt = 0
pttecg_xlist = []
pttecg_ylist = []   
ptt = rawdata_ptt()


#######  RAWDATA_TYPE_SPO2  ###########
accdata_cnt = 0
accdata_xlist = []
accdata_ylist = []
reddata_cnt = 0
reddata_xlist = []
reddata_ylist = []
infradata_cnt = 0
infradata_xlist = []
infradata_ylist = []

if RUNNING_MODE_NONE == running_type:
    print("未选择模式,退出....")
    sys.exit()

log_fobj = None
abs_filename = ''

tcpServerSocket = None
raw_fobj = None
rawdata_filename = ''
if RUNNING_MODE_SERVER == running_type:
#########    Server setting   ######### 
# 创建监听socket
    tcpServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 绑定IP地址和固定端口
    tcpServerSocket.bind(ADDRESS)
    print("服务器({})启动，".format(socket.gethostbyname(socket.gethostname())) + "监听端口{}...".format(ADDRESS[1]))
    tcpServerSocket.listen(5)


rawdata_type = RAWDATA_TYPE_PTT

# 打开交互模式    
# # 创建线程
# thread_draw = threading.Thread(target=draw_entry)
# # 启动线程
# thread_draw.start()
# thread_draw.join()
data = None    
try:
    while True:
        
        if RUNNING_MODE_FILE == running_type:
            Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
            abs_filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file

            if '' == abs_filename:
                print("未选择文件，退出...")
                sys.exit()
            log_fobj = open(abs_filename, mode='rb')
        
        now = time.strftime("%Y-%m-%d-%H-%M-%S",time.localtime(time.time())) 
        rawdata_filename=os.getcwd() + os.sep + r'dat' +os.sep + r'ble_rawdata-'+now+r'.dat'
        rawdata_filename = rawdata_filename.strip()
        print("Rawdata文件保存位置：%s"%(rawdata_filename))
        if RUNNING_MODE_SERVER == running_type:
            raw_fobj = open(rawdata_filename,'wb') #写入文件路径
            print('服务器正在运行，请连接到同一局域网。\n等待客户端连接...')
            # client_socket是专为这个客户端服务的socket，client_address是包含客户端IP和端口的元组
            client_socket, client_address = tcpServerSocket.accept()
            print('客户端{}已连接！'.format(client_address))
    
        try:
            while True:
                
                if RUNNING_MODE_FILE == running_type:      
                    time.sleep(0.1) # 休眠1秒
                    data = log_fobj.read(244)
                    
                if RUNNING_MODE_SERVER == running_type:
                    # 接收客户端发来的数据，阻塞，直到有数据到来
                    # 事实上，除非当前客户端关闭后，才会跳转到外层的while循环，即一次只能服务一个客户
                    # 如果客户端关闭了连接，data是空字符串
                    data = client_socket.recv(2048)
                    raw_fobj.write(data)
                            
                if data:
                    p_index = 0     
                    pk_cnt = pk_cnt + 1
                    data_size = data_size + len(data) 
                    data_ble.append(data)
            
                    x1 = []
                    y1 = []
                    x2 = []
                    y2 = []
                    x3 = []
                    y3 = []
                                
                    if(RAWDATA_TYPE_PTT == rawdata_type):                        
                        elemt_cnt, ptt_frames, remaining_stream, index = ptt.get_ptt_frames_v1(remaining_stream, data)
                                                
                        greenled_arr, ecg_arr, result = ptt.extract_ptt_data(ptt_frames, elemt_cnt)
                        res_cnt = res_cnt + elemt_cnt
            
                        x_append = [n for n in range(pttgreen_cnt, pttgreen_cnt + elemt_cnt)]
                        pttgreen_cnt = pttgreen_cnt + elemt_cnt
                        pttgreen_xlist = pttgreen_xlist + x_append 
                        pttgreen_ylist = pttgreen_ylist + greenled_arr
                        x1_len = len(pttgreen_xlist)
                        if x1_len >= 250:
                            x1 = np.array(pttgreen_xlist[x1_len-250:-1])
                            y1 = np.array(pttgreen_ylist[x1_len-250:-1])
                        else:
                            x1 = []
                            y1 = []
            
                        x_append = [n for n in range(pttecg_cnt, pttecg_cnt + elemt_cnt)]
                        pttecg_cnt = pttecg_cnt + elemt_cnt
                        pttecg_xlist = pttecg_xlist + x_append
                        pttecg_ylist = pttecg_ylist + ecg_arr
                        x2_len = len(pttgreen_xlist)
                        if x2_len > 250:
                            x2 = np.array(pttecg_xlist[-250:-1])
                            y2 = np.array(pttecg_ylist[-250:-1])
                        else:
                            x2 = []
                            y2 = []
                               
                    #if(== rawdata_type):
                    #   if len(reddata_xlist) > 250:
                    #     x1 = np.array(reddata_xlist[-250:])
                    #     y1 = np.array(reddata_ylist[-250:])
                    #
                    #                    if len(infradata_xlist) > 250:
                    #    x2 = np.array(infradata_xlist[-250:])
                    #    y2 = np.array(infradata_ylist[-250:])
                    #                    if len(accdata_xlist) > 250:
                    #    x3 = np.array(accdata_xlist[-250:])
                    #    y3 = np.array(accdata_ylist[-250:])
                    #
                                
                    plt.ion()    # Interactive，实时更新
                    plt.clf()
                    plt.autoscale()   
                                                    
                    plt.subplot(311)                        
                    plt.plot(x1, y1)
                
                    plt.subplot(312)    
                    plt.plot(x2, y2)
                                            
                    plt.subplot(313)
                    plt.plot(x3, y3)
                                    
                    plt.pause(0.001) # pause a bit so that plots are updated
                    time.sleep(0.001)
                    if is_ipython:
                        display.clear_output(wait=True)
                        display.display(plt.gcf())      
                                    
                else:            
                    break
        finally:
            
            if RUNNING_MODE_FILE == running_type:
                log_fobj.close()
                print('文件已经读完......')
        
            if RUNNING_MODE_SERVER == running_type:
                # 关闭为这个客户端服务的socket
                client_socket.close()
                print('客户端 {} 已断开！'.format(client_address))
        
                print(rawdata_filename)
                raw_fobj.close()
                print("起始序号：%d, 结束序号：%d, 本地文件长度：%d\n"%(start_idx, stop_idx, os.path.getsize(rawdata_filename)))
                print("Notify包数：%d, 单个Notify帧长：%d, 总共传输数据量：%d\n"%((stop_idx+1)-start_idx, 244, 244*((stop_idx+1)-start_idx)))

            start_flag = 0

finally:
    if RUNNING_MODE_SERVER == running_type:

            # 关闭监听socket，不再响应其它客户端连接
        tcpServerSocket.close()
