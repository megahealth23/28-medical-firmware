# 数据发送方法

通过软件定时器，定时器时间初步定为10ms，512byte往nandflash里面写入数据，nandflash存储器的地址间隔需要达到4k以上才能够防止出现读写错误的现象，然后512byte往com 端口发送数据

# 试验结果

cdc协议通过com端口打出16M数据需要5分钟左右

# 接收数据所需要的工具：

电脑端：安装串口xcom软件

手机端：安装nRF connect 手机app

# 串口xcom的操作指南

串口选择在设备管理器里面查看，选择usb串行设备对应的那个com口

每次接受数据前都先关闭一下串口，然后再重新开启，开启成功之后，左边的红灯标记会亮

把16进制和DTR√上，最后数据接受完了之后，点击 4 处的保存窗口将数据保存到你想要保存的路径

![1583404424934](C:\Users\acer\AppData\Roaming\Typora\typora-user-images\1583404424934.png)



# 手机软件nrf connect的操作指南

![1583823737049](C:\Users\acer\AppData\Roaming\Typora\typora-user-images\1583823737049.png)

出现如下弹框：

![1583823769859](C:\Users\acer\AppData\Roaming\Typora\typora-user-images\1583823769859.png)

在0x后面输入 B5AA   ,设备发送数据到串口软件(xcom)

输入 B5 ，设备停止发送数据到串口



note1:  该版本手机端APP无法通过广播无法收取数据

note2: flash里面的数据包存储格式，510byte+1byte+1byte 后面两个是校验位



