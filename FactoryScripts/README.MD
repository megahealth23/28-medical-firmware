
# 制作离线exe产测工具



pyinstaller -F BurnMain.py



## 虚拟环境制作与运行

    以D:\FACTORY\pythonvenv为例  

* 配置虚拟环境
* E盘新建文件夹FACTORY  
* 拷贝requirements.txt到E:\FACTORY  
* 从SVN上下载FactoryScripts（地址：）到E:\FACTORY\factTest
*  从GITHUB下载pynrfjprog-master（地址：<https://github.com/NordicSemiconductor/pynrfjprog>）到E:\FACTORY\factTest\pynrfjprog-master
* 配置执行如下

```
>: D: && cd D:\FACTORY  
 : python -m venv pythonvenv  
 : cd pythonvenv
 : .\Scripts\activate  
 : pip install -r D:\FACTORY\requirements.txt
```

补充：

 1）激活虚拟环境pythonvenv后， 使用`pip config set global.index-url <PYPI-URL> ` 来直接指定下载源的URL（这种命令行永久修改法不用手动修改配置文件）。以清华大学源为例，执行如下

```
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```


以下是国内互联网常用的pypi安装源URL，在国内互联网下载的速度非常快

| pypi安装源      | pypi安装源URL（PYPI-URL）                    |
| --------------- | -------------------------------------------- |
| pypi 清华大学源 | https://pypi.tuna.tsinghua.edu.cn/simple     |
| pypi 腾讯源     | http://mirrors.cloud.tencent.com/pypi/simple |
| pypi 阿里源     | https://mirrors.aliyun.com/pypi/simple/      |
| pypi 豆瓣源     | http://pypi.douban.com/simple/               |


 2）安装问题解决方法

​     a、QT5安装失败，去掉requirements.txt中的PyQt5和PyQt5-sip，单独安装时指定豆瓣源  

```
pip install pyqt5 -i https://pypi.douban.com/simple
```

​    b、UPX is not available问题，从UPX官网< [https://upx.github.io](https://upx.github.io/) >下载UPX.exe，放到D:\FACTORY\pythonvenv\Scripts\下。

​    c、打包好后，运行时报pynrfjprog.dll缺失或者参数错误，卸载pynrfjprog库后，单独用源码安装  

```
pip uninstall pynrfjprog
cd D:\FACTORY\factTest\pynrfjprog-master
python setup.py install
```

* 测试，以"D:\FACTORY\factTest\兆观生产烧录工具"产测程序路径为例  
```
>: D: && cd D:\FACTORY\factTest\兆观生产烧录工具  
 : python BurnMain.py  
```
* 退出虚拟环境  
```
>: deactivate
```

### 注意 
    1, 虚拟必须是英文路径，比如E:\FACTORY\pythonvenv  
    2，requirements.txt从工程中拷贝过去



## 制作可执行文件[exe]
    以"D:\FACTORY\factTest\build.bat"产测程序路径为例

* 检测build.bat的"venv_dir"虚拟环境地址是否正确  
* 运行build.bat
* 在dist文件家里查看是否有exe文件生成，如"兆观产测工具.exe"
* 关于配置信息，可打开BurnMain.spec查看、更改



## 注意
    windows下BurnMain.spec出现中文必须修改成gbk编码格式，其他OS，可以切换为utf8格式，不然会报错  

