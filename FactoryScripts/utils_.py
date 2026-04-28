
import os
import zipfile
import sys
import shutil
import codecs
import requests 
import qrcode
import PIL
import json
from device_sn import DeviceSN

def zip_dir(file_path, zfile_path):
    '''
    function:压缩
    params:
        file_path:要压缩的件路径,可以是文件夹
        zfile_path:解压缩路径
        
        zip_dir('log', 'log.zip')
    '''
    filelist = []
    if os.path.isfile(file_path):
        filelist.append(file_path)
    else :
        for root, dirs, files in os.walk(file_path):
            for name in files:
                filelist.append(os.path.join(root, name))
                # print('joined:',os.path.join(root, name),dirs)

    zf = zipfile.ZipFile(zfile_path, "w", zipfile.zlib.DEFLATED)
    for tar in filelist:
        arcname = tar[len(file_path):]
        # print(arcname,tar)
        zf.write(tar,arcname)
    zf.close()


def get_dir_path():
    cwd = os.getcwd()
    path = cwd[0:cwd.rindex('/')] + "/dir"
    if os.path.exists(path) and os.path.isdir(path):
        return path
    else:
        raise Exception("not found this path")


def unzip(src_zip, dest_dir):
    '''
        unzip('demo.zip', 'demo')
    '''
    if os.path.exists(dest_dir):
        shutil.rmtree(dest_dir)
    
    os.mkdir(dest_dir)

    if zipfile.is_zipfile(src_zip) and os.path.isdir(dest_dir):
        'unzip file'
  
        z = zipfile.ZipFile(src_zip, 'r')
        z.extractall(dest_dir)
        z.close()
        # os.remove(src_zip)
 
        # print("解压完成，开始转换编码了....")
        'change charset'
        # 修改文件夹名, 则先递归列举出最深层的子目录，然后是其兄弟目录，然后父目录，不然文件夹名会搜索不到
        for root_path, dir_names, file_names in os.walk(dest_dir, topdown=False):                
            for fn in file_names:
                # srcDir 下面的所有文件(非目录)，都会经过这个 path 这里了，至于为什么，你要去看 os.walk() 了。
                path = os.path.join(root_path, fn)
                if not zipfile.is_zipfile(path):
                    # print("before:", fn)
                    try:
                        fn = fn.encode('cp437').decode('gbk')
                        # print("after:", fn)
                        new_path = os.path.join(root_path, fn)
                        os.rename(path, new_path)
                    except Exception as e:
                        print("error:", e)
                        raise Exception("file name error.")

            for fn in dir_names:
                path = os.path.join(root_path, fn)
                try:
                    fn = fn.encode('cp437').decode('gbk')
                    new_path = os.path.join(root_path, fn)
                    os.rename(path, new_path)
                except Exception as e:
                    print("error:", e)
                    raise Exception("dir name error.")
    else:
        raise Exception("src zip is not a zip file")

def remove_empty(srcDir):
    if os.path.isdir(srcDir):
        for root_path, dir_names, file_names in os.walk(srcDir):
            for dn in dir_names:
                # srcDir 下面的所有目录，都会经过这个 dir_path 这里了，至于为什么，你要去看 os.walk() 了。
                dir_path = os.path.join(root_path, dn)
                if not len(os.listdir(dir_path)):
                    os.rmdir(dir_path)
                    print("rm dir:", dir_path)

def download_zip(url, dest_zip):
    '''
        download zip file
    '''
    # url = 'http://update.megahealth.cn/84fe5ea012f52cddfcbb.rar' 
    r = requests.get(url)
    print(r.status_code)
    if r.status_code == 200:
        with open(dest_zip, "wb") as code:
            code.write(r.content)
        return True
    else:
        return False


def findUsbDisk(name):
    '''根据U盘名称查找U盘是否插入'''
    disk_flags = os.popen('fsutil fsinfo drives').read().split()
    for disk in disk_flags[1:]:
        contxt = os.popen('fsutil fsinfo volumeinfo %s' % disk[:-1]).read().split('\n')[0]
        if name in contxt:
            return (disk, contxt)

def make_qrcode(message, img_file, width=152, height=152):
    imge = qrcode.make(message)
    imge = imge.resize((width, height), PIL.Image.ANTIALIAS)
    imge.save(img_file)

def get_macsn(macsn_json, hard_io_ver):
 
    # 判断是否还有可用 MAC SN
    if macsn_json['burned'] >= macsn_json['num']:
        return None

    # 处理SN
    startSN = macsn_json['START_SN']
    x = int(startSN[9:15]) + macsn_json['burned']

    sn = startSN[0:9] + str(x).zfill(6)
    n_sn = DeviceSN.code_sn(sn, hard_io_ver)
 
    # 处理MAC
    start_mac = int((macsn_json['START_MAC'].replace('-', '')[6:]), 16)

    new_mac = start_mac + macsn_json['burned']

    mac = hex(new_mac)[2:].zfill(0)
    mac = macsn_json['START_MAC'][:9] + mac[0:2] + '-' + mac[2:4] + '-' + mac[4:6]    
    return sn.upper(), mac.upper(), n_sn

def update_macsn(json_file, hard_io_ver):
    with open(json_file, 'r', encoding='utf8') as f:
        data = json.load(f)
        # 记录烧录数量
        burned = data['burned'] + 1
        if burned <= data['burned']:
            data['burned'] = burned
            json.dump(data, open(json_file, 'w'), sort_keys=True, indent=4)

        return get_macsn(data, hard_io_ver)
# unzip('burn.zip', 'burn')

# with codecs.open(r'E:\FACTORY\烧录工具\新穿戴联网\demo\0927金属戒指生产烧录及产测工具\说明.txt', 'r', encoding='utf-8') as f:
#     with codecs.open('hehe.txt', 'w', encoding='utf-8') as f1:
#         f1.write(f.read())