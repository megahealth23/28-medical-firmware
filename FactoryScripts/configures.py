try:
    import simplejson as json
except ImportError:
    import json

import os
from version_ import PC_VER
# from utils_ import download_zip, unzip

#********   MR  ********#
# MAC_SN_ADDR = 0x10001080 #NRF_UICR->CUSTOMER[0]
MAC_SN_ADDR = 0x000EE000 
UCIR_ADDR = 0x10000060

#********  EEG  ********#
DEV_SN_ADDR = 0xF0000     #size 0x1000

# in device_sn.py  HexAddr1    = "10000000" # 0x10 length, 0xD000 address, 0x00 datatype.
SIZEOF_BOOTSETTINGS_T = 0x10

FICR_ADDR   = 0x10000000  
DEVICEADDRTYPE_OFFSET =  0x0A0
DEVICEADDR_OFFSET  = 0x0A4
DEVICEADDRTYPE_ADDR  = FICR_ADDR + DEVICEADDRTYPE_OFFSET    #NRF_FICR->DEVICEADDRTYPE
DEVICEADDR_ADDR =  FICR_ADDR + DEVICEADDR_OFFSET    #NRF_FICR->DEVICEADDR[0]

EEG_TYPE_LIST = ["E01A"]

def config(path=None):
    '''
        返回一个字典文件
    '''
    if path == None:
        # path = "config.json"
        path = "conf.json"
    try:
        if not os.path.exists(path):
            return (False, "%s file is not exist." % path)

        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        if list(map(int, data['PC_VER'].split('.'))) > list(map(int, PC_VER.split('.'))):
            return (False, "PC烧录程序版本太低，请更新到%s以上" % data['PC_VER'])

        # factory_hex  产测固件、应用固件一体化.
        if not os.path.exists(data['factory_hex_path']):
            return (False, "%s file is not exist." % data['factory_hex_path'])

        hw_ver = str(int(data['hardware_ver'][1][4:6], 16)) + \
            "." + str(int(data['hardware_ver'][1][6:8], 16))
        if float(data['hardware_ver'][0]) != float(hw_ver):
            return (False, "hardware version error")

        for _, v in data['mac_sn'].items():
            js_file = v['path']
            if not os.path.exists(js_file):
                return (False, "%s file is not exist." % js_file)
            with open(js_file, 'r', encoding='utf-8') as jf:
                ctx = json.load(jf)
                if ctx['num'] != v['num']:
                    return (False, "%s file mac_sn num error." % js_file)

        return (True, data)

    except Exception as e:
        return (False, e)

if __name__ == '__main__':
    ret = config()
    print(ret)

