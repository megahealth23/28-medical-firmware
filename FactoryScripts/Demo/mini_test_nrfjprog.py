
try:
    from pynrfjprog import API, Hex
except ImportError:
    print("\n1) SourceCode's URL: \n\t <https://github.com/NordicSemiconductor/pynrfjprog>")
    print("2) Install: \n\t<python setup.py install>\n")
    print("or\t")
    print("    Run:\n\t <pip install pynrfjprog>\n")

from nrfjprog import NRF5xBurn, NRFErrorCode
from configures import config, MAC_SN_ADDR, UCIR_ADDR, EEG_TYPE_LIST, SIZEOF_BOOTSETTINGS_T
import time
import os
from device_sn import DeviceSN, SN2HEX
from pynrfjprog import LowLevel

try:
    import simplejson as json
except ImportError:
    import json

#*****  Using for MR   *****#
MHEAD = "4d52"  # 'MR'

#*****  Using for MR   *****#
EHEAD = "454547"  # 'EEG'

CONFIG = config()

TEST_SN = [69, 69, 71, 18, 21, 11, 0, 0, 6, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class BurnTest():
    def __init__(self, api):
        self.api = api
        
        global CONFIG
        try:
            if( CONFIG[0] ):
                self.config = CONFIG[1]
            else:
                self.config = None
                raise Exception(CONFIG[1])
        except Exception as e:
            print("self.config初始化失败: "+repr(e))
            os._exit(0)

    def burn_go(self):
        try:
            self.api.open()
            enum_list = self.api.enum_emu_snr()
            if(not enum_list):
                raise Exception(NRFErrorCode.emu_not_found)

            print("发现jlink：" + str(enum_list[0]))
            for x in range(20):
                try:
                    if(not self.api.is_open()):
                        self.api.open()
                    self.api.connect_to_emu_without_snr()
                    self.api.connect_to_device()
                    break
                except:
                    self.api.close()
                    print('查找设备连接:{}s'.format(x//2+1))
                    time.sleep(0.5)
                    if(x == 19):
                        print("连接jlink")
                        raise Exception(NRFErrorCode.dev_not_connect)

            print("连接jlink")
            print('读取MACSN')
            #查询是否已经烧录过
            
            if self.config["name"] in EEG_TYPE_LIST:
                print(self.config["name"])
                err, sn_msg = NRF5xBurn.read_devSn(self.api)
                print(sn_msg)
                if(not err):
                    print(sn_msg)
                    print('读取SN出错, {}'.format(repr(sn_msg)))
                     # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(sn_msg[0])

                print("read SN:{}".format(sn_msg))
                s = ''.join(hex(x)[2:] for x in sn_msg[0:3])
                print(s)

                err, chip_mac_msg = NRF5xBurn.read_deviceaddr(self.api)
                if(not err):
                    print(chip_mac_msg)
                    print('读取MAC出错, {}'.format(repr(chip_mac_msg)))
                     # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(chip_mac_msg[0])
                    chip_mac_s = (''.join("%02x-"%(x) for x in reversed(chip_mac_msg))).upper()[0:-1]
                    print("read MAC: {}".format(chip_mac_s))
                chip_mac_s = (''.join("%02x-"%(x) for x in reversed(chip_mac_msg))).upper()[0:-1]
                print("read MAC: {}".format(chip_mac_s))

                err, msg = NRF5xBurn.read_nordicid(self.api)
                #print(msg)
                if(err == False):
                    print('读取nordic id出错, {}'.format(repr(msg)))
                        # Reset the device and run.
                    self.api.erase_all()
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(msg[0])
                
                str_nrfid = msg
                print("MAC和SN烧录成功" + "芯片ID为：" + str_nrfid)

                
                _NOT_USED_, alls_array = SN2HEX.hexOfConfig("E01A52111000009")
                err, msg = NRF5xBurn.burn_devSn(self.api, alls_array)
                
        except Exception as e:
            if(self.api.is_open()):
                if(self.api.is_connected_to_device()):
                    self.api.sys_reset()
                    self.api.go()
                self.api.close()
            print(0, repr(e))



if __name__ == '__main__':

    # with LowLevel.API('NRF52') as api:
    #      api.enum_emu_snr()
    #      api.connect_to_emu_without_snr()
    #      api.erase_all()
    #      api.write_u32(SIZEOF_BOOTSETTINGS_T, 0x02, False)
    #      api.disconnect_from_emu()

    m_nrf52_api = API.API('NRF52')
    if( not NRF5xBurn.self_check(m_nrf52_api) ):
        raise Exception('请正确连接JLink')
        exit

    NRF52_obj = BurnTest(m_nrf52_api)
    NRF52_obj.burn_go()


                                      


