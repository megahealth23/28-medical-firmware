import time
import struct

from pynrfjprog import API, Hex
from configures import config, MAC_SN_ADDR, UCIR_ADDR, DEV_SN_ADDR, SIZEOF_BOOTSETTINGS_T, DEVICEADDRTYPE_ADDR, DEVICEADDR_ADDR

# table for calculating CRC
# this particular table was generated using pycrc v0.7.6, http://www.tty1.net/pycrc/
# using the configuration:
#  *    Width        = 16
#  *    Poly         = 0x1021
#  *    XorIn        = 0x0000
#  *    ReflectIn    = False
#  *    XorOut       = 0x0000
#  *    ReflectOut   = False
#  *    Algorithm    = table-driven
# by following command:
#   python pycrc.py --model xmodem --algorithm table-driven --generate c
CRC16_XMODEM_TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
        ]

BOOTSETTINGS = {'ADDR': 0x0007F000, 'SIZE': 0x1000}     #4k
MBR_PARAM    = {'ADDR': 0x0007E000, 'SIZE': 0x1000}     #4k
BOOTLOADER   = {'ADDR': 0x00076000, 'SIZE': 0x8000}     #32k
MAC_SN       = {'ADDR': 0x00075000, 'SIZE': 0x1000}     #4k
SWAP         = {'ADDR': 0x00074000, 'SIZE': 0x1000}     #4k
BOND         = {'ADDR': 0x00073000, 'SIZE': 0x1000}     #4k
USER_INFO    = {'ADDR': 0x00072000, 'SIZE': 0x1000}     #4k
APPLICATION  = {'ADDR': 0x0001C000, 'SIZE': 0x56000}    #344k
SOFTDIVECE   = {'ADDR': 0x00001000, 'SIZE': 0x1B000}    #108k
MBR_BOOT     = {'ADDR': 0x00000000, 'SIZE': 0x1000}     #4k

BANK0_ADDR = APPLICATION['ADDR']
BANK1_ADDR = APPLICATION['ADDR'] + APPLICATION['SIZE']//2   #47000

BANK_STR = {
    0x01 : 'VALID_APP',
    0xA5 : 'VALID_SD',
    0xAA : 'VALID_BOOT',
    0xFE : 'ERASED',
    0xFF : 'INVALID_APP',
    0 : 'INVALID_APP'
}

def _crc16xmodem(data, crc, table):
    """Calculate CRC16 using the given table.
    `data`      - data for calculating CRC, must be bytes
    `crc`       - initial value
    `table`     - table for caclulating CRC (list of 256 integers)
    Return calculated value of CRC
    """
    for byte in data:
        crc = ((crc<<8)&0xff00) ^ table[((crc>>8)&0xff)^byte]
    return crc & 0xffff


def crc16(data, crc=0xFFFF):
    """Calculate CRC-CCITT (XModem) variant of CRC16.
    `data`      - data for calculating CRC, must be bytes
    `crc`       - initial value
    Return calculated value of CRC
    """
    return _crc16xmodem(data, crc, CRC16_XMODEM_TABLE)

class BootSettingsStruct(object):
    '''
        内部DFU存储settings内容解析
    '''
    def __init__(self, byte_arr):
        self.raw_data = byte_arr
        self.parse()

    def parse(self):
        self.bank0, self.bank0_crc, self.bank1 = struct.unpack('<HHI', self.raw_data[0:8])

        self.bank0_size, self.sd_size, self.bl_size, self.app_size, self.sd_start  = struct.unpack('<5I', self.raw_data[8:28])
        
        self.fact_valid, self.fact_crc, self.fact_size = struct.unpack('<HHI', self.raw_data[28:36])

    def __repr__(self):
        tips = '---bootsettings messages:---\n'
        tips += '    bank0:              ' + BANK_STR[self.bank0] + '\n'
        tips += '    bank0 crc:          0x' + hex(self.bank0_crc)[2:].zfill(4) + '\n'
        tips += '    bank1:              ' + BANK_STR[self.bank1] + '\n'
        tips += '    bank0 size:         0x' + hex(self.bank0_size)[2:].zfill(8) + '\n'
        tips += '    sd size:            0x' + hex(self.sd_size)[2:].zfill(8) + '\n'
        tips += '    bl size:            0x' + hex(self.bl_size)[2:].zfill(8) + '\n'
        tips += '    app size:           0x' + hex(self.app_size)[2:].zfill(8) + '\n'
        tips += '    sd start:           0x' + hex(self.sd_start)[2:].zfill(8) + '\n'
        tips += '    factory valid:      0x' + hex(self.fact_valid)[2:].zfill(4) + '\n'
        tips += '    factory crc:        0x' + hex(self.fact_crc)[2:].zfill(4) + '\n'
        tips += '    factory size:       0x' + hex(self.fact_size)[2:].zfill(8) + '\n'

        return tips

def console_print(api, addr, size):
    '''
        内部存储dfu settings打印
    '''
    if(not api.is_connected_to_emu()):
        raise Exception("Operation failed, connect to the emulator first!")
    
    if(size < 64):
        raise Exception("Size if(small, must be largger than 256!")

    rd_list = api.read(addr, size)
    bss = BootSettingsStruct(bytearray(rd_list))
    print(bss)
    # 去掉注释，可查看二进制原始内容 
    for x in range(len(rd_list)//8):
        print([' '.join(hex(v)[2:].zfill(2) for v in rd_list[x*8 : x*8+8])])

def hex_print(parameter_list):
    for x in range(len(parameter_list)//8):
        print([' '.join(hex(v)[2:].zfill(2) for v in parameter_list[x*8 : x*8+8])])

def file_crc16(file):
    '''
        计算bin文件CRC32
    '''
    with open(file, 'rb') as f:
        value = f.read()
        return len(value), crc16(value)

def update_bootsettings(api, file):
    '''
        准备拷贝使用
    '''

    size, crc = file_crc16(file)

    btst = bytearray(api.read(BOOTSETTINGS['ADDR'], 64))

    # valid位置写4，代表有app
    btst[28:36] = struct.pack("<HHI", 4, crc, size)

    for x in range(5):    
        api.erase_page(BOOTSETTINGS['ADDR'])
        api.write(BOOTSETTINGS['ADDR'], btst, True)
        temp = bytearray(api.read(BOOTSETTINGS['ADDR'], 64))

        if(bytearray(temp) == btst):
            # print("参数你做的对")
            return True
        else:
            if(x == 4):
                return False
            continue

class NRFErrorCode(object):
    
    success = 'OK'
    emu_not_found = "没有发现Jlink设备"
    dev_not_connect = '设备未连接'
    dev_communicate_error = '设备连接异常'
    macsn_has_burned = '已经烧录过MAC和SN'
    macsn_burned_error = 'MAC和SN烧录失败'
    bin_burned_error = 'bin文件烧录失败'
    hex_burned_error = 'hex文件烧录失败'
    hex_parse_error = 'hex文件解析失败'
    emu_is_busy = 'JLink有别的操作在进行，请稍后在做'


class NRF5xBurn(object):
    
    @staticmethod
    def self_check(api):
        api.open()
        el = api.enum_emu_snr()
        api.close()
        return el

    @staticmethod
    def reset(api):
        if(api.is_open() == True):
            return False, (NRFErrorCode.emu_is_busy, )
        
        api.open()
        if(api.enum_emu_snr() is None):
            api.close()
            return False, (NRFErrorCode.emu_not_found, )

        try:
            api.connect_to_emu_without_snr()
            api.sys_reset()
            api.go()
            api.disconnect_from_emu()
            api.close()
            return True, NRFErrorCode.success
        except Exception as e:
            api.close()
            return False, (NRFErrorCode.dev_communicate_error, e)

    @staticmethod
    def eraseall(api):
        if(api.is_open() == True):
            return False, (NRFErrorCode.emu_is_busy, )
        
        device_family = API.DeviceFamily.NRF52
        api.close()                         # Close API so that correct family can be used to open
        
        # Re-Init API, open, connect, and erase device
        api = API.API(device_family)        # Initializing API with correct family type [API.DeviceFamily.NRF51 or ...NRF52]
        api.open()                          # Open the dll with the set family type
        #api.connect_to_emu_without_snr()    # Connect to emulator, it multiple are connected - pop up will        
        
        
        if(api.enum_emu_snr() is None):
            api.close()
            return False, (NRFErrorCode.emu_not_found, )

        try:
            api.connect_to_emu_without_snr()
            api.recover()
            api.erase_all()
            api.erase_uicr()
            # time.sleep(1)
            api.sys_reset()
            api.disconnect_from_emu()
            api.close()
            return True, NRFErrorCode.success
        except Exception as e:
            api.close()
            return False, (NRFErrorCode.dev_communicate_error, e)

    @staticmethod
    def burn_hex(api):
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )


    @staticmethod
    def burn_img(api):
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )

 #******************   Using for MR  ****************#
    @staticmethod
    def burn_macsn(api, macsn):
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )
        
        try:
            for _ in range(3):
                api.erase_page(MAC_SN_ADDR)

                api.write(MAC_SN_ADDR, macsn, True)
                read_back = api.read(MAC_SN_ADDR, len(macsn))
                if(read_back == macsn):
                    return True, NRFErrorCode.success
                continue
            return False, (NRFErrorCode.macsn_burned_error, )
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

 #******************   Using for MR  ****************#
    @staticmethod
    def read_macsn(api):
        '''读0x10001080 #NRF_UICR->CUSTOMER[0]前24字节内容并与alls比较，相同返回ture，不同返回false'''
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )

        try:
            data_list = api.read(MAC_SN_ADDR, 32)
            return True, data_list
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

    @staticmethod
    def read_nordicid(api): 
        '''DEVICEID 64 bit unique device identifier读8字节内容并与alls比较，相同返回ture，不同返回false
        DEVICEID[n] (n=0..1) Address offset: 0x060 + (n × 0x4)'''
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )

        try:
            nrfid_list = api.read(UCIR_ADDR, 8)
            nrfid_list = nrfid_list[0:4][::-1] + nrfid_list[4:][::-1]
            return True, ''.join(hex(x)[2:].zfill(2) for x in nrfid_list).upper()
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

 #******************   Using for EEG  ****************#
    @staticmethod
    def burn_devSn(api, devSn):

        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )
        
        try:
            for _ in range(3):
                api.erase_page(DEV_SN_ADDR)

                api.write(DEV_SN_ADDR, devSn, True)
                read_back = api.read(DEV_SN_ADDR, len(devSn))

                if(read_back == devSn):
                    return True, NRFErrorCode.success
                continue
            return False, (NRFErrorCode.macsn_burned_error, )
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

 #******************   Using for EEG  ****************#
    @staticmethod
    def read_devSn(api):
        '''读0xF0000 #BOOT_SETTINGS_ADDR前sizeof(BootSettings_t)字节内容并与alls比较，相同返回ture，不同返回false'''
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )

        try:
            data_list = api.read(DEV_SN_ADDR, SIZEOF_BOOTSETTINGS_T)
            return True, data_list
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

 #******************   Using for EEG  ****************#
    @staticmethod
    def read_deviceaddr(api):
        '''读0xF0000 #BOOT_SETTINGS_ADDR前sizeof(BootSettings_t)字节内容并与alls比较，相同返回ture，不同返回false'''
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )
        MAC_LIST = []
        try:
            data_list = api.read(DEVICEADDRTYPE_ADDR, 12)
            if(0xFF == data_list[0]):
#                print("Random address")
                RA_MASK = 0xC0
                MAC_LIST = data_list[4:10]
                MAC_LIST[5] = RA_MASK | MAC_LIST[5]
            else:
#                print("Public address")
                MAC_LIST = data_list[4:10]
            return True, MAC_LIST
        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e)  

    @staticmethod
    def burn_hex(api, hex_file):
        if(not api.is_connected_to_emu()):
            return False, (NRFErrorCode.dev_not_connect, )

        try:
            target_program = Hex.Hex(hex_file)
                        
            cnt = 0
            hex_list_len = len(target_program._segment_list)
            size = 0
            for segment in target_program:
                print(hex(segment.address), hex(len(segment.data)), hex(segment.address+len(segment.data)))
                size += len(segment.data)
            print(sum([len(s.data) for s in target_program]))
            print(size)
            print(size > 214180) 

        except Exception as e:
            return False, (NRFErrorCode.dev_communicate_error, e) 

def test():
    api = API.API('NRF52')
    print(NRF5xBurn.self_check(api))
    api.open()
    api.connect_to_emu_without_snr()

    console_print(api, BOOTSETTINGS['ADDR'], 64)

    api.sys_reset()
    api.go()
    print(api.is_connected_to_device())
    api.close()
