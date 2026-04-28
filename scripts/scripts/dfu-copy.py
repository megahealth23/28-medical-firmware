'''
    注意：api.read_u32(0x208) 等同于命令 >nrfjprog --memrd 0x208 --n 4
            结果为： 0xE890A00A，小端模式组织的uint32
            内存实际为；0A A0 90 E8
'''
from pynrfjprog import API
from pynrfjprog.API import QSPIEraseLen
import struct
import os
from zlib import crc32
import math
import time

# JLINKARM_DLL = None
JLINKARM_DLL = r'C:\Program Files\SEGGER\SEGGER Embedded Studio for ARM 3.40\bin\JLink_x64.dll'
# JLINKARM_DLL = 'D:/ring band/nRF5_SDK_15.0.0_a53641a/DFU-MAKE/JLink_V622g/JLink_x64.dll'

BOOTSETTINGS = {'ADDR': 0x000FF000, 'SIZE': 0x1000}     #4k
MBR_PARAM    = {'ADDR': 0x000FE000, 'SIZE': 0x1000}     #4k
BOOTLOADER   = {'ADDR': 0x000F6000, 'SIZE': 0x8000}     #32k
APP_DATA     = {'ADDR': 0x000C4000, 'SIZE': 0x32000}    #200k
APPLICATION  = {'ADDR': 0x00026000, 'SIZE': 0x9E000}    #632k
SOFTDIVECE   = {'ADDR': 0x00001000, 'SIZE': 0x25000}    #148k
MBR_BOOT     = {'ADDR': 0x00000000, 'SIZE': 0x1000}     #4k

EXT_IMG_MAX_NUM	= 8                         # 分配了8个application
# 注意，文档介绍的时候NRF52840 DK板是64Mb，即8MB
EXT_IMG_MAX_SIZE = 8*1024*1024              # 8M
EXT_APP_MAX_SIZE = 512*1024                 # 512K
EXT_APP_SETTINGS_SIZE =	0x1000              # 4K

EXT_APP_SETTINGS_LENGTH = 44                # 等于offsetof(ext_settings_t, page_rw_num)

class BootSettingsStruct(object):
    '''
        内部DFU存储settings内容解析
        /**@brief DFU settings for application and bank data.
        */
        typedef struct
        {
            uint32_t            crc;                /**< CRC for the stored DFU settings, not including the CRC itself. If 0xFFFFFFF, the CRC has never been calculated. */
            uint32_t            settings_version;   /**< Version of the current DFU settings struct layout. */
            uint32_t            app_version;        /**< Version of the last stored application. */
            uint32_t            bootloader_version; /**< Version of the last stored bootloader. */

            uint32_t            bank_layout;        /**< Bank layout: single bank or dual bank. This value can change. */
            uint32_t            bank_current;       /**< The bank that is currently used. */

            nrf_dfu_bank_t      bank_0;             /**< Bank 0. */
            nrf_dfu_bank_t      bank_1;             /**< Bank 1. */

            uint32_t            write_offset;       /**< Write offset for the current operation. */
            uint32_t            sd_size;            /**< Size of the SoftDevice. */

            dfu_progress_t      progress;           /**< Current DFU progress. */

            uint32_t            enter_buttonless_dfu;
            uint8_t             init_command[INIT_COMMAND_MAX_SIZE];  /**< Buffer for storing the init command. */

            uint32_t            boot_validation_crc;
            boot_validation_t   boot_validation_softdevice;
            boot_validation_t   boot_validation_app;
            boot_validation_t   boot_validation_bootloader;

            nrf_dfu_peer_data_t peer_data;          /**< Not included in calculated CRC. */
            nrf_dfu_adv_name_t  adv_name;           /**< Not included in calculated CRC. */
        } nrf_dfu_settings_t;
    '''
    def __init__(self, byte_arr):
        self.raw_data = byte_arr
        self.parse()

    def parse(self):
        '''
            CRC等于[settings_version, enter_buttonless_dfu]区间88bytes的CRC32
        '''
        self.crc, self.settings_version, self.app_version, self.bootloader_version,\
            self.bank_layout, self.bank_current = struct.unpack('6I', self.raw_data[0:24])

        self.bank0_size, self.bank0_crc, self.bank0_code, self.bank1_size,\
            self.bank1_crc, self.bank1_code =  struct.unpack('6I', self.raw_data[24:48])
        
        self.write_offset, self.sd_size =  struct.unpack('2I', self.raw_data[48:56])
        self.progress = struct.unpack('8I', self.raw_data[56:88])
        self.enter_buttonless_dfu = struct.unpack('I', self.raw_data[88:92])[0]

        self.init_command = []
        if len(self.raw_data) >= 256+92:
            self.init_command[:] = self.raw_data[92:256+92]

    def __repr__(self):
        tips = '---bootsettings messages:---\n'
        tips += '    CRC:                0x' + hex(self.crc)[2:].zfill(8) + '\n'
        tips += '    settings_version:   ' + str(self.settings_version) + '\n'
        tips += '    app_version:        ' + str(self.app_version) + '\n'
        tips += '    bootloader_version: ' + str(self.bootloader_version) + '\n'
        tips += '    bank_layout:        ' + str(self.bank_layout) + '\n'
        tips += '    bank_current:       ' + str(self.bank_current) + '\n'
        tips += '    bank0_size:         ' + str(self.bank0_size) + '\n'
        # tips += '    bank0_crc:          ' + str(self.bank0_crc) + '\n'
        tips += '    bank0_crc:          0x' + hex(self.bank0_crc)[2:].zfill(8) + '\n'

        tips += '    bank0_code:         ' + str(self.bank0_code) + '\n'
        tips += '    bank1_size:         ' + str(self.bank1_size) + '\n'
        # tips += '    bank1_crc:          ' + str(self.bank1_crc) + '\n'
        tips += '    bank1_crc:          0x' + hex(self.bank1_crc)[2:].zfill(8) + '\n'

        tips += '    bank1_code:         ' + str(self.bank1_code) + '\n'
        tips += '    write_offset:       ' + str(self.write_offset) + '\n'
        tips += '    sd_size:            ' + str(self.sd_size) + '\n'
        tips += '    update_start_addr:  0x' + hex(self.progress[4])[2:].zfill(8) + '\n'

        tips += '    enter_btnless_dfu:  ' + str(self.enter_buttonless_dfu) + '\n'
        return tips

def console_print(api, addr, size):
    '''
        内部存储dfu settings打印
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")
    
    if size < 256:
        raise Exception("Size if small, must be largger than 256!")

    rd_list = api.read(addr, size)
    bss = BootSettingsStruct(bytearray(rd_list))
    print(bss)
    ''' 去掉注释，可查看二进制原始内容 '''
    # for x in range(len(rd_list)//8):
    #     print([' '.join(hex(v)[2:].zfill(2) for v in rd_list[x*8 : x*8+8])])

def file_crc32(file):
    '''
        计算bin文件CRC32
    '''
    with open(file, 'rb') as f:
        value = f.read()
        return len(value), crc32(value)

def dfu_update_application(api, file, dat):
    '''
        同时更新application bin文件以及dfu settings bank1内容，指定存储到内部flash，
        重启后，BootLoader把这哥application更新为当前应用
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    bank1_addr = 0
    # 更新boootsettings
    size, crc = file_crc32(file)
    btst = bytearray(api.read(BOOTSETTINGS['ADDR'], 1024))
    bank0_size = struct.unpack("I", btst[24:28])[0]
    bank1_addr = ((APPLICATION['ADDR'] + bank0_size + 4096-1)//4096)*4096

    # print(btst[0:100])

    # bank1 配置
    btst[20:24] = struct.pack("I", 1)
    btst[36:48] = struct.pack("3I", size, crc, 1)
    
    # command_size command_offset
    btst[56:64] = struct.pack("2I", 0x8D, 0x8D)
    # update_start_address 配置
    btst[72:76] = struct.pack("I", bank1_addr)

    btst_crc = crc32(btst[4:92])

    # init_command
    with open(dat, 'rb') as datf:
        dat_bin = datf.read()
        btst[92: 92+0x8D] = dat_bin[:]
    btst[0:4] = struct.pack("I", btst_crc)

    for x in range(size//0x1000+1):
        api.erase_page(bank1_addr+x*0x1000)
    
    api.erase_page(BOOTSETTINGS['ADDR'])
    api.write(BOOTSETTINGS['ADDR'], btst, True)

    api.erase_page(MBR_PARAM['ADDR'])
    api.write(MBR_PARAM['ADDR'], btst, True)

    with open(file, 'rb') as f:
        api.write(bank1_addr, f.read(), True)

if __name__ == '__main__':
    # api = API.API('NRF52', jlink_arm_dll_path=JLINKARM_DLL)

    api = API.API('NRF52')
    api.open()
    api.enum_emu_snr()
    api.connect_to_emu_without_snr()

    # base_info_print(api)
    '''
        内部flash操作
    '''
    # dfu_update_application(api, r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VB.bin', r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VB.dat')
    dfu_update_application(api, r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VR.bin', r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VR.dat')
    console_print(api, BOOTSETTINGS['ADDR'], 256)


    api.sys_reset()
    api.go()
    api.disconnect_from_emu()
    api.close()
