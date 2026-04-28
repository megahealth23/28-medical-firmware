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

def base_info_print(api):
    '''
        DK 板及Jlink信息打印
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    print('|--------------------device info------------------------|')
    print(api.read_connected_emu_snr())
    print(api.read_connected_emu_fwstr())
    print(api.dll_version())
    print(api.read_device_version())
    print('|-----------------------end-----------------------------|\n')

def file_crc32(file):
    '''
        计算bin文件CRC32
    '''
    with open(file, 'rb') as f:
        value = f.read()
        return len(value), crc32(value)

def burn_image(api, file, addr):
    '''
        把application bin文件烧录到指定addr位置
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    fbin = open(file, r'rb')
    content = fbin.read()
    size = len(content)
    fbin.close()
    if (APPLICATION['ADDR']<addr<(APPLICATION['ADDR']+APPLICATION['SIZE']))\
        and size < APPLICATION['SIZE']//2:
        api.write(addr, content, True)
    else:
        raise Exception("file can't be receipted")

def update_application(api, file):
    '''
        同时更新application bin文件以及dfu settings bank1内容，指定存储到内部flash，
        重启后，BootLoader把这哥application更新为当前应用
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    bank1_addr = 0x75000
    # 更新boootsettings
    size, crc = file_crc32(file)
    btst = bytearray(api.read(BOOTSETTINGS['ADDR'], 1024))
    # print(btst[0:100])
    # bank1 配置
    btst[20:24] = struct.pack("I", 1)
    btst[36:48] = struct.pack("3I", size, crc, 1)
    
    # update_start_address 配置
    btst[72:76] = struct.pack("I", bank1_addr)

    btst_crc = crc32(btst[4:92])

    btst[0:4] = struct.pack("I", btst_crc)

    for x in range(size//0x1000+1):
        api.erase_page(bank1_addr+x*0x1000)
    
    api.erase_page(BOOTSETTINGS['ADDR'])
    api.write(BOOTSETTINGS['ADDR'], btst, True)

    api.erase_page(MBR_PARAM['ADDR'])
    api.write(MBR_PARAM['ADDR'], btst, True)

    with open(file, 'rb') as f:
        api.write(bank1_addr, f.read(), True)

    # api.pin_reset()
    # print(btst[0:100])

def update_application_new(api, file, dat):
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

    # api.pin_reset()
    # print(btst[0:100])

def EXT_APP_SETTINGS_ADDR(id):
    '''
        与id号对应的外部存储settings地址
    '''
    return (0x1000*id)

def EXT_APP_BIN_ADDR(id):
    '''
        与id号对应的外部存储bin文件地址，与settings尾部间隔2个4K
    '''
    return (0x1000*(EXT_IMG_MAX_NUM+2)+EXT_APP_MAX_SIZE*id)	#保留了8K的缓冲区

# typedef struct _ext_settings
# {
# 	uint32_t crc;	//the bank value
# 	nrf_dfu_bank_t bank;
# 	uint32_t ext_start_addr;
# 	uint32_t create_time;
# 	uint32_t last_modified_time;
# 	uint16_t app_type;
# 	uint16_t app_active;
# 	uint32_t app_version;
# 	uint8_t descrip[8];
# 	// imgae copy progress
# 	int page_rw_num;
# }ext_settings_t;

class ExtSettings(object):
    '''
        外部存储settings内容解析
    '''
    def __init__(self, id, img_dict):
        self.crc = 0
        self.image_size = img_dict['image_size']
        self.image_crc = img_dict['image_crc']
        self.bank_code = img_dict['bank_code']
        self.ext_start_addr = EXT_APP_BIN_ADDR(id)
        self.create_time = int(time.time())
        self.last_modified_time = self.create_time
        self.app_type = img_dict['app_type']
        self.app_active = 0
        self.app_version = img_dict['app_version']
        self.descrip = img_dict['descrip']
        print(hex(self.ext_start_addr))

    def get_bytes(self):
        '''
            计算外部settings的crc32值，返回组装好的bytearrays
        '''
        bytes_arr = struct.pack("6I2H1I8s", self.image_size, self.image_crc, self.bank_code,\
                                self.ext_start_addr, self.create_time, self.last_modified_time,\
                                self.app_type, self.app_active, self.app_version, self.descrip)
        self.crc = crc32(bytes_arr)
        # print(struct.unpack("6I2H1I8s", bytes_arr))
        return (struct.pack("I", self.crc) + bytes_arr)

    def __repr__(self):
        tips = "-----------extern image settings info-----------------" + '\n'
        tips += '       crc                ' + hex(self.crc) + '\n'
        tips += '       image_size         ' + hex(self.image_size) + '\n'
        tips += '       image_crc          ' + hex(self.image_crc) + '\n'
        tips += '       bank_code          ' + str(self.bank_code) + '\n'
        tips += '       ext_start_addr     ' + hex(self.ext_start_addr) + '\n'
        tips += '       create_time        ' + str(time.ctime(self.create_time)) + '\n'
        tips += '       last_modified_time ' + str(time.ctime(self.last_modified_time)) + '\n'
        tips += '       app_type           ' + hex(self.app_type) + '\n'
        tips += '       app_active         ' + hex(self.app_active) + '\n'
        tips += '       app_version        ' + hex(self.app_version) + '\n'
        tips += '       descrip            ' + str(self.descrip) + '\n'
        return tips

def burn_ext_img(api, id, file):
    '''
        把application bin文件以及对应的外部settings烧录到对应的id号外部存储
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")
    
    QSPI_BASE = EXT_APP_SETTINGS_ADDR(id)
    QSPI_BIN  = EXT_APP_BIN_ADDR(id)
    size, crc = file_crc32(file)
    print(hex(QSPI_BASE))
    print(hex(QSPI_BIN))
    print(hex(size), hex(crc))
    # return print(size, crc)
    api.qspi_init(retain_ram=True)

    # settings 参数设置
    api.qspi_erase(QSPI_BASE, QSPIEraseLen.ERASE4KB)

    img_dict = {'image_size': size, 'image_crc':crc, 'bank_code':1, 'app_type':0x01, 'app_version':0x01, 'descrip':bytearray(8)}
    ext = ExtSettings(id, img_dict)
    print(ext)
    api.qspi_write(QSPI_BASE, ext.get_bytes())
    
    # print(img_settings)
    # print(list(api.qspi_read(0, len(img_settings))))

    # bin 参数设置
    for addr in range(math.ceil(size/0x1000)):
        api.qspi_erase(QSPI_BIN+addr*0x1000, QSPIEraseLen.ERASE4KB)

    with open(file, 'rb') as f:
        api.qspi_write(QSPI_BIN, f.read())
        rd_bin = api.qspi_read(QSPI_BIN, size)
        if crc == crc32(rd_bin):
            print("ok")
        else:
            print("error")
    api.qspi_uninit()

def sacn_ext_image(api):
    '''
        列出外部存储有多少个有用的application，及其相关详细信息
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")
    
    api.qspi_init(retain_ram=True)
    
    images = []

    for x in range(EXT_IMG_MAX_NUM):
        base = EXT_APP_SETTINGS_ADDR(x)
        ext_sts_byts = api.qspi_read(base, EXT_APP_SETTINGS_LENGTH)
        parse = struct.unpack("7I2H1I8s", ext_sts_byts)
        if parse[0] == crc32(ext_sts_byts[4:]):
            valid_one = {'id':x, 'active':parse[8], 'app_type':parse[7], 'details':parse}
            images.append(valid_one)

    api.qspi_uninit()

    return images

def ext_check(api, id):
    '''
        坚持对应id处外部存储空间是否有相关的application
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    api.qspi_init(retain_ram=True)
    
    QSPI_BASE = EXT_APP_SETTINGS_ADDR(id)
    QSPI_BIN  = EXT_APP_BIN_ADDR(id)
    
    ext_sts_byts = api.qspi_read(QSPI_BASE, EXT_APP_SETTINGS_LENGTH)
    
    print(ext_sts_byts)
    parse = struct.unpack("7I2H1I8s", ext_sts_byts)
    print(parse)

    img_dict = {'image_size': parse[1], 'image_crc':parse[2], 'bank_code':parse[3], 'app_type':parse[7], 'app_version':parse[9], 'descrip':parse[10]}
    ext = ExtSettings(id, img_dict)
    ext.ext_start_addr = parse[4]
    ext.create_time = parse[5]
    ext.last_modified_time = parse[6]
    ext.app_active = parse[8]
    ext.crc = parse[0]
    print(ext)

    if parse[0] == crc32(ext_sts_byts[4:]):
        print("settings ok")
        bin_file = api.qspi_read(QSPI_BIN, ext.image_size)
        if ext.image_crc == crc32(bin_file):
            print("bin file ok")
        else:
            print("bin file error")
    else:
        print("settings error")

    api.qspi_uninit()

def ext_eraseall(api):
    '''
        擦除所有外部存储内容
    '''
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    api.qspi_init(retain_ram=True)

    for x in range(EXT_IMG_MAX_SIZE//(0x1000*16)):
        print(x)
        api.qspi_erase(x*0x1000*16, QSPIEraseLen.ERASE64KB)

    api.qspi_uninit()


def test(api):
    if not api.is_connected_to_emu():
        raise Exception("Operation failed, connect to the emulator first!")

    api.erase_page(0x75000)
    api.write(0x75000, bytearray([0x12, 0x34, 0x56, 0x78]), True)
    print(api.read_u32(0x75000))
    print(api.read(0x75000, 4))

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
    update_application_new(api, r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VB.bin', r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VB.dat')
    # update_application_new(api, r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VR.bin', r'E:\SVN\EEG\MegaEEGGroup\scripts\MegaEEG_VR.dat')
    console_print(api, BOOTSETTINGS['ADDR'], 256)
    console_print(api, MBR_PARAM['ADDR'], 256)
    # data = api.read(0x26000, 33620)
    # print(crc32(bytearray(data)))
    # data = api.read(0x8b000, 33604)
    # print(crc32(bytearray(data)))

    # with open('test.bin', 'wb') as f:
    #     f.write(bytearray(data))
   
    '''
        外部flash操作
    '''
    # burn_ext_img(api, 0, "one.bin")
    # burn_ext_img(api, 1, "two.bin")
    # ext_check(api, 1)
    # print(sacn_ext_image(api))
    api.sys_reset()
    api.go()
    api.disconnect_from_emu()
    api.close()
