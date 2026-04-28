import socket

MHEAD = "4D52"  # P11X

class C11X(object):
    SIZE = ["大号", "中号", "小号"]
    TYPE = ["C11E", "P11E", "P11F", "C11H", "P11G", "P11H"]

    def __init__(self, sn):
        ver_yy_mm = sn[0]*2**8+sn[1]
        self.sn_version = (ver_yy_mm>>13)&0x07
        self.year = (ver_yy_mm>>7)&0x03F
        self.month = (ver_yy_mm>>3)&0x0F
        self.number = sn[2]*2**16 + sn[3]*2**8 + sn[4]
        #self.medicine_consumer = (sn[5]>>5) & 0x01  废弃
        self.medicine_consumer = (sn[5]>>5) & 0x07; #高3位
        self.sizeType = sn[5]
        self.size = (sn[5]>>4) & 0x01
        self.type_ = sn[5] & 0x0F
        self.nSize = (sn[5] & 0xF0) >> 4

    def get_sn(self):
        if(self.type_ == 1) :
            sn_str = f'C11E{str(self.nSize+7)}{str(self.year).zfill(2)}{str(self.month).zfill(2)}{str(self.number).zfill(6)}'
        elif(self.sizeType == 0x1d):
            sn_str = f'C11E4{str(self.year).zfill(2)}{str(self.month).zfill(2)}{str(self.number).zfill(6)}'
        else:  
            sn_str = f'{self.TYPE[self.medicine_consumer]}{self.size+1}{str(self.year).zfill(2)}{str(self.month).zfill(2)}{str(self.number).zfill(6)}'
        return sn_str

    def __repr__(self):
        message = ''
        message += f'\nversion            : {self.sn_version}' 
        message += f'\nyear               : {self.year}'
        message += f'\nmonth              : {self.month}'
        message += f'\nnumber             : {self.number}'
        message += f'\nM/C                : {self.MED_CUS[self.medicine_consumer]}'
        message += f'\nsize               : {self.SIZE[self.size]}'
        message += f'\ntype               : {self.type_}'
        message += f'\nSN                 : {self.TYPE[self.medicine_consumer]}{self.size+2}{str(self.year).zfill(2)}{str(self.month).zfill(2)}{str(self.number).zfill(6)}'
                
        return message 

class DeviceSN(object):

    @staticmethod
    def parse_ver0(sn_bytes):
        device_type = ['E01A', 'P11B', 'P11C', 'P11D', 'E11D', None, None, 'P11T']
        type_val = sn_bytes[5]&0x07
        size = (sn_bytes[5]>>3)&0x0f
        sn = device_type[type_val] + str(size) + str(sn_bytes[0]).zfill(2) + str(sn_bytes[1]).zfill(2)
        sn += str(sn_bytes[2]*2**16+sn_bytes[3]*2**8+sn_bytes[4]).zfill(6)
        return sn

    @staticmethod
    def parse_ver1(sn_bytes): # 后4位 type  0x1d(C11E4)
        if (sn_bytes[5] & 0x0F) == 5 or (sn_bytes[5] == 0x1d ) \
            or (sn_bytes[5] & 0b1111 == 1 ): # C11Ex(C11H)
            c11x = C11X(sn_bytes)
            return c11x.get_sn()
        else:
            raise Exception(f'设备类型异常:{sn_bytes[5] & 0x0F}')

    @staticmethod
    def code_sn(sn_str, hard_io_ver):
        type    = sn_str[0:4]
        size    = int(sn_str[4:5])
        year    = int(sn_str[5:7])
        month   = int(sn_str[7:9])
        num     = int(sn_str[9:15])
        # hard_io = 1
        
        if ( type == 'C11F'):
            v_y_m   = (0b011 << 13)+ (year << 7) + (month << 3) + hard_io_ver
            sizeType = (0b000 << 5) + (size << 2) + 0b11
            rslt = str(hex(v_y_m) ).zfill(4)[2:7] + str( hex(num))[2:9].zfill(6) + str( hex(sizeType))[2:4].zfill(2)
        return rslt

    @staticmethod
    def decode_sn(sn_hex):
        v_y_m = int(sn_hex[0:4],16) # version year month 
        snVer = v_y_m >> 13

        if snVer == 0b011:
            t_s_m = int(sn_hex[10:11],16) #type size mark
            device_type = ['C11F']
            type_val = (t_s_m&0xE0) >> 5
            size = (t_s_m&0x18) >> 2
            yy = (v_y_m&0x1F80) >> 7
            mm = (v_y_m&0x78) >> 3
            sn_num = int(sn_hex[4:10],16)
            sn = device_type[type_val] + str(size) + str(yy).zfill(2) + str(mm).zfill(2) + str(sn_num).zfill(2)
            return sn
        else:
            raise Exception(f'版本号异常:{DeviceSN.get_ver(sn_bytes[0])}')

class SN2HEX(object):
    @staticmethod
    def hexChecksum(s):
        length = len(s)  #求长度
        #创建一个list，将传入的str的每两个数合在一起，再求和
        list1 = []
        if(length%2 == 1):    #如果str长度为单数，则抛出错误
            print('[!] 数据长度有误')
        else:
            #print(length)
            for i in range(0, length, 2):  #range（开始，结束-1，每次加多少）  这里即0——length-1  每次循环i+2
                hex_digit = s[i:i + 2]      #将传入的str的每两个数合在一起
                list1.append('0x'+hex_digit)    #再每个字符前+0x  但是它仍然是字符，但更便于下面通过int(list1[i], 16)转换成16进制
        #print(list1)
        sum = 0
        for i in range(int(length/2)):   #求和
            sum = int(list1[i], 16) + sum      #int(list1[i], 16)将16进制转换成10进制 int类型
        sum = sum%256
        sum = 256-sum
        #print('校验码: '+hex(sum))   #将sum和结果转换成16进制  hex(sum)
        return hex(sum)

    @staticmethod
    def hexOfSnMac(strSN, strMac, hard_io_ver):
        sn = DeviceSN.code_sn(strSN,hard_io_ver)
        hexSn = sn.upper()
        print(hexSn)

        s1 = strMac.replace('-','')
        if( len(s1) > 12):
            s1 = strMac.replace(':','')
            
        m = int( s1, 16)
        mac = hex(m).upper()
        #print(mac)
    
        #0xBCE59F4874C7 --> 'BC:E5:9F:48:74:C7'
        txtmac =mac[2:4] + ':' + mac[4:6] + ':' + mac[6:8] +\
            ':' + mac[8:10] + ':' + mac[10:12] + ':' + mac[12:14]
        #print(txtmac)

        ftFlag      = 1  # 2 bytes represent factory testing status.
        HexHeader   = ':020000041000EA'
        HexAddr1    = "10108000" # 0x10 length, 0x1080 address, 0x00 datatype.
        HexAddr2    = "10109000" # 0x10 length, 0x1090 address, 0x00 datatype.
        HexEnd      = ":00000001FF"

        sFt =  f'{ftFlag:0>4}' # Fill space in the string.
        #L1 = f'{mac[2:14]}' + hexSn + hexSn[0:4]  # padding 2 bytes.
        #checkSum = SN2HEX.hexChecksum( HexAddr1 + sFt + L1 )
        #sLine1 = ":" + HexAddr1 + sFt + L1 + str(checkSum).upper()[2:4]
        L1 =  HexAddr1 + str(MHEAD) + sFt + f'{mac[2:14]}' + hexSn
        checkSum = SN2HEX.hexChecksum( L1 )
        sLine1 = ":" + L1 + str(checkSum).upper()[2:4]
        h = HexHeader + '\n' + sLine1 + '\n' + HexEnd + '\n'
        return h

    @staticmethod
    def hexOfConfig(strSN, hard_io_ver):
        sn = DeviceSN.code_sn(strSN, hard_io_ver)
        hexSn = sn.upper()
#        print(hexSn)
#        print(DeviceSN.decode_sn(hexSn))

        blVer      = 12  # 2 bytes represent factory testing status.
        HexHeader   = ':02000004000FEB'
        HexAddr1    = "10000000" # 0x10 length, 0xD000 address, 0x00 datatype.
        HexEnd      = ":00000001FF"

        HEAD = '454547'
        sBL =  f'{blVer}'

        L1 = HexAddr1 + str(HEAD) + sBL  + hexSn +'00000000000000'
        checkSum = SN2HEX.hexChecksum( L1 )
        sLine1 = ":" + L1 + str(checkSum).upper()[2:4]
        hex_content = HexHeader + '\n' + sLine1 + '\n' + HexEnd + '\n'

        L2 = str(HEAD) + sBL  + hexSn +'00000000000000'
        byte_content = bytes.fromhex(L2)
#        print("L2: "+ L2)
#        print( byte_content)

        return hex_content, byte_content

if __name__ == "__main__":

    sn_str = "C11F10307989898"
    # "E01A02111000008"
    # "E01A12111000008"
    sn_hex = DeviceSN.code_sn(sn_str, 0)
    print (sn_hex)
    sn_str = DeviceSN.decode_sn(sn_hex)
    print (sn_str)
    sn_hex = DeviceSN.code_sn(sn_str, 0)
    print (sn_hex)