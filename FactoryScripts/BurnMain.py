# -*- coding: UTF-8 -*-
import sys
import shutil
from PyQt5.QtGui import QFont, QBrush, QColor, QStandardItemModel, QStandardItem, QIcon, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QThread
from PyQt5.QtWidgets import (QWidget, QApplication, QMainWindow, QMessageBox, QToolTip, QVBoxLayout,
                             QHBoxLayout, QGroupBox, QGridLayout, QLabel, QComboBox, QLineEdit, QDateEdit,
                             QPushButton, QProgressBar, QTableView, QAbstractItemView, QHeaderView, QFileDialog,
                             QAction, qApp, QDesktopWidget)
import time

try:
    from pynrfjprog import API, Hex
except ImportError:
    print("\n1) SourceCode's URL: \n\t <https://github.com/NordicSemiconductor/pynrfjprog>")
    print("2) Install: \n\t<python setup.py install>\n")
    print("or\t")
    print("    Run:\n\t <pip install pynrfjprog>\n")

import os
try:
    import simplejson as json
except ImportError:
    import json
import logging
import sqlite3
import random

from version_ import PC_VER
from utils_ import make_qrcode, get_macsn, findUsbDisk
from nrfjprog import NRF5xBurn, NRFErrorCode
from device_sn import DeviceSN, SN2HEX

from configures import config, MAC_SN_ADDR, UCIR_ADDR, EEG_TYPE_LIST


USB_NAME = 'KINGSTON'

GREEN_BUTTON = '''background-color: #0f0 ; height:40px; border-style: outset; \
border-width: 2px; border-radius: 10px; border-color: beige; font: bold 24px; min-width: 10em; padding: 6px;\
'''
YELLOW_BUTTON = '''background-color: yellow ; height:40px; border-style: outset; \
border-width: 2px; border-radius: 10px; border-color: beige; font: bold 24px; min-width: 10em; padding: 6px;\
'''
RED_BUTTON = '''background-color: #f00 ; height:40px; border-style: outset; \
border-width: 2px; border-radius: 10px; border-color: beige; font: bold 24px; min-width: 10em; padding: 6px;\
'''
WHITE_BUTTON = '''background-color: white ; height:40px; border-style: outset; \
border-width: 2px; border-radius: 10px; border-color: beige; font: bold 24px; min-width: 10em; padding: 6px;\
'''

BURN_BTN_COLOR = {'g': GREEN_BUTTON, 'r': RED_BUTTON,
                  'y': YELLOW_BUTTON, 'w': WHITE_BUTTON}

MHEAD = "4d52"  # 'MR'
EHEAD = "454547"  # 'EEG'

CONFIG = config()

base_path = os.getcwd()

DEBUG = False


def localLogger(enableConsole=False):
    # 第一步，创建一个logger
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)  # Log等级总开关

    # 第二步，创建一个handler，用于写入日志文件
    if( os.path.isdir('./log') != True):
        os.makedirs('./log')
    logfile = './log/logger.txt'
    fh = logging.FileHandler(logfile, mode='a')
    fh.setLevel(logging.DEBUG)  # 输出到file的log等级的开关

    # 第三步，再创建一个handler，用于输出到控制台
    if(enableConsole):
        ch = logging.StreamHandler()
        ch.setLevel(logging.WARNING)  # 输出到console的log等级的开关

    # 第四步，定义handler的输出格式
    formatter = logging.Formatter(
        "%(asctime)s - %(filename)s[line:%(lineno)4d] - %(levelname)8s: %(message)s")
    fh.setFormatter(formatter)
    if(enableConsole):
        ch.setFormatter(formatter)

    # 第五步，将logger添加到handler里面
    logger.addHandler(fh)
    if(enableConsole):
        logger.addHandler(ch)
    return logger


def RingsqlWrite(item):
    if(len(item) == 0):
        QMessageBox.warning("警告", "你写了个空文件！", QMessageBox.Yes)
        return
    if(os.path.isdir('./db') != True):
        os.makedirs('./db')

    cx = sqlite3.connect("./db/mringBurnRst.db")
    cu = cx.cursor()
    # 查询是否有表
    sql = r"SELECT count(*) FROM sqlite_master WHERE type='table' AND name='mringBurnRst'"
    ret = cx.execute(sql).fetchall()
    if(ret[0][0] == 0):
        cu.execute("create table mringBurnRst (id integer primary key, time varchar, " +
                   "nordicId varchar, hexBurnRst varchar, mac varchar, sn varchar, hw varchar)")
    else:
        sql = r"SELECT sql FROM sqlite_master WHERE type='table' AND name='mringBurnRst'"
        tab = cx.execute(sql).fetchall()
        if(tab[0][0].find('hw') == -1):
            cu.execute("alter table mringBurnRst add hw varchar")
    cx.execute("insert into mringBurnRst values (null,?,?,?,?,?,?)", tuple(item))
    cx.commit()
    cu.close()
    cx.close()


class ClickLabel(QLabel):
    clicked = pyqtSignal()

    def mouseReleaseEvent(self, QMouseEvent):
        if(QMouseEvent.button() == Qt.LeftButton):
            self.clicked.emit()

class MCRingBurnWindow(QMainWindow):

    qrclicked = pyqtSignal()
    sig_macsn = pyqtSignal(list)

    def __init__(self):
        super(MCRingBurnWindow, self).__init__()
        self.logger = localLogger()
        try:
            if( CONFIG[0] ):
                self.config = CONFIG[1]
            else:
                self.config = None
                raise Exception(CONFIG[1])

            self.initUi()
            self.initFile()

            self.resize(1500, 800)

            #居中
            screen = QDesktopWidget().screenGeometry()
            size = self.geometry()
            self.setGeometry((screen.width()-size.width())/2+screen.left(),
                             (screen.height()-size.height())/2,
                             size.width(), size.height())

            # 默认选择第一个
            if(not DEBUG):
                if(self.config and len(self.config['mac_sn'].keys()) == 1):
                    self.receive(list(self.config['mac_sn'].keys())[0])
            else:
                self.receive(list(self.config['mac_sn'].keys())[0])

            if(not DEBUG):
                self.api = API.API('NRF52')
                # self.api = API('NRF52')
                # self.api.open()
                if( not NRF5xBurn.self_check(self.api) ):
                    raise Exception('请正确连接JLink')
            else:
                self.api = None

            self.is_ready_burn = False

            self.burn_thd = BurnThread(self.api, self.logger,
                                       self.hexMegaRingFile, self.config)
            self.burn_thd.sig_burn_msg.connect(self.burn_result)
            self.burn_thd.sig_bar.connect(self.process_bar)
            self.burn_thd.sig_steps.connect(self.burnbtn_update)
            self.sig_macsn.connect(self.burn_thd.burn_on)
            self.burn_thd.start()

        except Exception as e:
            QMessageBox.critical(self, "错误", repr(e), QMessageBox.Yes)
            self.close()
            self.logger.warning("初始化失败: "+repr(e))
            os._exit(0)

    def initUi(self):
        centerWindow = QWidget()
        self.setCentralWidget(centerWindow)

        QToolTip.setFont(QFont('SansSerif', 20))
        self.createGridGroupBox()
        self.creatVboxGroupBox()
        self.creatFormGroupBox()
        self.createDetailsLayout()
        self.createQrGroupBox()
        self.createMenu()

        mainLayout = QVBoxLayout()
        hboxLayout = QHBoxLayout()
        hboxLayout.addStretch()
        hboxLayout.addWidget(self.detailsGroupBox, 2)
        hboxLayout.addWidget(self.gridGroupBox, 2)
        hboxLayout.addWidget(self.vboxGroupBox, 1)
        hboxLayout.addWidget(self.QrGroupBox, 1)
        mainLayout.addLayout(hboxLayout)

        mainLayout.addWidget(self.tableGroupBox)
        # self.setLayout(mainLayout)
        centerWindow.setLayout(mainLayout)
        self.hexMegaRingFile = None
        self.jsonMacSnFile = None
        self.devChooseSize = None
        self.logger.info("程序开启")

    def initFile(self):
        if(self.config != None):
            logging.info("查找到默认烧录文件")
            self.hexMegaRingFile = os.getcwd() + '\\' + \
                self.config['factory_hex_path'].replace('/', '\\')
            self.hexLineEdit.setText(self.hexMegaRingFile)
            self.logger.info("查找到默认HEX烧录文件，路径为：" + self.hexMegaRingFile)

        else:
            # QMessageBox.warning(self, "警告", "没有找到烧录文件，请手动选择！", QMessageBox.Yes)
            QMessageBox.critical(
                self, "配置文件出错", '请检查"config.json"文件！', QMessageBox.Yes)
            self.logger.error("没有找到烧录文件，请手动选择")

    def readMacSn(self):
        if(self.jsonMacSnFile is not None):
            with open(self.jsonMacSnFile, 'r', encoding='utf-8') as f:
                data = json.load(f)
                value = get_macsn(data, self.config['hardware_io_ver'])
                if(value):
                    if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                        self.macLineEdit.setText("%s忽略此项"%(self.config['name']))
                    else:
                        self.macLineEdit.setText(value[1])
                    self.snLineEdit.setText(value[0])
                    self.hex_sn = value[2]
                    self.logger.info(
                        "读取到有效的MAC:" + value[1] + " SN:" + value[0])
                    self.is_ready_burn = True
                else:
                    self.macLineEdit.setText("00:00:00:00:00:00")
                    self.snLineEdit.setText("000000000000000")
                    self.hex_sn = None
                    self.devTypeChooseBtn.setStyleSheet(
                        "background-color: #f00; font: bold 20px; height:60px;")
                    self.devTypeChooseBtn.setText('无效')
                    QMessageBox.warning(
                        self, "警告", "没有可用的MAC和SN了！", QMessageBox.Yes)
                    self.logger.error("没有可用的MAC和SN了！")
                    self.is_ready_burn = False
        else:
            self.macLineEdit.setText("00:00:00:00:00:00")
            self.snLineEdit.setText("000000000000000")
            self.devTypeChooseBtn.setStyleSheet(
                "background-color: #f00; font: bold 20px; height:60px;")
            self.devTypeChooseBtn.setText('无效')

            if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                QMessageBox.warning(
                    self, "警告", "没有SN的json文件！", QMessageBox.Yes)
                self.logger.error("没有SN的json文件！")
            else:
                QMessageBox.warning(
                    self, "警告", "没有MAC和SN的json文件！", QMessageBox.Yes)
                self.logger.error("没有MAC和SN的json文件！")

            self.is_ready_burn = False

    def updateMacSn(self):
        if(self.jsonMacSnFile is not None):
            with open(self.jsonMacSnFile, 'r') as f:
                data = json.load(f)

                if(data['burned'] < data['num']):
                    data['burned'] = data['burned'] + 1

                    for i, v in enumerate(self.devTitleRaw):
                        if(v == self.devChooseSize):
                            self.devBurnLabel[i].setText(str(data['burned']))
                            self.devUnBurnLabel[i].setText(
                                str(data['num']-data['burned']))

                    value = get_macsn(data, self.config['hardware_io_ver'])
                    json.dump(data, open(self.jsonMacSnFile, 'w'),
                              sort_keys=True, indent=4)

                    self.logger.info("MAC:" + self.macLineEdit.text() +
                                     " SN:" + self.snLineEdit.text() + "已被写入设备，并在json文件里标记成功")
                    if(value):   
                        if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                            self.macLineEdit.setText("%s忽略此项"%(self.config['name']))
                        else:
                            self.macLineEdit.setText(value[1])

                        self.snLineEdit.setText(value[0])
                        self.hex_sn = value[2]
                        
                        if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                            self.logger.info(
                                "新的 SN:" + value[0] + "已准备完毕")
                        else:
                            self.logger.info(
                                "新的MAC:" + value[1] + " SN:" + value[0] + "已准备完毕")
                        self.is_ready_burn = True
                    else:
                        self.macLineEdit.setText("00:00:00:00:00:00")
                        self.snLineEdit.setText("000000000000000")
                        self.hex_sn = None
                        self.devTypeChooseBtn.setStyleSheet(
                            "background-color: #f00; font: bold 20px; height:60px;")
                        self.devTypeChooseBtn.setText('无效')

                        if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                            self.logger.info("SN已使用完毕")
                        else:
                            self.logger.info("MAC和SN已使用完毕")

                        self.is_ready_burn = False
                else:
                    self.macLineEdit.setText("00:00:00:00:00:00")
                    self.snLineEdit.setText("000000000000000")
                    self.devTypeChooseBtn.setStyleSheet(
                        "background-color: #f00; font: bold 20px; height:60px;")
                    self.devTypeChooseBtn.setText('无效')

                    if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                        QMessageBox.warning(
                            self, "警告", "没有SN的json文件！", QMessageBox.Yes)
                        self.logger.error("没有SN的json文件！")
                    else:
                        QMessageBox.warning(
                            self, "警告", "没有MAC和SN的json文件！", QMessageBox.Yes)
                        self.logger.error("没有MAC和SN的json文件！")

                    self.is_ready_burn = False

        else:
            if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                QMessageBox.warning(
                    self, "警告", "没有SN的json文件！", QMessageBox.Yes)
                self.logger.error("没有SN的json文件！")
            else:
                QMessageBox.warning(
                    self, "警告", "没有MAC和SN的json文件！", QMessageBox.Yes)
                self.logger.error("没有MAC和SN的json文件！")

            self.is_ready_burn = False

    def show_msg_qrcode(self, msg):
        if(msg):
            make_qrcode(msg, 'qrcode.png')
            pix = QPixmap('qrcode.png')
            self.qr_label.setPixmap(pix)
        else:
            pix = QPixmap('invalid.jpg')
            self.qr_label.setPixmap(pix)

    def createQrGroupBox(self):
        self.QrGroupBox = QGroupBox("二维码")
        layout = QGridLayout()

        self.qr_label = QLabel()
        pix = QPixmap('invalid.jpg')
        self.qr_label.setPixmap(pix)
        self.qr_label.setStyleSheet("border: 2px solid black")
        self.qr_label.setScaledContents(True)
        layout.addWidget(self.qr_label)

        self.QrGroupBox.setLayout(layout)

    def createGridGroupBox(self):
        self.gridGroupBox = QGroupBox("准备栏:")
        self.gridGroupBox.setFont(QFont('Arial', 16))

        self.gridGroupBox.setToolTip("请检查所有烧录文件和连线是否正确")
        layout = QGridLayout()

        hexLabel = QLabel("待烧Hex:")
        self.hexLineEdit = QLineEdit("*.hex")
        self.hexLineEdit.setReadOnly(True)
        macLabel = QLabel("待烧MAC:")
        self.macLineEdit = QLineEdit("00:00:00:00:00:00")
        self.macLineEdit.setReadOnly(True)
        snLabel = QLabel("待烧SN:")
        self.snLineEdit = QLineEdit("000000000000000")
        self.snLineEdit.setReadOnly(True)
        self.hex_sn = None

        self.devTypeChooseBtn = QPushButton("无效")
        self.devTypeChooseBtn.setStyleSheet(
            "background-color: #f00; font: bold 20px; height:60px;")

        layout.setSpacing(10)
        layout.addWidget(hexLabel, 0, 0)
        layout.addWidget(self.hexLineEdit, 0, 1, 1, 3)

        layout.addWidget(macLabel, 2, 0)
        layout.addWidget(self.macLineEdit, 2, 1, 1, 2)
        layout.addWidget(snLabel, 3, 0)
        layout.addWidget(self.snLineEdit, 3, 1, 1, 2)
        layout.addWidget(self.devTypeChooseBtn, 2, 3, 2, 1)

        self.gridGroupBox.setLayout(layout)

        if self.config['name'] in EEG_TYPE_LIST:
            self.setWindowTitle("兆观脑电烧录工具V" + str(PC_VER))
        else:
            self.setWindowTitle("兆观指环烧录工具V" + str(PC_VER))

        # 注册信号槽
        # hexOpenBtn.clicked.connect(self.openHexFile)
        if(self.config != None):
            self.chooseWin = DevChooseWindow()
            self.chooseWin.choose_signal.connect(self.receive)
            self.devTypeChooseBtn.clicked.connect(self.chooseWin.handle_click)

    def scanInputCallback(self):
        b = QMessageBox.information(
            self, "准备烧录", "确定放置好单板？", QMessageBox.Yes | QMessageBox.No)
        if(b == QMessageBox.Yes):
            self.scanKeyLineEdit.setEnabled(False)
            self.scanKeyLineEdit.setText("")
            self.scanKeyID = self.scanKeyLineEdit.text().replace('\n', '')
            if(not self.burnDevice()):
                self.scanKeyLineEdit.setText("")
                self.scanKeyLineEdit.setEnabled(True)
        else:
            self.show_msg_qrcode(None)

    def creatVboxGroupBox(self):
        self.vboxGroupBox = QGroupBox("烧录:")
        layout = QVBoxLayout()
        self.burnBtn = QPushButton("烧录", self)
        # self.burnBtn.setFixedSize(400, 100)
        # self.burnBtn.setToolTip("请确保烧录文件选择正确")

        scanLabel = QLabel("条码框:")
        self.scanKeyLineEdit = QLineEdit()
        self.scanKeyID = ""
        self.scanKeyLineEdit.returnPressed.connect(self.scanInputCallback)
        self.scanKeyLineEdit.setStyleSheet("font: bold 20px; height:32px;")
        self.scanKeyLineEdit.setText("置入鼠标光标后按回车键")

        self.pbar = QProgressBar()
        self.pbar.setValue(0)


        self.burnLabel = QLabel("未烧录")
        self.burnLabel.setStyleSheet("font: bold 20px; height:32px;")

        self.burnBtn.setStyleSheet(BURN_BTN_COLOR['g'])
        # self.burnBtn.setDisabled(True)
        layout.addWidget(scanLabel)
        layout.addWidget(self.scanKeyLineEdit)
        layout.addWidget(self.burnBtn)
        layout.addWidget(self.pbar)
        layout.addWidget(self.burnLabel)
        self.vboxGroupBox.setLayout(layout)
        
        # 注册信号槽
        # self.burnBtn.clicked.connect(self.burnDevice)
        self.burn_btn_enable = False
        if(os.path.exists('button.txt')):
            with open('button.txt', 'r') as f:
                if('burn=1' in f.read()):
                    self.burn_btn_enable = True

        if(self.burn_btn_enable):
            self.burnBtn.clicked.connect(self.burn_click)

    def creatFormGroupBox(self):
        self.tableGroupBox = QGroupBox("输出栏:")
        layout = QVBoxLayout()
        self.model = QStandardItemModel(0, 5)
        self.model.setHorizontalHeaderLabels(
            ['日期', 'Nordic ID', 'HEX固件烧录', 'MAC地址', 'SN序列号', '硬件版本号'])
        self.tableView = QTableView()
        self.tableView.setModel(self.model)
        # 下面代码让表格100填满窗口
        self.tableView.horizontalHeader().setStretchLastSection(True)
        # 设置单元格禁止更改
        self.tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableView.horizontalHeader().setDefaultAlignment(Qt.AlignCenter)
        self.tableView.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.tableView)
        self.tableGroupBox.setLayout(layout)

    def aboutBox(self):
        QMessageBox.about(self, "上海兆观生产烧录软件",
                          ("公司：上海兆观\n生产烧录软件: V%s\n作者：\n" % PC_VER))

    def createDetailsLayout(self):
        self.detailsGroupBox = QGroupBox("本次生产需求:")
        self.detailsGroupBox.setFont(QFont('Arial', 16))
        layout = QGridLayout()

        if(self.config == None):
            return self.detailsGroupBox.setLayout(layout)

        hwLabel = QLabel("硬件: V%s" % (self.config['hardware_ver'][0]))
        swLabel = QLabel("固件: V%s\n" % (
            self.config['factory_firmware_ver'] ))
        btLabel = QLabel("Boot: V%s" % (self.config['boot_ver']))
        hwLabel.setFont(QFont("Roman times", 14, QFont.Bold))
        hwLabel.setAlignment(Qt.AlignCenter)
        swLabel.setFont(QFont("Roman times", 14, QFont.Bold))
        swLabel.setAlignment(Qt.AlignCenter)
        btLabel.setFont(QFont("Roman times", 14, QFont.Bold))
        btLabel.setAlignment(Qt.AlignCenter)

        keys = sorted(self.config['mac_sn'].keys())

        self.devTitleLabel = [QLabel(" ")]
        self.devTitleRaw = ['']
        self.devFullLabel = [QLabel("需求量")]
        self.devBurnLabel = [QLabel("烧录量")]
        self.devUnBurnLabel = [QLabel("剩余量")]

        burned_list = {}
        for k in keys:
            jp = os.getcwd() + '\\' + \
                self.config['mac_sn'][k]['path'].replace('/', '\\')
            burned_list[k] = 0
            with open(jp, 'r') as f:
                data = json.load(f)
                burned_list[k] = data['burned']

        for key in keys:
            self.devTitleLabel.append(
                QLabel('{}({})'.format(key, self.config['mac_sn'][key]["size"])))
            self.devTitleRaw.append(key)
            self.devFullLabel.append(
                QLabel("%d" % self.config['mac_sn'][key]["num"]))
            self.devBurnLabel.append(QLabel(str(burned_list[key])))
            self.devUnBurnLabel.append(
                QLabel("%d" % (self.config['mac_sn'][key]["num"] - burned_list[key])))

        layout.setSpacing(10)
        # 为了对齐好看
        if(len(keys) < 3):
            layout.addWidget(hwLabel, 0, 0)
            layout.addWidget(swLabel, 0, 1)
            layout.addWidget(btLabel, 0, 2)
        else:
            layout.addWidget(hwLabel, 0, 0)
            layout.addWidget(swLabel, 0, 2)
            layout.addWidget(btLabel, 0, 4)

        for i, v in enumerate(self.devTitleLabel):
            v.setFont(QFont("Roman times", 14, QFont.Bold))
            v.setAlignment(Qt.AlignCenter)
            layout.addWidget(v, 1, i)

        for i, v in enumerate(self.devFullLabel):
            if(i == 0):
                v.setFont(QFont("Roman times", 14, QFont.Bold))
            v.setAlignment(Qt.AlignCenter)
            layout.addWidget(v, 2, i)
        for i, v in enumerate(self.devBurnLabel):
            if(i == 0):
                v.setFont(QFont("Roman times", 14, QFont.Bold))
            v.setAlignment(Qt.AlignCenter)
            layout.addWidget(v, 3, i)
        for i, v in enumerate(self.devUnBurnLabel):
            if(i == 0):
                v.setFont(QFont("Roman times", 14, QFont.Bold))
            v.setAlignment(Qt.AlignCenter)
            layout.addWidget(v, 4, i)

        self.detailsGroupBox.setLayout(layout)

    def createMenu(self):
        eraseAction = QAction(QIcon(), '&擦除', self)
        # exitAction.setShortcut('Ctrl+A+T')
        eraseAction.setStatusTip('擦除全部芯片')
        eraseAction.triggered.connect(self.eraseAllAction)

        rstAction = QAction(QIcon(), '&重启', self)
        rstAction.triggered.connect(self.resetAction)

        copyAction = QAction(QIcon(), '&拷贝数据库及日志', self)
        copyAction.triggered.connect(self.copyLog)

        aboutAction = QAction(QIcon(), '&关于', self)
        aboutAction.setShortcut('Ctrl+H')
        aboutAction.triggered.connect(self.aboutBox)

        repairAction = QAction(QIcon(), '&修复', self)
        # repairAction.setShortcut('Ctrl+Alt+R')
        repairAction.triggered.connect(self.repairAction)

        self.statusBar()

        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&功能')
        aboutMenu = menubar.addMenu('&帮助')
        fileMenu.addAction(eraseAction)
        fileMenu.addAction(rstAction)
        fileMenu.addAction(copyAction)
        fileMenu.addAction(repairAction)
        aboutMenu.addAction(aboutAction)

    def modelAddItem(self, item):
        if(len(item) != 0):
            RingsqlWrite(item)
            newRow = self.model.rowCount()

            self.model.insertRow(newRow)
            for x in range(len(item)):
                self.model.setItem(newRow, x, QStandardItem(item[x]))
                if(item[x] == "失败"):
                    # 设置字符颜色
                    self.model.item(newRow, x).setForeground(
                        QBrush(QColor(255, 0, 0)))
                    self.model.item(newRow, x).setFont(
                        QFont("Roman times", 14, QFont.Bold))
                elif item[x] == "成功":
                    self.model.item(newRow, x).setForeground(
                        QBrush(QColor(0, 102, 0)))
                    self.model.item(newRow, x).setFont(
                        QFont("Roman times", 14, QFont.Bold))
                    # 设置字符位置
                self.model.item(newRow, x).setTextAlignment(Qt.AlignCenter)
            self.logger.info("本次烧录信息：" + ' '.join(t for t in item))

    def receive(self, msg):
        if(str(msg) == "无效"):
            self.devTypeChooseBtn.setStyleSheet(
                "background-color: #f00; font: bold 20px; height:60px;")
            self.devTypeChooseBtn.setText(msg)
            self.macLineEdit.setText("00:00:00:00:00:00")
            self.snLineEdit.setText("000000000000000")
            QMessageBox.warning(
                self, "警告", "没有找到烧录文件，请检查烧录文件！", QMessageBox.Yes)
            self.logger.error("没有找到烧录文件，请手动选择")
        else:
            if(self.devChooseSize != msg):
                for i, v in enumerate(self.devTitleRaw):
                    if(self.devChooseSize != None and v == self.devChooseSize):
                        self.devTitleLabel[i].setStyleSheet(
                            "background-color:gainsboro; background:transparent; color:black; font:bold 16px;")
                        self.devFullLabel[i].setStyleSheet(
                            "background-color:gainsboro; background:transparent; color:black; font:bold 16px;")
                        self.devBurnLabel[i].setStyleSheet(
                            "background-color:gainsboro; background:transparent; color:black; font:bold 16px;")
                        self.devUnBurnLabel[i].setStyleSheet(
                            "background-color:gainsboro; background:transparent; color:black; font:bold 16px;")

                    if(v == msg):
                        self.devTitleLabel[i].setStyleSheet(
                            "background-color: #0f0; color:blue; font:bold 16px;")
                        self.devFullLabel[i].setStyleSheet(
                            "background-color: #0f0;color:blue; font:bold 16px;")
                        self.devBurnLabel[i].setStyleSheet(
                            "background-color: #0f0;color:blue; font:bold 16px;")
                        self.devUnBurnLabel[i].setStyleSheet(
                            "background-color: #0f0;color:blue; font:bold 16px;")

            self.devChooseSize = msg
            self.devTypeChooseBtn.setStyleSheet(
                "background-color: #0f0; font: bold 20px; height:60px;")
            self.devTypeChooseBtn.setText(
                self.config['name'] + '\n' + msg + '({})'.format(self.config['mac_sn'][msg]['size']))
            self.jsonMacSnFile = os.getcwd() + '\\' + \
                self.config['mac_sn'][msg]['path'].replace('/', '\\')
            # print(self.jsonMacSnFile)
            self.readMacSn()

            if self.config['name'] in EEG_TYPE_LIST:
                self.logger.info("查找到默认SN烧录文件，路径为："+self.jsonMacSnFile)
            else:
                self.logger.info("查找到默认MAC和SN烧录文件，路径为："+self.jsonMacSnFile)

    def eraseAllAction(self):
        if(DEBUG):
            return
        b = QMessageBox.warning(self, "警告", "确定要擦除整个芯片？",
                                QMessageBox.Yes | QMessageBox.No)
        if(b == QMessageBox.No):
            return

        rst, err = NRF5xBurn.eraseall(self.api)
        if(rst):
            self.burnLabel.setText("擦除完毕")
            self.logger.info("手动擦除了设备")
        else:
            if(err[0] == NRFErrorCode.dev_communicate_error):
                QMessageBox.critical(
                    self, err[0], repr(err[1]), QMessageBox.Yes)
            else:
                QMessageBox.warning(self, "警告", err[0], QMessageBox.Yes)

    def resetAction(self):
        if(DEBUG):
            return
        rst, err = NRF5xBurn.reset(self.api)
        if(rst):
            self.burnLabel.setText("重启完毕")
            self.logger.info("手动重启了设备")
        else:
            if(err[0] == NRFErrorCode.dev_communicate_error):
                QMessageBox.critical(
                    self, err[0], repr(err[1]), QMessageBox.Yes)
            else:
                QMessageBox.warning(self, "警告", err[0], QMessageBox.Yes)

    def repairAction(self):
        if(DEBUG):
            return
        try:
            self.logger.info("修复固件")
            self.burnLabel.setText('开始修复')
            if(not self.api.is_open()):
                self.api.open()
            enum_list = self.api.enum_emu_snr()
            if(not enum_list):
                raise Exception(NRFErrorCode.emu_not_found)

            self.api.connect_to_emu_without_snr()
            self.api.connect_to_device()

            if self.config['name'] in EEG_TYPE_LIST:
                #查询脑电是否已经烧录过
                err, sn_msg = NRF5xBurn.read_devSn(self.api)
                if(not err):
                    # print(macsn)
                    self.logger.error('读取SN出错, {}'.format(repr(sn_msg)))
                    # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(sn_msg[0])
                self.logger.info("read SN:{}".format(sn_msg))
                s = ''.join(hex(x)[2:] for x in sn_msg[0:3])

                if( s in EHEAD ):
                    err, chip_mac_msg = NRF5xBurn.read_deviceaddr(self.api)
                    if(not err):
                        self.logger.info('读取MAC出错, {}'.format(repr(chip_mac_msg)))
                         # Reset the device and run.
                        self.api.sys_reset()
                        self.api.go()
                        raise Exception(chip_mac_msg[0])

                    burned_mac = ':'.join(hex(x)[2:] for x in reversed(chip_mac_msg)).upper()[0:-1]
                    burned_sn = DeviceSN.decode_sn(sn_msg[4:10])

                    self.logger.warning(
                        "修复已经烧录过MAC={}和SN={}的设备".format(burned_mac, burned_sn))
                else:
                    raise Exception('没有烧录MACSN，修复结束')
            else:
                #查询戒指是否已经烧录过
                err, macsn = NRF5xBurn.read_macsn(self.api)
                if(not err):
                    # print(macsn)
                    self.logger.error('读取MACSN出错, {}'.format(repr(macsn)))
                    # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(macsn[0])

                self.logger.info("read MACSN:{}".format(macsn))
                s = ''.join(hex(x)[2:] for x in macsn[0:2])
                if( s in MHEAD ):
                    burned_mac = ':'.join(hex(x)[2:] for x in macsn[4:10]).upper()
                    burned_sn = DeviceSN.decode_sn(macsn[10:16])

                    self.logger.warning(
                        "修复已经烧录过MAC={}和SN={}的设备".format(burned_mac, burned_sn))
                else:
                    raise Exception('没有烧录MACSN，修复结束')

            self.api.erase_all()
            self.logger.info("烧录前擦除全部")

            # Parse the hex file with the help of the HEX module
            self.logger.info("解析APP Hex文件")

            target_program = Hex.Hex(self.hexMegaRingFile)
            # Program the parsed hex into the device's memory.
            print('# Writing %s to device.' % self.hexMegaRingFile)
            cnt = 0
            hex_list_len = len(target_program._segment_list)
            for segment in target_program:
                self.api.write(segment.address, segment.data, True)
                cnt += 1
                if(hex_list_len <= 90):
                    self.pbar.setValue(cnt * (90 // hex_list_len))
                else:
                    self.pbar.setValue(cnt * (hex_list_len // 90))

            self.pbar.setValue(95)
            self.logger.info("APP Hex文件烧录完毕")

            err, msg = NRF5xBurn.read_nordicid(self.api)
            # print(msg)
            if(err == False):
                self.logger.error('读取nordic id出错, {}'.format(repr(msg)))
                # Reset the device and run.
                # self.api.erase_all()
                raise Exception(msg[0])
            self.pbar.setValue(100)

            str_nrfid = msg
            self.logger.info("MAC和SN烧录成功" + "芯片ID为：" + str_nrfid)
            # Reset the device and run.
            self.api.sys_reset()
            self.api.go()
            self.logger.info("重启设备")

            self.show_msg_qrcode('7\t{}\tMAC={},SN={}\tPASS'.format(
                time.strftime("%Y/%m/%d %H:%M:%S"), burned_mac, burned_sn))
            self.burnLabel.setText('修复结束')
            self.logger.info('修复结束')
            # Close the loaded DLL to free resources.
            self.api.close()
        except Exception as e:
            if(self.api.is_open()):
                if(self.api.is_connected_to_device()):
                    self.api.sys_reset()
                    self.api.go()
                self.api.close()
            self.show_msg_qrcode('7\t{}\tMAC={},SN={}\tFAIL\tB01'.format(
                time.strftime("%Y/%m/%d %H:%M:%S"), '00:00:00:00:00:00', '000000000000000'))
            self.pbar.setValue(0)
            self.burnLabel.setText('修复出错')
            self.logger.info('修复结束，原因:{}'.format(repr(e)))
            QMessageBox.critical(self, "修复出错", repr(e), QMessageBox.Yes)

    def burnDevice(self):
        if(self.burn_btn_enable):
            self.burnBtn.setDisabled(True)
        self.pbar.setValue(0)
        if(self.hexMegaRingFile is None or self.jsonMacSnFile is None or \
                (self.devTypeChooseBtn.text() == "无效") ):
            QMessageBox.warning(self, "警告", "没有选中文件！", QMessageBox.Yes)
            self.logger.error("没有选中文件就开启了烧录行为")

            if(self.burn_btn_enable):
                self.burnBtn.setEnabled(True)
            self.show_msg_qrcode(None)
            return False

        if(self.config['mac_sn'][self.devChooseSize]['num'] == int(self.devBurnLabel[self.devTitleRaw.index(self.devChooseSize)].text())):
            QMessageBox.warning(self, "警告", "本组MAC SN已经烧完", QMessageBox.Yes)

            if(self.burn_btn_enable):
                self.burnBtn.setEnabled(True)
            self.show_msg_qrcode(None)
            return False

        self.burnLabel.setText('开始烧录')

        self.sig_macsn.emit(self.writeMACSN2Device())

        return True

    def burn_click(self):
        return

    def writeMACSN2Device(self):
        alls_array = []

        if self.config['name'] in EEG_TYPE_LIST:
            _NOT_USED_, alls = SN2HEX.hexOfConfig(DeviceSN.decode_sn(self.hex_sn))
            alls_array = [x for x in alls]
            #return NRF5xBurn.burn_devSn(self.api, alls_array)

        else:
            '''
                head 4 byte, FactoryTestFlag 2 byte, mac 6 byte, sn 6 byte, hw 1byte
            '''
            # "mac_source": "auto",  optional_parm   1): "auto"; 2) "manual"
            if 'mac_src' in self.config.keys() and 'auto' == self.config['mac_src']:
       #         print('mac_src'+ self.config['mac_src'])
                if(not self.api.is_open()):
                    self.api.open()
                enum_list = self.api.enum_emu_snr()
                if(not enum_list):
                    raise Exception(NRFErrorCode.emu_not_found)

                self.api.connect_to_emu_without_snr()
                self.api.connect_to_device()
                err, chip_mac_msg = NRF5xBurn.read_deviceaddr(self.api)
                if(not err):
                    self.logger.info('读取MAC出错, {}'.format(repr(chip_mac_msg)))
                    # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(chip_mac_msg[0])
                self.api.close()
                mac = ''.join("%02x"%(x) for x in reversed(chip_mac_msg)).upper()
       #         print(mac)

            else:
                mac = self.macLineEdit.text().replace('-', '')

            #mac = self.macLineEdit.text().replace('-', '')

            head = MHEAD  # 代号, 标记, ex: MR
            ftFlag = "0000"
            alls = str(head) + ftFlag + str(mac) + str(self.hex_sn)

            alls = alls.upper()
 #           print(alls)
            self.logger.info("烧录的设备基本信息" + alls)
            alls_array = []
            for x in range(len(alls) // 2):
                alls_array.append(int(alls[2 * x:2 * x + 2], 16) & 0xFF)
            # return NRF5xBurn.burn_macsn(self.api, alls_array)
  #      print(alls_array)
        return alls_array
        # return NRF5xBurn.burn_macsn(self.api, alls_array)

    def copy_db(self, path):
        db_path = './db/'
        target_path = path + 'db\\'
        if(os.path.isdir(db_path) == True):
            if(os.path.isdir(target_path) != True):
                os.mkdir(target_path)
            if(os.path.isdir(db_path)):
                for root_path, dir_names, file_names in os.walk(db_path):
                    for file in file_names:
                        source_file = (root_path + file)
                        target_file = target_path + file
                        shutil.copyfile(source_file, target_file)
                        self.logger.info("已经将数据库{}拷贝完毕".format(source_file))

    def copyLogDbToLocal(self):
        num = self.config['mac_sn'][self.devChooseSize]['num']
        size = self.config['mac_sn'][self.devChooseSize]['size']
        ver = self.config['hardware_ver'][0]
        product_name = self.config['name'][-1]

        target_path = "C:\\MegaRing"
        source_log_path = "./log"
        source_db_path = "./db"
        try:
            if(not os.path.exists(target_path)):
                os.mkdir(target_path)

            target_path = target_path + '\\' + '烧录日志'
            if(not os.path.exists(target_path)):
                os.mkdir(target_path)

            # 拷贝log
            target_log_path = target_path + '\\' + 'log'
            if(not os.path.exists(target_log_path)):
                os.mkdir(target_log_path)

            target_log_path = target_log_path + '\\'
            if(not os.path.exists(target_log_path)):
                os.mkdir(target_log_path)

            for root, dirs, files in os.walk(source_log_path):
                for _, file_name in enumerate(files):
                    source_file = source_log_path + '/' + file_name
                    target_file = target_log_path + file_name
                    if(os.path.exists(source_file)):
                        shutil.copy(source_file, target_file)
            # 拷贝db
            target_db_path = target_path + '\\' + 'db'
            if(not os.path.exists(target_db_path)):
                os.mkdir(target_db_path)

            target_db_path = target_db_path + '\\'
            if(not os.path.exists(target_db_path)):
                os.mkdir(target_db_path)

            for root, dirs, files in os.walk(source_db_path):
                for _, file_name in enumerate(files):
                    source_file = source_db_path + '/' + file_name
                    target_file = target_db_path + file_name
                    if(os.path.exists(source_file)):
                        shutil.copy(source_file, target_file)

        except Exception as e:
            QMessageBox.critical(self, "错误", "{},请手动拷贝log和db文件到C盘:MegaRing路径下的相关文件夹".format(
                repr(e)), QMessageBox.Yes)

    def copyLog(self):
        reply = QMessageBox.question(
            self, "警告", "开始拷贝log吗？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if(reply == QMessageBox.Yes):
            # 判断是否插入U盘
            # PATH = findUsbDisk(USB_NAME)
            usb_name = self.config['usb_name']
            disk_name = findUsbDisk(usb_name)
            if(disk_name is None):
                QMessageBox.warning(self, "警告", "没有发现U盘", QMessageBox.Yes)
                self.logger.info("没有发现U盘")
            else:
                # 拷贝数据库
                self.copy_db(disk_name[0])
                # 拷贝日志
                TARGET_PATH = disk_name[0] + 'log'
                source_file = './log/logger.txt'

                if(os.path.exists(source_file)):
                    if(os.path.isdir(TARGET_PATH) != True):
                        os.mkdir(TARGET_PATH)
                    target_file = TARGET_PATH + '\\logger.txt'
                    self.logger.info("已经将log拷贝完毕")
                    shutil.copyfile(source_file, target_file)
                    QMessageBox.information(
                        self, "提示", "log拷贝完毕", QMessageBox.Yes)

    def process_bar(self, percent):
        self.pbar.setValue(percent)

    def burnbtn_update(self, color, steps):
        self.burnBtn.setStyleSheet(BURN_BTN_COLOR[color])
        self.burnBtn.setText(steps)

    def burn_result(self, err, msg, burning_mac):
        str_burn = "成功"
        str_nrfid = '00'*8

        if(err):
            self.burnLabel.setText('# 程序已经烧录完毕.')
            self.burnBtn.setStyleSheet(BURN_BTN_COLOR['g'])
            self.burnBtn.setText("成功")
            str_nrfid = msg
            #self.show_msg_qrcode('NRFID={}\nMAC={}\nSN={}\nPASS'.format(str_nrfid, self.macLineEdit.text(), self.snLineEdit.text()))

        else:
            QMessageBox.critical(self, "错误", msg, QMessageBox.Yes)
            self.burnBtn.setStyleSheet(BURN_BTN_COLOR['r'])
            self.burnBtn.setText("失败")
            str_burn = "失败"
            # self.show_msg_qrcode(None)
            self.burnLabel.setText('烧录失败')

        if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
            tableShow = [time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), str_nrfid, 
                        str_burn, burning_mac, self.snLineEdit.text(), 
                        self.config['hardware_ver'][0]]
        else:
            tableShow = [time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), str_nrfid, 
                        str_burn,self.macLineEdit.text(), self.snLineEdit.text(), 
                        self.config['hardware_ver'][0]]
        self.modelAddItem(tableShow)

        if(str_burn == "成功"):
            if (self.config['name'] in EEG_TYPE_LIST) or ('mac_src' in self.config.keys() and 'auto' == self.config['mac_src']):
                self.show_msg_qrcode('{},{},{},{},PASS,B00'.format(time.strftime("%Y/%m/%d %H:%M:%S"), self.scanKeyID,
                                    burning_mac.replace('-', ':'), self.snLineEdit.text()))
            else:
                self.show_msg_qrcode('{},{},{},{},PASS,B00'.format(time.strftime("%Y/%m/%d %H:%M:%S"), self.scanKeyID,
                                    self.macLineEdit.text().replace('-', ':'), self.snLineEdit.text()))
        else:
            if NRFErrorCode.macsn_has_burned in msg:
                _, burned_mac, burned_sn = msg.split(',')
                # Exception('已经烧录过MAC和SN,BC:E5:9F:48:97:2E,P11C81811000005')
                # print(burned_mac, burned_sn[:-2])
                # wr += '\tMAC={},SN={}\tPASS'.format(burned_mac, burned_sn[:-2])
                self.show_msg_qrcode('{},{},{},{},PASS,B00'.format(time.strftime("%Y/%m/%d %H:%M:%S"), self.scanKeyID,
                                burned_mac, burned_sn[:-2]))
            else:
                # wr += '\tMAC={},SN={}\tFAIL\tB01'.format('00:00:00:00:00:00', '000000000000000')
                self.show_msg_qrcode('{},{},{},{},FAIL,B01'.format(time.strftime("%Y/%m/%d %H:%M:%S"), self.scanKeyID,
                                '00:00:00:00:00:00', '000000000000000'))
        if(str_burn == "成功"):
            self.updateMacSn()      

        if(self.burn_btn_enable):
            self.burnBtn.setEnabled(True)

        self.scanKeyLineEdit.setText("")
        self.scanKeyLineEdit.setEnabled(True)
        self.scanKeyID = ""

    def closeEvent(self, event):
        self.hide()

        if('burn_thd' in dir(self)):
            self.burn_thd.working = False
            self.burn_thd.quit()
            self.burn_thd.wait()

        print('stop')
        if(self.config):
            self.copyLogDbToLocal()
            self.macLineEdit.setText("00:00:00:00:00:00")
            self.macLineEdit.setReadOnly(True)
            self.snLineEdit.setText("000000000000000")
            self.snLineEdit.setReadOnly(True)
            self.devTypeChooseBtn.setText("无效")
            self.devTypeChooseBtn.setStyleSheet(
                "background-color: #f00; font: bold 20px; height:80px;")

class BurnThread(QThread):
    sig_bar = pyqtSignal(int)
    sig_steps = pyqtSignal(str, str)
    sig_burn_msg = pyqtSignal(int, str, str)

    def __init__(self, api, logger, hexfile, config):
        super().__init__()
        self.working = True
        self.api = api
        self.logger = logger
        self.hexfile = hexfile
        self._burn = False
        self.rnd = 0
        self.config = config
        self.burned_sn = ''
        self.burned_mac = ''

    def run(self):

        while self.working:
            if(self._burn):
                # print(self.macsn)
                self.burn_go()
            self._burn = False
            time.sleep(.5)

        print('burn stop')

    def burn_go(self):
        try:
            t = time.time()
            self.api.open()
            enum_list = self.api.enum_emu_snr()
            if(not enum_list):
                raise Exception(NRFErrorCode.emu_not_found)

            self.logger.info("发现jlink：" + str(enum_list[0]))
            for x in range(20):
                try:
                    if(not self.api.is_open()):
                        self.api.open()
                    self.api.connect_to_emu_without_snr()
                    self.api.connect_to_device()
                    break
                except:
                    self.api.close()
                    self.sig_steps.emit('y', '查找设备连接:{}s'.format(x//2+1))
                    time.sleep(0.5)
                    if(x == 19):
                        self.logger.info("连接jlink")
                        raise Exception(NRFErrorCode.dev_not_connect)

            self.logger.info("连接jlink")
        
            self.sig_steps.emit('g', '读取MACSN')

            #初始化为空字符串，做保护
            self.burned_mac = ""
            self.burned_sn = ""

            #优化结构， 提前获取芯片MAC
            err, chip_mac_msg = NRF5xBurn.read_deviceaddr(self.api)
            if(not err):
                self.logger.info('读取MAC出错, {}'.format(repr(chip_mac_msg)))
            # Reset the device and run.
                self.api.sys_reset()
                self.api.go()
                raise Exception(chip_mac_msg[0])

            if self.config['name'] in EEG_TYPE_LIST:
                #查询脑电是否已经烧录过
                err, sn_msg = NRF5xBurn.read_devSn(self.api)
                if(not err):
                    # print(macsn)
                    self.logger.error('读取SN出错, {}'.format(repr(sn_msg)))
                    # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(sn_msg[0])
                self.logger.info("read SN:{}".format(sn_msg))
                s = ''.join(hex(x)[2:] for x in sn_msg[0:3])

                #无论EEG是否烧录，均使用当前芯片的MAC
                self.burned_mac = ':'.join(hex(x)[2:] for x in reversed(chip_mac_msg)).upper()

                if( s in EHEAD ):
                    strhex_sn = ''.join("%02x"%(x) for x in sn_msg[4:9])
                    self.burned_sn = DeviceSN.decode_sn(strhex_sn)

                    self.logger.warning(
                        "重复烧录已经烧录过MAC={}和SN={}的设备".format(self.burned_mac, self.burned_sn))
                    self.api.sys_reset()
                    self.api.go()
                    self.api.close()
                    raise Exception('{},{},{}'.format(
                        NRFErrorCode.macsn_has_burned, self.burned_mac, self.burned_sn))

            else:
                #查询戒指是否已经烧录过
                err, msg = NRF5xBurn.read_macsn(self.api)
                if(not err):
                    # print(msg)
                    self.logger.error('读取MACSN出错, {}'.format(repr(msg)))
                    # Reset the device and run.
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(msg[0])

                self.logger.info("read MACSN:{}".format(msg))
                s = ''.join(hex(x)[2:] for x in msg[0:2])
                if( s in MHEAD ):
                    self.burned_mac = ':'.join(hex(x)[2:] for x in msg[4:10]).upper()
                    strhex_sn = ''.join("%02x"%(x) for x in msg[10:16])
                    self.burned_sn = DeviceSN.decode_sn(strhex_sn)

                    self.logger.warning(
                        "重复烧录已经烧录过MAC={}和SN={}的设备".format(self.burned_mac, self.burned_sn))
                    self.api.sys_reset()
                    self.api.go()
                    self.api.close()
                    raise Exception('{},{},{}'.format(
                        NRFErrorCode.macsn_has_burned, self.burned_mac, self.burned_sn))
                else:
                    if 'mac_src' in self.config.keys() and 'auto' == self.config['mac_src']:
                        #使用当前芯片的MAC
                        self.burned_mac = ':'.join(hex(x)[2:] for x in reversed(chip_mac_msg)).upper()
                    else:
                        #使用传入的MAC
                        self.burned_mac = ':'.join("%02x"%(x) for x in self.macsn[4:10]).upper()


            self.sig_steps.emit('g', '烧录HEX文件')

            for hex_try in range(5):
                if(hex_try == 4):
                    self.logger.error("hex文件烧录重复尝试失败")
                    raise Exception(NRFErrorCode.hex_parse_error)

                # Parse the hex file with the help of the HEX module
                self.logger.info("解析APP Hex文件")

                target_program = Hex.Hex(self.hexfile)

                # Program the parsed hex into the device's memory.
                hexsize = sum([len(s.data) for s in target_program])

                hex_list_len = len(target_program._segment_list)

                for hex_burn in range(5):
                    self.api.erase_all()
                    self.logger.info("烧录前擦除全部")
                    target_item = target_program
                    cnt = 0
                    if(hex_burn == 4):
                        self.logger.error("hex文件烧录重复尝试失败")
                        raise Exception(NRFErrorCode.hex_burned_error)

                    for segment in target_item:
                        self.api.write(segment.address, segment.data, True)
                        rd_back = self.api.read(
                            segment.address, len(segment.data))
                        if(rd_back != segment.data):
                            break
                        cnt += 1
                        if(hex_list_len <= 90):
                            self.sig_bar.emit(cnt * (90 // hex_list_len))
                        else:
                            self.sig_bar.emit(cnt * (hex_list_len // 90))

                    if(cnt == hex_list_len):
                        self.logger.info("hex文件烧录结束")
                        break
                break

            self.sig_bar.emit(90)
            self.logger.info("APP Hex文件烧录完毕")

            self.sig_bar.emit(98)

            if self.config['name'] in EEG_TYPE_LIST:
  #              print("MAC("+ self.burned_mac +")和SN("+ self.burned_sn  +")烧录成功" + "EEG")

                err, msg = NRF5xBurn.burn_devSn(self.api, self.macsn)
                if(not err):
                    self.logger.error('写入MACSN出错, {}'.format(repr(msg)))
                    # Reset the device and run.
                    self.api.erase_all()
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(msg[0])
            else:
   #             print("MAC("+ self.burned_mac +")和SN("+ self.burned_sn  +")烧录成功" + "C11E")
                err, msg = NRF5xBurn.burn_macsn(self.api, self.macsn)
                if(not err):
                    self.logger.error('写入MACSN出错, {}'.format(repr(msg)))
                    # Reset the device and run.
                    self.api.erase_all()
                    self.api.sys_reset()
                    self.api.go()
                    raise Exception(msg[0])

 #           print("MAC("+ self.burned_mac +")和SN("+ self.burned_sn  +")烧录成功" + "start")

            self.sig_steps.emit('g', '读取nordic id')
            err, msg = NRF5xBurn.read_nordicid(self.api)
            # print(msg)
            if(err == False):
                self.logger.error('读取nordic id出错, {}'.format(repr(msg)))
                # Reset the device and run.
                self.api.erase_all()
                self.api.sys_reset()
                self.api.go()
                raise Exception(msg[0])
            self.sig_bar.emit(100)
            str_nrfid = msg
            self.logger.info("MAC和SN烧录成功" + "芯片ID为：" + str_nrfid)
 #           print("MAC("+ self.burned_mac +")和SN("+ self.burned_sn  +")烧录成功" + "芯片ID为：" + str_nrfid)

            # Reset the device and run.
            self.api.sys_reset()
            self.api.go()
            self.logger.info("重启设备")

            # Close the loaded DLL to free resources.
            self.api.close()
            self.sig_burn_msg.emit(1, str_nrfid, self.burned_mac)
            self.logger.info('烧录共消耗{}s'.format(time.time()-t))
        except Exception as e:
            if(self.api.is_open()):
                if(self.api.is_connected_to_device()):
                    self.api.sys_reset()
                    self.api.go()
                self.api.close()
            self.sig_burn_msg.emit(0, repr(e), self.burned_mac)

    def burn_on(self, macsn):
        # 没有锁，需要调用者加锁保护，不然会出现信息篡改
        if(not self._burn):
            self.sig_steps.emit('w', '发现新设备')
            self._burn = True
            self.macsn = macsn

class DevChooseWindow(QWidget):
    choose_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super(DevChooseWindow, self).__init__(parent)
        if(CONFIG[0]):
            self.config = CONFIG[1]
        else:
            self.config = None

        self.resize(300, 200)
        self.createGridGroupBox()
        # 固定前台
        # self.setWindowFlags(Qt.WindowStaysOnTopHint)
        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.gridGroupBox)

        self.setLayout(mainLayout)
        self.setWindowTitle("%s选择窗口" % self.config['name'])

    def createGridGroupBox(self):
        self.gridGroupBox = QGroupBox("选择栏:")
        layout = QGridLayout()
        if(self.config == None):
            self.gridGroupBox.setLayout(layout)
            return

        devLabel = QLabel("设备型号:")
        devSizeLabel = QLabel("设备尺寸:")
        devNameLabel = QLabel(self.config['name'])
        devNameLabel.setFont(QFont("Roman times", 14, QFont.Bold))

        self.devSizeCombo = QComboBox()
        keys = sorted(self.config['mac_sn'].keys())
        for index, key in enumerate(keys):
            self.devSizeCombo.insertItem(index, key)
        self.devSizeCombo.setCurrentIndex(0)

        pickBtn = QPushButton("确定选择")

        layout.setSpacing(10)
        layout.addWidget(devLabel, 0, 0)
        layout.addWidget(devNameLabel, 0, 1)
        layout.addWidget(devSizeLabel, 1, 0)
        layout.addWidget(self.devSizeCombo, 1, 1, 1, 1)
        layout.addWidget(pickBtn, 2, 2)
        self.gridGroupBox.setLayout(layout)

        # 注册信号槽
        pickBtn.clicked.connect(self.transfer)

    def transfer(self):
        if(self.config == None):
            value = "无效"
            # QMessageBox.warning(self, "警告", "没有选择戒指型号或尺寸！", QMessageBox.Yes)
        else:
            value = self.devSizeCombo.currentText()
            # print(value)
        self.close()
        self.devSizeCombo.setCurrentIndex(0)
        self.choose_signal.emit(str(value))

    def handle_click(self):
        if(self.config == None):
            QMessageBox.critical(
                self, "错误", "没有找到烧录文件，请检查烧录文件！", QMessageBox.Yes)
            return

        if(not self.isVisible()):
            self.show()

    def closeEvent(self, event):
        # self.close()
        pass


if(__name__ == '__main__'):
    app = QApplication(sys.argv)
    ex = MCRingBurnWindow()
    
    ex.setObjectName("MainWindow")
    #ex.setStyleSheet("#MainWindow{border-image:url(./images/python.jpg);}")
    ex.setStyleSheet("#MainWindow{background-color:rgb(200,220,200)}")

    ex.setFont(QFont('Arial', 24))

    ex.show()
    sys.exit(app.exec_())
