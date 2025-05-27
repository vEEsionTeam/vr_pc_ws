
# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'veesionOmChCR.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"vEEsionGUI")
        MainWindow.resize(424, 950)
        MainWindow.setWindowIcon(QIcon("logo.png"))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.frame = QFrame(self.centralwidget)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(10, 80, 201, 251))
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.pushButton_connect = QPushButton(self.frame)
        self.pushButton_connect.setObjectName(u"pushButton_connect")
        self.pushButton_connect.setGeometry(QRect(10, 40, 181, 21))
        self.textEdit = QTextEdit(self.frame)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(60, 10, 131, 21))
        self.label_rpi_ip = QLabel(self.frame)
        self.label_rpi_ip.setObjectName(u"label_rpi_ip")
        self.label_rpi_ip.setGeometry(QRect(10, 10, 51, 17))
        self.label_rpi_ip.setFont(QFont("Ubuntu", 11))
        self.label_rpi_ip.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_rpi_status = QLabel(self.frame)
        self.label_rpi_status.setObjectName(u"label_rpi_status")
        self.label_rpi_status.setGeometry(QRect(10, 70, 55, 17))
        self.label_rpi_status.setFont(QFont("Ubuntu", 11))
        self.label_rpi_status.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.textBrowser_2 = QTextBrowser(self.frame)
        self.textBrowser_2.setObjectName(u"textBrowser_2")
        self.textBrowser_2.setGeometry(QRect(70, 70, 121, 21))
        self.label_cam = QLabel(self.frame)
        self.label_cam.setObjectName(u"label_cam")
        self.label_cam.setGeometry(QRect(10, 100, 61, 17))
        self.label_cam.setFont(QFont("Ubuntu", 11))
        self.label_cam.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_imu = QLabel(self.frame)
        self.label_imu.setObjectName(u"label_imu")
        self.label_imu.setGeometry(QRect(10, 130, 71, 17))
        self.label_imu.setFont(QFont("Ubuntu", 11))
        self.label_imu.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_client = QLabel(self.frame)
        self.label_client.setObjectName(u"label_client")
        self.label_client.setGeometry(QRect(10, 160, 71, 17))
        self.label_client.setFont(QFont("Ubuntu", 11))
        self.label_client.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_play = QLabel(self.frame)
        self.label_play.setObjectName(u"label_play")
        self.label_play.setGeometry(QRect(10, 190, 51, 17))
        self.label_play.setFont(QFont("Ubuntu", 11))
        self.label_play.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.pb_run_cam = QPushButton(self.frame)
        self.pb_run_cam.setObjectName(u"pb_run_cam")
        self.pb_run_cam.setGeometry(QRect(70, 100, 51, 25))
        self.pb_stop_cam = QPushButton(self.frame)
        self.pb_stop_cam.setObjectName(u"pb_stop_cam")
        self.pb_stop_cam.setGeometry(QRect(120, 100, 51, 25))
        self.pb_run_imu = QPushButton(self.frame)
        self.pb_run_imu.setObjectName(u"pb_run_imu")
        self.pb_run_imu.setGeometry(QRect(70, 130, 51, 25))
        self.pb_stop_imu = QPushButton(self.frame)
        self.pb_stop_imu.setObjectName(u"pb_stop_imu")
        self.pb_stop_imu.setGeometry(QRect(120, 130, 51, 25))
        self.pb_run_client = QPushButton(self.frame)
        self.pb_run_client.setObjectName(u"pb_run_client")
        self.pb_run_client.setGeometry(QRect(70, 160, 51, 25))
        self.pb_stop_client = QPushButton(self.frame)
        self.pb_stop_client.setObjectName(u"pb_stop_client")
        self.pb_stop_client.setGeometry(QRect(120, 160, 51, 25))
        self.pb_run_play = QPushButton(self.frame)
        self.pb_run_play.setObjectName(u"pb_run_play")
        self.pb_run_play.setGeometry(QRect(70, 190, 51, 25))
        self.pb_stop_play = QPushButton(self.frame)
        self.pb_stop_play.setObjectName(u"pb_stop_play")
        self.pb_stop_play.setGeometry(QRect(120, 190, 51, 25))
        self.frame_2 = QFrame(self.centralwidget)
        self.frame_2.setObjectName(u"frame_2")
        self.frame_2.setGeometry(QRect(220, 80, 201, 251))
        self.frame_2.setFrameShape(QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Raised)
        self.label_pc_ip = QLabel(self.frame_2)
        self.label_pc_ip.setObjectName(u"label_pc_ip")
        self.label_pc_ip.setGeometry(QRect(10, 10, 51, 20))
        self.label_pc_ip.setFont(QFont("Ubuntu", 11))
        self.label_pc_ip.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_pc_status = QLabel(self.frame_2)
        self.label_pc_status.setObjectName(u"label_pc_status")
        self.label_pc_status.setGeometry(QRect(10, 40, 81, 17))
        self.label_pc_status.setFont(QFont("Ubuntu", 11))
        self.label_pc_status.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.checkBox_points = QCheckBox(self.frame_2)
        self.checkBox_points.setObjectName(u"checkBox_points")
        self.checkBox_points.setGeometry(QRect(120, 70, 81, 23))
        self.checkBox_img = QCheckBox(self.frame_2)
        self.checkBox_img.setObjectName(u"checkBox_img")
        self.checkBox_img.setGeometry(QRect(10, 70, 51, 23))
        self.checkBox_path = QCheckBox(self.frame_2)
        self.checkBox_path.setObjectName(u"checkBox_path")
        self.checkBox_path.setGeometry(QRect(60, 70, 51, 23))
        self.label_server = QLabel(self.frame_2)
        self.label_server.setObjectName(u"label_server")
        self.label_server.setGeometry(QRect(10, 100, 71, 17))
        self.label_server.setFont(QFont("Ubuntu", 11))
        self.label_server.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.label_gazebo = QLabel(self.frame_2)
        self.label_gazebo.setObjectName(u"label_gazebo")
        self.label_gazebo.setGeometry(QRect(10, 160, 71, 17))
        self.label_gazebo.setFont(QFont("Ubuntu", 11))
        self.label_gazebo.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.pb_run_server = QPushButton(self.frame_2)
        self.pb_run_server.setObjectName(u"pb_run_server")
        self.pb_run_server.setGeometry(QRect(90, 100, 51, 25))
        self.label_rviz = QLabel(self.frame_2)
        self.label_rviz.setObjectName(u"label_rviz")
        self.label_rviz.setGeometry(QRect(10, 130, 71, 17))
        self.label_rviz.setFont(QFont("Ubuntu", 11))
        self.label_rviz.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.pb_stop_server = QPushButton(self.frame_2)
        self.pb_stop_server.setObjectName(u"pb_stop_server")
        self.pb_stop_server.setGeometry(QRect(140, 100, 51, 25))
        self.pb_run_rviz = QPushButton(self.frame_2)
        self.pb_run_rviz.setObjectName(u"pb_run_rviz")
        self.pb_run_rviz.setGeometry(QRect(90, 130, 51, 25))
        self.pb_stop_rviz = QPushButton(self.frame_2)
        self.pb_stop_rviz.setObjectName(u"pb_stop_rviz")
        self.pb_stop_rviz.setGeometry(QRect(140, 130, 51, 25))
        self.pb_run_gazebo = QPushButton(self.frame_2)
        self.pb_run_gazebo.setObjectName(u"pb_run_gazebo")
        self.pb_run_gazebo.setGeometry(QRect(90, 160, 51, 25))
        self.pb_stop_gazebo = QPushButton(self.frame_2)
        self.pb_stop_gazebo.setObjectName(u"pb_stop_gazebo")
        self.pb_stop_gazebo.setGeometry(QRect(140, 160, 51, 25))
        self.comboBox_worlds = QComboBox(self.frame_2)
        self.comboBox_worlds.addItem("")
        self.comboBox_worlds.addItem("")
        self.comboBox_worlds.addItem("")
        self.comboBox_worlds.setObjectName(u"comboBox_worlds")
        self.comboBox_worlds.setGeometry(QRect(90, 190, 101, 20))
        self.label_worlds = QLabel(self.frame_2)
        self.label_worlds.setObjectName(u"label_worlds")
        self.label_worlds.setGeometry(QRect(10, 190, 67, 17))
        self.label_worlds.setFont(QFont("Ubuntu", 11))
        self.label_worlds.setStyleSheet("color: #03194d; font-weight: bold;")  
        self.textBrowser_3 = QTextBrowser(self.frame_2)
        self.textBrowser_3.setObjectName(u"textBrowser_3")
        self.textBrowser_3.setGeometry(QRect(65, 40, 125, 21))
        self.textBrowser_3.setText("Closed")

        self.textEdit_2 = QTextEdit(self.frame_2)
        self.textEdit_2.setObjectName(u"textEdit_2")
        self.textEdit_2.setGeometry(QRect(60, 10, 131, 21))
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(10, 340, 411, 251))
        self.tab_algorithm = QWidget()
        self.tab_algorithm.setObjectName(u"tab_algorithm")
        self.textBrowser = QTextBrowser(self.tab_algorithm)
        self.textBrowser.setObjectName(u"textBrowser")
        self.textBrowser.setGeometry(QRect(0, 10, 401, 201))
        self.tabWidget.addTab(self.tab_algorithm, "")
        self.tab_camimu = QWidget()
        self.tab_camimu.setObjectName(u"tab_camimu")
        self.textBrowser_4 = QTextBrowser(self.tab_camimu)
        self.textBrowser_4.setObjectName(u"textBrowser_4")
        self.textBrowser_4.setGeometry(QRect(0, 10, 401, 201))
        self.tabWidget.addTab(self.tab_camimu, "")
        self.tab_comm = QWidget()
        self.tab_comm.setObjectName(u"tab_comm")
        self.textBrowser_5 = QTextBrowser(self.tab_comm)
        self.textBrowser_5.setObjectName(u"textBrowser_5")
        self.textBrowser_5.setGeometry(QRect(0, 10, 401, 201))
        self.tabWidget.addTab(self.tab_comm, "")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.textBrowser_6 = QTextBrowser(self.tab)
        self.textBrowser_6.setObjectName(u"textBrowser_6")
        self.textBrowser_6.setGeometry(QRect(0, 10, 401, 201))
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.textBrowser_7 = QTextBrowser(self.tab_2)
        self.textBrowser_7.setObjectName(u"textBrowser_7")
        self.textBrowser_7.setGeometry(QRect(0, 10, 401, 201))
        self.tabWidget.addTab(self.tab_2, "")
        self.label = QLabel(self.centralwidget)

        self.logo_label = QLabel(self.centralwidget)
        self.logo_label.setGeometry(QRect(15, 5, 70, 70))  # Adjust size and position
        self.logo_label.setPixmap(QPixmap("logo.png").scaled(70, 65, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo_label.setFixedSize(70, 65)


        self.logo_label.setScaledContents(True)

        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(100, 30, 321, 41))
        font = QFont()
        font.setFamily(u"Saab")
        font.setPointSize(12)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setStyleSheet("color: #0997B1;")  # Turquesa blue

        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(100, 0, 321, 41))
        font1 = QFont()
        font1.setFamily(u"Saab")
        font1.setPointSize(14)
        font1.setBold(True)
        font1.setItalic(False)
        font1.setWeight(75)
        self.label_2.setFont(font1)
        self.label_2.setStyleSheet("color: #03194d;")  # Dark blue

        self.pb_test1_get = QPushButton(self.centralwidget)
        self.pb_test1_get.setObjectName(u"pb_test1_get")
        self.pb_test1_get.setGeometry(QRect(90, 610, 81, 21))
        self.label_test1 = QLabel(self.centralwidget)
        self.label_test1.setObjectName(u"label_test1")
        self.label_test1.setGeometry(QRect(10, 610, 51, 20))
        self.pb_test1_delete = QPushButton(self.centralwidget)
        self.pb_test1_delete.setObjectName(u"pb_test1_delete")
        self.pb_test1_delete.setGeometry(QRect(180, 610, 71, 21))
        self.textBrowser_test1 = QTextBrowser(self.centralwidget)
        self.textBrowser_test1.setObjectName(u"textBrowser_test1")
        self.textBrowser_test1.setGeometry(QRect(10, 640, 401, 51))
        self.pb_test2_delete = QPushButton(self.centralwidget)
        self.pb_test2_delete.setObjectName(u"pb_test2_delete")
        self.pb_test2_delete.setGeometry(QRect(180, 710, 71, 21))
        self.pb_test2_get = QPushButton(self.centralwidget)
        self.pb_test2_get.setObjectName(u"pb_test2_get")
        self.pb_test2_get.setGeometry(QRect(90, 710, 81, 21))
        self.label_test2 = QLabel(self.centralwidget)
        self.label_test2.setObjectName(u"label_test2")
        self.label_test2.setGeometry(QRect(10, 707, 51, 20))
        self.textBrowser_test2 = QTextBrowser(self.centralwidget)
        self.textBrowser_test2.setObjectName(u"textBrowser_test2")
        self.textBrowser_test2.setGeometry(QRect(10, 740, 401, 51))
        self.pb_test_difference = QPushButton(self.centralwidget)
        self.pb_test_difference.setObjectName(u"pb_test_difference")
        self.pb_test_difference.setGeometry(QRect(90, 800, 81, 21))
        self.textBrowser_test_result = QTextBrowser(self.centralwidget)
        self.textBrowser_test_result.setObjectName(u"textBrowser_test_result")
        self.textBrowser_test_result.setGeometry(QRect(10, 830, 401,70))
        self.label_testresult = QLabel(self.centralwidget)
        self.label_testresult.setObjectName(u"label_testresult")
        self.label_testresult.setGeometry(QRect(10, 800, 51, 20))


        MainWindow.setCentralWidget(self.centralwidget)
        self.tabWidget.raise_()
        self.frame.raise_()
        self.frame_2.raise_()
        self.label.raise_()
        self.label_2.raise_()
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 443, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"vEEsionGUI", None))
        self.pushButton_connect.setText(QCoreApplication.translate("MainWindow", u"Connect", None))
        self.label_rpi_ip.setText(QCoreApplication.translate("MainWindow", u"RPI IP :", None))
        self.label_rpi_status.setText(QCoreApplication.translate("MainWindow", u"Status :", None))
        self.label_cam.setText(QCoreApplication.translate("MainWindow", u"Camera:", None))
        self.label_imu.setText(QCoreApplication.translate("MainWindow", u"IMU:", None))
        self.label_client.setText(QCoreApplication.translate("MainWindow", u"Client:", None))
        self.label_play.setText(QCoreApplication.translate("MainWindow", u"VINS:", None))
        self.pb_run_cam.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_cam.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pb_run_imu.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_imu.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pb_run_client.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_client.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pb_run_play.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_play.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.label_pc_ip.setText(QCoreApplication.translate("MainWindow", u"PC IP :", None))
        self.label_pc_status.setText(QCoreApplication.translate("MainWindow", u"Status :", None))
        self.checkBox_points.setText(QCoreApplication.translate("MainWindow", u"Points", None))
        self.checkBox_img.setText(QCoreApplication.translate("MainWindow", u"Img", None))
        self.checkBox_path.setText(QCoreApplication.translate("MainWindow", u"Path", None))
        self.label_server.setText(QCoreApplication.translate("MainWindow", u"Server:", None))
        self.label_gazebo.setText(QCoreApplication.translate("MainWindow", u"Gazebo:", None))
        self.pb_run_server.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.label_rviz.setText(QCoreApplication.translate("MainWindow", u"Rviz:", None))
        self.pb_stop_server.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pb_run_rviz.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_rviz.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pb_run_gazebo.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.pb_stop_gazebo.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.comboBox_worlds.setItemText(0, QCoreApplication.translate("MainWindow", u"Cafe", None))
        self.comboBox_worlds.setItemText(1, QCoreApplication.translate("MainWindow", u"Warehouse", None))
        self.comboBox_worlds.setItemText(2, QCoreApplication.translate("MainWindow", u"Lab", None))

        self.label_worlds.setText(QCoreApplication.translate("MainWindow", u"Worlds:", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_algorithm), QCoreApplication.translate("MainWindow", u"Main", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_camimu), QCoreApplication.translate("MainWindow", u"Cam", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_comm), QCoreApplication.translate("MainWindow", u"IMU", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Server", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Client", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Visual-Inertial Navigation GUI", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"VR Headset", None))

        self.pb_test1_get.setText(QCoreApplication.translate("MainWindow", u"Get Status", None))
        self.label_test1.setText(QCoreApplication.translate("MainWindow", u"Test1", None))
        self.pb_test1_delete.setText(QCoreApplication.translate("MainWindow", u"Delete", None))
        self.pb_test2_delete.setText(QCoreApplication.translate("MainWindow", u"Delete", None))
        self.pb_test2_get.setText(QCoreApplication.translate("MainWindow", u"Get Status", None))
        self.label_test2.setText(QCoreApplication.translate("MainWindow", u"Test2", None))
        self.pb_test_difference.setText(QCoreApplication.translate("MainWindow", u"Difference", None))
        self.label_testresult.setText(QCoreApplication.translate("MainWindow", u"Result", None))


        self.frame.setStyleSheet("""
            QFrame#frame {
                background-color: transparent;
                border: 1px solid #03194d; /* Dark blue border */
                border-radius: 10px;
            }
        """)
    
        self.frame_2.setStyleSheet("""
            QFrame#frame_2 {
                background-color: transparent;
                border: 1px solid #03194d; /* Dark blue border */
                border-radius: 10px;
            }
        """)
        self.tabWidget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #03194d;
                border-radius: 10px;
                padding: 5px;
            }

            QTabBar::tab {
                background: #ffffff;
                border: 1px solid #03194d;
                border-top-left-radius: 10px;
                border-top-right-radius: 10px;
                padding: 5px 10px;
                margin-right: 2px;
            }

            QTabBar::tab:selected {
                background: #03194d;     /* Navy blue background */
                color: white;            /* White text */
            }

            QTabBar::tab:!selected {
                background: #f9f9f9;
                color: #777777;
            }
        """)
        button_style = """
        QPushButton {
            background-color: #03194d;
            color: white;
            border: 1px solid #03194d;
            border-radius: 6px;
            font-weight: bold;
        }

        QPushButton:hover:enabled {
            background-color: #062670;
        }

        QPushButton:pressed:enabled {
            background-color: #021134;
        }

        QPushButton:disabled {
            background-color: #4a5a78;
            color: #F5F5F5	;
            border: 1px solid #aaaaaa;
        }
        """


        # Example:
        self.pb_run_cam.setStyleSheet(button_style)
        self.pb_stop_cam.setStyleSheet(button_style)
        self.pb_run_imu.setStyleSheet(button_style)
        self.pb_stop_imu.setStyleSheet(button_style)
        self.pb_run_client.setStyleSheet(button_style)
        self.pb_stop_client.setStyleSheet(button_style)
        self.pb_run_play.setStyleSheet(button_style)
        self.pb_stop_play.setStyleSheet(button_style)
        self.pb_run_server.setStyleSheet(button_style)
        self.pb_stop_server.setStyleSheet(button_style)
        self.pb_run_rviz.setStyleSheet(button_style)
        self.pb_stop_rviz.setStyleSheet(button_style)
        self.pb_run_gazebo.setStyleSheet(button_style)
        self.pb_stop_gazebo.setStyleSheet(button_style)
        self.pushButton_connect.setStyleSheet(button_style)
        checkbox_style = """
        QCheckBox {
            color: #03194d;
            font-weight: bold;
            spacing: 5px;
        }
        """
        # Apply to all checkboxes
        self.checkBox_points.setStyleSheet(checkbox_style)
        self.checkBox_img.setStyleSheet(checkbox_style)
        self.checkBox_path.setStyleSheet(checkbox_style)


        self.comboBox_worlds.setStyleSheet("""
        QComboBox {
            background-color: #ffffff;
            color: #03194d;  /* This changes the text color of the selected item */
            border: 1px solid #03194d;
            border-radius: 5px;
            padding: 2px 10px;
        }

        QComboBox::drop-down {
            border-left: 1px solid #03194d;
            width: 20px;
        }

        QComboBox QAbstractItemView {
            background-color: #ffffff;
            selection-background-color: #03194d;
            selection-color: white;
        }
        """)
        self.textEdit.setStyleSheet("""
            color: #03194d;         
            font-family: Ubuntu;     
            font-size: 12pt;        
        """)

        self.textEdit_2.setStyleSheet("""
            color: #03194d;
            font-family: Ubuntu;
            font-size: 12pt;
        """)
        self.textBrowser_2.setStyleSheet("""
            color: #03194d;         
            font-family: Ubuntu;     
            font-size: 11pt;        
            background-color: #ffffff; 
        """)

        self.textBrowser_3.setStyleSheet("""
            color: #03194d;
            font-family: Ubuntu;
            font-size: 11pt;
            background-color: #ffffff;
        """)
        style = """
            color: #03194d;
            background-color: #ffffff;
            border: 1px solid #03194d;
            border-radius: 10px;
            padding: 5px;
        """

        self.textBrowser.setStyleSheet(style)
        self.textBrowser_4.setStyleSheet(style)
        self.textBrowser_5.setStyleSheet(style)
        self.textBrowser_6.setStyleSheet(style)
        self.textBrowser_7.setStyleSheet(style)



    # retranslateUi



