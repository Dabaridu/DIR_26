# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'window.ui'
#
# Created by: PyQt5 UI code generator 5.14.0
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(316, 195)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.btn_run1 = QtWidgets.QPushButton(self.centralwidget)
        self.btn_run1.setGeometry(QtCore.QRect(20, 60, 88, 61))
        self.btn_run1.setObjectName("btn_run1")
        self.btn_run2 = QtWidgets.QPushButton(self.centralwidget)
        self.btn_run2.setGeometry(QtCore.QRect(110, 60, 88, 61))
        self.btn_run2.setObjectName("btn_run2")
        self.btn_run3 = QtWidgets.QPushButton(self.centralwidget)
        self.btn_run3.setGeometry(QtCore.QRect(200, 60, 91, 61))
        self.btn_run3.setObjectName("btn_run3")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(30, 10, 54, 17))
        self.label.setObjectName("label")
        self.label_servoStatus = QtWidgets.QLabel(self.centralwidget)
        self.label_servoStatus.setGeometry(QtCore.QRect(90, 10, 101, 17))
        self.label_servoStatus.setObjectName("label_servoStatus")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(30, 30, 54, 17))
        self.label_3.setObjectName("label_3")
        self.label_jobStatus = QtWidgets.QLabel(self.centralwidget)
        self.label_jobStatus.setGeometry(QtCore.QRect(90, 30, 101, 17))
        self.label_jobStatus.setObjectName("label_jobStatus")
        self.btn_refresh = QtWidgets.QPushButton(self.centralwidget)
        self.btn_refresh.setGeometry(QtCore.QRect(200, 10, 91, 41))
        self.btn_refresh.setObjectName("btn_refresh")
        self.btn_servoOff = QtWidgets.QPushButton(self.centralwidget)
        self.btn_servoOff.setGeometry(QtCore.QRect(20, 130, 271, 41))
        self.btn_servoOff.setObjectName("btn_servoOff")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Run job Example - HC-10"))
        self.btn_run1.setText(_translate("MainWindow", "Run speed 1"))
        self.btn_run2.setText(_translate("MainWindow", "Run speed 2"))
        self.btn_run3.setText(_translate("MainWindow", "Run speed 3"))
        self.label.setText(_translate("MainWindow", "Servo:"))
        self.label_servoStatus.setText(_translate("MainWindow", "Off"))
        self.label_3.setText(_translate("MainWindow", "Job:"))
        self.label_jobStatus.setText(_translate("MainWindow", "None"))
        self.btn_refresh.setText(_translate("MainWindow", "Go home"))
        self.btn_servoOff.setText(_translate("MainWindow", "Force servo off"))
