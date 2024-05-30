# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'navigation.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.txtLogs = QtWidgets.QTextEdit(self.centralwidget)
        self.txtLogs.setGeometry(QtCore.QRect(30, 150, 741, 391))
        self.txtLogs.setObjectName("txtLogs")
        self.btnNavigate = QtWidgets.QPushButton(self.centralwidget)
        self.btnNavigate.setGeometry(QtCore.QRect(660, 50, 101, 31))
        self.btnNavigate.setObjectName("btnNavigate")
        self.labelLat = QtWidgets.QLabel(self.centralwidget)
        self.labelLat.setGeometry(QtCore.QRect(420, 40, 67, 17))
        self.labelLat.setObjectName("labelLat")
        self.labelLon = QtWidgets.QLabel(self.centralwidget)
        self.labelLon.setGeometry(QtCore.QRect(420, 80, 81, 17))
        self.labelLon.setObjectName("labelLon")
        self.editLatitude = QtWidgets.QLineEdit(self.centralwidget)
        self.editLatitude.setGeometry(QtCore.QRect(500, 30, 131, 31))
        self.editLatitude.setObjectName("editLatitude")
        self.editLongitude = QtWidgets.QLineEdit(self.centralwidget)
        self.editLongitude.setGeometry(QtCore.QRect(500, 70, 131, 31))
        self.editLongitude.setObjectName("editLongitude")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btnNavigate.setText(_translate("MainWindow", "Navigate"))
        self.labelLat.setText(_translate("MainWindow", "Latitude"))
        self.labelLon.setText(_translate("MainWindow", "Longitude"))
