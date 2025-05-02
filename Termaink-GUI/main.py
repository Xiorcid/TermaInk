import time
from datetime import datetime
from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem, QFileDialog, QMessageBox, QErrorMessage
from PyQt6.QtSerialPort import QSerialPortInfo
from PyQt6.QtGui import QPixmap, QAction
from PyQt6.QtCore import QTimer
from pathlib import Path, PurePath
import serial
from sys import exit
import os
import calendar

from fontTools.misc.timeTools import timestampNow, timestampToString

app = QtWidgets.QApplication([])
ui = uic.loadUi("termaink.ui")

arrayOfTemp = [None]
lastMeasurementTimestamp = timestampNow()
epoch_diff = calendar.timegm((1904, 1, 1, 0, 0, 0, 0, 0, 0))

useCelsius = True

#some flags
devConnected = False
devPort = None

def updateports():
    portlist = []
    ports = QSerialPortInfo().availablePorts()
    for port in ports:
        portlist.append(port.portName())
    ui.com.clear()
    ui.com.addItems(portlist)


def openport():
    global devConnected, devPort
    if devConnected:
        return 1
    try:
        ser.baudrate = 115200
        ser.port = ui.com.currentText()
        ser.open()
        ser.write(bytes("test", 'utf-8'))
        ret = ser.readline().rstrip().decode("utf-8")
        print(ret)
        if ret != "Termaink Ready":
            ser.close()
            showCOMerror("Communication error")
            return 1
        devPort = ser
        ui.state.setText(f"Connected on {ser.name}")
        devConnected = True
    except Exception as ex:
        print(ex)
        showCOMerror(str(ex))
        return 1

def closeport():
    global devConnected, devPort
    if not devConnected:
        return 1
    devPort.close()
    devConnected = False
    ui.state.setText(f"Not Connected")



def showCOMerror(error):
    ui.state.setText("Error")
    errBox = QMessageBox()
    errBox.setIcon(QMessageBox.Icon.Critical)
    errBox.setText(error)
    errBox.setWindowTitle("Termaink GUI")
    errBox.exec()


def saveFile():
    file_dialog = QFileDialog()
    file_dialog.setWindowTitle("Save File")
    file_dialog.setFileMode(QFileDialog.FileMode.AnyFile)
    file_dialog.setViewMode(QFileDialog.ViewMode.Detail)
    file_dialog.setNameFilter("*.csv")
    file_dialog.setDefaultSuffix(".csv")
    if file_dialog.exec():
        selected_files = file_dialog.selectedFiles()
        print("Selected File:", selected_files[0])
        with open(selected_files[0], "w") as file:
            for i in range(len(arrayOfTemp)):
                file.write(f"{timestampToString(lastMeasurementTimestamp-1800*i)}, {arrayOfTemp[i] if useCelsius else round(arrayOfTemp[i] * 9/5 + 32, 2)}\n")


def getData():
    global lastMeasurementTimestamp, devConnected
    if not devConnected:
        return 1
    n = 0
    try:
        print("Receiving data")
        ser.write(bytes("get", 'utf-8'))
        arrayOfTemp.clear()
        max = ser.readline().rstrip().decode("utf-8").replace("\r", "").replace("\x00", "")
        ui.progressBar.setMaximum(int(max))
        while True:
            ret = ser.readline().rstrip().decode("utf-8").replace("\r", "").replace("\x00", "")
            n+=1
            print(ret)
            if 'T' in ret:
                ui.progressBar.setValue(int(max))
                lastMeasurementTimestamp = int(ret.replace("T", ""))
                lastMeasurementTimestamp = timestampConv(lastMeasurementTimestamp)
                fillTable()
                return
            try:
                arrayOfTemp.append(round(float(ret)/100, 2))
            except Exception as ex:
                print(ex)
            ui.progressBar.setValue(n)
    except Exception as ex:
        showCOMerror(ex)

def timestampConv(time):
    source_date_epoch = os.environ.get("SOURCE_DATE_EPOCH")
    if source_date_epoch is not None:
        return int(source_date_epoch) - epoch_diff
    return int(time - epoch_diff)

def syncRTC():
    global devConnected, devPort
    if not devConnected:
        return 1
    time_now = datetime.now()
    print(f"s{time_now.hour:02}.{time_now.minute:02}.{time_now.second:02}.{time_now.year-2000:02}.{time_now.month:02}.{time_now.day:02}\0")
    ser.write(bytes(f"s{time_now.hour:02}.{time_now.minute:02}.{time_now.second:02}.{time_now.year-2000:02}.{time_now.month:02}.{time_now.day:02}\0", 'utf-8'))
    if "OK" not in ser.readline().rstrip().decode("utf-8"):
        return 1
    msgBox = QMessageBox()
    msgBox.setText("The RTC has been synchronised.")
    msgBox.setWindowTitle("Termaink GUI")
    msgBox.exec()

def changeUnit():
    global useCelsius
    useCelsius = not useCelsius
    fillTable()

def configureUI():
    ui.update.clicked.connect(updateports)
    ui.conn.clicked.connect(openport)
    ui.disconn.clicked.connect(closeport)
    ui.open.clicked.connect(saveFile)
    ui.sync.clicked.connect(syncRTC)
    ui.progressBar.setValue(0)
    ui.save.clicked.connect(getData)
    ui.actionExit.triggered.connect(exit)
    ui.actionSave.triggered.connect(saveFile)
    ui.setWindowTitle(f"Termaink GUI")
    ui.tableWidget.setColumnCount(2)
    ui.actionUse_Farenheit.triggered.connect(changeUnit)
    ui.tableWidget.setRowCount(1)
    ui.tableWidget.setColumnWidth(0, 200)
    ui.tableWidget.setHorizontalHeaderLabels(["Time", "Temperature"])
    fillTable()

def fillTable():
    ui.tableWidget.setRowCount(len(arrayOfTemp))
    for n in range(len(arrayOfTemp)):
        ui.tableWidget.setItem(n, 1, QTableWidgetItem(str(arrayOfTemp[n] if useCelsius else round(arrayOfTemp[n] * 9/5 + 32, 2))))
        ui.tableWidget.setItem(n, 0, QTableWidgetItem(str(timestampToString(lastMeasurementTimestamp-1800*n))))

def setProgState(code):
    # 0 - Error
    match code:
        case 0:
            ui.state.setText(f"Not connected")

if __name__ == "__main__":
    ser = serial.Serial()
    updateports()
    configureUI()
    ui.show()
    exit(app.exec())
