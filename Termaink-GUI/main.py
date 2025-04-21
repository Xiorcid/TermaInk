import userpaths
from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem, QFileDialog, QMessageBox, QErrorMessage
from PyQt6.QtSerialPort import QSerialPortInfo
from PyQt6.QtGui import QPixmap, QAction
from PyQt6.QtCore import QTimer
from pathlib import Path, PurePath
import serial
from sys import exit

from fontTools.misc.timeTools import timestampNow, timestampToString

app = QtWidgets.QApplication([])
ui = uic.loadUi("termaink.ui")

arrayOfTemp = [25.2, 26.6, 14.88, 6.66]
lastMeasurementTimestamp = timestampNow()

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
        if ret != "Termaink Ready":
            ser.close()
            showCOMerror()
            return 1
        devPort = ser
        ui.state.setText(f"Connected on {ser.name}")
        devConnected = True
    except Exception as ex:
        print(ex)
        showCOMerror()
        return 1


def showCOMerror():
    ui.state.setText("Error")
    errBox = QErrorMessage()
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
    global lastMeasurementTimestamp
    ser.write(bytes("sync", 'utf-8'))
    timeReceived = False
    while True:
        ret = ser.readline().rstrip().decode("utf-8")
        if ret == "EOD":
            return
        if not timeReceived:
            lastMeasurementTimestamp = int(ret)
        else:
            arrayOfTemp.append(float(ret))

def syncRTC():
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
    ui.open.clicked.connect(saveFile)
    ui.sync.clicked.connect(syncRTC)
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
