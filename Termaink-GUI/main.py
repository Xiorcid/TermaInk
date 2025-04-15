from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem
from PyQt6.QtSerialPort import QSerialPortInfo
from PyQt6.QtGui import QPixmap, QAction
from PyQt6.QtCore import QTimer
from pathlib import Path
import serial
from sys import exit

from fontTools.misc.timeTools import timestampNow, timestampToString

app = QtWidgets.QApplication([])
ui = uic.loadUi("termaink.ui")

arrayOfTemp = [25.2, 26.6, 14.88, 6.66]
lastMeasurementTimestamp = timestampNow()

useCelsius = False

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
            return 1
        devPort = ser
        ui.state.setText(f"Connected on {ser.name}")
        devConnected = True
    except Exception as ex:
        print(ex)
        ui.state.setText("Error")


def saveFile():
    #new code
    pass

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

def configureUI():
    ui.update.clicked.connect(updateports)
    ui.conn.clicked.connect(openport)
    ui.actionExit.triggered.connect(exit)
    ui.actionSave.triggered.connect(saveFile)
    ui.setWindowTitle(f"Termaink GUI")
    ui.tableWidget.setColumnCount(2)
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
