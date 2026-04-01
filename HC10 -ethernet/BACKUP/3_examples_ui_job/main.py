import sys

#import support libraries
import numpy as np
import time
import os
import struct

#import ui libraries
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from PyQt5 import QtCore

#imprt ui and robot controller
from window import Ui_MainWindow
from yrc_high_speed_ethernet import ClientOfYRC, UDPRequest, UDPAnswer

#define ui class (not needed for cli applicaion)
class ApplicationWindow(QtWidgets.QMainWindow):

    #initialize ui and monitoring thread
    def __init__(self):
        super(ApplicationWindow, self).__init__()
        
        #setup main window
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #connect to robot controller
        self.yrc = ClientOfYRC('192.168.65.249')

        #setup wait counter (robot requires about 2 seconds before it starts sending the Running = True status)
        self.job_counter = 0

        #Turn servo on and run "GO-HOME" job
        self.yrc.servo_on()
        self.goHome()

        #setup button functions
        self.ui.btn_run1.clicked.connect(self.speed1)
        self.ui.btn_run2.clicked.connect(self.speed2)
        self.ui.btn_run3.clicked.connect(self.speed3)
        self.ui.btn_refresh.clicked.connect(self.goHomeServo)
        self.ui.btn_servoOff.clicked.connect(self.yrc.servo_off)

        #setup monitoring thread
        self.statusThread = statusThread(self.yrc, self)
        #set monitoring thread callback(needed for ui updates)
        self.statusThread.status.connect(self.statusCallback2)
        #start thread
        self.statusThread.start()

        ###initialization end, ui becmoes visible here

    #define speeds for job speed 1 button (this is the "job speed 1" button function)
    def speed1(self):
        #job counter must be reset before running job, otherwise the servo will turn off before the job finshes execution.
        self.job_counter = 0
        self.yrc.servo_on()
        self.runJobSpeed(1000, 250, 2500)

    #define speeds for job speed 2 button (this is the "job speed 2" button function)
    def speed2(self):
        self.job_counter = 0
        self.yrc.servo_on()
        self.runJobSpeed(2000, 500, 5000)

    #define speeds for job speed 3 button (this is the "job speed 3" button function)
    def speed3(self):
        self.job_counter = 0
        self.yrc.servo_on()
        self.runJobSpeed(3000, 1000, 10000)

    #go home button function
    def goHomeServo(self):
        self.job_counter = 0
        self.yrc.servo_on()
        self.goHome()
        
    #defines what should happen on ui exit
    def closeEvent(self, e):
        print("exiting")
        #stop thread
        self.statusThread.running = False
        #wait for it to finish
        self.statusThread.wait()
        #turn off servo power
        self.yrc.servo_off()
        #get app from global and attempt to quit
        app = GlobalData().get("app")
        comp = GlobalData().get("comp")
        if app is not None and comp is None:
            app.quit()  # QtWidgets.QApplication.instance().quit()
        e.accept()

    #function defines all job related commands to execute the GO-HOME job
    def goHome(self):
        #disable ui buttons so user cant attempt to run multiple jobs
        self.disableButtons()

        #selects job to run and starting line number. If successfull, run job
        if(self.yrc.job_select("GO-HOME-ETHERNET-EXAMPLE", 0)):
            #if the run command was successfull, return true
            if(self.yrc.job_start()):
                return True
        return False

    #function defines all job related commands to execute the example job with a certan speed
    def runJobSpeed(self, TCPspeed, ROTspeed, Jspeed):
        self.disableButtons()

        #sets variable that is defined in the job as joint speed. This value runs from 10000 to 0. It is defined as percent speed * 100.
        if(self.yrc.double_int_variable_write(Jspeed, 51)):

            #sets variable that is defined in the job as translational TCP speed. The vaue is defined as mm/s * 100. TODO: confirm value corelations to acctual values
            if(self.yrc.int_variable_write(TCPspeed, 50)):

                #sets variable that is defined in the job as rotational TCP speed. The vaue is defined as deg/s * 100. TODO: confirm value corelations to acctual values
                if(self.yrc.double_int_variable_write(ROTspeed, 50)):
                    #wait for the values to acctually be set; you could reduce the wait ammount.
                    time.sleep(1)

                    #select and run the selected job
                    if(self.yrc.job_select("ETHERNET-EXAMPLE-JOB", 0)):
                        if(self.yrc.job_start()):

                            #if everything executed return true
                            return True
        return False

    #this is the status thread callback. It renders ui data, relevant to the robot's status, as well as handles the servo off commands at job finish. It is called about every 0.5 seconds from the thread
    def statusCallback2(self, data):
        if(data["ServoOn"]):
            self.ui.label_servoStatus.setText("On")
        else:
            self.ui.label_servoStatus.setText("Off")
        if(data["Running"]):
            self.ui.label_jobStatus.setText("Running")
            self.disableButtons()
        else:
            self.ui.label_jobStatus.setText("None")
            self.yrc.servo_off()
            self.enableButtons()
    
    #helper functions to enable/disable buttons
    def enableButtons(self):
        self.ui.btn_refresh.setEnabled(True)
        self.ui.btn_run1.setEnabled(True)
        self.ui.btn_run2.setEnabled(True)
        self.ui.btn_run3.setEnabled(True)

    def disableButtons(self):
        self.ui.btn_refresh.setEnabled(False)
        self.ui.btn_run1.setEnabled(False)
        self.ui.btn_run2.setEnabled(False)
        self.ui.btn_run3.setEnabled(False)


#this defines the status thread
class statusThread(QtCore.QThread):

    #define thread signal for callback
    status = QtCore.pyqtSignal(dict)

    #init thread
    def __init__(self, yrc, parent):
        QtCore.QThread.__init__(self)

        #set robot controller variable
        self.yrc = yrc

        #needed for job_counter
        self.parent = parent

        #set running variable
        self.running = True

    #thread running function
    def run(self):
        while self.running:

            #read robot status data
            status_data = self.yrc.status_information_reading()

            #if job is executing for less than 3 seconds
            if(status_data['ServoOn'] and self.parent.job_counter < 2):

                #wait, increase wait counter
                time.sleep(1)
                self.parent.job_counter += 1
            else:

                #call thread callback (updates ui, at job end -> servo_off)
                self.status.emit(status_data)

            #execute thread every 0.5s
            time.sleep(0.5)
        

#helper class for ui     
class GlobalData:
    data = dict()

    def get(self, key):
        if key in self.data.keys():
            return self.data.get(key)
        else:
            return None

    def save(self, key, value):
        self.data.update({key: value})
        

#main function, executes ui class   
def main():

    #set ui main window
    app = QtWidgets.QApplication(sys.argv)

    #show ui main window
    application = ApplicationWindow()
    application.show()

    #save app to global data (needed for on exit function)
    GlobalData().save("app", app)
    GlobalData().save("application", application)

    #execute ui program
    app.exec_()


#if file is called as program, run main
if __name__ == "__main__":
    main()
