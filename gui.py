import sys
import collections
import collections.abc
try:
    
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from dronekit import connect
from drone import Drone
from datetime import datetime,time
#takeoff için butona basınca uyarı penceresi açan class
class TakeoffWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("takeoff info")
        self.setGeometry(400,200,300,200)
        self.setStyleSheet("background-color:#2F343A;color:white;font-weight:bold;")
        
        layout=QVBoxLayout()
        
        self.altitude_info=QLabel("Enter takeoff altitude:",self)
        self.altitude_info.setFont(QFont("Arial",12))

        self.altitude_input=QDoubleSpinBox(self)
        self.altitude_input.setRange(1,100)
        self.altitude_input.setValue(10)
        self.altitude_input.setSuffix(" m")
        self.altitude_input.setFont(QFont("Arial",12))
        self.altitude_input.setStyleSheet("background-color:#2F343A;color:white;font-weight:bold;")

        layout.addWidget(self.altitude_info)
        layout.addWidget(self.altitude_input)

        self.button_box=QDialogButtonBox(QDialogButtonBox.Ok|QDialogButtonBox.Cancel)
        self.button_box.setStyleSheet("background-color:#2F343A;color:white;")

        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)
        self.setLayout(layout)

    def get_altitude(self):
        return self.altitude_input.value()
#connect için sağ üstte bağlanma butonu oluşturan class
class Connection(QMainWindow):
    def __init__(self,drone):
        super().__init__()
        self.is_connected=False
        self.drone=drone
    
        self.connect_btn=QPushButton("CONNECT",self)
        self.connect_btn.setGeometry(1500,0,100,75)
        self.connect_btn.setStyleSheet("background-color:#2F343A;color:white;")

        self.combo_str=QComboBox(self)
        self.combo_str.addItems(["tcp","udp"])
        self.combo_str.setGeometry(1350,0,150,30)
        self.combo_str.setStyleSheet("background-color:#2F343A;color:white;")

        self.label=QLineEdit(self)
        self.label.setGeometry(1350,30,150,45)
        self.label.setStyleSheet("background-color:#2F343A;color:white;")

        self.connect_btn.clicked.connect(self.connected)
    
    def connected(self):
        if self.is_connected:
            self.drone.disconnect()
            self.is_connected=False
            self.update_gui()
            return
        question=QMessageBox.question(self,"Connect","Are you sure you want to connect?",QMessageBox.Yes|QMessageBox.No)
        if question==QMessageBox.Yes:
            port=self.combo_str.currentText()
            port_number=self.label.text()

            if not port_number.isdigit():
                QMessageBox.warning(self,"Invalid Port","Please enter a valid port number.")
                return
            connection_string=f"{port}:{port_number}"
            self.connect_btn.setText("CONNECTING")
            self.connect_btn.setStyleSheet("color:green;")

            if self.drone.connectDrone(connection_string):
                self.is_connected=True
            else:
                self.is_connected=False
            self.update_gui()
        else:
            self.add_log("connection is cancelled by user")

    #gui güncelleme fonksiyonu
    def update_gui(self):
        if self.is_connected():
            self.connect_btn.setText("DISCONNECT")
            self.connect_btn.setStyleSheet("background-color:#2F343A;color:rgb(0, 255, 0);")
        else:
            self.connect_btn.setText("CONNECT")
            self.connect_btn.setStyleSheet("background-color:#2F343A;color:rgb(255, 0, 0);")

#uygulama ana classı
class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("System")
        self.setGeometry(140,90,1600,900)
        self.setWindowIcon(QIcon("indir.jpg"))
        self.drone=Drone()
        #bağlanma classını oluşturup drone objesini ona gönderiyoruz
        self.connnection=Connection(self.drone)
        self.initUI()
    def initUI(self):
        my_widget=QWidget()
        self.setCentralWidget(my_widget)

        main_layout=QHBoxLayout()
        my_widget.setLayout(main_layout)

        self.map=QLabel()
        self.map.setToolTip("location of map")
        self.map.setStyleSheet("background-color:#2F343A;")

        left_layout = QVBoxLayout()
        main_layout.addLayout(left_layout, 1)

        main_layout.addWidget(self.map,2)

        self.camera=QLabel()
        self.camera.setToolTip("location of camera footage")
        self.camera.setStyleSheet("background-color:#2F343A;")

        left_layout.addWidget(self.camera, 3)

        self.btn_layout=QHBoxLayout()
        self.telemetry_btn=QPushButton("Telemetry",self)
        self.comound_btn=QPushButton("Komut",self)
        self.message_btn=QPushButton("Mesaj",self)
        self.btn_layout.addWidget(self.message_btn)
        self.btn_layout.addWidget(self.telemetry_btn)
        self.btn_layout.addWidget(self.comound_btn)
        left_layout.addLayout(self.btn_layout)

        self.stacked=QStackedWidget()
        left_layout.addWidget(self.stacked,2)

        self.page_telemetry=QWidget()
        telemetry_layout = QGridLayout()
        
        self.page_telemetry.setLayout(telemetry_layout)

        telemetry_layout.setRowStretch(0, 1)
        telemetry_layout.setRowStretch(1, 1)
        telemetry_layout.setColumnStretch(0, 1)
        telemetry_layout.setColumnStretch(1, 1)


        self.label1=QLabel("ARMED",self)
        self.label1.setFont(QFont("Arial",14))
        self.label1.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")
        self.label1.setAlignment(Qt.AlignBottom)

        self.label2=QLabel("MODE",self)
        self.label2.setFont(QFont("Arial",14))
        self.label2.setStyleSheet("background-color:#2F343A;color:rgb(128, 128, 128);font-weight:bold;font-style:italic;text-decoration:underline;")
        self.label2.setAlignment(Qt.AlignBottom)

        self.label3=QLabel("ALTİTUDE",self)
        self.label3.setFont(QFont("Arial",14))
        self.label3.setStyleSheet("background-color:#2F343A;color:rgb(128, 128, 128);font-weight:bold;font-style:italic;text-decoration:underline;")
        self.label3.setAlignment(Qt.AlignBottom)

        self.label4=QLabel("VELOCİTY",self)
        self.label4.setFont(QFont("Arial",14))
        self.label4.setStyleSheet("background-color:#2F343A;color:rgb(128, 128, 128);font-weight:bold;font-style:italic;text-decoration:underline;")
        self.label4.setAlignment(Qt.AlignBottom)

        telemetry_layout.addWidget(self.label1, 0, 0)
        telemetry_layout.addWidget(self.label2, 0, 1)
        telemetry_layout.addWidget(self.label3, 1, 0)
        telemetry_layout.addWidget(self.label4, 1, 1)

        self.stacked.addWidget(self.page_telemetry)

        self.page_command=QWidget()
        command_layout = QVBoxLayout()
        self.page_command.setLayout(command_layout)

        self.btn_arm=QPushButton("Force",self)
        self.btn_arm.setFont(QFont("Arial",18))
        self.btn_arm.setFixedHeight(50)
        self.btn_arm.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.combo_box=QComboBox(self)
        self.combo_box.setFont(QFont("Arial",18))
        self.combo_box.setFixedHeight(50)
        self.combo_box.addItems(["STABILIZE","LOITER","LAND","GUIDED","AUTO"])
        self.combo_box.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.btn_mode=QPushButton("Apply",self)
        self.btn_mode.setFont(QFont("Arial",18))
        self.btn_mode.setFixedHeight(50)
        self.btn_mode.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.btn_rtl=QPushButton("Geri Dön",self)
        self.btn_rtl.setFont(QFont("Arial",18))
        self.btn_rtl.setFixedHeight(50)
        self.btn_rtl.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")
        
        self.takeoff_btn=QPushButton("Takeoff",self)
        self.takeoff_btn.setFont(QFont("Arial",18))
        self.takeoff_btn.setFixedHeight(50)
        self.takeoff_btn.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.page_message=QWidget()
        message_layout=QVBoxLayout()
        self.page_message.setLayout(message_layout)

        self.log_box=QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Arial",10))
        self.log_box.setStyleSheet("background-color:#2F343A;color:white;font-weight:bold;")

        message_layout.addWidget(self.log_box)
        command_layout.addWidget(self.btn_arm)
        command_layout.addWidget(self.combo_box)
        command_layout.addWidget(self.btn_mode)
        command_layout.addWidget(self.btn_rtl)
        command_layout.addWidget(self.takeoff_btn)

        self.stacked.addWidget(self.page_command)
        self.stacked.addWidget(self.page_message)

        self.telemetry_btn.clicked.connect(lambda: self.stacked.setCurrentWidget(self.page_telemetry))
        self.comound_btn.clicked.connect(lambda: self.stacked.setCurrentWidget(self.page_command))
        self.message_btn.clicked.connect(lambda: self.stacked.setCurrentWidget(self.page_message))

        self.btn_arm.clicked.connect(self.toggle_arm)
        self.btn_mode.clicked.connect(self.change_mode)
        self.btn_rtl.clicked.connect(self.drone_rtl)
        self.takeoff_btn.clicked.connect(self.takeoff_info)

        self.timer=QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(1000)
    
        self.update_telemetry()

    def takeoff_info(self):
        valid=True
        mode_list=["STABILIZE","LOITER","LAND"]
        for i in mode_list:
            if self.drone.get_mode()==i:
                self.add_log(f"drone is in {i} mode,takeoff command is not allowed")
                valid=False
                break
        takeoff_obj=TakeoffWindow()
        if valid:
            if takeoff_obj.exec_()==QDialog.Accepted:
                altitude=takeoff_obj.get_altitude()
                self.drone.take_off(altitude)
                self.add_log(f"drone is commanded to takeoff to {altitude} m")
        else:
            self.add_log("takeoff command is cancelled due to invalid mode")
    def add_log(self,message):
        time=datetime.now().strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{time}] {message}")

    def toggle_arm(self):
        question=QMessageBox.question(self,"ARM/DISARM","Are you sure you want to change arm state?",QMessageBox.Yes|QMessageBox.No)
        if question==QMessageBox.Yes:
            if self.drone.is_arm():
                self.drone.dis_arm()
                self.add_log("drone is forced to disarm")
            else:
                self.drone.arm()
                self.add_log("drone is forced to arm")
        else:
            self.add_log("arm state change is cancelled by user")
    def change_mode(self):
        self.selected_mode=self.combo_box.currentText()
        self.drone.set_mode(self.selected_mode)
        self.add_log(f"drone mode is changed to {self.selected_mode}")
    def drone_rtl(self):
        self.drone.rtl()
        self.add_log("drone is commanded to return to launch")
    def update_telemetry(self):
        if not self.drone.is_connected():
            self.add_log("drone is not connected")
            self.label1.setText("Not ready to arm")
            self.label1.setAlignment(Qt.AlignCenter)
            self.label1.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")
            return 
        if self.drone.is_arm():
            self.label1.setText("DISARM")
            self.label1.setStyleSheet("background-color:#2F343A;color:rgb(128, 128, 128);font-weight:bold;font-style:italic;text-decoration:underline;")
        else:
            self.label1.setText("ARM")
            self.label1.setStyleSheet("background-color:#2F343A;color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")
        self.velocity=self.drone.velocity()
        self.label4.setText(f"{self.velocity:.2f} m/s")
        self.label4.alignment(Qt.AlignCenter)
        self.label4.setStyleSheet("color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.altitude=self.drone.altitude()
        self.label3.setText(f"{self.altitude:.2f} m")
        self.label3.setAlignment(Qt.AlignCenter)
        self.label3.setStyleSheet("color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

        self.current_mode=self.drone.get_mode()
        self.label2.setText(f"{self.current_mode}")
        self.label2.setAlignment(Qt.AlignCenter)
        self.label2.setStyleSheet("color:rgb(168, 5, 16);font-weight:bold;font-style:italic;text-decoration:underline;")

def main():
    app=QApplication(sys.argv)    
    win=MyApp()
    win.show()
    sys.exit(app.exec_())
main()

