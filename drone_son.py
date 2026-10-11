import collections
import collections.abc

# Eski DroneKit'in Python 3.10+ uyumluluk düzeltmesi.
# Bu blok mutlaka "from dronekit import ..." satırından önce çalışmalı.
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

import math
import time
import threading

class Drone:
    LINK_HEARTBEAT_MAX_AGE_S = 5.0
    def __init__(self):
        self.vehicle=None
        self.current_history = collections.deque(maxlen=10)
        self.servo_current_amps = 0.0
        self.rc_channels_raw = [0, 0, 0, 0, 0, 0]
        # Mission Planner > Servo Output ekranındaki gerçek çıkış PWM değerleri
        # SERVO_OUTPUT_RAW mesajından güncellenecek.
        self.servo_outputs_raw = {i: 0 for i in range(1, 17)}
        self._last_servo_output_time = 0.0
        self._last_statustext = ""
        self._last_ack = {}
        self._calib_callback = None  # Python tarafı buraya callback koyacak
        self.current_mission_seq = -1
        self.is_vtol_aircraft = False
        self.params_ready = False
        self.last_mission_upload_status = ""
        # Aynı anda normal görev ve recovery görevi yazılmasını engeller.
        # Lock bilinçli olarak non-blocking alınır; ikinci Write kuyruğa girmez.
        self._mission_upload_lock = threading.Lock()

        # EXTENDED_SYS_STATE mesajından güncellenecek:
        # 1=plane'e geçiş, 2=drone'a geçiş, 3=drone, 4=plane.
        self.current_vtol_state = None

    def wait_for_parameters_ready(
        self,
        timeout=5.0
    ):
        if not self.is_connected():
            print(
                "[-] Parametre bekleme iptal: "
                "heartbeat yok."
            )
            return False

        check_params = [
            "SYSID_THISMAV",
            "ARMING_CHECK",
            "Q_ENABLE",
            "SERVO1_FUNCTION",
            "SCR_USER1",
            "SCR_USER3",
        ]

        target_sys = (
            self.vehicle._master.target_system
            or 1
        )

        target_comp = (
            self.vehicle._master.target_component
            or 1
        )

        started = time.monotonic()
        last_request_s = 0.0

        while (
            time.monotonic() - started
            < timeout
        ):
            if not self.is_connected():
                print(
                    "[-] Parametre beklerken "
                    "heartbeat kayboldu."
                )
                return False

            found = {}

            for name in check_params:
                try:
                    value = (
                        self.vehicle.parameters.get(
                            name,
                            None
                        )
                    )

                    if value is not None:
                        found[name] = value

                except Exception:
                    pass

            if found:
                self.params_ready = True

                print(
                    f"[+] Parametre cache aktif: "
                    f"{found}"
                )

                return True

            now = time.monotonic()

            if (
                now - last_request_s
                >= 1.0
            ):
                for name in check_params:
                    try:
                        self.vehicle._master.mav.param_request_read_send(
                            target_sys,
                            target_comp,
                            name.encode("utf-8"),
                            -1
                        )

                    except Exception:
                        pass

                try:
                    self.vehicle.flush()
                except Exception:
                    pass

                last_request_s = now

            time.sleep(0.2)

        self.params_ready = False

        print(
            "[!] Parametre cache henüz hazır değil. "
            "MAVLink bağlantısı korunuyor."
        )

        return False

    def connectDrone(self, connection_string, baudrate=57600):
        print(f"[*] {connection_string} portu açılıyor, Telemetri bağlantısı bekleniyor...")
        
        # Telemetrinin birbirini bulması 10-15 saniye sürebilir.
        # DroneKit'in anında pes etmemesi için 3 defa deneme döngüsü kuruyoruz.
        for attempt in range(1, 4):
            try:
                print(f"[*] Bağlantı denemesi {attempt}/3...")
                self.vehicle = connect(
                    connection_string,
                    baud=baudrate,
                    wait_ready=False,
                    timeout=20,  # 60 saniye bekletip kitlemek yerine 20 saniyelik bloklar halinde deniyoruz
                    heartbeat_timeout=120
                )
                
                self.setup_listeners()
                
                print("[+] MAVLink bağlantısı kuruldu.")
                print("[*] Parametreler hızlı kontrol ediliyor...")
                
                params_ok = self.wait_for_parameters_ready(timeout=3)
                
                if params_ok:
                    self.params_ready = True
                    print("[+] Parametreler hazır.")
                else:
                    self.params_ready = False
                    print("[!] Bağlantı var ama parametreler henüz okunamadı.")
                    
                return True
                
            except Exception as e:
                print(f"[-] Deneme {attempt} başarısız: {e}")
                # Telemetriler senkronize olmaya çalışıyorsa 3 saniye zaman tanı ve tekrar dene
                time.sleep(3)
                
        print("[-] Telemetri linki kurulamadı. Işıkların SABİT YEŞİL yandığından emin olun.")
        self.vehicle = None
        self.params_ready = False
        return False
    def set_param_safe(
        self,
        name,
        value,
        retries=4,
        delay=0.35,
        initial_wait=2.0
    ):
        """
        Parametreyi GUI'yi ve MAVLink akışını gereksiz yere bloklamadan yazar.

        - ARM durumunda yazmaz.
        - Parametre hazır değilse kısa süre bekler.
        - MAVLink PARAM_SET kullanır.
        - Yazmayı geri okuyarak doğrular.
        - Başarısızlık bağlantıyı kapatmaz.
        """
        if not self.is_connected():
            print(f"[-] {name} yazılamadı: heartbeat yok.")
            return False

        try:
            if bool(self.vehicle.armed):
                print(
                    f"[-] {name} yazılamadı: araç ARM durumda."
                )
                return False
        except Exception:
            return False

        if not getattr(self, "params_ready", False):
            self.wait_for_parameters_ready(timeout=initial_wait)

        def same_value(a, b):
            try:
                a = float(a)
                b = float(b)
                if not (math.isfinite(a) and math.isfinite(b)):
                    return False
                # SCR koordinat kesirleri için daha hassas doğrulama.
                tolerance = 5e-7 if str(name).startswith("SCR_USER") else 0.01
                return abs(a - b) <= tolerance
            except Exception:
                return str(a) == str(b)

        for attempt in range(1, retries + 1):
            if not self.is_connected():
                print(f"[-] {name}: heartbeat kayboldu.")
                return False

            try:
                old_value = self.vehicle.parameters.get(name, None)

                if same_value(old_value, value):
                    return True

                target_sys = self.vehicle._master.target_system or 1
                target_comp = self.vehicle._master.target_component or 1

                self.vehicle._master.mav.param_set_send(
                    target_sys,
                    target_comp,
                    name.encode("utf-8"),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                self.vehicle.flush()
                time.sleep(delay)

                new_value = self.vehicle.parameters.get(name, None)
                if same_value(new_value, value):
                    print(
                        f"[+] {name} doğrulandı: "
                        f"{new_value}"
                    )
                    return True

                # Cache geç güncellenmişse tekil PARAM_REQUEST_READ gönder.
                try:
                    self.vehicle._master.mav.param_request_read_send(
                        target_sys,
                        target_comp,
                        name.encode("utf-8"),
                        -1
                    )
                except Exception:
                    pass

                print(
                    f"[PARAM] {name} doğrulanamadı "
                    f"({attempt}/{retries}): "
                    f"okunan={new_value}, beklenen={value}"
                )

            except Exception as e:
                print(
                    f"[-] {name} yazma denemesi "
                    f"{attempt}/{retries}: {e}"
                )

            time.sleep(delay)

        print(f"[-] {name} yazılamadı/doğrulanamadı.")
        return False
    def set_q_option_bits(self, bits_to_set=0, bits_to_clear=0):
        """
        Q_OPTIONS bitmask ayarı.
        Var olan Q_OPTIONS değerini tamamen ezmez; sadece istediğimiz bitleri açar/kapatır.
        """
        if not self.is_connected():
            return False

        try:
            current = self.vehicle.parameters.get('Q_OPTIONS', 0)
            if current is None:
                current = 0

            current = int(current)
            new_value = (current | bits_to_set) & (~bits_to_clear)

            return self.set_param_safe('Q_OPTIONS', new_value)

        except Exception as e:
            print(f"[-] Q_OPTIONS ayarlanamadı: {e}")
            return False
    def setup_failsafes(self):
        if not self.is_connected():
            return False
        try:
            # 2 = Görevdeyken (AUTO) sinyal kopsa bile GÖREVİ TAMAMLA, manuelken RTL yap
            self.vehicle.parameters['FS_GCS_ENABLE'] = 2
            # 1 = Batarya bitiyorsa RTL (Eve Dön) yap
            self.vehicle.parameters['FS_BATT_ENABLE'] = 1
            #1 = GPS koptuğu an rüzgarla uçup gitmemek için LAND (İniş) yap
            self.vehicle.parameters['FS_EKF_ACTION'] = 1
            current_mis = self.vehicle.parameters.get('MIS_OPTIONS', 0)
            self.vehicle.parameters['MIS_OPTIONS'] = int(current_mis) | 4
            self.vehicle.parameters['Q_TRANS_DECEL']=4.0 
            self.vehicle.parameters['TECS_LAND_ARSPD']=10.0 #iniş noktasına yaklaşırken ki hızını kontrol ediyoruz burada
            
            return True
        except Exception as e:
            print(f"[-] DİKKAT: Failsafe parametreleri ayarlanamadı: {e}")
            return False
    def setup_listeners(self):
        if not self.is_connected():
            return
        try:
            target_sys = self.vehicle._master.target_system
            target_comp = self.vehicle._master.target_component

            # RC_CHANNELS İSTEĞİ 5 Hz'den 2 Hz'e DÜŞÜRÜLDÜ (Telemetriyi rahatlatmak için)
            msg = self.vehicle.message_factory.request_data_stream_encode(
                target_sys, 
                target_comp, 
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 
                2,  # <-- BURASI 5 İDİ, 2 YAPTIK
                1  
            )
            self.vehicle.send_mavlink(msg)
            print(f"[+] Uçağa 'RC Kumanda Yayını' emri (Sys:{target_sys}, Comp:{target_comp}) gönderildi (2 Hz)!")
            
            def rc_listener(vehicle, name, message):
                self.rc_channels_raw[0] = message.chan1_raw
                self.rc_channels_raw[1] = message.chan2_raw
                self.rc_channels_raw[2] = message.chan3_raw
                self.rc_channels_raw[3] = message.chan4_raw
                self.rc_channels_raw[4] = message.chan5_raw
                self.rc_channels_raw[5] = message.chan6_raw

            def servo_output_listener(vehicle, name, message):
                try:
                    fields = [
                        "servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw",
                        "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw",
                        "servo9_raw", "servo10_raw", "servo11_raw", "servo12_raw",
                        "servo13_raw", "servo14_raw", "servo15_raw", "servo16_raw"
                    ]
                    for servo_no, field in enumerate(fields, start=1):
                        if hasattr(message, field):
                            val = getattr(message, field)
                            if val is not None:
                                self.servo_outputs_raw[servo_no] = int(val)
                    self._last_servo_output_time = time.time()
                except Exception as e:
                    pass

            def statustext_listener(vehicle, name, message):
                self._last_statustext = message.text
                print(f"[PIXHAWK] {message.text}")

            def command_ack_listener(vehicle, name, message):
                self._last_ack = {'command': message.command, 'result': message.result}
                print(f"[ACK] CMD:{message.command} RESULT:{message.result}")

            def mission_current_listener(vehicle, name, message):
                try:
                    self.current_mission_seq = int(message.seq)
                except Exception:
                    pass

            def extended_sys_state_listener(vehicle, name, message):
                try:
                    self.current_vtol_state = int(message.vtol_state)
                except Exception:
                    self.current_vtol_state = None

            def mag_cal_progress_listener(vehicle, name, message):
                pct = message.completion_pct
                if hasattr(self, 'compass_cb') and self.compass_cb:
                    self.compass_cb(pct)

            def mag_cal_report_listener(vehicle, name, message):
                if hasattr(self, 'compass_cb') and self.compass_cb:
                    self.compass_cb(100.0)

            # Dinleyicileri ekle
            self.vehicle.add_message_listener('MAG_CAL_PROGRESS', mag_cal_progress_listener)
            self.vehicle.add_message_listener('MAG_CAL_REPORT', mag_cal_report_listener)
            self.vehicle.add_message_listener('STATUSTEXT', statustext_listener)
            self.vehicle.add_message_listener('COMMAND_ACK', command_ack_listener)
            self.vehicle.add_message_listener('RC_CHANNELS', rc_listener)
            self.vehicle.add_message_listener('RC_CHANNELS_RAW', rc_listener)
            self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', servo_output_listener)
            self.vehicle.add_message_listener('MISSION_CURRENT', mission_current_listener)
            self.vehicle.add_message_listener('EXTENDED_SYS_STATE', extended_sys_state_listener)

            # EXTENDED_SYS_STATE isteği (0.5 sn = 2 Hz)
            try:
                self.vehicle._master.mav.command_long_send(
                    target_sys, target_comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500000, 0, 0, 0, 0, 0
                )
                print("[+] EXTENDED_SYS_STATE yayını istendi: 2 Hz")
            except Exception as e:
                pass
            
            # SERVO_OUTPUT_RAW İSTEĞİ 10 Hz'den 2 Hz'e DÜŞÜRÜLDÜ (Telemetriyi boğmamak için)
            try:
                self.vehicle._master.mav.command_long_send(
                    target_sys, target_comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 
                    500000,  # <-- BURASI 100000 (10 Hz) İDİ, 500000 (2 Hz) YAPTIK
                    0, 0, 0, 0, 0
                )
                print("[+] SERVO_OUTPUT_RAW yayını istendi: 2 Hz")
            except Exception as e:
                pass
                
        except Exception as e:
            print(f"[-] DİKKAT: Dinleyici ayarlanırken hata: {e}")
    def get_battery(self):
        if self.is_connected() and self.vehicle.battery:
            volt = self.vehicle.battery.voltage or 0.0
            level = self.vehicle.battery.level or 0
            current = self.vehicle.battery.current or 0.0 # AKIMI DİREKT BURADAN ÇEKİYORUZ
            return volt, level, current
        return 0.0, 0, 0.0
    def get_rc_channels(self):
        if self.is_connected():
            return (
                self.rc_channels_raw[0], # ch1
                self.rc_channels_raw[1], # ch2
                self.rc_channels_raw[2], # ch3
                self.rc_channels_raw[3], # ch4
                self.rc_channels_raw[4], # ch5
                self.rc_channels_raw[5]  # ch6
            )
        return 0, 0, 0, 0, 0, 0
    
    def get_servo_outputs(self):
        """
        SERVO_OUTPUT_RAW'dan gelen son gerçek PWM çıkışlarını döndürür.
        Format:
        {
            1: 1500,
            2: 1570,
            ...
            16: 0
        }
        """
        if not self.is_connected():
            return {i: 0 for i in range(1, 17)}

        try:
            return dict(self.servo_outputs_raw)
        except Exception:
            return {i: 0 for i in range(1, 17)}


    def get_servo_function_params(self, count=16):
        """
        SERVO1_FUNCTION ... SERVO16_FUNCTION parametrelerini okur.
        Mission Planner'daki Function combobox'larının karşılığı budur.
        """
        result = {}

        if not self.is_connected():
            return {i: 0 for i in range(1, count + 1)}

        for servo_no in range(1, count + 1):
            param_name = f"SERVO{servo_no}_FUNCTION"
            try:
                value = self.vehicle.parameters.get(param_name, None)
                result[servo_no] = int(float(value)) if value is not None else 0
            except Exception:
                result[servo_no] = 0

        return result

    def get_servo_reverse_params(self, count=16):
        """
        SERVO1_REVERSED ... SERVO16_REVERSED parametrelerini okur.
        Mission Planner'daki Reverse checkbox karşılığıdır.
        """
        result = {}

        if not self.is_connected():
            return {i: False for i in range(1, count + 1)}

        for servo_no in range(1, count + 1):
            param_name = f"SERVO{servo_no}_REVERSED"
            try:
                value = self.vehicle.parameters.get(param_name, 0)
                result[servo_no] = bool(int(float(value)))
            except Exception:
                result[servo_no] = False

        return result

    def set_servo_function_param(self, servo_no, function_id):
        """
        SERVOx_FUNCTION değiştirir.
        Güvenlik için araç ARM durumdayken değiştirme.
        """
        if not self.is_connected():
            print("[-] Servo function yazılamadı: araç bağlı değil.")
            return False

        if self.vehicle.armed:
            print("[-] Servo function yazılamadı: araç ARM durumda.")
            return False

        param_name = f"SERVO{int(servo_no)}_FUNCTION"
        return self.set_param_safe(param_name, int(function_id))

    def set_servo_reverse_param(self, servo_no, reversed_state):
        """
        SERVOx_REVERSED değiştirir.
        """
        if not self.is_connected():
            print("[-] Servo reverse yazılamadı: araç bağlı değil.")
            return False

        if self.vehicle.armed:
            print("[-] Servo reverse yazılamadı: araç ARM durumda.")
            return False

        param_name = f"SERVO{int(servo_no)}_REVERSED"
        return self.set_param_safe(param_name, 1 if reversed_state else 0)

    def run_motor_test(self, motor_num, throttle_percent=5, duration_sec=3,motor_count=1):
        if not self.is_connected():
            return False
        # MAV_CMD_DO_MOTOR_TEST komutu
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 
            0,  
            motor_num,        # Param 1: Motor Numarası
            0,                # Param 2: Gaz Tipi 
            throttle_percent, # Param 3: Yüzde kaç güçle dönsün?
            duration_sec,     # Param 4: Kaç saniye dönsün? 
            motor_count,      # Param 5: YENİ! Kaç motor test edilsin? (Hepsi için 4 yollayacağız)
            0, 0  
        )
        self.vehicle.send_mavlink(msg)
        return True
    def stop_motor_test(self):
        if not self.is_connected():
            return False
        # 4 motorun hepsine anında %0 Güç (0 saniye) gönderip durduruyoruz!
        for i in range(1, 5):
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 
                0,  
                i,  # Motor Numarası
                0,  # Gaz Tipi 
                0,  # %0 GÜÇ (DUR)
                0,  # 0 Saniye
                1,  
                0, 0  
            )
            self.vehicle.send_mavlink(msg)
        return True
    def reboot_vehicle(self):
        if not self.is_connected():
            return False
        try:
            # Uçağa "Yeniden Başlat" emri yolluyoruz (Parametre 1 = 1 ise Reboot atar)
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 
                0,  
                1, 0, 0, 0, 0, 0, 0  
            )
            self.vehicle.send_mavlink(msg)
            return True
        except Exception as e:
            print(f"[-] Reboot Hatası: {e}")
            return False
    def disconnect(self):
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle=None
    def has_vehicle(self):
        return self.vehicle is not None
    def link_state(self):
        if self.vehicle is None:
            return "DISCONNECTED"

        age = self.heartbeat_age()

        if age is None:
            return "UNKNOWN"

        if age <= self.LINK_HEARTBEAT_MAX_AGE_S:
            return "OK"

        if age <= 12.0:
            return "DEGRADED"

        return "LOST"
    def heartbeat_age(self):
        if self.vehicle is None:
            return None

        try:
            age = float(self.vehicle.last_heartbeat)

            if not math.isfinite(age):
                return None

            if age < 0.0:
                return None

            return age

        except (TypeError, ValueError, AttributeError):
            return None
    def is_connected(self):
        if self.vehicle is None:
            return False

        age = self.heartbeat_age()

        if age is None:
            return False

        return age <= self.LINK_HEARTBEAT_MAX_AGE_S
    def is_arm(self):
        return self.vehicle.armed
    def arm(self):
        self.vehicle.armed=True
    def dis_arm(self):
        self.vehicle.armed=False
    def velocity(self):
        if not self.is_connected():
            return 0.0

        try:
            velocity = self.vehicle.velocity

            if velocity is None or len(velocity) < 2:
                return 0.0

            vx = velocity[0]
            vy = velocity[1]

            vx = float(vx) if vx is not None else 0.0
            vy = float(vy) if vy is not None else 0.0

            if not math.isfinite(vx):
                vx = 0.0

            if not math.isfinite(vy):
                vy = 0.0

            return math.hypot(vx, vy)

        except Exception:
            return 0.0
    def altitude(self):
        if not self.is_connected():
            return 0.0

        try:
            frame = self.vehicle.location.global_relative_frame

            if frame is None:
                return 0.0

            altitude = getattr(
                frame,
                "alt",
                None
            )

            if altitude is None:
                return 0.0

            altitude = float(altitude)

            if not math.isfinite(altitude):
                return 0.0

            return altitude

        except Exception:
            return 0.0
    def set_mode(self,mode_name):
        self.vehicle.mode=VehicleMode(mode_name)
    def get_mode(self):
        return self.vehicle.mode.name
    def rtl(self):
        self.vehicle.mode=VehicleMode("RTL")
    def qrtl(self):
        if not self.is_connected():
            print("[-] QRTL verilemedi: araç bağlı değil.")
            return False

        try:
            self.vehicle.mode = VehicleMode("QRTL")

            ok = self._wait_until(
                lambda: self.vehicle.mode.name == "QRTL",
                timeout=5
            )

            if not ok:
                print("[-] QRTL moda geçilemedi.")
                return False

            print("[+] Araç QRTL moda geçti: 4 dikey motorla home'a dönüş/iniş başlatıldı.")
            return True

        except Exception as e:
            print(f"[-] QRTL mode hatası: {e}")
            return False
    def take_off(self,altitude):
        self.vehicle.simple_takeoff(altitude)
    def land(self):
        if not self.is_connected():
            return False
        try:
            self.vehicle.mode=VehicleMode("LAND")
            return True
        except Exception as e:
            print(f"land hatasi{e}")
            return False
    def configure_sim_descent_params(self):
        """
        SITL/simülasyon için kontrollü hızlı yaklaşma/iniş parametreleri.
        Gerçek uçuş için körlemesine kullanma.
        """
        if not self.is_connected():
            return False
        
        self.set_param_safe('WPNAV_RADIUS',150) 

        # Waypoint yatay hızı: 600 cm/s = 6 m/s
        self.set_param_safe('WPNAV_SPEED',600)

        # Waypoint alçalma hızı
        self.set_param_safe('WPNAV_SPEED_DN',300)

        #
        self.set_param_safe('WPNAV_ACCEL',500)

        self.set_param_safe('ANGLE_MAX',4500)
        return True
    
    def setup_kamikaze_params(self):
        """
        Normal drone için hızlı yaklaşma + kısa bekleme + kontrollü iniş profili.
        Amaç:
        - Hedef hattına hızlı gelsin.
        - Hedefin önünde/üstünde kısa beklesin.
        - LAND komutunda çok sert alçalmasın.
        """
        if not self.is_connected():
            return False

        print("[*] Drone görev parametreleri yazılıyor...")

        self.set_param_safe('AUTO_OPTIONS', 2)

        # Hızlı yatay yaklaşma
        self.set_param_safe('WPNAV_SPEED', 600)      # 6 m/s

        # Alçalma hızı
        # 300 = 3 m/s. Hızlıdır ama 400 kadar sert değil.
        self.set_param_safe('WPNAV_SPEED_DN', 300)

        # Waypoint kabul yarıçapı
        # 150 = 1.5 m. 200'e göre daha hassas, 100 kadar zorlayıcı değil.
        self.set_param_safe('WPNAV_RADIUS', 150)

        # İvmelenme
        self.set_param_safe('WPNAV_ACCEL', 500)

        # Maksimum yatış
        self.set_param_safe('ANGLE_MAX', 4500)

        print("[+] Drone görev parametreleri yazıldı.")
        return True
    
    def _wait_until(self, condition_fn, timeout=8, step=0.2):
        start = time.time()

        while time.time() - start < timeout:
            try:
                if condition_fn():
                    return True
            except Exception:
                pass

            time.sleep(step)

        return False

    def _fixed_wing_recovery_allowed(self):
        if not self.is_connected():
            return False, "MAVLink bağlantısı yok."

        if not getattr(self, "is_vtol_aircraft", False):
            return False, "Araç VTOL/QuadPlane olarak tanımlı değil."

        if not bool(self.vehicle.armed):
            return False, "Araç ARM değil."

        mode_name = str(
            getattr(self.vehicle.mode, "name", "")
        ).strip().upper()

        if mode_name != "AUTO":
            return False, f"Araç AUTO modunda değil: {mode_name}"

        state = self.current_vtol_state

        transition_to_fw = getattr(
            mavutil.mavlink,
            "MAV_VTOL_STATE_TRANSITION_TO_FW",
            1
        )
        transition_to_mc = getattr(
            mavutil.mavlink,
            "MAV_VTOL_STATE_TRANSITION_TO_MC",
            2
        )
        multicopter = getattr(
            mavutil.mavlink,
            "MAV_VTOL_STATE_MC",
            3
        )

        if state == transition_to_fw:
            return False, "Uçak plane moduna geçişini henüz tamamlamadı."

        if state in (transition_to_mc, multicopter):
            return False, (
                "Araç dikey moda geçti/geçiyor; "
                "sabit-kanat recovery iptal edildi."
            )

        # state=None veya 0 eski firmware/mesaj alınamaması olabilir.
        # state=4 normal sabit-kanat durumudur.
        return True, ""

    def set_current_mission_item(self, seq=0):
        if not self.is_connected():
            return False

        try:
            seq = int(seq)
            fc_seq = seq + 1

            # DroneKit tarafında görev başlangıç sırası
            self.vehicle.commands.next = fc_seq

            # MAVLink tarafında görev başlangıç sırası
            msg = self.vehicle.message_factory.mission_set_current_encode(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                fc_seq
            )

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

            time.sleep(0.5)

            print(f"[+] Görev başlangıç sırası {seq} olarak ayarlandı.")
            return True

        except Exception as e:
            print(f"[-] Görev başlangıç sırası ayarlanamadı: {e}")
            return False

    def start_carrier_mission(self, start_seq=0, force_disable_arming_check=False):
        """
        VTOL üzerinden bırakılan drone'un önceden yüklenmiş görevini hızlı başlatır.

        Beklenen durum:
        - Drone yerde STABILIZE modda ARM edilmiş olacak.
        - Drone görevi önceden yüklenmiş olacak.
        - Servo açıldıktan sonra bu fonksiyon sadece mission seq=1 yapıp AUTO'ya alacak.
        """

        if not self.is_connected():
            print("[-] Drone bağlı değil, görev başlatılamadı.")
            return False

        try:
            print("[*] Taşıyıcıdan ayrılan drone görevi başlatılıyor...")

            if force_disable_arming_check:
                print('[!] ARMING_CHECK kapatılması engellendi; güvenlik kontrolü açık kalacak.')

            # 1) Görev başlangıcını ilk gerçek mission item yap
            ok = self.set_current_mission_item(start_seq)
            if not ok:
                print("[-] Görev başlangıç item'ı ayarlanamadı.")
                return False

            # 2) Bırakma anında kanal override kalmasın
            try:
                self.vehicle.channels.overrides.clear()
            except Exception:
                try:
                    self.vehicle.channels.overrides['3'] = None
                except Exception:
                    pass

            # 3) Drone zaten yerde ARM edilmiş olmalı
            if not self.vehicle.armed:
                print("[!] Drone disarm görünüyor. Bu görevde drone yerde STABILIZE modda ARM edilmiş olmalı.")
                return False

            print(f"[+] Drone zaten ARM durumda. Mevcut mod: {self.vehicle.mode.name}")

            # 4) AUTO'ya birkaç kez dene
            print("[*] Drone AUTO moduna alınıyor...")

            for attempt in range(1, 6):
                self.vehicle.mode = VehicleMode("AUTO")
                self.vehicle.flush()

                ok = self._wait_until(
                    lambda: self.vehicle.mode.name == "AUTO",
                    timeout=0.8
                )

                if ok:
                    print("[+] Drone AUTO moda geçti. Yüklenen görev başladı.")

                    try:
                        self.vehicle.channels.overrides.clear()
                    except Exception:
                        try:
                            self.vehicle.channels.overrides['3'] = None
                        except Exception:
                            pass
                    return True
                last_text = getattr(self, "_last_statustext", "")
                print(
                    f"[!] AUTO denemesi başarısız ({attempt}/5). "
                    f"Mod: {self.vehicle.mode.name} | Son mesaj: {last_text}"
                )

            print("[-] KRİTİK: Drone AUTO moda geçemedi.")
            print("[-] Olası sebepler: GPS/EKF hazır değil, görev geçersiz, RC mode switch modu eziyor, failsafe aktif.")
            return False

        except Exception as e:
            print(f"[-] Taşıyıcı drone görev başlatma hatası: {e}")
            return False
        
    def _verify_uploaded_mission(self, expected_items, timeout_s=15.0):
        """Pixhawk'tan mission'ı geri okuyup YKI listesiyle doğrular."""
        if not self.is_connected():
            return False

        commands = self.vehicle.commands
        actual_items = []
        
        print("[*] Pixhawk'tan görev geri okunarak doğrulanıyor...")
        
        # Telemetri yavaşlıkları için 3 defa indirmeyi dene
        for attempt in range(3):
            try:
                commands.download()
                commands.wait_ready(timeout=5)
                actual_items = list(commands)
                if len(actual_items) == len(expected_items):
                    break
            except Exception as e:
                pass
            time.sleep(1)

        if len(actual_items) != len(expected_items):
            print(
                f"[-] MISSION VERIFY BAŞARISIZ: "
                f"Beklenen={len(expected_items)}, "
                f"Gelen={len(actual_items)}"
            )
            return False

        def close(a, b, tol):
            try:
                return abs(float(a) - float(b)) <= tol
            except Exception:
                return False

        for index, (expected, actual) in enumerate(zip(expected_items, actual_items)):
            if int(getattr(actual, "command", -1)) != expected["command"]:
                print(f"[-] MISSION VERIFY: item {index} command farklı.")
                return False

            for key, tol in (("param1", 0.15), ("param2", 0.15),
                             ("param3", 0.15), ("param4", 0.15),
                             ("alt", 0.30)):
                if not close(getattr(actual, key, None), expected[key], tol):
                    print(f"[-] MISSION VERIFY: item {index} {key} farklı.")
                    return False

            if not close(getattr(actual, "x", None), expected["lat"], 2e-6):
                print(f"[-] MISSION VERIFY: item {index} latitude farklı.")
                return False

            if not close(getattr(actual, "y", None), expected["lon"], 2e-6):
                print(f"[-] MISSION VERIFY: item {index} longitude farklı.")
                return False

        print(f"[+] MISSION VERIFIED: {len(actual_items)}/{len(expected_items)} item Pixhawk'tan geri okundu.")
        return True

    def _request_mission_count_fast(
        self,
        timeout_s=4.0,
        retries=3
    ):
        """
        Pixhawk'tan sadece mission toplamını ister.
        HOME dahil edilmez.
        7 YKI satırı için beklenen cevap 7'dir.
        """

        if self.vehicle is None:
            return None

        event = threading.Event()
        result = {"count": None}

        def on_mission_count(_vehicle, _name, message):
            try:
                result["count"] = int(message.count)
            except Exception:
                result["count"] = None

            event.set()

        self.vehicle.add_message_listener(
            "MISSION_COUNT",
            on_mission_count
        )

        try:
            target_sys = (
                self.vehicle._master.target_system or 1
            )
            target_comp = (
                self.vehicle._master.target_component or 1
            )

            mission_type = getattr(
                mavutil.mavlink,
                "MAV_MISSION_TYPE_MISSION",
                0
            )

            for attempt in range(retries):
                event.clear()
                result["count"] = None

                try:
                    self.vehicle._master.mav.mission_request_list_send(
                        target_sys,
                        target_comp,
                        mission_type
                    )
                except TypeError:
                    self.vehicle._master.mav.mission_request_list_send(
                        target_sys,
                        target_comp
                    )

                self.vehicle.flush()

                if event.wait(timeout_s):
                    if result["count"] is not None:
                        return result["count"]

                time.sleep(0.5)

            return None

        finally:
            self.vehicle.remove_message_listener(
                "MISSION_COUNT",
                on_mission_count
            )

    def _verify_mission_fast(
        self,
        expected_items,
        timeout_s=3.0,
        retries=2
    ):
        """
        Tam görev satırlarını indirmeden yalnızca Pixhawk'taki
        mission toplamını doğrular.
        """

        expected_count = len(expected_items)

        actual_count = self._request_mission_count_fast(
            timeout_s=timeout_s,
            retries=retries
        )

        if actual_count is None:
            print(
                "[-] Pixhawk'tan MISSION_COUNT alınamadı."
            )
            return False

        print(
            f"[*] Pixhawk mission sayısı: {actual_count} | "
            f"Beklenen: {expected_count}"
        )

        if actual_count != expected_count:
            print(
                "[-] MISSION COUNT DOĞRULAMASI BAŞARISIZ."
            )
            return False

        print(
            f"[+] MISSION VERIFIED: "
            f"Pixhawk'ta {actual_count} görev satırı var."
        )
        return True

    def upload_mission(self, mission_data):
        """Mission ekranındaki normal Write işlemi."""
        if not self._mission_upload_lock.acquire(blocking=False):
            self.last_mission_upload_status = (
                "Başka bir mission yüklemesi hâlâ devam ediyor."
            )
            print(
                "[-] Görev yüklenemedi: başka bir mission yüklemesi "
                "zaten devam ediyor."
            )
            return False

        try:
            result = self._upload_mission_impl(
                mission_data=mission_data,
                start_seq=0,
                in_flight=False
            )
            if not self.last_mission_upload_status:
                self.last_mission_upload_status = (
                    "Mission yükleme başarılı."
                    if result
                    else "Mission yükleme başarısız."
                )
            return result
        finally:
            self._mission_upload_lock.release()

    def _upload_mission_impl(
        self,
        mission_data,
        start_seq=0,
        in_flight=False
    ):
        self.last_mission_upload_status = ""

        if not self.is_connected():
            self.last_mission_upload_status = "Araç bağlı değil."
            print("[-] Görev yüklenemedi: araç bağlı değil.")
            return False

        if not isinstance(mission_data, (list, tuple)):
            print("[-] Görev yüklenemedi: görev listesi geçersiz.")
            return False
        if len(mission_data) == 0:
            print("[-] Görev yüklenemedi: görev listesi boş.")
            return False

        try:
            start_seq = int(start_seq)
        except (TypeError, ValueError):
            print("[-] Geçersiz görev başlangıç sırası.")
            return False
        if start_seq < 0 or start_seq >= len(mission_data):
            print(
                f"[-] Başlangıç sırası görev dışında: "
                f"{start_seq}/{len(mission_data)}"
            )
            return False
        if in_flight:
            allowed, reason = self._fixed_wing_recovery_allowed()
            if not allowed:
                print(f"[-] Recovery yüklemesi engellendi: {reason}")
                return False
        else:
            try:
                if bool(self.vehicle.armed):
                    print("[-] Normal mission Write engellendi: araç ARM durumda.")
                    return False
            except Exception:
                return False

        for index, wp in enumerate(mission_data):
            if not isinstance(wp, dict):
                print(f"[-] Mission WP{index + 1}: sözlük değil.")
                return False

            try:
                lat = float(wp.get("lat", 0.0))
                lon = float(wp.get("lon", wp.get("lng", 0.0)))
                alt = float(wp.get("alt", 0.0))
            except (TypeError, ValueError):
                print(f"[-] Mission WP{index + 1}: koordinat/irtifa geçersiz.")
                return False

            if not all(math.isfinite(v) for v in (lat, lon, alt)):
                print(f"[-] Mission WP{index + 1}: NaN/sonsuz değer var.")
                return False

            if not (-90.0 <= lat <= 90.0):
                print(f"[-] Mission WP{index + 1}: latitude geçersiz.")
                return False

            if not (-180.0 <= lon <= 180.0):
                print(f"[-] Mission WP{index + 1}: longitude geçersiz.")
                return False

        cmds = self.vehicle.commands

        # Komutları önce yerelde oluşturuyoruz. Böylece yeni görevde bir
        # mod/parametre hatası varsa Pixhawk'taki mevcut göreve dokunulmaz.
        # Pixhawk ancak bütün yerel kontroller geçtikten sonra temizlenir.
        commands_to_upload = []
        expected_items = []

        home_cmd = Command(
            0, 0, 0, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
            0, 1, 
            0, 0, 0, 0, 
            0.0, 0.0, 0.0
        )
        commands_to_upload.append(home_cmd)

        for wp in mission_data:
            mode = str(wp.get('mode', '')).strip().upper() 

            is_vtol = getattr(self, "is_vtol_aircraft", False)

            if is_vtol:
                forbidden_modes = ["RTL",  "TAKEOFF"]
            else:
                forbidden_modes = [
                    "VTOL_LAND",
                    "ALT_SERVO_AC",
                    "UST_SERVO_AC",
                    "ROLE_KES"
                ]
            if mode in forbidden_modes:
                print(f"[-] GÖREV REDDEDİLDİ: {mode} bu araç tipi için uygun değil.")
                return False

            p1, p2, p3, p4 = 0.0, 0.0, 0.0, 0.0
            
            cmd_lat = float(wp.get('lat', 0.0))
            cmd_lon = float(wp.get('lon', wp.get('lng', 0.0)))
            cmd_alt = float(wp.get('alt', 0.0))
            
            command_type = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

            if mode == "WAYPOINT":
                try:
                    p2 = max(
                        0.0,
                        float(
                            wp.get(
                                "acceptance_radius_m",
                                0.0,
                            )
                        ),
                    )
                except (TypeError, ValueError):
                    p2 = 0.0

            elif mode == "TAKEOFF":
                command_type = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif mode == "LAND":
                command_type = mavutil.mavlink.MAV_CMD_NAV_LAND
            elif mode == "VTOL_TAKEOFF":
                command_type = mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF
            elif mode == "VTOL_LAND":
                command_type = mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND

            elif mode == "RTL":
                command_type = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0

            elif mode == 'ALT_SERVO_AC':
                command_type = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                p1 = 5.0        
                p2 = 2078.0     
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0

            elif mode == 'UST_SERVO_AC':
                command_type = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                p1 = 6.0       
                p2 = 2125.0     
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0

            elif mode == 'ROLE_KES':
                command_type = mavutil.mavlink.MAV_CMD_DO_SET_RELAY
                p1 = 0.0      
                p2 = 0.0      
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0
    
            elif mode == 'BEKLE':
                command_type = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
                p1 = float(wp.get('wait', 5.0))  

            elif mode == 'UZUN_BEKLE':
                command_type = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
                p1 = float(wp.get('wait', 10.0))

            elif mode == 'HOLD':
                command_type = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
                p1 = 0.0
            
            elif mode == "QRTL":
                command_type = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
                p1 = 1.0
                
            elif mode == 'TRANSITION_DRONE':
                command_type = mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION
                p1 = 3.0      
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0
                
            elif mode == 'TRANSITION_PLANE':
                command_type = mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION
                p1 = 4.0      
                cmd_lat, cmd_lon, cmd_alt = 0.0, 0.0, 0.0
                
            # ANA WAYPOINT EKLENİYOR
            cmd = Command(
                0, 0, 0, 
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                command_type, 
                0, 1, 
                p1, p2, p3, p4, 
                cmd_lat, cmd_lon, cmd_alt
            )
            expected_items.append({
                "command": int(command_type),
                "param1": float(p1),
                "param2": float(p2),
                "param3": float(p3),
                "param4": float(p4),
                "lat": float(cmd_lat),
                "lon": float(cmd_lon),
                "alt": float(cmd_alt),
            })
            commands_to_upload.append(cmd)
            
            # HIZ KONTROLÜ
            if mode == "WAYPOINT":
                survey_kind = wp.get("survey_kind", "")
                
                # LEAD_IN: Taramaya giriyor, HIZLAN
                if survey_kind == "LEAD_IN":
                    speed_cmd = Command(
                        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1,
                        0, 13.0, -1, 0, 0.0, 0.0, 0.0
                    )
                    expected_items.append({
                        "command": int(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED),
                        "param1": 0.0, "param2": 13.0, "param3": -1.0, "param4": 0.0,
                        "lat": 0.0, "lon": 0.0, "alt": 0.0,
                    })
                    commands_to_upload.append(speed_cmd)
                    
                # SCAN_END: Tarama bitti, YAVAŞLA
                elif survey_kind == "SCAN_END":
                    speed_cmd = Command(
                        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1,
                        0, 13.0, -1, 0, 0.0, 0.0, 0.0
                    )
                    expected_items.append({
                        "command": int(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED),
                        "param1": 0.0, "param2": 13.0, "param3": -1.0, "param4": 0.0,
                        "lat": 0.0, "lon": 0.0, "alt": 0.0,
                    })
                    commands_to_upload.append(speed_cmd)

        def reset_local_command_sequence():
            """Yerel komut listesini her denemede eksiksiz yeniden kurar."""
            cmds.clear()
            for command in commands_to_upload:
                cmds.add(command)

        try:
            # Bu, ilk gerçek Write işlemidir. Buraya kadar olan kontroller
            # başarısız olursa Pixhawk'taki eski görev silinmez.
            reset_local_command_sequence()
        except Exception as e:
            print(f"[-] Mission komut listesi hazırlanamadı: {e}")
            return False

        # `commands.upload()` normal bir şekilde döndüyse MAVLink mission
        # transferi tamamlanmıştır. Hızlı MISSION_COUNT isteği yalnızca ek
        # bilgi/diagnostic olarak kullanılacak; HOME veya yavaş link yüzünden
        # başarılı Write işlemi başarısız sayılmayacak.
        upload_completed = False
        count_confirmed = False
        actual_count = None
        max_attempts = 3

        for attempt in range(1, max_attempts + 1):
            try:
                if attempt > 1:
                    # Önceki deneme yarıda kaldıysa ArduPilot'un yarım
                    # transfer durumunu sıfırla ve tüm listeyi tekrar ekle.
                    reset_local_command_sequence()
                    time.sleep(1.0)

                print(
                    f"[*] Mission Write denemesi {attempt}/{max_attempts} "
                    f"({len(expected_items)} görev satırı)..."
                )

                upload_timeout = 15 if in_flight else 60
                try:
                    cmds.upload(timeout=upload_timeout)
                except TypeError:
                    cmds.upload()
                
                self.vehicle.flush()
                upload_completed = True

                # Tam mission download yapılmaz. Yalnızca Pixhawk'ın tuttuğu
                # mission satır sayısı istenir; bu MISSION_COUNT isteği görevi
                # silmez ve yavaş telemetride gereksiz veri taşımaz. Bu
                # kontrolden sonra kesinlikle ikinci bir clear+upload yapılmaz.
                try:
                    time.sleep(0.5)
                    actual_count = self._request_mission_count_fast(
                        timeout_s=3.0,
                        retries=2
                    )
                except Exception as count_error:
                    actual_count = None
                    print(
                        "[!] Write tamamlandı; hızlı MISSION_COUNT kontrolü "
                        f"alınamadı: {count_error}"
                    )

                if actual_count == len(expected_items):
                    print(
                        f"[+] MISSION VERIFIED: Pixhawk'ta "
                        f"{actual_count} görev satırı var."
                    )
                    count_confirmed = True

                if actual_count is None:
                    # Upload tamamlandığı için görevi başarısız saymıyoruz.
                    # Sayım cevabı gelmedi diye görevi tekrar temizleyip
                    # göndermek, yavaş linkte asıl riski büyütür.
                    print(
                        "[!] Pixhawk MISSION_COUNT cevabı alınamadı; "
                        "Write tamamlandı, yeniden silme yapılmayacak."
                    )
                elif not count_confirmed:
                    # Tam geri okumadaki dummy HOME veya telemetri yoğunluğu
                    # gibi sayı tabanı/okuma farkları başarılı upload'ı
                    # reddetmemelidir.
                    print(
                        f"[!] Write tamamlandı; hızlı Pixhawk mission sayısı "
                        f"{actual_count}, beklenen {len(expected_items)}. "
                        "Sayı kontrolü uyarı olarak bırakıldı; yeniden "
                        "clear+upload yapılmayacak."
                    )

                # `upload()` exception vermeden döndü. Bundan sonra count
                # kontrolü ne gelirse gelsin aynı görevi tekrar silip yazma.
                break

            except Exception as e:
                print(
                    f"[-] Mission upload başarısız "
                    f"(Deneme {attempt}/{max_attempts}): {e}"
                )

            if attempt < max_attempts:
                print("[*] Mission yeniden gönderilecek...")
                time.sleep(2.0)
                
        if not upload_completed:
            self.last_mission_upload_status = (
                "Mission upload timeout/exception verdi; görev Pixhawk'a "
                "tamamlanmış olarak gönderilemedi."
            )
            print(
                "[-] GÖREV YÜKLENEMEDİ: upload() tamamlanmadı. "
                "AUTO/kalkış yapmayın."
            )
            return False

        mission_name = "Recovery mission" if in_flight else "Normal mission"
        if count_confirmed:
            self.last_mission_upload_status = (
                f"{mission_name} Pixhawk'ta {len(expected_items)} satır olarak "
                "yazıldı ve hızlı sayı kontrolüyle doğrulandı."
            )
        elif actual_count is None:
            self.last_mission_upload_status = (
                f"{mission_name} upload() tarafından tamamlandı; hızlı "
                "MISSION_COUNT teyidi alınamadı."
            )
        else:
            self.last_mission_upload_status = (
                f"{mission_name} upload() tarafından tamamlandı; Pixhawk "
                f"mission sayısı {actual_count}, beklenen {len(expected_items)}. "
                "Sayı farkı yalnızca uyarı olarak bırakıldı."
            )
        print(
            f"[+] {mission_name} Pixhawk'a yazıldı: "
            f"{len(expected_items)} görev satırı. "
            f"Hızlı sayı teyidi={'OK' if count_confirmed else 'uyarı'}"
        )

        if in_flight:
            allowed, reason = self._fixed_wing_recovery_allowed()
            if not allowed:
                print(
                    "[!] Recovery yüklendi fakat aktif edilmedi: "
                    f"{reason}"
                )
                return False
        try:
            self.current_mission_seq = -1
            fc_seq = start_seq + 1

            msg = self.vehicle.message_factory.mission_set_current_encode(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                fc_seq
            )

            self.vehicle.send_mavlink(msg)
            self.vehicle.commands.next = fc_seq
            self.vehicle.flush()

        except Exception as e:
            self.last_mission_upload_status = (
                "Görev doğrulandı fakat başlangıç sırası ayarlanamadı."
            )
            print(f"[-] Görev başlangıcı ayarlanamadı: {e}")
            return False

        if in_flight:
            fc_seq = start_seq + 1
            activated = self._wait_until(
                lambda: self.get_mission_next() >= fc_seq,
                timeout=3.0,
                step=0.1
            )

            if not activated:
                print(
                    "[-] Recovery yüklendi fakat MISSION_CURRENT "
                    "doğrulanamadı."
                )
                return False

            print(
                f"[+] Recovery aktif: mission seq={self.get_mission_next()}"
            )

        return True

    def upload_recovery_mission_in_flight(
        self,
        recovery_mission,
        start_seq=0
    ):
        """
        Eksik tarama hatlarını uçuş sırasında yükler.

        Bu fonksiyon:
        - aracı ARM etmez,
        - AUTO moda zorlamaz,
        - VTOL transition göndermez,
        - hız/roll parametresi değiştirmez.
        """

        allowed, reason = self._fixed_wing_recovery_allowed()
        if not allowed:
            print(f"[-] Recovery başlamadı: {reason}")
            return False

        if not isinstance(recovery_mission, (list, tuple)):
            print("[-] Recovery görevi liste değil.")
            return False

        if len(recovery_mission) == 0:
            print("[-] Recovery görevi boş.")
            return False

        allowed_modes = {
            "",
            "WAYPOINT",
            "LAND",
            "VTOL_TAKEOFF",
            "VTOL_LAND",
            "QRTL",
            "ALT_SERVO_AC",
            "UST_SERVO_AC",
            "ROLE_KES",
            "BEKLE",
            "UZUN_BEKLE",
            "HOLD",
            "TRANSITION_DRONE",
            "TRANSITION_PLANE",
        }

        coordinate_modes = {
            "",
            "WAYPOINT",
            "LAND",
            "VTOL_TAKEOFF",
            "VTOL_LAND",
            "QRTL",
            "BEKLE",
            "UZUN_BEKLE",
            "HOLD",
        }

        # Bütün görev doğrulanmadan mevcut göreve dokunma.
        for index, item in enumerate(recovery_mission, start=1):
            if not isinstance(item, dict):
                print(f"[-] Recovery WP{index} sözlük değil.")
                return False

            mode = str(
                item.get("mode", "WAYPOINT")
            ).strip().upper()

            if mode not in allowed_modes:
                print(
                    f"[-] Recovery WP{index}: "
                    f"desteklenmeyen mod {mode}"
                )
                return False

        if mode in coordinate_modes:
            try:
                lat = float(item.get("lat"))
                lon = float(
                    item.get("lon", item.get("lng"))
                )
                alt = float(item.get("alt"))
            except (TypeError, ValueError):
                print(
                    f"[-] Recovery WP{index}: "
                    "koordinat/irtifa geçersiz."
                )
                return False

            if not all(map(math.isfinite, (lat, lon, alt))):
                print(f"[-] Recovery WP{index}: NaN/sonsuz değer.")
                return False

            if not (-90.0 <= lat <= 90.0):
                print(f"[-] Recovery WP{index}: latitude geçersiz.")
                return False

            if not (-180.0 <= lon <= 180.0):
                print(f"[-] Recovery WP{index}: longitude geçersiz.")
                return False

        if not self._mission_upload_lock.acquire(blocking=False):
            self.last_mission_upload_status = (
                "Başka bir mission yüklemesi hâlâ devam ediyor."
            )
            print(
                "[-] Recovery yüklenemedi: başka bir mission yüklemesi "
                "zaten devam ediyor."
            )
            return False

        try:
            print(
                "[*] Uçuş içi recovery yükleniyor: "
                f"{len(recovery_mission)} satır."
            )

            return self._upload_mission_impl(
                mission_data=recovery_mission,
                start_seq=start_seq,
                in_flight=True
            )
        finally:
            self._mission_upload_lock.release()
    
    def location(self):
        if not self.is_connected():
            return None, None

        try:
            frame = self.vehicle.location.global_relative_frame

            if frame is None:
                return None, None

            lat = getattr(frame, "lat", None)
            lon = getattr(frame, "lon", None)

            lat = float(lat) if lat is not None else None
            lon = float(lon) if lon is not None else None

            if lat is not None and not math.isfinite(lat):
                lat = None

            if lon is not None and not math.isfinite(lon):
                lon = None

            return lat, lon

        except Exception:
            return None, None
    def get_attitude(self):
        if not self.is_connected():
            return 0.0, 0.0

        try:
            attitude = self.vehicle.attitude

            if attitude is None:
                return 0.0, 0.0

            pitch_rad = getattr(attitude, "pitch", None)
            roll_rad = getattr(attitude, "roll", None)

            pitch = (
                math.degrees(float(pitch_rad))
                if pitch_rad is not None
                else 0.0
            )

            roll = (
                math.degrees(float(roll_rad))
                if roll_rad is not None
                else 0.0
            )

            if not math.isfinite(pitch):
                pitch = 0.0

            if not math.isfinite(roll):
                roll = 0.0

            return pitch, roll

        except Exception:
            return 0.0, 0.0
    def get_gps(self):
        if not self.is_connected():
            return 0, 0

        try:
            gps = self.vehicle.gps_0

            if gps is None:
                return 0, 0

            satellites = getattr(
                gps,
                "satellites_visible",
                0
            )

            fix_type = getattr(
                gps,
                "fix_type",
                0
            )

            satellites = (
                int(satellites)
                if satellites is not None
                else 0
            )

            fix_type = (
                int(fix_type)
                if fix_type is not None
                else 0
            )

            return satellites, fix_type

        except Exception:
            return 0, 0
    def get_heading(self):
        if not self.is_connected():
            return 0.0

        try:
            heading = getattr(
                self.vehicle,
                "heading",
                None
            )

            if heading is None:
                return 0.0

            heading = float(heading)

            if not math.isfinite(heading):
                return 0.0

            return heading

        except Exception:
            return 0.0
    
    def get_mission_next(self):
        if not self.is_connected():
            return -1
        # Öncelik: MAVLink MISSION_CURRENT listener verisi
        try:
            seq = int(getattr(self, "current_mission_seq", -1))
            if seq >= 0:
                return seq
        except Exception:
            pass
        # Yedek: DroneKit commands.next
        try:
            return int(self.vehicle.commands.next)
        except Exception:
            return -1

    def start_accel_calibration_step(self, step_index):
        """
        6 pozisyonlu accelerometer kalibrasyonu.
        step_index=0 → kalibrasyonu başlat
        step_index=1..6 → o pozisyonu kaydet
        """
        if not self.is_connected():
            return False
        try:
            if step_index == 0:
            # Kalibrasyonu başlat: param5=1
                msg = self.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0, 0, 0, 0, 1, 0, 0
                )
            else:
            # Sonraki pozisyonu kaydet: param5=step_index
                msg = self.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0, 0, 0, 0,
                    float(step_index),
                    0, 0
                )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            return True
        except Exception as e:
            print(f"Accel kalibrasyon hatası: {e}")
            return False
    def start_compass_calibration(self):
        """Compass kalibrasyonunu başlat"""
        if not self.is_connected():
            return False
        try:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0, 1, 0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            return True
        except Exception as e:
            print(f"Compass kalibrasyon hatası: {e}")
            return False

    def cancel_calibration(self):
        """Tüm kalibrasyonları iptal et"""
        if not self.is_connected():
            return False
        try:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            return True
        except Exception as e:
            return False

class VTOL(Drone):
    def __init__(self):
        super().__init__()
        self.is_vtol_aircraft = True
        print("VTOL moduna geçildi")
    
    def disable_forward_throttle_outputs(self, scan_until=32,allow_disable=False):
        """
        Fixed-wing / pusher throttle çıkışlarını kapatır.
        SERVOx_FUNCTION = 70 olan çıkışları bulur ve 0 yapar.

        70 = Throttle / forward motor output.
        0  = Disabled.

        Not:
        - SERVO3 diye sabitlemiyoruz.
        - Hangi SERVO çıkışında 70 varsa onu buluyoruz.
        - Araç ARM ise asla parametre değiştirmiyoruz.
        """
        if not allow_disable:
            print("[-] Pusher kapatma BLOKE edildi. 5-motor testinde SERVO7 korunuyor.")
            print("[-] Gerçekten 4-motor saf dikey profile dönmek istiyorsan allow_disable=True ile çağır.")
            return False

        if not self.is_connected():
            print("[-] Pusher kapatma iptal: araç bağlı değil.")
            return False

        if self.vehicle.armed:
            print("[-] Pusher kapatma iptal: araç ARM durumda. Uçuşta SERVO parametresi değiştirilmez.")
            return False

        found_outputs = []
        disabled_outputs = []

        print("[*] Pusher/throttle çıkışı taranıyor: SERVO1_FUNCTION ... SERVO32_FUNCTION")

        for servo_no in range(1, scan_until + 1):
            param_name = f"SERVO{servo_no}_FUNCTION"

            try:
                value = self.vehicle.parameters.get(param_name, None)

                # Bazı kartlarda SERVO17+ olmayabilir, yoksa geç.
                if value is None:
                    continue

                value_int = int(float(value))

                # 70 = Throttle / fixed-wing forward motor
                if value_int == 70:
                    found_outputs.append(param_name)
                    print(f"[!] Pusher/throttle çıkışı bulundu: {param_name} = {value_int}")

                    ok = self.set_param_safe(param_name, 0)

                    if ok:
                        disabled_outputs.append(param_name)
                        print(f"[+] {param_name} kapatıldı: 70 -> 0")
                    else:
                        print(f"[-] {param_name} kapatılamadı!")

            except Exception as e:
                print(f"[!] {param_name} kontrol edilirken hata: {e}")

        if not found_outputs:
            print("[*] SERVOx_FUNCTION = 70 bulunamadı. Pusher/throttle çıkışı zaten kapalı olabilir.")
            return True

        if found_outputs and len(disabled_outputs) == len(found_outputs):
            print(f"[+] Tüm pusher/throttle çıkışları kapatıldı: {disabled_outputs}")
            return True

        print(f"[-] Bazı pusher çıkışları kapatılamadı. Bulunan: {found_outputs}, Kapatılan: {disabled_outputs}")
        return False

    def enable_4vtol_mode(self):
        if not self.is_connected():
            return False

        def arka_planda_ayarla():
            print("[*] VTOL güvenli 4-motor görev ayarları için 5 sn bekleniyor...")
            time.sleep(5)

            if not self.is_connected():
                print("[-] VTOL ayarı iptal: bağlantı yok.")
                return

            # Uçuşta parametre yazmak istemiyoruz.
            if self.vehicle.armed:
                print("[-] VTOL ayarı iptal: araç ARM durumda. Parametreler uçuşta yazılmayacak.")
                return

            print("[*] Güvenli VTOL görev ayarları yazılıyor...")

            self.disable_forward_throttle_outputs(scan_until=32, allow_disable=True)

            # ------------------------------------------------------------
            # 1) Pre-arm güvenliği
            # ------------------------------------------------------------
            # 1 = tüm uygun arming check'leri açık tut.
            # Gerçek uçuşta tüm arming check'leri açık tut.
            self.set_param_safe('ARMING_CHECK', 1)

            self.set_param_safe('Q_RTL_MODE', 3)
            # ------------------------------------------------------------
            # 2) QuadPlane temel ayarları
            # ------------------------------------------------------------
            # Bu ikisini sadece frame tipiniz gerçekten quad/x ise kullan.
            # Daha önce Mission Planner'da doğru ayarlandıysa yine aynı değeri yazar.
            self.set_param_safe('Q_ENABLE', 1)
            self.set_param_safe('Q_FRAME_CLASS', 1)  # 1 = Quad
            self.set_param_safe('Q_FRAME_TYPE', 1)   # 1 = X frame

            # ------------------------------------------------------------
            # 3) 4 dikey motorla yavaş, sakin waypoint uçuşu
            # ------------------------------------------------------------
            # Q_WP_SPEED cm/s: 300 = 3 m/s, 350 = 3.5 m/s.
            # İlk gerçek test için 300 daha sakin.
            self.set_param_safe('Q_WP_SPEED', 450)
            self.set_param_safe("Q_ASSIST_SPEED", 15) 

            # Q_WP_ACCEL cm/s/s: 50 minimum/yumuşak ivmelenme.
            self.set_param_safe('Q_WP_ACCEL', 50)

            # Waypoint kabul yarıçapı. 500 cm = 5 m.
            # Çok küçük olursa rüzgarda noktaya oturmaya çalışıp daire/yalpa yapabilir.
            self.set_param_safe('Q_WP_RADIUS', 200)

            # Dikey kalkış/iniş hızları.
            self.set_param_safe('Q_WP_SPEED_UP', 150)  # 1.5 m/s çıkış
            self.set_param_safe('Q_WP_SPEED_DN', 80)   # 0.8 m/s iniş/dönüş alçalma

            # Yatma açısı. 1500 = 15 derece gibi düşün.
            # Rüzgarlı havada 2000 gerekebilir ama ilk testte 1500 daha sakindir.
            self.set_param_safe('Q_A_ANGLE_MAX', 15) 

            # ------------------------------------------------------------
            # 4) Yatay motor desteğini yazılımsal olarak azalt
            # ------------------------------------------------------------
            # QuadPlane VTOL hareketinde ileri motor desteğini kapat.
            self.set_param_safe('Q_FWD_THR_USE', 0)

            # VTOL hareket sırasında ileri itiş katkısını sıfırla.
            self.set_param_safe('Q_VFWD_GAIN', 0)

            # Hava hız sensörü kullanmıyorsanız bunu kapalı tutmak mantıklı.
            # Airspeed sensörünüz gerçekten varsa bu satırları körlemesine kullanmayın.
            self.set_param_safe('ARSPD_USE', 0)
            self.set_param_safe('ARSPD_TYPE', 0)

            # GUIDED kullanırsanız hedefte fixed-wing circle yerine VTOL hover davranışı için.
            self.set_param_safe('Q_GUIDED_MODE', 1)

            # ------------------------------------------------------------
            # 5) QRTL'yi pure VTOL dönüş yapacak hale getir
            # ------------------------------------------------------------
            QOPT_FIXED_WING_APPROACH = 1 << 4      # 16
            QOPT_QRTL_ON_VTOL_FAILSAFE = 1 << 5    # 32
            QOPT_PURE_VTOL_QRTL = 1 << 16          # 65536
            QOPT_NORMAL_RTL_ON_VTOL_FAILSAFE = 1 << 20  # 1048576

            # Bit 16: QRTL'de fixed-wing approach'u kapat, pure VTOL dönüş/iniş yap.
            # Bit 4 açıksa fixed-wing approach davranışı isteyebilir, o yüzden kapatıyoruz.
            # Bit 20 açıksa VTOL failsafe normal RTL'ye kayabilir, o yüzden kapatıyoruz.
            # Bit 5: VTOL failsafe durumlarında QLAND yerine QRTL tercih etsin.
            self.set_q_option_bits(
                bits_to_set=QOPT_PURE_VTOL_QRTL | QOPT_QRTL_ON_VTOL_FAILSAFE,
                bits_to_clear=QOPT_FIXED_WING_APPROACH | QOPT_NORMAL_RTL_ON_VTOL_FAILSAFE
            )

            # ------------------------------------------------------------
            # 6) QRTL dönüş irtifası
            # ------------------------------------------------------------
            # 15 m: senin istediğin yarışma/video akışına uygun.
            # Engel varsa bu değer artırılmalı.
            self.set_param_safe('Q_RTL_ALT', 15)

            print("[+] VTOL 4-motor güvenli görev ayarları tamamlandı.")
            print("[!] Not: AUTO görevde TRANSITION_PLANE ve mission RTL kullanma.")

        t = threading.Thread(target=arka_planda_ayarla)
        t.daemon = True
        t.start()

        return True
    
    def enable_5motor_forward_assist_mode(self, pusher_servo_no=8):
        """
        Assisted Fixed-Wing / 5 motor profili.
        """
        if not self.is_connected():
            return False

        def arka_planda_ayarla():
            print("[*] Assisted Fixed-Wing 5-motor ayarları için 5 sn bekleniyor...")
            time.sleep(5)

            if not self.is_connected():
                print("[-] 5-motor ayarı iptal: bağlantı yok.")
                return

            if self.vehicle.armed:
                print("[-] 5-motor ayarı iptal: araç ARM durumda. Parametreler uçuşta yazılmayacak.")
                return

            print("[*] Assisted Fixed-Wing / 5-motor ayarları yazılıyor...")

            # 1) Güvenlik
            

            # 2) QuadPlane temel ayarları
            self.set_param_safe('Q_ENABLE', 1)
            self.set_param_safe('Q_FRAME_CLASS', 1)
            self.set_param_safe('Q_FRAME_TYPE', 1)

            # 3) Ön/arka pusher motor çıkışı
            self.set_param_safe(f'SERVO{pusher_servo_no}_FUNCTION', 70)
            self.set_param_safe(f'SERVO{pusher_servo_no}_MIN', 1000)
            self.set_param_safe(f'SERVO{pusher_servo_no}_TRIM', 1000)
            self.set_param_safe(f'SERVO{pusher_servo_no}_MAX', 1900)

            # 4) VTOL/Q-mode kalkış ve iniş hareket ayarları (SERT İNİŞ BURADA ÇÖZÜLÜYOR)
            self.set_param_safe('Q_WP_SPEED', 450)
            self.set_param_safe('Q_WP_RADIUS', 500)
            self.set_param_safe('Q_WP_ACCEL', 100)
            self.set_param_safe('Q_WP_SPEED_UP', 150)
            
            # İniş Hızları
            self.set_param_safe('Q_WP_SPEED_DN', 20)       # Normal alçalma hızı: 60 cm/s
            self.set_param_safe('Q_LAND_FINAL_ALT', 7)    # Son 5 metrede yavaşla
            self.set_param_safe('Q_LAND_FINAL_SPD', 0.5)    # 
            self.set_param_safe('Q_A_ANGLE_MAX', 20)       # Rüzgara karşı daha iyi direnç için 15'ten 20'ye alındı

            # 5) Assisted Fixed-Wing / Transition throttle ayarları
            if pusher_servo_no == 4:
                # SITL profili
                self.set_param_safe('Q_ASSIST_SPEED', 10)
                self.set_param_safe('Q_ASSIST_ANGLE', 35)

                self.set_param_safe('THR_MIN', 0)
                self.set_param_safe('THR_MAX', 100)
                self.set_param_safe('TRIM_THROTTLE', 55)

                self.set_param_safe('TKOFF_THR_MIN', 50)
                self.set_param_safe('TKOFF_THR_MAX', 100)

                self.set_param_safe('AIRSPEED_CRUISE', 15)
                self.set_param_safe('AIRSPEED_MIN',13)
                self.set_param_safe('WP_RADIUS',15)
                self.set_param_safe('AIRSPEED_MAX',22)
            else:
                # GERÇEK UÇAK PROFİLİ
                self.set_param_safe('Q_ASSIST_SPEED', 10)
                self.set_param_safe('Q_ASSIST_ANGLE', 35)

                self.set_param_safe('THR_MIN', 0)
                self.set_param_safe('THR_MAX', 100)

                # 65 yerine 55-60 daha kontrollü başlangıç.
                self.set_param_safe('TRIM_THROTTLE', 60)

                self.set_param_safe('TKOFF_THR_MIN', 50)
                self.set_param_safe('TKOFF_THR_MAX', 95)

                self.set_param_safe('AIRSPEED_CRUISE', 15)
                self.set_param_safe('AIRSPEED_MIN', 13)
                self.set_param_safe('AIRSPEED_MAX',22)
                self.set_param_safe('WP_RADIUS',9)

            # YAN YAN GİTME SORUNU ÇÖZÜMÜ
            self.set_param_safe('Q_FWD_THR_USE', 0)
            self.set_param_safe('Q_FWD_THR_GAIN', 0)
            self.set_param_safe('Q_VFWD_GAIN', 0)
            self.set_param_safe('Q_VFWD_ALT', 0)
            
            self.set_param_safe('Q_WVANE_ENABLE', 1)        
            
            # TRANSITION ZAMAN AŞIMI ÇÖZÜMÜ
            # TRANSITION ZAMAN AŞIMI / AKILLI DEVAM
            self.set_param_safe('Q_TRANS_FAIL',0)
            self.set_param_safe('Q_TRANS_FAIL_ACT', 1)

            # GUIDED hedeflerde VTOL davranışı tercih edilsin.
            self.set_param_safe('Q_GUIDED_MODE', 1)

            # 7) QRTL pure VTOL dönüş/iniş ayarı
            self.set_param_safe('Q_RTL_MODE', 3)

            QOPT_FIXED_WING_APPROACH = 1 << 4
            QOPT_QRTL_ON_VTOL_FAILSAFE = 1 << 5
            QOPT_PURE_VTOL_QRTL = 1 << 16
            QOPT_FORCE_FW_TRANSITION_ON_FAIL = 1 << 19
            QOPT_NORMAL_RTL_ON_VTOL_FAILSAFE = 1 << 20

            self.set_q_option_bits(
                bits_to_set=(
                    QOPT_PURE_VTOL_QRTL |
                    QOPT_QRTL_ON_VTOL_FAILSAFE |
                    QOPT_FORCE_FW_TRANSITION_ON_FAIL
                ),
                bits_to_clear=(
                    QOPT_FIXED_WING_APPROACH |
                    QOPT_NORMAL_RTL_ON_VTOL_FAILSAFE
                )
            )
            self.print_transition_param_check()

            self.set_param_safe('Q_RTL_ALT', 15) # Dönüş irtifası güvenli olsun diye 20 metre yapıldı

            print("[+] Assisted Fixed-Wing / 5-motor ayarları tamamlandı.")
            print("[!] Bu profilde mission şu mantıkta olmalı:")
            print("    VTOL_TAKEOFF -> TRANSITION_PLANE -> WAYPOINT -> TRANSITION_DRONE -> BEKLE -> QRTL")

        t = threading.Thread(target=arka_planda_ayarla)
        t.daemon = True
        t.start()
        return True
    
    def print_transition_param_check(self):
        """
        Transition ile ilgili kritik parametreleri tek tek ekrana basar.
        Bunu ayarlar yazıldıktan sonra çağıracağız.
        """
        if not self.is_connected():
            return False

        names = [
            'Q_OPTIONS',
            'Q_TRANS_FAIL',
            'Q_TRANS_FAIL_ACT',
            'Q_ASSIST_SPEED',
            'Q_ASSIST_ANGLE',
            'AIRSPEED_MIN',
            'AIRSPEED_CRUISE',
            'ARSPD_USE',
            'THR_MIN',
            'THR_MAX',
            'TRIM_THROTTLE',
            'TKOFF_THR_MIN',
            'TKOFF_THR_MAX',
        ]

        print("\n========== TRANSITION PARAM CHECK ==========")
        for name in names:
            try:
                print(f"[CHECK] {name} = {self.vehicle.parameters.get(name, None)}")
            except Exception as e:
                print(f"[CHECK] {name} okunamadı: {e}")
        print("===========================================\n")

        return True

    def enable_vtol_mode(self):
        """
        Bu görev videosunda varsayılan VTOL profili 5-motor forward assist profilidir.

        Eski enable_vtol_mode artık ön motoru kapatan 4-motor profili çalıştırmayacak.
        Yanlışlıkla DroneManager enable_vtol_mode çağırsa bile SERVO7 korunur.
        """

        print("[*] enable_vtol_mode çağrıldı.")
        print("[*] Bu görev için güvenli 5-motor forward assist profiline yönlendiriliyor.")

        return self.enable_5motor_forward_assist_mode(pusher_servo_no=8)

    def do_vtol_transition(self, state):
        if not self.is_connected() or self.vehicle is None:
            print("Uyarı: Uçağa bağlı değilken transition yapılamaz.")
            return False

        try:
            state = int(state)

            if state not in [3, 4]:
                print("[-] Geçersiz transition state. 3=VTOL, 4=Plane olmalı.")
                return False

            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                0,
                state,
                0, 0, 0, 0, 0, 0
            )

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

            if state == 4:
                print("[+] Geçiş emri gönderildi: PLANE / FIXED-WING MODU")
            else:
                print("[+] Geçiş emri gönderildi: VTOL / DRONE MODU")

            return True

        except Exception as e:
            print(f"Transition hatası: {e}")
            return False
    def take_off(self, altitude):
        if not self.is_connected():
            return False
        try:
            # 1. Mod kontrolü — sadece GUIDED kabul ediyor
            current_mode = self.vehicle.mode.name
            if current_mode != "GUIDED":
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1.5)  # Mod geçişini bekle
                print("[*] GUIDED moda geçildi, takeoff hazırlanıyor...")

            # 2. Arm değilse arm et
            if not self.vehicle.armed:
                self.vehicle.armed = True
                timeout = 8
                start = time.time()
                while not self.vehicle.armed:
                    if time.time() - start > timeout:
                        print("[-] Arm zaman aşımı!")
                        return False
                    time.sleep(0.3)
                print("[+] Arm başarılı.")
                time.sleep(0.8)  # Pixhawk stabilize olsun

            # 3. Takeoff — VTOL için simple_takeoff en güvenilir
            self.vehicle.simple_takeoff(altitude)
            print(f"[+] VTOL Takeoff komutu gönderildi. Hedef: {altitude}m")

            # 4. Hedef irtifaya ulaşana kadar bekle (arka planda)
            # Bu sayede QLOITER geçişi güvenli irtifada olur
        
            def monitor_climb():
                while True:
                    current_alt = self.vehicle.location.global_relative_frame.alt or 0
                    print(f"[~] İrtifa: {current_alt:.1f}m / Hedef: {altitude}m")
                    if current_alt >= altitude * 0.85:
                        print(f"[+] Hedef irtifaya ulaşıldı ({current_alt:.1f}m). QLOITER güvenli.")
                        break
                    if not self.vehicle.armed:
                        print("[-] Drone disarm oldu, izleme durduruluyor.")
                        break
                    time.sleep(1)

            t = threading.Thread(target=monitor_climb, daemon=True)
            t.start()
            return True
        except Exception as e:
            print(f"[-] VTOL Kalkış Hatası: {e}")
            return False
    def rtl(self):
        """
        VTOL için RTL butonu da güvenli QRTL çalıştırsın.
        Ön motor forward assist önce kapansın.
        """
        return self.qrtl()
    
    def set_mode(self, mode_name):
        """
        VTOL için mode değiştirme güvenlik kapısı.
        QRTL veya RTL istenirse direkt mode basmak yerine güvenli qrtl() çalışır.
        """

        if not self.is_connected():
            print("[-] Mode değiştirilemedi: araç bağlı değil.")
            return False

        mode_name = str(mode_name).upper().strip()

        if mode_name in ["QRTL", "RTL"]:
            print("[*] Mode isteği QRTL/RTL. Güvenli qrtl() fonksiyonuna yönlendiriliyor.")
            return self.qrtl()

        try:
            self.vehicle.mode = VehicleMode(mode_name)
            self.vehicle.flush()
            print(f"[+] Mode set edildi: {mode_name}")
            return True

        except Exception as e:
            print(f"[-] Mode set hatası: {e}")
            return False

    def land(self):
        if not self.is_connected():
            return False
        try:
            self.vehicle.mode=VehicleMode("QLAND")
            return True
        except Exception as e:
            print(f"land hatasi{e}")
            return False
        
    def test_pusher_throttle_output(self, pwm=1500, duration=3):
        """
        Pusher / ön motor throttle çıkış testi.

        SITL:
        - pusher SERVO3 ise SERVO3 output yükselmeli.

        Gerçek uçak:
        - pusher SERVO7 ise SERVO7 output yükselmeli.
        - GERÇEK UÇAKTA PERVANE SÖKÜLÜ OLMADAN KULLANMA.
        """

        if not self.is_connected():
            print("[-] Pusher test yapılamadı: araç bağlı değil.")
            return False

        try:
            print("[*] Pusher throttle test başlıyor.")
            print("[!] Gerçek uçakta pervane takılıysa BU TESTİ YAPMA.")

            # Fixed-wing throttle doğrudan MANUAL modda net görülür.
            self.vehicle.mode = VehicleMode("MANUAL")
            self.vehicle.flush()
            time.sleep(1.5)

            print(f"[*] Mevcut mod: {self.vehicle.mode.name}")

            if not self.vehicle.armed:
                print("[*] Araç ARM ediliyor...")
                self.vehicle.armed = True

                start = time.time()
                while not self.vehicle.armed:
                    if time.time() - start > 8:
                        print("[-] ARM zaman aşımı. Pusher test iptal.")
                        return False
                    time.sleep(0.2)

            print("[+] Araç ARM. RC3 throttle override veriliyor...")

            # RC kanal 3 = throttle input
            self.vehicle.channels.overrides['3'] = int(pwm)
            self.vehicle.flush()

            print(f"[+] RC3 throttle override: {pwm}")
            print(f"[*] {duration} saniye Mission Planner Servo Output ekranından pusher çıkışını izle.")
            time.sleep(duration)

            print("[*] Throttle minimuma çekiliyor...")
            self.vehicle.channels.overrides['3'] = 1000
            self.vehicle.flush()
            time.sleep(0.5)

            print("[*] Override temizleniyor...")
            self.vehicle.channels.overrides.clear()
            self.vehicle.flush()

            print("[+] Pusher throttle test bitti.")
            return True

        except Exception as e:
            print(f"[-] Pusher throttle test hatası: {e}")

            try:
                self.vehicle.channels.overrides['3'] = 1000
                self.vehicle.channels.overrides.clear()
                self.vehicle.flush()
            except Exception:
                pass
            return False

    def qrtl(self):
        """
        5 motor forward assist modundan QRTL'ye geçerken:
        - Ön motor Q-mode forward assist kapatılır.
        - Sonra QRTL verilir.
        Böylece dönüş/iniş 4 dikey motorla beklenir.
        """

        if not self.is_connected():
            print("[-] QRTL verilemedi: araç bağlı değil.")
            return False

        try:
            print("[*] QRTL öncesi ön motor forward assist kapatılıyor...")

            # SERVO7_FUNCTION uçuşta değiştirilmez.
            # Sadece Q-mode forward assist kapatılır.
            self.set_param_safe('Q_FWD_THR_USE', 0)
            self.set_param_safe('Q_VFWD_GAIN', 0)

            time.sleep(0.5)

            print("[*] QRTL modu veriliyor...")
            self.vehicle.mode = VehicleMode("QRTL")
            self.vehicle.flush()

            ok = self._wait_until(
                lambda: self.vehicle.mode.name == "QRTL",
                timeout=5
            )

            if not ok:
                print("[-] QRTL moda geçilemedi.")
                return False

            print("[+] Araç QRTL moda geçti. Ön motor desteği kapalı, 4 dikey motorla dönüş/iniş beklenir.")
            return True

        except Exception as e:
            print(f"[-] QRTL mode hatası: {e}")
            return False
