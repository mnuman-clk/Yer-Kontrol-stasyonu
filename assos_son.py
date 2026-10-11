import sys
import os
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
from drone_son import Drone,VTOL
from datetime import datetime
import time
from assos_gui import Ui_MainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView,QWebEngineSettings,QWebEngineProfile
import cv2
import json
import serial.tools.list_ports
import threading
import math
from pymavlink import mavutil
from survey_planner_fixedwing import (
    SurveyCoverageTracker,
    build_recovery_plan,
    build_survey_plan,
    regularize_four_point_rectangle,
    # Geçiş süresince eski fonksiyon çalışabilsin.
    # Sabit-kanat metotlarını bağladıktan sonra kullanılmayacak.
)
import shutil
import subprocess
import numpy as np
import socket

# Rota hesabında kullanılacak güvenli yatay görüş açısı.
SURVEY_HORIZONTAL_FOV_DEG = 60.0

# Jetson görev kodunun çalıştığı GERÇEK çıktı görüntüsü (Güncellendi).
SURVEY_FRAME_WIDTH_PX = 1456
SURVEY_FRAME_HEIGHT_PX = 1088

# Poligon planlayıcısının kullanacağı tam kamera yer izi.
# 20m irtifa ve 60 FOV için tam genişlik.
SURVEY_FULL_FRAME_SWATH_M = 23.0

# Komşu tarama merkez hatları arasındaki mesafe (BATARYA DOSTU AYAR).
SURVEY_LANE_SPACING_M = 10.0

# Kameranın uçuş doğrultusunda yerde gördüğü yaklaşık uzunluk.
SURVEY_FULL_FRAME_ALONG_M = 17.0

# Jetson görev kodundaki merkezi kabul alanı (800x800 piksel ROI):
SURVEY_ROI_HALF_SIZE_PX = 400
SURVEY_ROI_SIZE_PX = SURVEY_ROI_HALF_SIZE_PX * 2

# Varsayılan görev değerleri.
SURVEY_DEFAULT_ALTITUDE_M = 20.0
SURVEY_DEFAULT_SPEED_MPS = 13.0
SURVEY_TURN_SPEED_MPS = 13.0

# GPS ve rüzgâr sapmaları sonrası kabul ettiğimiz GÜVENLİ kapsama.
# Hat aralığımız 10m olduğu için bunu da 10m'ye eşitliyoruz.
SURVEY_SAFE_SWATH_M = 10.0

# Kamera izi 10 metreden küçük olursa yine en az %15 bindirme isteriz.
SURVEY_MIN_SIDE_OVERLAP_RATIO = 0.15

# Sabit-kanat rota tasarımı.
# Uçağın gerçek ROLL_LIMIT_DEG değeri 45° olsa da rotayı 40° ile
# planlayarak 5° takip/rüzgâr payı bırakıyoruz.
SURVEY_DESIGN_BANK_DEG = 40.0
SURVEY_TURN_RADIUS_SAFETY = 1.10

# Pixhawk'taki WP_RADIUS değeri.
SURVEY_WP_RADIUS_M =12.0

# Matematiksel dönüş eğrisinin Pixhawk waypoint aralığı.
SURVEY_TURN_WP_SPACING_M = 22.5

# Gerçek telemetri örneğinin taranmış sayılması için sınırlar.
SURVEY_CROSS_TRACK_LIMIT_M = 1.0
SURVEY_HEADING_LIMIT_DEG = 8.0
SURVEY_ROLL_LIMIT_DEG = 10.0
SURVEY_ALTITUDE_LIMIT_M = 3.0

# Bir metrekareden küçük geometrik artık için tekrar turu üretme.
SURVEY_MISSING_AREA_TOLERANCE_M2 = 1.0

# Sonsuz recovery döngüsünü engeller.
SURVEY_MAX_RECOVERY_ROUNDS = 2

def calculate_survey_camera_metrics(altitude_agl_m):
    """
    Seçilen irtifada 380x380 ROI'nin yerde gördüğü alanı
    ve izin verilen en büyük şerit aralığını hesaplar.

    altitude_agl_m:
        Kameranın yerden yüksekliği. MSL değil, AGL olmalıdır.
    """
    altitude_agl_m = float(altitude_agl_m)

    if altitude_agl_m <= 0.0:
        raise ValueError("Tarama irtifasi pozitif olmali.")

    if not (0.0 < SURVEY_HORIZONTAL_FOV_DEG < 180.0):
        raise ValueError("Kamera FOV degeri 0 ile 180 arasinda olmali.")

    if SURVEY_ROI_SIZE_PX > SURVEY_FRAME_WIDTH_PX:
        raise ValueError("ROI genisligi kamera görüntüsünden büyük.")

    if SURVEY_ROI_SIZE_PX > SURVEY_FRAME_HEIGHT_PX:
        raise ValueError("ROI yüksekligi kamera görüntüsünden büyük.")

    # 60 derecelik yatay FOV'dan piksel cinsinden odak uzaklığı.
    fx_px = (
        SURVEY_FRAME_WIDTH_PX / 2.0
    ) / math.tan(
        math.radians(SURVEY_HORIZONTAL_FOV_DEG) / 2.0
    )

    # Kare piksel ve yere dik kamera varsayımında fy yaklaşık fx'tir.
    fy_px = fx_px

    # Görüntüde yatay eksen, uçağın uçuş hattına dik yöndür.
    cross_track_footprint_m = (
        altitude_agl_m * SURVEY_ROI_SIZE_PX / fx_px
    )

    # Görüntüde dikey eksen, uçuş doğrultusudur.
    along_track_footprint_m = (
        altitude_agl_m * SURVEY_ROI_SIZE_PX / fy_px
    )

    # En az %15 bindirme kullanıldığında izin verilen teorik aralık.
    requested_spacing_m = (
        cross_track_footprint_m
        * (1.0 - SURVEY_MIN_SIDE_OVERLAP_RATIO)
    )

    # Teorik sonuç daha geniş olsa bile güvenli kabulümüz 5 m.
    max_lane_spacing_m = min(
        SURVEY_SAFE_SWATH_M,
        requested_spacing_m,
    )

    return {
        "fx_px": fx_px,
        "fy_px": fy_px,
        "cross_track_m": cross_track_footprint_m,
        "along_track_m": along_track_footprint_m,
        "max_lane_spacing_m": max_lane_spacing_m,

        # Gerçekte oluşan bindirme oranı.
        "actual_overlap_ratio": (
            1.0
            - max_lane_spacing_m
            / cross_track_footprint_m
        ),
    }

def calculate_required_lane_count(area_width_m, altitude_agl_m):
    """
    Alanın tarama yönüne dik genişliğini tamamen kapatmak için
    gereken minimum paralel uçuş şeridi sayısını hesaplar.
    """
    area_width_m = float(area_width_m)

    if area_width_m <= 0.0:
        raise ValueError("Alan genisligi pozitif olmali.")

    metrics = calculate_survey_camera_metrics(altitude_agl_m)

    footprint_m = min(
        metrics["cross_track_m"],
        SURVEY_SAFE_SWATH_M,
    )
    max_spacing_m = metrics["max_lane_spacing_m"]

    # Tek ROI bütün genişliği kapatıyorsa tek geçiş yeterlidir.
    if area_width_m <= footprint_m:
        return 1

    center_span_m = area_width_m - footprint_m

    return 1 + math.ceil(center_span_m / max_spacing_m)

def calculate_even_lane_spacing(
    area_width_m,
    altitude_agl_m,
    lane_count,
):
    """
    İlk ve son şeritleri sınırlardan yarım ROI içeride tutarak
    şeritleri alan genişliğine eşit dağıtır.
    """
    area_width_m = float(area_width_m)
    lane_count = int(lane_count)

    metrics = calculate_survey_camera_metrics(altitude_agl_m)
    footprint_m = min(
        metrics["cross_track_m"],
        SURVEY_SAFE_SWATH_M,
    )

    if lane_count <= 1:
        return 0.0

    if area_width_m <= footprint_m:
        return 0.0

    return (
        area_width_m - footprint_m
    ) / (lane_count - 1)

SURVEY_EARTH_RADIUS_M = 6378137.0

def survey_polygon_latlon_to_xy(latlon_points):
    """
    [(lat, lon), ...] biçimindeki poligon noktalarını,
    poligonun merkezini referans alan [(x, y), ...]
    metre koordinatlarına dönüştürür.

    x: doğu yönü, metre
    y: kuzey yönü, metre
    """
    if len(latlon_points) < 3:
        raise ValueError(
            "Metre dönüşümü için en az 3 poligon noktasi gerekli."
        )

    # Poligon köşelerinin ortalamasını yerel koordinat merkezi yapıyoruz.
    origin_lat = sum(
        point[0] for point in latlon_points
    ) / len(latlon_points)

    origin_lon = sum(
        point[1] for point in latlon_points
    ) / len(latlon_points)

    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    cos_origin_lat = math.cos(origin_lat_rad)

    if abs(cos_origin_lat) < 1e-12:
        raise ValueError(
            "Kutuplara çok yakin koordinatlarda dönüşüm yapilamaz."
        )

    xy_points = []

    for lat, lon in latlon_points:
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        # Longitude farkı doğu-batı mesafesine çevrilir.
        x_m = (
            SURVEY_EARTH_RADIUS_M
            * (lon_rad - origin_lon_rad)
            * cos_origin_lat
        )

        # Latitude farkı kuzey-güney mesafesine çevrilir.
        y_m = (
            SURVEY_EARTH_RADIUS_M
            * (lat_rad - origin_lat_rad)
        )

        xy_points.append((x_m, y_m))

    return xy_points, (origin_lat, origin_lon)

def survey_xy_to_latlon(x_m, y_m, origin_latlon):
    """
    Yerel metre koordinatını tekrar latitude-longitude
    koordinatına dönüştürür.

    Rota algoritmasının ürettiği waypoint'leri Leaflet ve
    Pixhawk'ın anlayacağı koordinatlara çevirmek için kullanılacak.
    """
    origin_lat, origin_lon = origin_latlon

    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    cos_origin_lat = math.cos(origin_lat_rad)

    if abs(cos_origin_lat) < 1e-12:
        raise ValueError(
            "Kutuplara çok yakin koordinatlarda dönüşüm yapilamaz."
        )

    lat_rad = (
        origin_lat_rad
        + float(y_m) / SURVEY_EARTH_RADIUS_M
    )

    lon_rad = (
        origin_lon_rad
        + float(x_m)
        / (SURVEY_EARTH_RADIUS_M * cos_origin_lat)
    )

    return math.degrees(lat_rad), math.degrees(lon_rad)

def survey_polygon_signed_area_m2(xy_points):
    """
    Shoelace formülüyle poligonun işaretli alanını hesaplar.

    Pozitif/negatif işaret köşelerin saat yönünde veya
    saat yönünün tersinde girildiğini gösterir.
    """
    area_twice = 0.0
    point_count = len(xy_points)

    for index in range(point_count):
        x1, y1 = xy_points[index]
        x2, y2 = xy_points[(index + 1) % point_count]

        area_twice += x1 * y2 - x2 * y1

    return area_twice / 2.0

def survey_polygon_perimeter_m(xy_points):
    """
    Poligonun bütün kenar uzunluklarını toplayarak
    metre cinsinden çevresini hesaplar.
    """
    perimeter_m = 0.0
    point_count = len(xy_points)

    for index in range(point_count):
        x1, y1 = xy_points[index]
        x2, y2 = xy_points[(index + 1) % point_count]

        perimeter_m += math.hypot(
            x2 - x1,
            y2 - y1,
        )

    return perimeter_m

def survey_cross_product(point_a, point_b, point_c):
    """
    A-B-C noktalarının dönüş yönünü hesaplar.

    Pozitif: sola dönüş
    Negatif: sağa dönüş
    Sıfıra yakın: aynı doğru üzerinde
    """
    return (
        (point_b[0] - point_a[0])
        * (point_c[1] - point_a[1])
        -
        (point_b[1] - point_a[1])
        * (point_c[0] - point_a[0])
    )

def survey_point_on_segment(point_a, point_b, point_c):
    """
    C noktasının A-B doğru parçası üzerinde olup olmadığını kontrol eder.
    """
    epsilon = 1e-9

    return (
        min(point_a[0], point_b[0]) - epsilon
        <= point_c[0]
        <= max(point_a[0], point_b[0]) + epsilon
        and
        min(point_a[1], point_b[1]) - epsilon
        <= point_c[1]
        <= max(point_a[1], point_b[1]) + epsilon
    )

def survey_segments_intersect(a1, a2, b1, b2):
    """
    İki doğru parçasının kesişip kesişmediğini kontrol eder.
    """
    epsilon = 1e-9

    cross_1 = survey_cross_product(a1, a2, b1)
    cross_2 = survey_cross_product(a1, a2, b2)
    cross_3 = survey_cross_product(b1, b2, a1)
    cross_4 = survey_cross_product(b1, b2, a2)

    if (
        ((cross_1 > epsilon and cross_2 < -epsilon)
         or (cross_1 < -epsilon and cross_2 > epsilon))
        and
        ((cross_3 > epsilon and cross_4 < -epsilon)
         or (cross_3 < -epsilon and cross_4 > epsilon))
    ):
        return True

    if (
        abs(cross_1) <= epsilon
        and survey_point_on_segment(a1, a2, b1)
    ):
        return True

    if (
        abs(cross_2) <= epsilon
        and survey_point_on_segment(a1, a2, b2)
    ):
        return True

    if (
        abs(cross_3) <= epsilon
        and survey_point_on_segment(b1, b2, a1)
    ):
        return True

    if (
        abs(cross_4) <= epsilon
        and survey_point_on_segment(b1, b2, a2)
    ):
        return True

    return False

def validate_survey_polygon_xy(xy_points):
    """
    Metreye çevrilmiş poligonun geometrik olarak kullanılabilir
    olup olmadığını kontrol eder.

    Döndürdüğü bilgiler:
    - area_m2: poligon alanı
    - perimeter_m: poligon çevresi
    """
    point_count = len(xy_points)

    if point_count < 3:
        raise ValueError(
            "Poligonda en az 3 nokta olmali."
        )

    # Çok kısa veya sıfır uzunluklu kenarları engeller.
    for index in range(point_count):
        point_a = xy_points[index]
        point_b = xy_points[(index + 1) % point_count]

        edge_length_m = math.hypot(
            point_b[0] - point_a[0],
            point_b[1] - point_a[1],
        )

        if edge_length_m < 0.05:
            raise ValueError(
                f"Poligonun {index + 1}. kenari "
                f"5 santimetreden kisa."
            )

    # Komşu olmayan bütün kenar çiftlerini kontrol eder.
    for first_index in range(point_count):
        a1 = xy_points[first_index]
        a2 = xy_points[(first_index + 1) % point_count]

        for second_index in range(
            first_index + 1,
            point_count,
        ):
            # Aynı veya komşu kenarlar doğal olarak ortak köşe taşır.
            if second_index == first_index:
                continue

            if second_index == (first_index + 1) % point_count:
                continue

            if first_index == (second_index + 1) % point_count:
                continue

            b1 = xy_points[second_index]
            b2 = xy_points[(second_index + 1) % point_count]

            if survey_segments_intersect(a1, a2, b1, b2):
                raise ValueError(
                    "Poligon kendi üzerine kesişiyor. "
                    "Köşeleri alanın çevresinde sirayla seçin."
                )

    signed_area_m2 = survey_polygon_signed_area_m2(
        xy_points
    )
    area_m2 = abs(signed_area_m2)

    if area_m2 < 0.25:
        raise ValueError(
            "Poligon alani tarama için çok küçük."
        )

    perimeter_m = survey_polygon_perimeter_m(
        xy_points
    )

    return {
        "area_m2": area_m2,
        "perimeter_m": perimeter_m,
        "clockwise": signed_area_m2 < 0.0,
    }

class DroneManager(QObject):
    log_signal=pyqtSignal(str)
    def __init__(self,vehicle_class=Drone):
        super().__init__()
        self.drone=vehicle_class()
        self.connected=False
        self._connecting = False
        self._connection_lock = threading.Lock()
        self._connection_string = None
        self._baudrate = 57600

        self._manual_disconnect = False
        self._ever_connected = False

        self._link_lost_since = None
        self._last_reconnect_attempt_s = 0.0

        self.AUTO_RECONNECT_AFTER_S = 12.0
        self.AUTO_RECONNECT_COOLDOWN_S = 5.0
    
    # connect ve disconnect kısmı;
    def connect(self, connection_string, baudrate=57600):
        self._connection_string = connection_string
        self._baudrate = int(baudrate)
        self._manual_disconnect = False
        with self._connection_lock:

            if self._connecting:
                self.log_signal.emit(
                    "Bağlantı zaten kurulmaya çalışılıyor."
                )
                return False

            if self.is_connected():
                self.log_signal.emit(
                    "Araç zaten bağlı."
                )
                return False

            self._connecting = True

        self.log_signal.emit(
            "Bağlantı deneniyor, lütfen bekleyin..."
        )
        def arka_planda_baglan():
            try:
                # Eski/zombi Vehicle nesnesi varsa önce kapat.
                if getattr(self.drone, "vehicle", None) is not None:
                    try:
                        self.drone.disconnect()
                    except Exception:
                        pass

                self.connected = False

                result = self.drone.connectDrone(
                    connection_string,
                    baudrate=baudrate
                )

                self.connected = bool(result)

                if self.connected and self.drone.is_connected():
                    self._ever_connected = True
                    self._link_lost_since = None

                    self.log_signal.emit(
                        "Drone connected successfully."
                    )

                    if getattr(
                        self.drone,
                        "is_vtol_aircraft",
                        False
                    ):
                        self.log_signal.emit(
                            "VTOL bağlantısı hazır. "
                            "Otomatik parametre profili uygulanmadı."
                        )

                else:
                    self.connected = False

                    self.log_signal.emit(
                        "Bağlantı kuruldu ancak heartbeat doğrulanamadı."
                    )

            except Exception as error:

                self.connected = False

                self.log_signal.emit(
                    f"Error connecting to drone: {error}"
                )

                print(
                    f"Error connecting to drone: {error}"
                )

            finally:
                with self._connection_lock:
                    self._connecting = False

        worker = threading.Thread(
            target=arka_planda_baglan,
            daemon=True
        )

        worker.start()

        return True
    def disconnect(self):
        # Kullanıcı bilinçli olarak bağlantıyı kesti.
        # Bundan sonra otomatik reconnect yapılmayacak.
        self._manual_disconnect = True
        self._link_lost_since = None

        try:
            if (
                getattr(
                    self.drone,
                    "vehicle",
                    None
                )
                is not None
            ):
                self.drone.disconnect()

        except Exception as e:
            self.log_signal.emit(
                f"Error disconnecting "
                f"from drone: {e}"
            )

        finally:
            self.connected = False

            self.log_signal.emit(
                "Drone disconnected successfully."
            )
    def request_auto_reconnect(self):
        """
        Heartbeat kaybolunca ilk anda bağlantıyı kapatmaz.

        Bağlantı 12 saniyeden uzun süre kayıpsa
        eski DroneKit oturumunu kapatıp aynı COM/baud
        ile yeniden bağlanmayı dener.

        Bu işlem:
        - ARM/DISARM göndermez
        - mode değiştirmez
        - mission değiştirmez
        - sadece YKI <-> MAVLink bağlantısını yeniler
        """

        # Kullanıcı Disconnect'e bastıysa dokunma.
        if self._manual_disconnect:
            return False

        # Daha önce başarılı bağlantı kurulmadıysa
        # otomatik reconnect mantığını çalıştırma.
        if not self._ever_connected:
            return False

        if not self._connection_string:
            return False

        # Heartbeat yeniden geldiyse sayaç sıfır.
        if self.drone.is_connected():
            self._link_lost_since = None
            return False

        # Zaten yeni bağlantı kuruluyorsa ikinci tane başlatma.
        if self._connecting:
            return False

        now = time.monotonic()

        if self._link_lost_since is None:
            self._link_lost_since = now
            return False

        lost_for = (
            now
            - self._link_lost_since
        )

        # Geçici 1-5-10 saniyelik kesintide COM'a dokunma.
        if (
            lost_for
            < self.AUTO_RECONNECT_AFTER_S
        ):
            return False

        # Sürekli connect spam'i yapma.
        if (
            now
            - self._last_reconnect_attempt_s
            < self.AUTO_RECONNECT_COOLDOWN_S
        ):
            return False

        self._last_reconnect_attempt_s = now

        self.log_signal.emit(
            "MAVLink heartbeat "
            f"{lost_for:.1f}s alınamadı. "
            "YKI bağlantı oturumu otomatik "
            "olarak yeniden kurulacak."
        )

        # connect() zaten background thread başlatıyor.
        self.connect(
            self._connection_string,
            baudrate=self._baudrate
        )

        return True
    def is_connected(self):
        return self.connected and self.drone.is_connected()
    
    #arm ve disarm kısmı;
    def arm(self):
        if self.is_connected():
            self.drone.arm()
            self.log_signal.emit("Drone armed.")
    def disarm(self):
        if self.is_connected():
            self.drone.dis_arm()
            self.log_signal.emit("Drone disarmed.")
    def toggle_arm(self):
        try:
            if not self.is_connected():
                self.log_signal.emit("Drone not connected.")
                return False    
            if self.is_connected():
                if self.drone.is_arm():
                    self.drone.dis_arm()
                    self.log_signal.emit("Drone forced to disarm.")
                else:
                    self.drone.arm()
                    self.log_signal.emit("Drone forced to arm.")
            return True
        except Exception as e:
            self.log_signal.emit(f"Error toggling arm state: {e}")
            return False

    #mode kısmı;
    def set_mode(self,mode_name):
        try:    
            if not self.is_connected():
                self.log_signal.emit("Drone not connected.")
                return False
            if self.is_connected():
                self.drone.set_mode(mode_name)
                self.log_signal.emit(f"Drone mode set to {mode_name}.")
        except Exception as e:
            self.log_signal.emit(f"Error setting mode: {e}")
            return False
    def get_mode(self):
        if not self.is_connected():
            self.log_signal.emit("Cannot get mode: Drone not connected.")
            return "DISCONNECTED"
        return self.drone.get_mode()

    def rtl(self):
        try:
            if not self.is_connected():
                self.log_signal.emit("Drone not connected.")
                return False
            if self.is_connected():
                self.drone.rtl()
        except Exception as e:
            self.log_signal.emit(f"Error setting RTL mode: {e}")
            return False
        
    def qrtl(self):
        try:
            if not self.is_connected():
                self.log_signal.emit("Drone not connected.")
                return False

            if hasattr(self.drone, "qrtl"):
                ok = self.drone.qrtl()

                if ok:
                    self.log_signal.emit("QRTL mode set: 4 dikey motorla dönüş/iniş başlatıldı.")
                else:
                    self.log_signal.emit("QRTL mode geçişi başarısız.")
                return ok

            self.log_signal.emit("Bu araçta QRTL fonksiyonu yok.")
            return False

        except Exception as e:
            self.log_signal.emit(f"Error setting QRTL mode: {e}")
            return False

    #takeoff kısmı;
    def takeoff(self,altitude):
        try:
            if not self.is_connected():
                self.log_signal.emit("Drone not connected.")
                return False    
            if self.is_connected():
                self.drone.take_off(altitude)
        except Exception as e:
            self.log_signal.emit(f"Error during takeoff: {e}")
            return False

    #velocity ve altitude kısmı;
    def velocity(self):
        if not self.is_connected():
            return 0.0
        return self.drone.velocity()
    
    def altitude(self):
        if not self.is_connected():
            return 0.0
        return self.drone.altitude()

class TelemetryManager(QThread):
    telemetry_signal = pyqtSignal(dict)

    def __init__(self, drone_manager):
        super().__init__()

        self.drone_manager = drone_manager
        self._is_running = False
        self.interval = 100

        self._last_error_log_s = 0.0

    def start(self, interval=100):
        self.interval = interval

        if not self._is_running:
            self._is_running = True
            super().start()

    def stop(self):
        self._is_running = False
        self.requestInterruption()

        if self.isRunning():
            self.wait(500)

    @staticmethod
    def _safe_call(function, default):
        try:
            return function()
        except Exception:
            return default

    def run(self):

        while (
            self._is_running
            and not self.isInterruptionRequested()
        ):

            drone = self.drone_manager.drone

            heartbeat_age = self._safe_call(
                drone.heartbeat_age,
                None
            )

            link_alive = self._safe_call(
                self.drone_manager.is_connected,
                False
            )

            # ------------------------------------------------------
            # MAVLINK BAĞLANTISI YOKSA DA GUI'YE PAKET YOLLA
            # Böylece ekran eski değerde donmuş gibi kalmaz.
            # ------------------------------------------------------
            if not link_alive:
                self.drone_manager.request_auto_reconnect()

                data = {
                    "altitude": None,
                    "velocity": None,
                    "mode": "TELEMETRY LOST",
                    "arm": None,

                    "lat": None,
                    "lon": None,

                    "pitch": 0.0,
                    "roll": 0.0,

                    "voltage": 0.0,
                    "level": 0,

                    "satellites": 0,
                    "fix": 0,
                    "heading": 0.0,

                    "servo_current": 0.0,

                    "ch1": 0,
                    "ch2": 0,
                    "ch3": 0,
                    "ch4": 0,
                    "ch5": 0,
                    "ch6": 0,

                    "mission_next": -1,
                    "servo_outputs": {},

                    "statustext": "",

                    "heartbeat_age": heartbeat_age,
                    "link_alive": False,
                }

                self.telemetry_signal.emit(data)

                self.msleep(self.interval)
                continue

            # ------------------------------------------------------
            # BAĞLANTI VAR
            # Her veri kaynağı AYRI ayrı korunuyor.
            # ------------------------------------------------------

            lat, lon = self._safe_call(
                drone.location,
                (None, None)
            )

            pitch, roll = self._safe_call(
                drone.get_attitude,
                (0.0, 0.0)
            )

            volt, level, current = self._safe_call(
                drone.get_battery,
                (0.0, 0, 0.0)
            )

            satellite, fix = self._safe_call(
                drone.get_gps,
                (0, 0)
            )

            heading = self._safe_call(
                drone.get_heading,
                0.0
            )

            rc_channels = self._safe_call(
                drone.get_rc_channels,
                (0, 0, 0, 0, 0, 0)
            )

            servo_outputs = self._safe_call(
                drone.get_servo_outputs,
                {}
            )

            altitude = self._safe_call(
                drone.altitude,
                0.0
            )

            velocity = self._safe_call(
                drone.velocity,
                0.0
            )

            mode = self._safe_call(
                drone.get_mode,
                "UNKNOWN"
            )

            arm = self._safe_call(
                drone.is_arm,
                False
            )

            mission_next = self._safe_call(
                drone.get_mission_next,
                -1
            )

            data = {
                "altitude": altitude,
                "velocity": velocity,
                "mode": mode,
                "arm": arm,

                "lat": lat,
                "lon": lon,

                "pitch": pitch,
                "roll": roll,

                "voltage": volt,
                "level": level,

                "satellites": satellite,
                "fix": fix,
                "heading": heading,

                "servo_current": current,

                "ch1": rc_channels[0],
                "ch2": rc_channels[1],
                "ch3": rc_channels[2],
                "ch4": rc_channels[3],
                "ch5": rc_channels[4],
                "ch6": rc_channels[5],

                "mission_next": mission_next,
                "servo_outputs": servo_outputs,

                "statustext": getattr(
                    drone,
                    "_last_statustext",
                    ""
                ),

                "heartbeat_age": heartbeat_age,
                "link_alive": True,
            }

            self.telemetry_signal.emit(data)

            self.msleep(self.interval)

class TelemetryPage(QWidget):
    def __init__(self,drone_manager):
        super().__init__()
        self.drone_manager=drone_manager

        self.main_layout=QVBoxLayout()

        self.hud_view=QWebEngineView()
        self.hud_view.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        self.hud_view.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        #html yolu
        base_path = os.path.dirname(os.path.abspath(__file__))
        hud_path = os.path.join(base_path, "suni_ufuk.html")
        self.hud_view.load(QUrl.fromLocalFile(os.path.abspath(hud_path)))
        self.hud_view.setMinimumHeight(350)
        self.main_layout.addWidget(self.hud_view)

        self.grid_layout=QGridLayout()
        self.grid_layout.setSpacing(20)

        self.label1=QLabel("Arm State")
        self.label1.setFont(QFont("Arial",14))
        self.label1.setFixedHeight(50)
        self.label1.setStyleSheet("color:red")

        self.label2=QLabel("Mode")
        self.label2.setFont(QFont("Arial",14))  
        self.label2.setFixedHeight(50)
        self.label2.setStyleSheet("color:yellow")

        self.label3=QLabel("Altitude")
        self.label3.setFont(QFont("Arial",14)) 
        self.label3.setFixedHeight(50)
        self.label3.setStyleSheet("color:green")

        self.label4=QLabel("Velocity")
        self.label4.setFont(QFont("Arial",14))
        self.label4.setFixedHeight(50)
        self.label4.setStyleSheet("color:orange")

        self.label5=QLabel("Battery:0.0V %0")
        self.label5.setFont(QFont("Arial",14))
        self.label5.setFixedHeight(50)
        self.label5.setStyleSheet("color:#9b59b6; font-size:14px; font-weight:bold;")

        self.label6=QLabel("GPS:0 Sats (No Fix)")
        self.label6.setFont(QFont("Arial",14))
        self.label6.setFixedHeight(30)
        self.label6.setStyleSheet("color:#1abc9c; font-size:14px; font-weight:bold;")

        self.label7=QLabel("RC->Bekleniyor")
        self.label7.setFont(QFont("Arial",14))
        self.label7.setFixedHeight(30)
        self.label7.setStyleSheet("color: #e67e22;")

        self.label8=QLabel("Şartel->Bekleniyor")
        self.label8.setFont(QFont("Arial",14))
        self.label8.setFixedHeight(30)
        self.label8.setStyleSheet("color: #e67e22;")

        self.grid_layout.addWidget(self.label1,0,0)
        self.grid_layout.addWidget(self.label2,0,1)
        self.grid_layout.addWidget(self.label3,1,0)
        self.grid_layout.addWidget(self.label4,1,1)
        self.grid_layout.addWidget(self.label5,2,0)
        self.grid_layout.addWidget(self.label6,2,1)
        self.grid_layout.addWidget(self.label7,3,0)
        self.grid_layout.addWidget(self.label8,3,1)

        self.main_layout.addLayout(self.grid_layout)
        self.mode_layout=QHBoxLayout()

        self.arm_button=QPushButton("Force Arm/Disarm")
        self.arm_button.setFont(QFont("Arial",12))
        self.arm_button.setFixedHeight(40)
        self.arm_button.setStyleSheet("background-color:lightgray")

        self.mode_box=QComboBox()
        self.mode_box.setFont(QFont("Arial",12))
        self.mode_box.setFixedHeight(40)
        self.mode_box.addItems(["STABILIZE","GUIDED","LOITER","AUTO","LAND", "QLOITER", "QHOVER", "FBWA", "CRUISE","QRTL","QLAND"])
        self.mode_box.setStyleSheet("background-color:lightgray")

        self.apply_btn=QPushButton("Apply")
        self.apply_btn.setFont(QFont("Arial",12))
        self.apply_btn.setFixedHeight(40)
        self.apply_btn.setStyleSheet("background-color:lightgray")

        self.mode_layout.addWidget(self.mode_box)
        self.mode_layout.addWidget(self.apply_btn)
        
        self.main_layout.addLayout(self.mode_layout)

        self.rtl_btn=QPushButton("RTL MODE")
        self.rtl_btn.setFont(QFont("Arial",12))
        self.rtl_btn.setFixedHeight(40)
        self.rtl_btn.setStyleSheet("background-color:lightgray")

        self.takeoff_btn=QPushButton("Takeoff")
        self.takeoff_btn.setFont(QFont("Arial",12))
        self.takeoff_btn.setFixedHeight(40)
        self.takeoff_btn.setStyleSheet("background-color:lightgray")

        self.vtol_quad_bttn=QPushButton("DRONE MODE")
        self.vtol_quad_bttn.setFont(QFont("Arial",12))
        self.vtol_quad_bttn.setFixedHeight(40)
        self.vtol_quad_bttn.setStyleSheet("background-color: #8e44ad; color: white;")

        self.vtol_plane_bttn=QPushButton("VTOL MODE")
        self.vtol_plane_bttn.setFont(QFont("Arial",14))
        self.vtol_plane_bttn.setFixedHeight(40)
        self.vtol_plane_bttn.setStyleSheet("background-color: #2980b9; color: white;")

        self.vtol_quad_bttn.hide()
        self.vtol_plane_bttn.hide()

        # --- KAMİKAZE DALIŞ BUTONU ---
        self.attack_btn = QPushButton("KAMİKAZE DALIŞI BAŞLAT!")
        self.attack_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.attack_btn.setFixedHeight(40)
        self.attack_btn.setStyleSheet("background-color: red; color: white;")
        self.attack_btn.hide()
        self.main_layout.addWidget(self.attack_btn)

        btn_style="background-color:#343b47; color:white; padding:8px; font-weight:bold;"
        for buton in[self.arm_button, self.takeoff_btn, self.rtl_btn, self.apply_btn, self.mode_box]:
            buton.setStyleSheet(btn_style)

        self.main_layout.addWidget(self.arm_button)
        self.main_layout.addWidget(self.rtl_btn)
        self.main_layout.addWidget(self.takeoff_btn)
        self.main_layout.addWidget(self.vtol_plane_bttn)
        self.main_layout.addWidget(self.vtol_quad_bttn)

        self.main_layout.addStretch()
        self.setLayout(self.main_layout)

        self.setup_connections()
    def setup_connections(self):
        self.arm_button.clicked.connect(lambda: self.drone_manager.toggle_arm())
        self.rtl_btn.clicked.connect(lambda: self.drone_manager.rtl())
        self.takeoff_btn.clicked.connect(self.takeoff)
        self.apply_btn.clicked.connect(lambda: self.drone_manager.set_mode(self.mode_box.currentText()))

        self.vtol_quad_bttn.clicked.connect(self.cmd_vtol_quad)
        self.vtol_plane_bttn.clicked.connect(self.cmd_vtol_plane)
        self.attack_btn.clicked.connect(self.trigger_kamikaze)
    
    def cmd_vtol_quad(self):
        if hasattr(self.drone_manager.drone, 'do_vtol_transition'):
            self.drone_manager.drone.do_vtol_transition(3) # 3 = Drone Modu
            self.drone_manager.log_signal.emit("Geçiş komutu gönderildi:DRONE MODU")
        else:
            self.drone_manager.log_signal.emit("Uyarı: Bu uçak VTOL yeteneğine sahip değil.")

    def cmd_vtol_plane(self):
        if hasattr(self.drone_manager.drone, 'do_vtol_transition'):
            self.drone_manager.drone.do_vtol_transition(4) # 4 = Uçak Modu
            self.drone_manager.log_signal.emit("Geçiş komutu gönderildi:VTOL MODU")
        else:
            self.drone_manager.log_signal.emit("Uyarı: Bu uçak VTOL yeteneğine sahip değil.")
    
    def takeoff(self):
        if not self.drone_manager.is_connected():
            QMessageBox.warning(self, "Hata", "Önce uçağa bağlanın!")
            return

        altitude, ok = QInputDialog.getDouble(
            self, "Takeoff İrtifası", "Kalkış irtifasını girin (metre):\n\n⚠️ Uçak GUIDED modunda kalkacak ve havada güvenle asılı kalacaktır.", 15, min=5.0, max=120.0, decimals=1
        )
        if ok:
            self.drone_manager.log_signal.emit(f"Kalkış başlatılıyor. Uçak GUIDED modunda havalanacak ve havada kilitlenecek.")
            self.drone_manager.takeoff(altitude)
    def trigger_kamikaze(self):
        if not self.drone_manager.is_connected():
            self.drone_manager.log_signal.emit("HATA: Drone'a bağlı değilsiniz!")
            return

        self.drone_manager.log_signal.emit("Drone görevi manuel olarak başlatılıyor...")
        self.drone_manager.drone.start_carrier_mission(
            start_seq=0,
            force_disable_arming_check=False
        )
    
    def telemetry_update(self,data):
        """Telemetri kaybında None değerleri GUI slotunu düşürmez."""
        def safe_number(value, default=0.0):
            try:
                value = float(value)
                return value if math.isfinite(value) else default
            except (TypeError, ValueError):
                return default

        link_alive = bool(data.get("link_alive", False))

        if not link_alive:
            self.label1.setText("UNKNOWN")
            self.label2.setText(str(data.get("mode", "TELEMETRY LOST")))
            self.label3.setText("--")
            self.label4.setText("--")
            self.label5.setText("Battery:-- | Telemetry Lost")
            self.label6.setText("GPS:-- | Telemetry Lost")
            self.label7.setText("RC -> --")
            self.label8.setText("Switches -> --")
            return

        volt = safe_number(data.get("voltage"), 0.0)
        lvl = int(safe_number(data.get("level"), 0))
        sats = int(safe_number(data.get("satellites"), 0))
        fix = int(safe_number(data.get("fix"), 0))
        amps = safe_number(data.get("servo_current"), 0.0)

        c1 = int(safe_number(data.get("ch1"), 0))
        c2 = int(safe_number(data.get("ch2"), 0))
        c3 = int(safe_number(data.get("ch3"), 0))
        c4 = int(safe_number(data.get("ch4"), 0))
        c5 = int(safe_number(data.get("ch5"), 0))
        c6 = int(safe_number(data.get("ch6"), 0))

        altitude = safe_number(data.get("altitude"), 0.0)
        velocity = safe_number(data.get("velocity"), 0.0)
        pitch = safe_number(data.get("pitch"), 0.0)
        roll = safe_number(data.get("roll"), 0.0)

        arm_value = data.get("arm")
        self.label1.setText(
            "UNKNOWN" if arm_value is None else str(bool(arm_value))
        )
        self.label2.setText(str(data.get("mode", "UNKNOWN")))
        self.label3.setText(f"{altitude:.1f} m")
        self.label4.setText(f"{velocity:.1f} m/s")
        self.label5.setText(
            f"Battery:{volt:.2f}V level:{lvl}% | {amps:.2f}A"
        )
        fix_str = (
            "3D Fix" if fix >= 3
            else ("2D Fix" if fix == 2 else "No Fix")
        )
        self.label6.setText(f"GPS:{sats} Sats:{fix_str}")
        self.label7.setText(
            f"RC -> R:{c1} | P:{c2} | Gaz:{c3} | Y:{c4}"
        )
        self.label8.setText(
            f"Switches -> CH5(Mod):{c5} | CH6:{c6}"
        )

        self.hud_view.page().runJavaScript(
            f"""
            if (typeof updateHUD === "function") {{
                updateHUD({pitch}, {roll}, {altitude});
            }}
            """
        )

class RadioBridgeThread(QThread):
    log_signal = pyqtSignal(str)

    # ============================================================
    # VTOL PIXHAWK PARAMETRELERİ
    # ============================================================

    # SCR_USER1/2 tam koordinat değildir.
    # Favori.lua içindeki LAT_BASE_DEG / LON_BASE_DEG değerlerinden
    # geriye kalan işaretli ondalık kısımlardır.
    VTOL_TARGET_LAT_PARAM = "SCR_USER1"
    VTOL_TARGET_LON_PARAM = "SCR_USER2"
    VTOL_HANDSHAKE_PARAM = "SCR_USER3"

    # ============================================================
    # DRONE PIXHAWK PARAMETRELERİ
    # ============================================================

    DRONE_TARGET_LAT_PARAM = "SCR_USER1"
    DRONE_TARGET_LON_PARAM = "SCR_USER2"
    DRONE_TRIGGER_PARAM = "SCR_USER3"
    DRONE_MODE_PARAM = "SCR_USER4"
    DRONE_MODE_VALUE = 0.0

    # ============================================================
    # DRONE BIRAKMA SİSTEMİ
    # ============================================================

    SERVO_DRONE_RELEASE = 12
    RELEASE_PWM = 2011
    SEPARATION_DELAY_S = 1.0

    SERVO_COMMAND_REPEAT = 3
    SERVO_COMMAND_GAP_MS = 50

    # Drone DISARM bilgisi tek okumayla kabul edilmez.
    # En az bu süre boyunca DISARM görülmelidir.
    DISARM_CONFIRM_S = 1.0

    # ============================================================
    # PARAMETRE / TELEMETRİ AYARLARI
    # ============================================================

    # Koordinatların kesir kısmı float32 olarak taşındığı için
    # eski 1e-5 toleransından daha hassastır.
    PARAM_TOLERANCE = 5e-7
    PARAM_RETRY_S = 1.0

    MAX_HEARTBEAT_AGE_S = 3.0
    # 3 saniyeden sonra veri eski kabul edilir.
    # 8 saniyede, yalnızca Lua tetiklendiyse görev tamamlandı sayılır.
    ALLOW_HEARTBEAT_TIMEOUT_ACK = True
    HEARTBEAT_TIMEOUT_ACK_S = 8.0

    # Drone bırakılmadan önce hem GUIDED hem ARM olmak zorunda.
    REQUIRE_DRONE_ARMED_BEFORE_RELEASE = True
    REQUIRE_DRONE_GUIDED_BEFORE_RELEASE = True

    def __init__(self, vtol_manager, drone_manager):
        super().__init__()

        self.vtol_manager = vtol_manager
        self.drone_manager = drone_manager

        self._is_running = False
        self._state = "WAIT_REQUEST"
        self._last_drone_ready_command_s = 0.0

        # İsimler tank_lat/tank_lon olsa da burada tutulan değerler
        # tam koordinat değil, Lua için kesir kısımlarıdır.
        self.tank_lat = None
        self.tank_lon = None

        self._program_items = []
        self._program_index = 0

        # Aynı SCR_USER adı iki farklı Pixhawk'ta bulunduğu için
        # vehicle kimliği de anahtara dahil edilir.
        self._last_param_write = {}

        self._separation_started_s = None
        self._mission_triggered_s = None
        self._drone_disarm_seen_s = None
        self._mission_started = False
        self._drone_link_loss_started_s = None
        self._completion_reason = None

        self._last_status_log_s = 0.0

    # ============================================================
    # THREAD KONTROLÜ
    # ============================================================

    def start_bridge(self):
        if self.isRunning():
            return

        self._is_running = True
        self.start()

    def stop_bridge(self):
        self._is_running = False
        self.requestInterruption()
        self.wait(3000)

    # ============================================================
    # YARDIMCI FONKSİYONLAR
    # ============================================================

    def _set_state(self, new_state, detail):
        if new_state != self._state:
            self.log_signal.emit(
                f"RADIO BRIDGE: {self._state} -> "
                f"{new_state} | {detail}"
            )

            # Yeni state'e girince ilk durum mesajı hemen basılabilsin.
            self._last_status_log_s = 0.0

        self._state = new_state

    def _rate_log(self, message, interval_s=7.0):
        now = time.monotonic()

        # Tekrarlanan durum mesajları en fazla 7 saniyede bir basılsın.
        interval_s = max(7.0, float(interval_s))

        if now - self._last_status_log_s >= interval_s:
            self._last_status_log_s = now
            self.log_signal.emit(message)

    @staticmethod
    def _read_parameter(vehicle, name):
        try:
            value = float(vehicle.parameters.get(name))

            if math.isfinite(value):
                return value

        except Exception:
            pass

        return None

    @classmethod
    def _matches(cls, current, expected):
        if current is None:
            return False

        return (
            abs(float(current) - float(expected))
            <= cls.PARAM_TOLERANCE
        )

    @staticmethod
    def _valid_fractional_coordinate(
        lat_fraction,
        lon_fraction
    ):
        if lat_fraction is None or lon_fraction is None:
            return False

        try:
            lat_fraction = float(lat_fraction)
            lon_fraction = float(lon_fraction)
        except (TypeError, ValueError):
            return False

        if not (
            math.isfinite(lat_fraction)
            and math.isfinite(lon_fraction)
        ):
            return False

        # Lua yalnızca [-1, +1) aralığındaki kesirleri bekliyor.
        if abs(lat_fraction) >= 1.0:
            return False

        if abs(lon_fraction) >= 1.0:
            return False

        return True

    def _drone_scr_parameters_ready(self, drone):
        required_params = [
            self.DRONE_TARGET_LAT_PARAM,
            self.DRONE_TARGET_LON_PARAM,
            self.DRONE_TRIGGER_PARAM,
            self.DRONE_MODE_PARAM,
        ]

        missing = []

        for name in required_params:
            value = self._read_parameter(
                drone,
                name
            )

            if value is None:
                missing.append(name)

        if missing:
            self._rate_log(
                (
                    "RADIO BRIDGE: Küçük drone SCR parametreleri "
                    "henüz hazır değil: "
                    + ", ".join(missing)
                ),
                1.0
            )
            return False

        return True

    @classmethod
    def _heartbeat_is_fresh(cls, vehicle):
        try:
            heartbeat_age = float(vehicle.last_heartbeat)

            return (
                math.isfinite(heartbeat_age)
                and heartbeat_age
                <= cls.MAX_HEARTBEAT_AGE_S
            )

        except Exception:
            return False

    def _write_confirmed(self, vehicle, name, value):
        """
        Parametreyi ham MAVLink mesajı ile yaz ve güncelleme iste.
        Sonraki döngülerde geri okuyarak doğrula.
        """

        if vehicle is None:
            self._rate_log(
                f"RADIO BRIDGE: {name} yazılamıyor; vehicle=None.",
                1.0
            )
            return False

        current = self._read_parameter(vehicle, name)

        if self._matches(current, value):
            self.log_signal.emit(
                f"RADIO BRIDGE ONAY: {name}={current:.9f}"
            )
            return True

        now = time.monotonic()
        key = (id(vehicle), name)

        last_write = self._last_param_write.get(
            key,
            0.0
        )

        if now - last_write < self.PARAM_RETRY_S:
            return False

        try:
            # 1. DroneKit onbellegini atla, Dogrudan MAVLink Set mesaji gonder
            if isinstance(name, str):
                encoded_name = name.encode("ascii")
            else:
                encoded_name = name

            # 9 = MAV_PARAM_TYPE_REAL32
            msg_set = vehicle.message_factory.param_set_encode(
                0, 0, encoded_name, float(value), 9
            )
            vehicle.send_mavlink(msg_set)

            # 2. FC'nin yeni degeri bize geri gondermesi icin durt
            # Boylece DroneKit'in onbellegi gercek Pixhawk verisiyle guncellenir
            msg_req = vehicle.message_factory.param_request_read_encode(
                0, 0, encoded_name, -1
            )
            vehicle.send_mavlink(msg_req)

            self._last_param_write[key] = now

            self.log_signal.emit(
                f"RADIO BRIDGE YAZ (MAVLink): {name}="
                f"{float(value):.9f} | "
                f"önceki={current}"
            )

        except Exception as error:
            self._rate_log(
                f"RADIO BRIDGE PARAM HATA: "
                f"{name}={value} yazılamadı: {error}",
                1.0
            )

        return False

    def _send_release_servo(self, vtol):

        for attempt in range(
            self.SERVO_COMMAND_REPEAT
        ):
            message = (
                vtol.message_factory.command_long_encode(
                    0,
                    0,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    self.SERVO_DRONE_RELEASE,
                    self.RELEASE_PWM,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )

            vtol.send_mavlink(message)
            vtol.flush()

            self.log_signal.emit(
                (
                    f"RADIO BRIDGE SERVO: "
                    f"SERVO{self.SERVO_DRONE_RELEASE}="
                    f"{self.RELEASE_PWM} gönderildi "
                    f"({attempt + 1}/"
                    f"{self.SERVO_COMMAND_REPEAT})"
                )
            )

            if (
                attempt + 1
                < self.SERVO_COMMAND_REPEAT
            ):
                self.msleep(
                    self.SERVO_COMMAND_GAP_MS
                )

    def _reset_for_new_mission(self):
        self.tank_lat = None
        self.tank_lon = None

        self._program_items = []
        self._program_index = 0

        self._last_param_write.clear()

        self._separation_started_s = None
        self._mission_triggered_s = None
        self._drone_disarm_seen_s = None
        self._mission_started = False
        self._drone_link_loss_started_s = None
        self._completion_reason = None

        self._set_state(
            "WAIT_REQUEST",
            "Yeni görev için hazır"
        )

    # ============================================================
    # RADIO BRIDGE ANA AKIŞI
    # ============================================================

    def run(self):
        self.log_signal.emit(
            "RADIO BRIDGE başladı; "
            "VTOL SCR_USER3=1 isteği bekleniyor."
        )

        while (
            self._is_running
            and not self.isInterruptionRequested()
        ):
            self.msleep(100)

            try:
                # Her iki araç da bağlı olmalıdır.
                now = time.monotonic()

                # VTOL bağlantısı her durumda gereklidir; çünkü SCR_USER3=2
                # VTOL Pixhawk'a yazılacaktır.
                if not self.vtol_manager.is_connected():
                    continue

                vtol = getattr(
                    self.vtol_manager.drone,
                    "vehicle",
                    None
                )

                if vtol is None:
                    continue

                if not self._heartbeat_is_fresh(vtol):
                    self._rate_log(
                        "RADIO BRIDGE: VTOL heartbeat eski/yok; "
                        "SCR_USER3=2 yazılamıyor."
                    )
                    continue

                # ============================================================
                # VTOL isteğini küçük drone bağlantısından BAĞIMSIZ yakala
                # ============================================================
                if self._state == "WAIT_REQUEST":

                    request = self._read_parameter(
                        vtol,
                        self.VTOL_HANDSHAKE_PARAM
                    )

                    if self._matches(request, 2.0):
                        self._set_state(
                            "DONE",
                            "VTOL SCR_USER3 zaten 2"
                        )
                        continue

                    if self._matches(request, 1.0):

                        self.tank_lat = self._read_parameter(
                            vtol,
                            self.VTOL_TARGET_LAT_PARAM
                        )

                        self.tank_lon = self._read_parameter(
                            vtol,
                            self.VTOL_TARGET_LON_PARAM
                        )

                        if not self._valid_fractional_coordinate(
                            self.tank_lat,
                            self.tank_lon
                        ):
                            self._rate_log(
                                "RADIO BRIDGE HATA: VTOL SCR3=1 fakat "
                                f"SCR1={self.tank_lat}, SCR2={self.tank_lon} geçersiz."
                            )
                            continue

                        self._set_state(
                            "WAIT_DRONE_LINK",
                            (
                                f"VTOL isteği ALINDI. "
                                f"SCR1={self.tank_lat:.9f}, "
                                f"SCR2={self.tank_lon:.9f}. "
                                "Küçük drone bağlantısı bekleniyor."
                            )
                        )

                drone = getattr(
                    self.drone_manager.drone,
                    "vehicle",
                    None
                )

                try:
                    drone_manager_connected = bool(
                        self.drone_manager.is_connected()
                    )
                except Exception:
                    drone_manager_connected = False

                try:
                    drone_heartbeat_age = float(drone.last_heartbeat)
                except (TypeError, ValueError, AttributeError):
                    drone_heartbeat_age = None

                drone_heartbeat_fresh = (
                    drone_manager_connected
                    and drone_heartbeat_age is not None
                    and math.isfinite(drone_heartbeat_age)
                    and 0.0 <= drone_heartbeat_age
                    <= self.MAX_HEARTBEAT_AGE_S
                )

                if drone_heartbeat_fresh:
                    # Bağlantı geri geldiyse kayıp sayacını sıfırla.
                    self._drone_link_loss_started_s = None

                else:
                    # Son heartbeat zamanı biliniyorsa kayıp başlangıcını
                    # last_heartbeat değerinden geriye doğru hesapla.
                    if self._drone_link_loss_started_s is None:
                        if (
                            drone_heartbeat_age is not None
                            and math.isfinite(drone_heartbeat_age)
                            and drone_heartbeat_age >= 0.0
                        ):
                            self._drone_link_loss_started_s = (
                                now - drone_heartbeat_age
                            )
                        else:
                            self._drone_link_loss_started_s = now

                    link_loss_elapsed = (
                        now - self._drone_link_loss_started_s
                    )

                    mission_elapsed = 0.0

                    if self._mission_triggered_s is not None:
                        mission_elapsed = (
                            now - self._mission_triggered_s
                        )

                    heartbeat_timeout_completion = (
                        self.ALLOW_HEARTBEAT_TIMEOUT_ACK
                        and self._state == "WAIT_DRONE_DISARM"
                        and self._mission_started
                        and mission_elapsed
                        >= self.HEARTBEAT_TIMEOUT_ACK_S
                        and link_loss_elapsed
                        >= self.HEARTBEAT_TIMEOUT_ACK_S
                    )

                    if heartbeat_timeout_completion:
                        self._completion_reason = "HEARTBEAT_TIMEOUT"

                        self._set_state(
                            "ACK_VTOL",
                            (
                                f"Lua tetiklendikten sonra drone heartbeat "
                                f"{link_loss_elapsed:.1f}s alınamadı; "
                                "8 saniyelik failsafe gereği VTOL onayı verilecek"
                            )
                        )

                        # Burada continue YOK.
                        # Aynı döngüde ACK_VTOL bölümüne geçilecek.

                    elif (
                        self._state == "ACK_VTOL"
                        and self._completion_reason
                        in ("DISARM", "HEARTBEAT_TIMEOUT")
                    ):
                        # Tamamlanma kararı daha önce verildi.
                        # Drone bağlantısı olmasa da VTOL'a 2 yazılabilir.
                        pass

                    elif self._state == "DONE":
                        # Görev tamamlandı; drone bağlantısı geri gelene kadar
                        # yeni görev reset kontrolü yapılmaz.
                        continue

                    else:
                        self._rate_log(
                            (
                                "RADIO BRIDGE: Drone heartbeat eski/yok; "
                                f"kayıp süresi={link_loss_elapsed:.1f}s. "
                                "Lua tetiklenmeden heartbeat timeout "
                                "VTOL onayı oluşturamaz."
                            ),
                            1.0
                        )
                        continue

                # =================================================
                # 1. KÜÇÜK DRONE BAĞLANTISINI / SCR3 DURUMUNU BEKLE
                # =================================================

                if self._state == "WAIT_DRONE_LINK":

                    # Buraya geldiğimizde:
                    # VTOL SCR_USER3=1 ALINDI.
                    # VTOL SCR_USER1/2 zaten self.tank_lat/lon içine alındı.

                    if not drone_heartbeat_fresh:
                        self._rate_log(
                            "RADIO BRIDGE: VTOL isteği alındı fakat "
                            "küçük drone heartbeat bekleniyor.",
                            1.0
                        )
                        continue

                    if not self._drone_scr_parameters_ready(
                        drone
                    ):
                        continue    

                    drone_trigger = self._read_parameter(
                        drone,
                        self.DRONE_TRIGGER_PARAM
                    )

                    # DroneKit parametre cache'i henüz SCR_USER3'ü
                    # yüklememiş olabilir.
                    if drone_trigger is None:
                        self._rate_log(
                            "RADIO BRIDGE: Drone SCR_USER3 henüz "
                            "okunamadı. Parametre verisi bekleniyor.",
                            1.0
                        )
                        continue

                    self._rate_log(
                        (
                            f"RADIO BRIDGE: Drone bağlantısı hazır | "
                            f"heartbeat={drone_heartbeat_age:.2f}s | "
                            f"SCR_USER3={drone_trigger}"
                        ),
                        2.0
                    )

                    # -------------------------------------------------
                    # Lua zaten önceden tetiklenmişse yeniden tetikleme
                    # -------------------------------------------------
                    if self._matches(drone_trigger, 1.0):

                        drone_lat = self._read_parameter(
                            drone,
                            self.DRONE_TARGET_LAT_PARAM
                        )

                        drone_lon = self._read_parameter(
                            drone,
                            self.DRONE_TARGET_LON_PARAM
                        )

                        coordinates_match = (
                            self._matches(
                                drone_lat,
                                self.tank_lat
                            )
                            and
                            self._matches(
                                drone_lon,
                                self.tank_lon
                            )
                        )

                        if coordinates_match:
                            self._mission_started = True
                            self._mission_triggered_s = (
                                time.monotonic()
                            )
                            self._drone_disarm_seen_s = None

                            self._set_state(
                                "WAIT_DRONE_DISARM",
                                (
                                    "Drone SCR_USER3 zaten 1 ve "
                                    "koordinatlar doğru. "
                                    "Servo/tetik tekrarlanmayacak; "
                                    "DISARM bekleniyor."
                                )
                            )

                        else:
                            self._rate_log(
                                (
                                    "RADIO BRIDGE HATA: Drone SCR_USER3=1 "
                                    "fakat drone SCR_USER1/2 değerleri "
                                    "yeni VTOL hedefiyle uyuşmuyor."
                                ),
                                1.0
                            )

                        continue

                    # -------------------------------------------------
                    # Normal yeni görevde SCR_USER3 kesinlikle 0 olmalı
                    # -------------------------------------------------
                    if not self._matches(
                        drone_trigger,
                        0.0
                    ):
                        self._rate_log(
                            (
                                "RADIO BRIDGE HATA: "
                                f"Drone SCR_USER3={drone_trigger}. "
                                "Yeni görev için 0 olması gerekiyor."
                            ),
                            1.0
                        )
                        continue

                    # -------------------------------------------------
                    # Küçük drone'a gönderilecek parametreleri hazırla
                    # -------------------------------------------------
                    self._program_items = [
                        (
                            self.DRONE_TARGET_LAT_PARAM,
                            self.tank_lat
                        ),
                        (
                            self.DRONE_TARGET_LON_PARAM,
                            self.tank_lon
                        ),
                        (
                            self.DRONE_MODE_PARAM,
                            self.DRONE_MODE_VALUE
                        ),
                    ]

                    self._program_index = 0

                    self._set_state(
                        "PROGRAM_DRONE",
                        (
                            "Drone SCR programlama başlıyor | "
                            f"SCR_USER1={self.tank_lat:.9f} | "
                            f"SCR_USER2={self.tank_lon:.9f} | "
                            f"SCR_USER4={self.DRONE_MODE_VALUE:.1f}"
                        )
                    )

                elif self._state == "PROGRAM_DRONE":
                    if self._program_index >= len(
                        self._program_items
                    ):
                        self._set_state(
                            "WAIT_DRONE_READY",
                            "Drone parametreleri doğrulandı"
                        )
                        continue

                    name, value = self._program_items[
                        self._program_index
                    ]

                    if self._write_confirmed(
                        drone,
                        name,
                        value
                    ):
                        self._program_index += 1

                # =================================================
                # 3. DRONE GUIDED + ARM KONTROLÜ
                # =================================================

                elif self._state == "WAIT_DRONE_READY":
                    drone_mode = str(
                        getattr(
                            getattr(drone, "mode", None),
                            "name",
                            ""
                        )
                    ).upper()

                    if (
                        self.REQUIRE_DRONE_GUIDED_BEFORE_RELEASE
                        and drone_mode != "GUIDED"
                    ):
                        self._rate_log(
                            "RADIO BRIDGE: Drone GUIDED modda "
                            f"değil; mevcut mod="
                            f"{drone_mode or 'BILINMIYOR'}. "
                            "Bırakma bekletiliyor."
                        )
                        continue

                    if (
                        self.REQUIRE_DRONE_ARMED_BEFORE_RELEASE
                        and not bool(drone.armed)
                    ):
                        self._rate_log(
                            "RADIO BRIDGE: Drone ARM değil; "
                            "bırakma güvenlik için bekletiliyor."
                        )
                        continue

                    self._set_state(
                        "RELEASE_DRONE",
                        "Drone GUIDED ve ARM; bırakmaya hazır"
                    )

                # =================================================
                # 4. VTOL SERVO 15'i AÇ
                # =================================================

                elif self._state == "RELEASE_DRONE":
                    drone_mode = str(
                        getattr(getattr(drone, "mode", None), "name", "")
                    ).upper()

                    if (
                        self.REQUIRE_DRONE_GUIDED_BEFORE_RELEASE
                        and drone_mode != "GUIDED"
                    ):
                        self._set_state(
                            "WAIT_DRONE_READY",
                            "Servo öncesi GUIDED durumu kaybedildi"
                        )
                        continue

                    if (
                        self.REQUIRE_DRONE_ARMED_BEFORE_RELEASE
                        and not bool(getattr(drone, "armed", False))
                    ):
                        self._set_state(
                            "WAIT_DRONE_READY",
                            "Servo öncesi drone DISARM oldu"
                        )
                        continue

                    self._send_release_servo(vtol)
                    self._separation_started_s = time.monotonic()

                    self._set_state(
                        "WAIT_SEPARATION",
                        (
                            f"SERVO{self.SERVO_DRONE_RELEASE}="
                            f"{self.RELEASE_PWM}; ayrılma süresi bekleniyor"
                        )
                    )

                # =================================================
                # 5. FİZİKSEL AYRILMAYI BEKLE
                # =================================================

                elif self._state == "WAIT_SEPARATION":
                    if self._separation_started_s is None:
                        self._set_state(
                            "RELEASE_DRONE",
                            "Servo komutu yeniden gönderilecek"
                        )
                        continue

                    elapsed = (
                        time.monotonic()
                        - self._separation_started_s
                    )

                    if elapsed >= self.SEPARATION_DELAY_S:
                        self._set_state(
                            "TRIGGER_LUA",
                            "Fiziksel ayrılma süresi tamamlandı"
                        )

                # =================================================
                # 6. DRONE LUA TETİĞİNİ YAZ
                # =================================================

                elif self._state == "TRIGGER_LUA":
                    drone_mode = str(
                        getattr(getattr(drone, "mode", None), "name", "")
                    ).upper()

                    if (
                        self.REQUIRE_DRONE_GUIDED_BEFORE_RELEASE
                        and drone_mode != "GUIDED"
                    ):
                        self._rate_log(
                            "RADIO BRIDGE HATA: Ayrılma sonrasında drone "
                            f"GUIDED değil; mevcut mod={drone_mode or 'BILINMIYOR'}. "
                            "Lua tetiklenmiyor ve VTOL'e 2 gönderilmiyor."
                        )
                        continue

                    if (
                        self.REQUIRE_DRONE_ARMED_BEFORE_RELEASE
                        and not bool(getattr(drone, "armed", False))
                    ):
                        self._rate_log(
                            "RADIO BRIDGE HATA: Ayrılma sonrasında drone "
                            "DISARM. Lua tetiklenmiyor ve VTOL'e 2 gönderilmiyor."
                        )
                        continue

                    if self._write_confirmed(
                        drone,
                        self.DRONE_TRIGGER_PARAM,
                        1.0
                    ):
                        self._mission_started = True
                        self._mission_triggered_s = time.monotonic()
                        self._drone_disarm_seen_s = None
                        self._drone_link_loss_started_s = None
                        self._completion_reason = None

                        self._set_state(
                            "WAIT_DRONE_DISARM",
                            (
                                "Drone SCR_USER3=1 doğrulandı; "
                                "drone görevi ve DISARM bekleniyor"
                            )
                        )

                # =================================================
                # 7. DRONE GÖREVİ VE DISARM BEKLE
                # =================================================

                elif self._state == "WAIT_DRONE_DISARM":
                    if not self._mission_started:
                        self._rate_log(
                            "RADIO BRIDGE HATA: Lua görevinin "
                            "başlatıldığı doğrulanmadan DISARM "
                            "kontrolüne geçildi."
                        )
                        continue

                    drone_armed = getattr(
                        drone,
                        "armed",
                        None
                    )

                    if drone_armed is None:
                        self._rate_log(
                            "RADIO BRIDGE: Drone ARM durumu "
                            "okunamıyor; VTOL'e 2 gönderilmiyor."
                        )
                        continue

                    # Drone hâlâ uçuyor/görevde.
                    if bool(drone_armed):
                        self._drone_disarm_seen_s = None

                        elapsed_text = ""

                        if self._mission_triggered_s is not None:
                            mission_elapsed = (
                                time.monotonic()
                                - self._mission_triggered_s
                            )
                            elapsed_text = (
                                f" | görev süresi="
                                f"{mission_elapsed:.1f}s"
                            )

                        self._rate_log(
                            "RADIO BRIDGE: Drone hâlâ ARM; "
                            "görev/iniş devam ediyor. "
                            "VTOL SCR_USER3=1 olarak kalacak"
                            f"{elapsed_text}."
                        )
                        continue

                    # İlk DISARM gözlemi.
                    if self._drone_disarm_seen_s is None:
                        self._drone_disarm_seen_s = (
                            time.monotonic()
                        )

                        self.log_signal.emit(
                            "RADIO BRIDGE: Drone DISARM görüldü; "
                            f"{self.DISARM_CONFIRM_S:.1f}s "
                            "doğrulama bekleniyor."
                        )
                        continue

                    disarm_elapsed = (
                        time.monotonic()
                        - self._drone_disarm_seen_s
                    )

                    if disarm_elapsed < self.DISARM_CONFIRM_S:
                        continue

                    # Kritik nokta:
                    # VTOL SCR_USER3=2 yalnızca drone gerçekten
                    # DISARM olduktan sonra gönderilecektir.
                    self._completion_reason = "DISARM"
                    self._set_state(
                        "ACK_VTOL",
                        (
                            "Drone DISARM doğrulandı; "
                            "artık VTOL'e görev tamamlandı "
                            "onayı gönderilebilir"
                        )
                    )

                # =================================================
                # 8. VTOL SCR_USER3=2 ONAYINI YAZ
                # =================================================

                elif self._state == "ACK_VTOL":
                    if self._completion_reason == "DISARM":
                        completion_text = "Drone DISARM doğrulaması"

                    elif self._completion_reason == "HEARTBEAT_TIMEOUT":
                        completion_text = (
                            "Lua sonrası 8 saniyelik drone heartbeat kaybı"
                        )

                    else:
                        self._rate_log(
                            "RADIO BRIDGE HATA: Tamamlanma nedeni yok; "
                            "VTOL SCR_USER3=2 yazılmıyor."
                        )
                        continue

                    if self._write_confirmed(
                        vtol,
                        self.VTOL_HANDSHAKE_PARAM,
                        2.0
                    ):
                        self._set_state(
                            "DONE",
                            (
                                f"{completion_text} nedeniyle "
                                "VTOL SCR_USER3=2 doğrulandı; "
                                "VTOL artık hastaneye gidebilir"
                            )
                        )

                # =================================================
                # 9. YENİ GÖREV RESETİNİ BEKLE
                # =================================================

                elif self._state == "DONE":
                    vtol_state = self._read_parameter(
                        vtol,
                        self.VTOL_HANDSHAKE_PARAM
                    )

                    drone_trigger = self._read_parameter(
                        drone,
                        self.DRONE_TRIGGER_PARAM
                    )

                    if (
                        self._matches(vtol_state, 0.0)
                        and self._matches(
                            drone_trigger,
                            0.0
                        )
                    ):
                        self._reset_for_new_mission()

            except Exception as error:
                self._rate_log(
                    f"RADIO BRIDGE DÖNGÜ HATASI: {error}",
                    1.0
                )   

class MessagePage(QWidget):
    def __init__(self):
        super().__init__()

        self.main_layout=QVBoxLayout()
        
        self.message_box=QTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.document().setMaximumBlockCount(1000)
        
        self.main_layout.addWidget(self.message_box)
        self.setLayout(self.main_layout)
    def append_text(self,text):
        now=datetime.now().strftime("%H:%M:%S")
        self.message_box.append(f"[{now}] - {text}")

class ConnectPage(QWidget):
    def __init__(self,drone_manager):
        super().__init__()
        self.drone_manager=drone_manager
        self.setup_ui()
        self.setup_connections()
        self.scan_ports() #portlar bununla taranıyor
    def setup_ui(self):
        self.main_layout = QVBoxLayout()
        
        title = QLabel("BAĞLANTI YÖNETİCİSİ")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(title)

        self.form_layout = QGridLayout()
        
        self.form_layout.addWidget(QLabel("Bağlantı Portu:"), 0, 0)
        self.port_combo = QComboBox()
        self.form_layout.addWidget(self.port_combo, 0, 1)

        self.refresh_btn = QPushButton("Yenile")
        self.form_layout.addWidget(self.refresh_btn, 0, 2)

        self.form_layout.addWidget(QLabel("Baud Rate:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200"]) # Telemetri: 57600, USB: 115200
        self.form_layout.addWidget(self.baud_combo, 1, 1)

        self.main_layout.addLayout(self.form_layout)

        self.btn_layout = QHBoxLayout()
        self.connect_button = QPushButton("Bağlan")
        self.connect_button.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; padding: 10px;")
        
        self.disconnect_button = QPushButton("Bağlantıyı Kes")
        self.disconnect_button.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; padding: 10px;")
        
        self.btn_layout.addWidget(self.connect_button)
        self.btn_layout.addWidget(self.disconnect_button)
        self.main_layout.addLayout(self.btn_layout)

        # Durum Yazısı
        self.status_label = QLabel("🔴 Disconnected")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #FF0000; padding: 20px;")
        self.main_layout.addWidget(self.status_label)
        
        self.main_layout.addStretch()
        self.setLayout(self.main_layout)
    def setup_connections(self):
        self.refresh_btn.clicked.connect(self.scan_ports)
        self.connect_button.clicked.connect(self.handle_connect)
        self.disconnect_button.clicked.connect(self.drone_manager.disconnect)
    
    def scan_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("SITL Simülasyon (127.0.0.1:5762)")
        # Bilgisayara takılı USB/COM portlarını bul
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")

    def handle_connect(self):
        selection = self.port_combo.currentText()
        baudrate = int(self.baud_combo.currentText())

        # Eğer Simülasyon seçildiyse:
        if "SITL" in selection:
            connection_string = "tcp:127.0.0.1:5762"
            print(f"[*] Simülasyona bağlanılıyor: {connection_string}")
        # Eğer gerçek bir COM portu seçildiyse:
        else:
            # "COM3 - USB Serial Port" yazısından sadece "COM3" kısmını alıyoruz
            connection_string = selection.split(" ")[0]
            print(f"[*] Gerçek Donanıma bağlanılıyor: {connection_string} (Baud: {baudrate})")

        # Drone.py içindeki bağlantı fonksiyonuna hem adresi hem de hızı gönder
        self.drone_manager.connect(connection_string, baudrate=baudrate)

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)
    status_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    # MikroTik/UDP hattındaki küçük paket sıralama sorunlarını düzeltir.
    RTP_JITTER_LATENCY_MS = 80

    FRAME_STALL_TIMEOUT_S = 2.5
    FIRST_FRAME_TIMEOUT_S = 5.0
    RECONNECT_DELAY_MS = 500

    # Eksik/bozuk JPEG geldiğinde belleğin sınırsız büyümesini engeller.
    MAX_JPEG_BUFFER_BYTES = 8 * 1024 * 1024

    def __init__(
        self,
        port=5000,
        width=1280,
        height=720,
        parent=None
    ):
        super().__init__(parent)

        self.port = port
        self.width = width
        self.height = height

        self._run_flag = False
        self.process = None
        self.local_socket = None
        self.local_tcp_port = None

    @staticmethod
    def _find_free_local_tcp_port():
        """
        GStreamer ile Python arasında kullanılacak
        boş bir yerel TCP portu bulur.
        """
        with socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        ) as probe_socket:

            probe_socket.bind(("127.0.0.1", 0))

            return probe_socket.getsockname()[1]

    def _build_gstreamer_command(self, gst_launch):
        """
        UDP 5000 üzerinden gelen RTP/JPEG yayınını alır.

        Terminal komutundaki autovideosink yerine JPEG karelerini
        Python tarafına gönderir.
        """

        rtp_caps = (
            "application/x-rtp,"
            "media=(string)video,"
            "clock-rate=(int)90000,"
            "encoding-name=(string)JPEG,"
            "payload=(int)26"
        )

        command = [
            gst_launch,
            "-q",

            # gst-launch komutundaki:
            # udpsrc port=5000 buffer-size=524288
            "udpsrc",
            f"port={self.port}",
            "buffer-size=524288",
            f"caps={rtp_caps}",

            "!",

            # Ağdan karışık sırada gelebilen RTP paketlerini düzenler.
            "rtpjitterbuffer",
            f"latency={self.RTP_JITTER_LATENCY_MS}",
            "drop-on-latency=true",

            "!",

            # RTP paketlerinin içinden JPEG karesini çıkarır.
            "rtpjpegdepay",

            "!",

            # Gecikme oluşursa eski kareleri biriktirme.
            "queue",
            "leaky=downstream",
            "max-size-buffers=2",
            "max-size-bytes=0",
            "max-size-time=0",

            "!",

            # JPEG karelerini sadece bilgisayar içerisinde
            # Python soketine gönderir.
            "tcpserversink",
            "host=127.0.0.1",
            f"port={self.local_tcp_port}",
            "sync=false"
        ]

        return command

    def _close_resources(self):
        """
        Yerel soketi ve gst-launch işlemini güvenli şekilde kapatır.
        """

        local_socket = self.local_socket
        self.local_socket = None

        if local_socket is not None:
            try:
                local_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            try:
                local_socket.close()
            except OSError:
                pass

        process = self.process
        self.process = None

        if process is not None:
            try:
                if process.poll() is None:
                    process.terminate()

                    try:
                        process.wait(timeout=1.0)

                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)

            except OSError:
                pass

    def _connect_local_socket(self):
        """
        gst-launch tarafından açılan yerel TCP kanalına bağlanır.
        """

        deadline = time.monotonic() + 5.0

        while (
            self._run_flag
            and time.monotonic() < deadline
        ):
            if (
                self.process is None
                or self.process.poll() is not None
            ):
                return False

            try:
                self.local_socket = socket.create_connection(
                    ("127.0.0.1", self.local_tcp_port),
                    timeout=0.5
                )

                self.local_socket.settimeout(0.5)

                return True

            except OSError:
                self.msleep(100)

        return False

    def run(self):

        self._run_flag = True

        gst_launch = (
            shutil.which("gst-launch-1.0")
            or shutil.which("gst-launch-1.0.exe")
        )

        if gst_launch is None:
            self.error_signal.emit(
                "gst-launch-1.0 bulunamadı."
            )
            self._run_flag = False
            return

        while self._run_flag:

            cycle_started_s = time.monotonic()
            last_frame_s = None

            try:
                self.local_tcp_port = (
                    self._find_free_local_tcp_port()
                )

                command = (
                    self._build_gstreamer_command(
                        gst_launch
                    )
                )

                creation_flags = (
                    subprocess.CREATE_NO_WINDOW
                    if os.name == "nt"
                    else 0
                )

                self.process = subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    creationflags=creation_flags
                )

                if not self._connect_local_socket():

                    if self._run_flag:
                        self.status_signal.emit(
                            "Video alıcısı yeniden "
                            "bağlanıyor..."
                        )

                    continue

                self.status_signal.emit(
                    f"UDP {self.port} yayını bekleniyor..."
                )

                jpeg_buffer = bytearray()
                stream_announced = False

                while self._run_flag:

                    try:
                        chunk = self.local_socket.recv(
                            65536
                        )

                    except socket.timeout:

                        now = time.monotonic()

                        # Daha hiç frame gelmediyse
                        if last_frame_s is None:

                            if (
                                now - cycle_started_s
                                >= self.FIRST_FRAME_TIMEOUT_S
                            ):
                                self.status_signal.emit(
                                    "Video başlamadı; "
                                    "alıcı yeniden başlatılıyor..."
                                )
                                break

                            continue

                        # Daha önce frame geldi ama sonra kesildi
                        if (
                            now - last_frame_s
                            >= self.FRAME_STALL_TIMEOUT_S
                        ):
                            self.status_signal.emit(
                                "Video akışı kesildi; "
                                "otomatik yeniden bağlanılıyor..."
                            )
                            break

                        continue

                    except OSError:
                        break

                    if not chunk:
                        break

                    jpeg_buffer.extend(chunk)

                    while self._run_flag:

                        start_index = (
                            jpeg_buffer.find(
                                b"\xff\xd8"
                            )
                        )

                        if start_index < 0:

                            if (
                                len(jpeg_buffer)
                                > self.MAX_JPEG_BUFFER_BYTES
                            ):
                                jpeg_buffer.clear()

                            break

                        end_index = jpeg_buffer.find(
                            b"\xff\xd9",
                            start_index + 2
                        )

                        if end_index < 0:

                            if start_index > 0:
                                del jpeg_buffer[
                                    :start_index
                                ]

                            if (
                                len(jpeg_buffer)
                                > self.MAX_JPEG_BUFFER_BYTES
                            ):
                                jpeg_buffer.clear()

                            break

                        jpeg_data = bytes(
                            jpeg_buffer[
                                start_index:
                                end_index + 2
                            ]
                        )

                        del jpeg_buffer[
                            :end_index + 2
                        ]

                        frame = cv2.imdecode(
                            np.frombuffer(
                                jpeg_data,
                                dtype=np.uint8
                            ),
                            cv2.IMREAD_COLOR
                        )

                        if frame is None:
                            continue

                        # Başarılı gerçek frame
                        last_frame_s = (
                            time.monotonic()
                        )

                        rgb_frame = cv2.cvtColor(
                            frame,
                            cv2.COLOR_BGR2RGB
                        )

                        h, w, channels = (
                            rgb_frame.shape
                        )

                        qt_image = QImage(
                            rgb_frame.data,
                            w,
                            h,
                            w * channels,
                            QImage.Format_RGB888
                        ).copy()

                        self.change_pixmap_signal.emit(
                            qt_image
                        )

                        if not stream_announced:
                            self.status_signal.emit("Yayında")
                            stream_announced = True

            except Exception as error:

                if self._run_flag:
                    self.status_signal.emit(
                        f"Video hata: {error} | "
                        "yeniden bağlanılıyor..."
                    )

            finally:
                self._close_resources()

            if self._run_flag:
                self.msleep(
                    self.RECONNECT_DELAY_MS
                )

        self._run_flag = False
        self._close_resources()

    def stop(self):
        self._run_flag = False
        self.requestInterruption()

        local_socket = self.local_socket

        if local_socket is not None:
            try:
                local_socket.shutdown(
                    socket.SHUT_RDWR
                )
            except OSError:
                pass

            try:
                local_socket.close()
            except OSError:
                pass

class CameraPage(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Kamera Görüntüsü")
        self.resize(800, 600)

        self.main_layout = QVBoxLayout(self)

        self.video_label = QLabel("Camera stream")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet(
            "background-color: black; color: white;"
        )
        self.video_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )

        self.start_btn = QPushButton("Start Camera")
        self.start_btn.setFont(QFont("Arial", 12))
        self.start_btn.setStyleSheet(
            "background-color:#27ae60;"
            "color:white;"
            "font-weight:bold;"
        )

        self.stop_btn = QPushButton("Stop Camera")
        self.stop_btn.setFont(QFont("Arial", 12))
        self.stop_btn.setStyleSheet(
            "background-color:#c0392b;"
            "color:white;"
            "font-weight:bold;"
        )
        self.stop_btn.setEnabled(False)

        self.main_layout.addWidget(self.video_label)
        self.main_layout.addWidget(self.start_btn)
        self.main_layout.addWidget(self.stop_btn)

        self.main_layout.setStretch(0, 1)
        self.main_layout.setStretch(1, 0)
        self.main_layout.setStretch(2, 0)

        # Nesne hazırdır ama Start Camera basılana kadar çalışmaz.
        self.thread = VideoThread(
            port=5000,
            width=1280,
            height=720,
            parent=self
        )

        self.thread.change_pixmap_signal.connect(
            self.update_image
        )
        self.thread.status_signal.connect(
            self.update_camera_status
        )
        self.thread.error_signal.connect(
            self.show_error
        )

        self.start_btn.clicked.connect(
            self.start_camera
        )
        self.stop_btn.clicked.connect(
            self.stop_camera
        )

    def start_camera(self):
        if self.thread.isRunning():
            return

        self.video_label.clear()
        self.video_label.setText(
            "UDP 5000 yayını bekleniyor..."
        )

        self.start_btn.setEnabled(False)
        self.start_btn.setText("Bağlanıyor...")
        self.stop_btn.setEnabled(True)

        self.thread.start()

    def stop_camera(self):
        self.thread.stop()

        self.video_label.clear()
        self.video_label.setText("Camera stream")

        self.start_btn.setText("Start Camera")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    @pyqtSlot(str)
    def update_camera_status(self, message):
        if message == "Yayında":
            self.start_btn.setText("Yayında")
        else:
            self.start_btn.setText("Bağlanıyor...")
            self.video_label.setText(message)

    @pyqtSlot(QImage)
    def update_image(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)

        scaled_pixmap = pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatio,
            Qt.FastTransformation
        )

        self.video_label.setPixmap(scaled_pixmap)

    @pyqtSlot(str)
    def show_error(self, error_message):

        self.video_label.setText(
            f"Video hatası:\n{error_message}"
        )

        self.start_btn.setText(
            "Yeniden bağlanıyor..."
        )

        QMessageBox.warning(
            self,
            "Kamera Hatası",
            error_message
        )

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()
  
class HomePage(QWidget):
    def __init__(self):
        super().__init__()

        self.main_layout = QVBoxLayout()
        self.map_view = QWebEngineView()
        self.map_ready = False

        base_path = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.abspath(os.path.join(base_path, "map.html"))

        self.map_view.loadFinished.connect(self.on_map_loaded)
        self.map_view.load(QUrl.fromLocalFile(map_path))

        self.main_layout.addWidget(self.map_view)
        self.setLayout(self.main_layout)

    def on_map_loaded(self, success):
        self.map_ready = bool(success)
        if not success:
            print("[HARİTA] map.html yüklenemedi.")

    def set_connected(self, lat, lon, heading=0):
        if not self.map_ready:
            return

        try:
            lat = float(lat)
            lon = float(lon)
            heading = float(heading or 0)
        except (TypeError, ValueError):
            return

        js = f"""
        (function() {{
            if (typeof setConnected !== "function") {{
                console.error("setConnected fonksiyonu map.html içinde bulunamadı.");
                return;
            }}

            setConnected({lat}, {lon}, {heading});
        }})();
        """

        self.map_view.page().runJavaScript(js)

    def set_disconnect(self):
        if not self.map_ready:
            return

        self.map_view.page().runJavaScript("""
        if (typeof setDisconnected === "function") {
            setDisconnected();
        }
        """)

    def update_drone_position(self, lat, lon):
        return

class MissionPage(QWidget):
    log_signal=pyqtSignal(str)
    mission_uploaded_signal=pyqtSignal(list)
    survey_recovery_upload_result = pyqtSignal(bool, str)
    mission_upload_state = pyqtSignal(bool)
    def __init__(self,home_page,drone_manager):
        super().__init__()
        
        self.home_page=home_page
        self.drone_manager=drone_manager
        self.mission_data=[]
        self.survey_polygon_latlon = []
        # Poligonun yerel metre koordinatları.
        self.survey_polygon_xy = []

        # Metre koordinatlarının (0, 0) kabul ettiği GPS merkezi.
        self.survey_origin_latlon = None

        # Poligonun alan/çevre bilgileri.
        self.survey_polygon_geometry = None
        # İlk oluşturulan sabit-kanat planı.
        self.survey_plan = None

        # Planlayıcının oluşturduğu, değiştirilmemesi gereken waypointler.
        self.survey_plan_mission = []

        # Grid öncesindeki TAKEOFF, TRANSITION vb.
        self.survey_pre_mission = []

        # Kullanıcının tarama rotasının sonuna eklediği QRTL, transition,
        # servo vb. görev komutları.
        self.survey_post_mission = []

        # Gerçek uçuş kapsama takipçisi.
        self.survey_tracker = None
        self.survey_tracking_enabled = False

        # Haritayı her telemetri paketinde çizmemek için zaman/log kontrolü.
        self.survey_last_coverage_draw_s = 0.0
        self.survey_last_log_percent = -1

        # Recovery durumları.
        self.survey_recovery_round = 0
        self.survey_recovery_pending = False
        self.survey_recovery_uploading = False
        self.survey_recovery_plan = None
        self.survey_recovery_mission = []

        # Şu anda uçulan ana/recovery rotası.
        self.survey_active_route_plan = None
        self.survey_active_route_name = None

        # Eksik drone.py metodu için aynı uyarıyı sürekli basmayı engeller.
        self.survey_missing_uploader_warned = False
        self.servo_alert_seq = None
        self.servo_alert_sent = False
        self._mission_upload_active = False
        
        self.main_layout=QVBoxLayout()
        self.map_view=QWebEngineView()
        self.map_ready = False
        survey_test = calculate_survey_camera_metrics(
            SURVEY_DEFAULT_ALTITUDE_M
        )
        survey_test_lane_count = calculate_required_lane_count(
            30.0,
            SURVEY_DEFAULT_ALTITUDE_M,
        )
        survey_test_spacing = calculate_even_lane_spacing(
            30.0,
            SURVEY_DEFAULT_ALTITUDE_M,
            survey_test_lane_count,
        )
        print(
            "[SURVEY TEST] "
        f"ROI={SURVEY_ROI_SIZE_PX}x{SURVEY_ROI_SIZE_PX}px | "
        f"irtifa={SURVEY_DEFAULT_ALTITUDE_M:.1f}m | "
        f"yer kapsama={survey_test['cross_track_m']:.3f}m | "
        f"maks aralik={survey_test['max_lane_spacing_m']:.3f}m | "
        f"30m serit={survey_test_lane_count} | "
        f"esit aralik={survey_test_spacing:.3f}m"
        )

        #html yolu
        base_path=os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.abspath(
            os.path.join(base_path, "mission_map.html")
        )
        print("[MISSION MAP PATH]", map_path)

        self.map_view.loadFinished.connect(self.on_map_loaded)
        self.map_view.load(QUrl.fromLocalFile(map_path))
        self.map_view.setMinimumHeight(400)

        self.main_layout.addWidget(self.map_view)

        self.table=QTableWidget()
        self.table.setStyleSheet("""
            QTableWidget{
                background-color:#2c313c;
                color:#ffffff;
                gridline-color:#555555;
            }
            QHeaderView::section{
                background-color:#1f232a;
                color:#ffffff;
                padding:4px;
                border: 1px solid #444;
            }
        """)
        self.table.setColumnCount(6)

        self.table.setHorizontalHeaderLabels(
            [
                "#",
                "Latitude",
                "Longitude",
                "Altitude",
                "Mode",
                "Önceki WP (m)",
            ]
        )

        self.table.setColumnWidth(4, 150)
        self.table.setColumnWidth(5, 115)

        self.main_layout.addWidget(self.table)

        self.bottom_layout=QHBoxLayout() #alttaki butonlar için layout ekleme yeri

        self.clear_button = QPushButton("Clear Mission")
        self.send_button = QPushButton("Write Pist")

        self.save_mission_button = QPushButton("Save Mission")
        self.load_mission_button = QPushButton("Load Mission")

        self.save_mission_button.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        self.load_mission_button.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        
        # POLİGON BUTONLARI EKLENDİ
        self.polygon_toggle_btn = QPushButton("Poligon Çiz: KAPALI")
        self.polygon_toggle_btn.setCheckable(True)
        self.survey_btn = QPushButton("Alanı Tara (Grid)")
        self.polygon_toggle_btn.setStyleSheet("background-color: #34495e; color: white;")
        
        self.polygon_clear_btn = QPushButton("Poligon Temizle")
        self.polygon_clear_btn.setStyleSheet("background-color: #e74c3c; color: white;")

        self.bottom_layout.addWidget(self.polygon_toggle_btn)
        self.bottom_layout.addWidget(self.survey_btn)
        self.bottom_layout.addWidget(self.polygon_clear_btn)

        self.bottom_layout.addWidget(self.load_mission_button)
        self.bottom_layout.addWidget(self.save_mission_button)

        self.bottom_layout.addWidget(self.clear_button)
        self.bottom_layout.addWidget(self.send_button)
    
        self.main_layout.addLayout(self.bottom_layout)
        self.setLayout(self.main_layout)

        self.clear_button.clicked.connect(self.clear_mission)
        self.send_button.clicked.connect(self.send_mission)
        self.save_mission_button.clicked.connect(self.save_mission_to_file)
        self.load_mission_button.clicked.connect(self.load_mission_from_file)
        self.polygon_toggle_btn.toggled.connect(self.toggle_polygon_mode)
        self.survey_btn.clicked.connect(self.request_fixed_wing_survey_preview)
        self.polygon_clear_btn.clicked.connect(self.clear_polygon)
        self.table.itemChanged.connect(self.table_item_changed)
        self.mission_upload_state.connect(self._set_mission_upload_state)

    def _set_mission_upload_state(self, active):
        """Write sırasında ikinci gönderimi ve buton tekrarını engeller."""
        self._mission_upload_active = bool(active)
        self.send_button.setEnabled(not self._mission_upload_active)
        self.send_button.setText(
            "Write Sürüyor..."
            if self._mission_upload_active
            else "Write Pist"
        )

    def on_map_loaded(self, success):
        self.map_ready = bool(success)
        print(f"[MISSION MAP] loaded={success}")

        if success:
            self.start_sync_timer()

    def start_sync_timer(self):
        if hasattr(self, "timer") and self.timer.isActive():
            return
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.request_mission_data)
        self.timer.start(300)

    def get_mission_mode_items(self):
        return [
            "WAYPOINT",
            "TAKEOFF",
            "VTOL_TAKEOFF",
            "VTOL_LAND",
            "LAND",
            "RTL",
            "TRANSITION_DRONE",
            "TRANSITION_PLANE",
            "ALT_SERVO_AC",
            "UST_SERVO_AC",
            "ROLE_KES",
            "BEKLE",
            "QRTL",
            "HOLD",
            "UZUN_BEKLE"
        ]

    def table_item_changed(self, item):
        row = item.row()
        column = item.column()
        if column==3:
            try:
                alt=float(item.text())
                self.map_view.page().runJavaScript(
                    f"updateAltitude({row}, {alt});"
                )
            except:
                return
        if column not in (1,2):
            return
        try:
            lat = float(self.table.item(row, 1).text())
            lon = float(self.table.item(row, 2).text())
        except:
            return
        js_code = f"update_location({row}, {lat}, {lon});"
        self.map_view.page().runJavaScript(js_code)
        self.refresh_waypoint_distances()

    def request_mission_data(self):
        self.map_view.page().runJavaScript("getMissionData();",self.update_table)

    def update_table(self,data):
        if not data:
            return
        self.table.blockSignals(True)
        if len(data)!=self.table.rowCount():
            self.table.setRowCount(0)
            for index,wp in enumerate(data):
                self.table.insertRow(index)

                #waypoint numarası
                self.table.setItem(index,0,QTableWidgetItem(str(index+1)))
                #latitude
                self.table.setItem(index,1,QTableWidgetItem(str(wp["lat"])))
                #longtitude(
                self.table.setItem(index,2,QTableWidgetItem(str(wp["lon"])))
                #altitude
                self.table.setItem(
                    index,
                    3,
                    QTableWidgetItem(
                    str(float(wp.get("alt", SURVEY_DEFAULT_ALTITUDE_M)))
                    ),
                )

                #mod
                combo=QComboBox()
                combo.setStyleSheet("background-color:white;color:black;")
                combo.addItems(self.get_mission_mode_items())
                combo.setCurrentText(wp["mode"])
                combo.activated[str].connect(lambda text, r=index: self.mode_changed(r, text))
                self.table.setCellWidget(index,4,combo)
                self.table.setItem(
                    index,
                    5,
                    QTableWidgetItem("-"),
                )
        else:
            for index,wp in enumerate(data):
                self.table.item(index,1).setText(f"{wp['lat']:.6f}")
                self.table.item(index,2).setText(f"{wp['lon']:.6f}")
        self.refresh_waypoint_distances()
        self.table.blockSignals(False)
        self.mission_data=data
   
        self.servo_alert_seq = None
        #self.servo_alert_sent = False 

        for i, wp in enumerate(self.mission_data):
            mode = str(wp.get("mode", "")).strip().upper()
            if mode == "UST_SERVO_AC":
                # HOME mission listesine eklenmez; DroneKit mission seq 0-based.
                self.servo_alert_seq = i
                self.log_signal.emit(
                    f"ÜST SERVO takip aktif: Mission item {self.servo_alert_seq} okununca mesaj verilecek."
                )
                break
    @staticmethod
    def waypoint_distance_m(
        lat1,
        lon1,
        lat2,
        lon2
    ):
        """
        İki GPS koordinatı arasındaki yatay mesafeyi Haversine
        formülüyle metre cinsinden hesaplar.
        """
        lat1_rad = math.radians(float(lat1))
        lat2_rad = math.radians(float(lat2))

        d_lat = lat2_rad - lat1_rad
        d_lon = math.radians(
            float(lon2) - float(lon1)
        )

        value = (
            math.sin(d_lat / 2.0) ** 2
            + math.cos(lat1_rad)
            * math.cos(lat2_rad)
            * math.sin(d_lon / 2.0) ** 2
        )

        return (
            2.0
            * SURVEY_EARTH_RADIUS_M
            * math.atan2(
                math.sqrt(value),
                math.sqrt(max(0.0, 1.0 - value)),
            )
        )

    def refresh_waypoint_distances(self):
        """
        Her waypointin bir önceki waypointten uzaklığını tablonun
        son sütununa yazar.
        """
        for row in range(self.table.rowCount()):
            distance_item = self.table.item(row, 5)

            if distance_item is None:
                distance_item = QTableWidgetItem("-")

                # Kullanıcı bu hesaplanan hücreyi düzenleyemesin.
                distance_item.setFlags(
                    distance_item.flags()
                    & ~Qt.ItemIsEditable
                )

                self.table.setItem(
                    row,
                    5,
                    distance_item,
                )

            if row == 0:
                distance_item.setText("-")
                continue

            try:
                lat1 = float(
                    self.table.item(row - 1, 1).text()
                )
                lon1 = float(
                    self.table.item(row - 1, 2).text()
                )
                lat2 = float(
                    self.table.item(row, 1).text()
                )
                lon2 = float(
                    self.table.item(row, 2).text()
                )

                distance = self.waypoint_distance_m(
                    lat1,
                    lon1,
                    lat2,
                    lon2,
                )

                distance_item.setText(
                    f"{distance:.1f}"
                )

            except (
                AttributeError,
                TypeError,
                ValueError,
            ):
                distance_item.setText("-")
    def mode_changed(self,r,mode):
        if mode in  ["TAKEOFF","VTOL_TAKEOFF"] and self.drone_manager.is_connected():
            lat, lon = self.drone_manager.drone.location()
            if lat and lon:
                self.table.item(r, 1).setText(f"{lat:.6f}")
                self.table.item(r, 2).setText(f"{lon:.6f}")
                self.map_view.page().runJavaScript(f"update_location({r}, {lat}, {lon});")
                self.log_signal.emit(f"Waypoint {r+1} ucağın konumuna hizalandı.")
        self.map_view.page().runJavaScript(f"if (markers[{r}]) {{ markers[{r}].mode='{mode}'; }}")
        self.log_signal.emit(f"Waypoint {r+1} modu {mode} olarak değiştirildi.")
    
    def keyPressEvent(self,event):
        if event.key()==Qt.Key_Delete:
            selected_row=self.table.currentRow()
            if selected_row>=0:
                self.map_view.page().runJavaScript(f"""
                    if (markers[{selected_row}]) {{
                        map.removeLayer(markers[{selected_row}]);
                        markers.splice({selected_row},1);
                        updatePolyline();
                    }}
                """)

    def clear_mission(self):
        self.mission_data.clear()
        self.table.setRowCount(0)
        self.map_view.page().runJavaScript("clearMission();")

    def get_default_mission_folder(self):
        """
        Görev dosyalarının varsayılan kaydedileceği klasör.
        Uygulama dosyasının yanında saved_missions klasörü açar.
        """
        base_path = os.path.dirname(os.path.abspath(__file__))
        folder = os.path.join(base_path, "saved_missions")
        os.makedirs(folder, exist_ok=True)
        return folder

    def build_mission_list_from_table(self, map_data=None):
        """
        Mevcut görev bilgisini tablo + harita verisinden toplar.

        Öncelik:
        - lat/lon: tablodan alınır, eksikse haritadan alınır
        - alt: tablodan alınır
        - mode: tablodaki combobox'tan alınır
        """
        mission = []
        map_data = map_data or []

        row_count = self.table.rowCount()

        for row in range(row_count):
            try:
                map_wp = map_data[row] if row < len(map_data) else {}

                lat_item = self.table.item(row, 1)
                lon_item = self.table.item(row, 2)
                alt_item = self.table.item(row, 3)
                combo = self.table.cellWidget(row, 4)

                lat = float(lat_item.text()) if lat_item and lat_item.text() else float(map_wp.get("lat", 0.0))
                lon = float(lon_item.text()) if lon_item and lon_item.text() else float(map_wp.get("lon", 0.0))
                alt = float(alt_item.text()) if alt_item and alt_item.text() else float(map_wp.get("alt", 10.0))
                mode = combo.currentText() if combo else str(map_wp.get("mode", "WAYPOINT"))

                wp = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "mode": mode
                }

                # İleride BEKLE süresini ayrı tutarsan kaybolmasın diye opsiyonel koruma
                if "wait" in map_wp:
                    wp["wait"] = map_wp["wait"]

                mission.append(wp)
                for key in (
                    "survey_kind",
                    "survey_pass",
                    "acceptance_radius_m",
                    "is_survey_waypoint",
                ):
                    if key in map_wp:
                        wp[key] = map_wp[key]

            except Exception as e:
                self.log_signal.emit(f"Mission satırı okunamadı. Satır {row + 1}: {e}")

        # Write Pist'te yalnızca tabloda bulunan görev satırları otoritedir.
        # Haritadaki eski/stale JS noktaları boş tabloya geri eklenmez.
        return mission

    def save_mission_to_file(self):
        """
        Mevcut görevi .ykimission dosyasına kaydeder.
        """
        default_name = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ykimission"
        default_path = os.path.join(self.get_default_mission_folder(), default_name)

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Görevi Kaydet",
            default_path,
            "YKI Mission (*.ykimission);;JSON File (*.json)"
        )

        if not file_path:
            return

        if not file_path.endswith(".ykimission") and not file_path.endswith(".json"):
            file_path += ".ykimission"

        self.map_view.page().runJavaScript(
            "getMissionData();",
            lambda data, path=file_path: self.finish_save_mission_to_file(data, path)
        )

    def finish_save_mission_to_file(self, map_data, file_path):
        try:
            mission = self.build_mission_list_from_table(map_data)

            if not mission:
                QMessageBox.warning(self, "Görev Kaydetme", "Kaydedilecek görev bulunamadı.")
                return

            payload = {
                "format": "ASSOS_YKI_MISSION",
                "version": 1,
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "mission": mission
            }

            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=4)

            self.log_signal.emit(f"Görev dosyaya kaydedildi: {file_path}")
            QMessageBox.information(self, "Görev Kaydedildi", f"Görev başarıyla kaydedildi:\n{file_path}")

        except Exception as e:
            self.log_signal.emit(f"Görev kaydetme hatası: {e}")
            QMessageBox.critical(self, "Görev Kaydetme Hatası", str(e))

    def load_mission_from_file(self):
        """
        .ykimission / .json dosyasından görevi okuyup tabloya ve haritaya basar.
        """
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Görev Dosyası Aç",
            self.get_default_mission_folder(),
            "YKI Mission (*.ykimission);;JSON File (*.json);;All Files (*)"
        )

        if not file_path:
            return

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                payload = json.load(f)

            # Eski format desteği:
            # Eğer dosya direkt listeyse onu da kabul et.
            if isinstance(payload, list):
                mission = payload
            else:
                mission = payload.get("mission", [])

            mission = self.normalize_loaded_mission(mission)

            if not mission:
                QMessageBox.warning(self, "Görev Yükleme", "Dosyada geçerli görev bulunamadı.")
                return

            self.apply_loaded_mission(mission)

            self.log_signal.emit(f"Görev dosyadan yüklendi: {file_path}")
            QMessageBox.information(self, "Görev Yüklendi", f"Görev başarıyla yüklendi:\n{file_path}")

        except Exception as e:
            self.log_signal.emit(f"Görev yükleme hatası: {e}")
            QMessageBox.critical(self, "Görev Yükleme Hatası", str(e))

    def normalize_loaded_mission(self, mission):
        """
        Dosyadan gelen görevi temizler.
        Her satırda lat/lon/alt/mode olmasını garanti eder.
        """
        clean = []
        allowed_modes = self.get_mission_mode_items()

        for i, wp in enumerate(mission):
            try:
                lat = float(wp.get("lat"))
                lon = float(wp.get("lon"))
                alt = float(wp.get("alt", 10.0))

                mode = str(wp.get("mode", "WAYPOINT")).strip().upper()
                if mode not in allowed_modes:
                    mode = "WAYPOINT"

                item = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "mode": mode
                }

                if "wait" in wp:
                    item["wait"] = wp["wait"]

                clean.append(item)
                for key in (
                    "survey_kind",
                    "survey_pass",
                    "acceptance_radius_m",
                    "is_survey_waypoint",
                ):
                    if key in wp:
                        item[key] = wp[key]

            except Exception as e:
                self.log_signal.emit(f"Dosyadaki {i + 1}. görev satırı atlandı: {e}")

        return clean

    def populate_table_from_mission(self, mission):
        """
        Load edilen görevi tabloya eksiksiz basar.
        DİKKAT: Mevcut update_table() altitude'u 10 yapıyor.
        Bu yüzden load için ayrı fonksiyon kullanıyoruz.
        """
        self.table.blockSignals(True)
        self.table.setRowCount(0)

        for index, wp in enumerate(mission):
            self.table.insertRow(index)

            self.table.setItem(index, 0, QTableWidgetItem(str(index + 1)))
            self.table.setItem(index, 1, QTableWidgetItem(f"{float(wp['lat']):.7f}"))
            self.table.setItem(index, 2, QTableWidgetItem(f"{float(wp['lon']):.7f}"))
            self.table.setItem(index, 3, QTableWidgetItem(str(float(wp.get("alt", 10.0)))))

            combo = QComboBox()
            combo.setStyleSheet("background-color:white;color:black;")
            combo.addItems(self.get_mission_mode_items())
            combo.setCurrentText(str(wp.get("mode", "WAYPOINT")).upper())
            combo.activated[str].connect(lambda text, r=index: self.mode_changed(r, text))
            self.table.setCellWidget(index, 4, combo)
            distance_item = QTableWidgetItem("-")
            distance_item.setFlags(
                distance_item.flags()
                & ~Qt.ItemIsEditable
            )
            self.table.setItem(
                index,
                5,
                distance_item,
            )
        self.refresh_waypoint_distances()
        self.table.blockSignals(False)

    def refresh_servo_alert_seq_from_mission(self):
        """
        Yüklenen görevde UST_SERVO_AC varsa servo takip seq bilgisini yeniden kurar.
        """
        self.servo_alert_seq = None
        self.servo_alert_sent = False

        for i, wp in enumerate(self.mission_data):
            mode = str(wp.get("mode", "")).strip().upper()
            if mode == "UST_SERVO_AC":
                self.servo_alert_seq = i
                self.log_signal.emit(
                    f"ÜST SERVO takip aktif: Mission item {self.servo_alert_seq} okununca mesaj verilecek."
                )
                break

    def apply_loaded_mission(self, mission):
        """
        Dosyadan okunan görevi tabloya, mission_data'ya, mission haritasına ve home haritasına basar.
        """
        self.mission_data = mission

        # Tabloyu doldur
        self.populate_table_from_mission(mission)

        # Servo takip bilgisi varsa tekrar hazırla
        self.refresh_servo_alert_seq_from_mission()

        js_data = json.dumps(mission)

        # Mission haritasına markerları tekrar çiz
        self.map_view.page().runJavaScript(f"loadMissionFromPython({js_data});")

        # Home haritasında da rota çizilsin
        self.home_page.map_view.page().runJavaScript(f"drawMission({js_data});")

        self.log_signal.emit(f"{len(mission)} görev satırı tabloya ve haritaya yüklendi.")

    def request_survey_polygon(self):
        """
        Tarama butonuna basıldığında mission_map.html içindeki
        getPolygonPoints() fonksiyonunu çalıştırır.

        JavaScript sonucu hazır olduğunda
        on_survey_polygon_received() otomatik çağrılır.
        """
        if not self.map_ready:
            message = "Mission haritasi henüz yüklenmedi."
            self.log_signal.emit(message)
            QMessageBox.warning(
                self,
                "Harita Hazır Değil",
                message,
            )
            return

        self.map_view.page().runJavaScript(
            "getPolygonPoints();",
            self.on_survey_polygon_received,
        )

    def validate_survey_polygon_points(self, raw_points):
        """
        JavaScript'ten gelen poligon verisini kontrol eder ve
        [(lat, lon), ...] biçimine dönüştürür.

        Kontroller:
        - Veri bir liste mi?
        - En az üç geçerli köşe var mı?
        - Latitude ve longitude sayısal mı?
        - Koordinatlar dünya sınırları içinde mi?
        - Art arda aynı noktalar eklenmiş mi?
        """
        if not isinstance(raw_points, list):
            raise ValueError(
                "Haritadan geçerli bir poligon listesi alınamadı."
            )

        normalized_points = []

        for index, point in enumerate(raw_points):
            if not isinstance(point, dict):
                raise ValueError(
                    f"Poligonun {index + 1}. noktasi geçersiz."
                )

            lat_value = point.get("lat")

            # mission_map.html boylam için 'lng' kullanıyor.
            # İleride 'lon' gelirse onu da kabul ediyoruz.
            if "lng" in point:
                lon_value = point.get("lng")
            else:
                lon_value = point.get("lon")

            try:
                lat = float(lat_value)
                lon = float(lon_value)
            except (TypeError, ValueError):
                raise ValueError(
                    f"Poligonun {index + 1}. koordinati sayisal değil."
                )

            if not math.isfinite(lat) or not math.isfinite(lon):
                raise ValueError(
                    f"Poligonun {index + 1}. koordinati sonlu değil."
                )

            if not (-90.0 <= lat <= 90.0):
                raise ValueError(
                    f"{index + 1}. noktanin latitude değeri geçersiz."
                )

            if not (-180.0 <= lon <= 180.0):
                raise ValueError(
                    f"{index + 1}. noktanin longitude değeri geçersiz."
                )

            # Kullanıcı aynı yere art arda iki defa tıklarsa
            # ikinci noktayı poligona eklemiyoruz.
            if normalized_points:
                previous_lat, previous_lon = normalized_points[-1]

                same_as_previous = (
                    abs(lat - previous_lat) < 1e-10
                    and abs(lon - previous_lon) < 1e-10
                )

                if same_as_previous:
                    continue

            normalized_points.append((lat, lon))

        # Bazı poligon sistemleri şekli kapatmak için son noktaya
        # ilk noktayı tekrar ekler. Biz kapatmayı matematikte yapacağız.
        if len(normalized_points) >= 2:
            first_lat, first_lon = normalized_points[0]
            last_lat, last_lon = normalized_points[-1]

            last_equals_first = (
                abs(first_lat - last_lat) < 1e-10
                and abs(first_lon - last_lon) < 1e-10
            )

            if last_equals_first:
                normalized_points.pop()

        if len(normalized_points) < 3:
            raise ValueError(
                "Tarama poligonu için en az 3 farklı nokta gerekli."
            )

        return normalized_points

    def on_survey_polygon_received(self, raw_points):
        """
        Haritadan gelen poligonu:

        1. Latitude-longitude olarak doğrular.
        2. Metre koordinatlarına dönüştürür.
        3. Alan, çevre ve kesişme kontrolü yapar.
        4. Sonraki rota aşaması için saklar.
        """
        try:
            rough_polygon_latlon = (
                self.validate_survey_polygon_points(
                    raw_points
                )
            )

            # GPS koordinatlarını metre düzlemine çevir.
            rough_polygon_xy, origin_latlon = (
                survey_polygon_latlon_to_xy(
                    rough_polygon_latlon
                )
            )

            # Yaklaşık koyulan 4 köşeyi düzgün bir dikdörtgene çevir.
            polygon_xy = regularize_four_point_rectangle(
                rough_polygon_xy
            )

            # Düzeltilen dikdörtgeni tekrar GPS koordinatlarına çevir.
            polygon_latlon = [
                survey_xy_to_latlon(
                    x,
                    y,
                    origin_latlon,
                )
                for x, y in polygon_xy
            ]

            geometry = validate_survey_polygon_xy(
                polygon_xy
            )

        except ValueError as error:
            self.survey_polygon_latlon = []
            self.survey_polygon_xy = []
            self.survey_origin_latlon = None
            self.survey_polygon_geometry = None

            message = f"Poligon hatasi: {error}"
            self.log_signal.emit(message)

            QMessageBox.warning(
                self,
                "Geçersiz Poligon",
                str(error),
            )
            return

        self.survey_polygon_latlon = polygon_latlon
        self.survey_polygon_xy = polygon_xy
        self.survey_origin_latlon = origin_latlon
        self.survey_polygon_geometry = geometry

        self.log_signal.emit(
            f"Poligon hazir: "
            f"{len(polygon_xy)} köşe, "
            f"{geometry['area_m2']:.1f} m² alan, "
            f"{geometry['perimeter_m']:.1f} m çevre."
        )

        print(
            "[SURVEY GEOMETRY] "
            f"köşe={len(polygon_xy)} | "
            f"alan={geometry['area_m2']:.2f}m² | "
            f"çevre={geometry['perimeter_m']:.2f}m | "
            f"yön={'saat yönü' if geometry['clockwise'] else 'saat yönü tersi'}"
        )

        print(
            "[SURVEY ORIGIN] "
            f"lat={origin_latlon[0]:.8f} "
            f"lon={origin_latlon[1]:.8f}"
        )

        for index, (x_m, y_m) in enumerate(
            polygon_xy,
            start=1,
        ):
            print(
                f"[SURVEY XY] P{index}: "
                f"x={x_m:.2f}m y={y_m:.2f}m"
            )

    def request_fixed_wing_survey_preview(self):
        """
        Haritadaki kırmızı poligonu alarak sabit-kanat planı oluşturur.
        """
        if not self.map_ready:
            QMessageBox.warning(
                self,
                "Sabit Kanat Tarama",
                "Mission haritası henüz yüklenmedi.",
            )
            return

        self.map_view.page().runJavaScript(
            "getPolygonPoints();",
            self.create_fixed_wing_survey_preview,
        )

    def get_survey_home_latlon(self):
        """
        Öncelik sırası:
        1. Pixhawk HOME
        2. Aracın mevcut konumu
        3. Bağlantı yoksa None
        """
        if not self.drone_manager.is_connected():
            return None

        try:
            drone = self.drone_manager.drone
            vehicle = getattr(
                drone,
                "vehicle",
                None,
            )

            home = getattr(
                vehicle,
                "home_location",
                None,
            )

            if home is not None:
                lat = float(home.lat)
                lon = float(home.lon)

                valid = (
                    math.isfinite(lat)
                    and math.isfinite(lon)
                    and (
                        abs(lat) > 1e-8
                        or abs(lon) > 1e-8
                    )
                )

                if valid:
                    return lat, lon

        except Exception:
            pass

        try:
            lat, lon = (
                self.drone_manager.drone.location()
            )

            if lat is not None and lon is not None:
                return float(lat), float(lon)

        except Exception:
            pass

        return None

    @staticmethod
    def latlon_to_survey_xy(
        lat,
        lon,
        origin_latlon
    ):
        """
        GPS koordinatını poligonun yerel metre koordinatına çevirir.

        x = doğu
        y = kuzey
        """
        origin_lat, origin_lon = origin_latlon

        x_m = (
            SURVEY_EARTH_RADIUS_M
            * math.radians(
                float(lon) - origin_lon
            )
            * math.cos(
                math.radians(origin_lat)
            )
        )

        y_m = (
            SURVEY_EARTH_RADIUS_M
            * math.radians(
                float(lat) - origin_lat
            )
        )

        return x_m, y_m

    def survey_xy_path_to_latlon(self, points):
        """
        [(x,y), ...] yolunu [[lat,lon], ...] biçimine çevirir.
        """
        return [
            list(
                survey_xy_to_latlon(
                    x,
                    y,
                    self.survey_origin_latlon,
                )
            )
            for x, y in points
        ]

    def survey_xy_rings_to_latlon(self, rings):
        """
        Shapely poligon halkalarını Leaflet koordinatlarına çevirir.
        """
        return [
            self.survey_xy_path_to_latlon(ring)
            for ring in rings
        ]

    def create_fixed_wing_survey_preview(
        self,
        polygon_points
    ):
        """
        Kırmızı poligonu doğrular, HOME'a en yakın başlangıcı seçer,
        sabit-kanat dönüşlerini üretir ve waypointleri tabloya basar.
        """
        try:
            rough_polygon_latlon = (
                self.validate_survey_polygon_points(
                    polygon_points
                )
            )
            # Kaba seçilen köşeleri metre düzlemine çevir.
            rough_polygon_xy, origin_latlon = (
                survey_polygon_latlon_to_xy(
                    rough_polygon_latlon
                )
            )
            # Dört yaklaşık köşeyi düzgün dikdörtgene çevir.
            polygon_xy = regularize_four_point_rectangle(
                rough_polygon_xy
            )
            # Düzeltilmiş dikdörtgeni tekrar GPS koordinatına çevir.
            polygon_latlon = [
                survey_xy_to_latlon(
                    x,
                    y,
                    origin_latlon,
                )
                for x, y in polygon_xy
            ]
            geometry = validate_survey_polygon_xy(
                polygon_xy
            )
            self.survey_polygon_latlon = (
                polygon_latlon
            )
            self.survey_polygon_xy = polygon_xy
            self.survey_origin_latlon = (
                origin_latlon
            )
            self.survey_polygon_geometry = (
                geometry
            )
            home_latlon = (
                self.get_survey_home_latlon()
            )
    
            home_xy = None
    
            if home_latlon is not None:
                home_xy = self.latlon_to_survey_xy(
                    home_latlon[0],
                    home_latlon[1],
                    origin_latlon,
                )
    
            plan = build_survey_plan(
                polygon_xy=polygon_xy,
                altitude_m=(
                    SURVEY_DEFAULT_ALTITUDE_M
                ),
                
                horizontal_fov_deg=(
                    SURVEY_HORIZONTAL_FOV_DEG
                ),
                frame_width_px=(
                    SURVEY_FRAME_WIDTH_PX
                ),
                roi_size_px=(
                    SURVEY_ROI_SIZE_PX
                ),
                full_frame_swath_m=(
                    SURVEY_FULL_FRAME_SWATH_M
                ),
                along_track_swath_m=(
                    SURVEY_FULL_FRAME_ALONG_M
                ),
                lane_spacing_m=(
                    SURVEY_LANE_SPACING_M
                ),
                overlap_ratio=(
                    SURVEY_MIN_SIDE_OVERLAP_RATIO
                ),
                planning_speed_mps=(
                    SURVEY_DEFAULT_SPEED_MPS
                ),
                design_bank_deg=(
                    SURVEY_DESIGN_BANK_DEG
                ),
                turn_radius_safety=(
                    SURVEY_TURN_RADIUS_SAFETY
                ),
                wp_radius_m=(
                    SURVEY_WP_RADIUS_M
                ),
                mission_turn_step_m=(
                    SURVEY_TURN_WP_SPACING_M
                ),
                home_xy=home_xy,
            )
            rectangle_js_points = [
                {
                    "lat": lat,
                    "lng": lon,
                }
                for lat, lon in polygon_latlon
            ]

            self.map_view.page().runJavaScript(
                "replacePolygonPoints("
                + json.dumps(
                    rectangle_js_points,
                    ensure_ascii=False,
                )
                + ");"
            )
    
            mission = []
    
            for route_point in plan["route_points"]:
                lat, lon = survey_xy_to_latlon(
                    route_point["x"],
                    route_point["y"],
                    origin_latlon,
                )
    
                mission.append(
                    {
                        "lat": lat,
                        "lon": lon,
                        "lng": lon,
                        "alt": SURVEY_DEFAULT_ALTITUDE_M,
                        "mode": "WAYPOINT",

                        # Bunun otomatik grid waypointi olduğunu belirtir.
                        "is_survey_waypoint": True,

                        "survey_kind": route_point["kind"],
                        "survey_pass": (
                            route_point.get("scan_pass", -1) + 1
                        ),
                        "acceptance_radius_m": float(
                            route_point.get(
                                "acceptance_radius_m",
                                0.0,
                            )
                        ),
                    }
                )
    
            route_corridor = (
                self.survey_xy_rings_to_latlon(
                    plan["route_corridor_rings"]
                )
            )
    
            allowed_boundary = (
                self.survey_xy_rings_to_latlon(
                    plan["allowed_polygon_rings"]
                )
            )
    
            safe_coverage = (
                self.survey_xy_rings_to_latlon(
                    plan["coverage_rings"]
                )
            )
    
            scan_lines = [
                self.survey_xy_path_to_latlon(
                    line
                )
                for line in plan["scan_lines"]
            ]
    
            lead_lines = [
                self.survey_xy_path_to_latlon(
                    line
                )
                for line in plan["lead_lines"]
            ]
    
            turn_lines = [
                self.survey_xy_path_to_latlon(
                    line
                )
                for line in plan["turn_lines"]
            ]
    
            # Plan ve değiştirilemez otomatik görev bölümü.
            self.survey_plan = plan
            self.survey_active_route_plan = plan
            self.survey_active_route_name = "MAIN"
    
            self.survey_plan_mission = [
                dict(item)
                for item in mission
            ]
    
            self.survey_post_mission = []
    
            # Gerçek uçuş tamamlanma takipçisi oluşturulur fakat
            # Write Pist başarılı olana kadar aktif edilmez.
            self.survey_tracker = (
                SurveyCoverageTracker(
                    polygon_xy=polygon_xy,
                    scan_passes=plan["scan_passes"],
                    safe_swath_m=(
                        SURVEY_SAFE_SWATH_M
                    ),
                    target_altitude_m=(
                        SURVEY_DEFAULT_ALTITUDE_M
                    ),
                    cross_track_limit_m=(
                        SURVEY_CROSS_TRACK_LIMIT_M
                    ),
                    heading_limit_deg=(
                        SURVEY_HEADING_LIMIT_DEG
                    ),
                    roll_limit_deg=(
                        SURVEY_ROLL_LIMIT_DEG
                    ),
                    altitude_limit_m=(
                        SURVEY_ALTITUDE_LIMIT_M
                    ),
                )
            )
    
            self.survey_tracking_enabled = False
            self.survey_recovery_round = 0
            self.survey_recovery_pending = False
    
            summary = plan["summary"]
    
            arguments = [
                mission,
                route_corridor,
                allowed_boundary,
                scan_lines,
                lead_lines,
                turn_lines,
                safe_coverage,
                summary,
            ]
    
            argument_json = [
                json.dumps(
                    value,
                    ensure_ascii=False,
                )
                for value in arguments
            ]
    
            fixed_wing_call = (
                "loadFixedWingSurveyPreview("
                + ",".join(argument_json)
                + ");"
            )
    
            # HTML'yi bir sonraki aşamada güncelleyeceğiz.
            # Şimdilik eski HTML varsa en azından waypointleri gösterir.
            fallback_call = (
                "loadSurveyWaypoints("
                + argument_json[0]
                + ");"
            )
    
            js_code = (
                "(function(){"
                "if (typeof loadFixedWingSurveyPreview === 'function') {"
                + fixed_wing_call
                + "} else if (typeof loadSurveyWaypoints === 'function') {"
                + fallback_call
                + "} else {"
                "console.error('Survey preview fonksiyonu bulunamadı.');"
                "}"
                "})();"
            )
    
            js_code = (
                "(function(){"
                "if (typeof loadFixedWingSurveyPreview === 'function') {"
                "return "
                + fixed_wing_call
                + "} else if (typeof loadSurveyWaypoints === 'function') {"
                "return "
                + fallback_call
                + "} else {"
                "console.error('Survey preview fonksiyonu bulunamadı.');"
                "return null;"
                "}"
                "})();"
            )

            self.map_view.page().runJavaScript(
                js_code,
                self.on_survey_mission_merged,
            )
    
            self.polygon_toggle_btn.setChecked(
                False
            )
    
            self.log_signal.emit(
                "[FIXED-WING SURVEY] "
                f"hat={summary['scan_pass_count']} | "
                f"WP={summary['waypoint_count']} | "
                f"tam görüntü={summary['footprint_m']:.2f}m | "
                f"ROI={summary['roi_footprint_m']:.3f}m | "
                f"hat aralığı="
                f"{summary['max_lane_spacing_m']:.2f}m | "
                f"bindirme=%"
                f"{summary['actual_overlap_percent']:.1f} | "
                f"R={summary['turn_radius_m']:.1f}m | "
                f"en uzak dış="
                f"{summary['max_outside_distance_m']:.1f}m | "
                f"rota={summary['route_length_m']:.1f}m | "
                f"süre≈{summary['estimated_time_s']:.1f}s"
            )
    
        except Exception as error:
            self.survey_plan = None
            self.survey_plan_mission = []
            self.survey_tracker = None
            self.survey_tracking_enabled = False
    
            self.log_signal.emit(
                f"[FIXED-WING SURVEY ERROR] {error}"
            )
    
            QMessageBox.critical(
                self,
                "Sabit Kanat Tarama Hatası",
                str(error),
            )

    def on_survey_mission_merged(self, merged_mission):
        """
        Görev sırası:
        manuel ön görevler + otomatik grid + manuel son görevler
        """
        if not isinstance(merged_mission, list):
            self.log_signal.emit(
                "[FIXED-WING SURVEY ERROR] "
                "Birleştirilmiş görev haritadan okunamadı."
            )
            return

        mission = self.normalize_loaded_mission(
            merged_mission
        )

        if not mission:
            self.log_signal.emit(
                "[FIXED-WING SURVEY ERROR] "
                "Birleştirilmiş görev boş."
            )
            return

        self.mission_data = mission
        self.populate_table_from_mission(mission)
        self.refresh_servo_alert_seq_from_mission()

        survey_indexes = [
            index
            for index, waypoint in enumerate(mission)
            if bool(
                waypoint.get(
                    "is_survey_waypoint",
                    False,
                )
            )
        ]

        if survey_indexes:
            first_survey = survey_indexes[0]
            last_survey = survey_indexes[-1]

            self.survey_pre_mission = [
                dict(item)
                for item in mission[:first_survey]
            ]

            self.survey_post_mission = [
                dict(item)
                for item in mission[last_survey + 1:]
            ]
        else:
            self.survey_pre_mission = [
                dict(item)
                for item in mission
            ]
            self.survey_post_mission = []

        self.log_signal.emit(
            "[FIXED-WING SURVEY MERGE] "
            f"ön görev={len(self.survey_pre_mission)} | "
            f"grid={len(survey_indexes)} | "
            f"son görev={len(self.survey_post_mission)} | "
            f"toplam={len(mission)}"
        )
    
    def toggle_polygon_mode(self, checked):
        if checked:
            self.polygon_toggle_btn.setText("Poligon Çiz: AÇIK")
            self.polygon_toggle_btn.setStyleSheet("background-color: #27ae60; color: white;")
            self.map_view.page().runJavaScript("setPolygonMode(true);")
        else:
            self.polygon_toggle_btn.setText("Poligon Çiz: KAPALI")
            self.polygon_toggle_btn.setStyleSheet("background-color: #34495e; color: white;")
            self.map_view.page().runJavaScript("setPolygonMode(false);")

    def clear_polygon(self):
        """
        Haritadaki ve Python tarafındaki bütün poligon
        ve hesaplama verilerini temizler.
        """
        self.polygon_toggle_btn.setChecked(False)

        self.survey_polygon_latlon = []
        self.survey_polygon_xy = []
        self.survey_origin_latlon = None
        self.survey_polygon_geometry = None

        self.map_view.page().runJavaScript(
            "clearPolygon();"
        )

        self.log_signal.emit("Tarama poligonu temizlendi.")
    
    def validate_mission(self):
        # Write işleminin tek kaynağı mevcut tablo olsun.
        # Eski mission_data veya haritadaki stale JS noktaları yetkili değildir.
        if self.table.rowCount() < 2:
            return False

        modes = []
        for i in range(self.table.rowCount()):
            combo = self.table.cellWidget(i, 4)
            if combo is None:
                return False
            modes.append(combo.currentText())
        takeoff_modes = {"TAKEOFF", "VTOL_TAKEOFF"}
        for takeoff_mode in takeoff_modes:
            if takeoff_mode in modes and modes.index(takeoff_mode) != 0:
                self.log_signal.emit(
                    f"GÖREV GÖNDERİLMEDİ: {takeoff_mode} ilk görev satırı olmalı."
                )
                return False
        if "RTL" in modes and modes.index("RTL") != len(modes) - 1:
            return False
        if "QRTL" in modes:
            qrtl_index = modes.index("QRTL")
            # QRTL ya son satır olabilir ya da hemen arkasından HOLD gelebilir.
            if qrtl_index != len(modes) - 1:
                if not (qrtl_index == len(modes) - 2 and modes[qrtl_index + 1] == "HOLD"):
                    return False
        return True
    
    def send_mission(self):
        if not self.validate_mission():
            return
        self.map_view.page().runJavaScript(
            "getMissionData();",self.finish_send_mission
        )
    def finish_send_mission(self, data):
        # JavaScript'ten gelen liste yalnızca harita görselleştirmesidir.
        # Gerçek Write listesi mevcut QTableWidget'tan yeniden kurulur.
        mission = self.build_mission_list_from_table(data)
        if len(mission) < 2:
            self.log_signal.emit(
                "GÖREV GÖNDERİLMEDİ: tabloda geçerli görev satırı yok."
            )
            return

        if self._mission_upload_active:
            self.log_signal.emit(
                "GÖREV GÖNDERİLMEDİ: önceki mission Write işlemi hâlâ sürüyor."
            )
            return

        self.mission_data = mission
        # Worker'ın daha sonra değişen self.mission_data'yı kullanmaması için
        # bu Write işleminin sabit bir kopyasını alıyoruz.
        mission_to_upload = [dict(item) for item in mission]
        js_data = json.dumps(self.mission_data)
        self.home_page.map_view.page().runJavaScript(f"drawMission({js_data});")
        
        if self.drone_manager.is_connected():

            self._set_mission_upload_state(True)

            def upload_worker():
                try:
                    self.log_signal.emit("Görev yükleniyor, lütfen bekleyin...")

                    ok = self.drone_manager.drone.upload_mission(
                        mission_to_upload
                    )

                    if ok:
                        detail = getattr(
                            self.drone_manager.drone,
                            "last_mission_upload_status",
                            "",
                        )
                        suffix = f" {detail}" if detail else ""
                        self.log_signal.emit(
                            "Görev Pixhawk'a yazıldı; upload işlemi "
                            f"tamamlandı.{suffix}"
                        )
                        self.mission_uploaded_signal.emit(
                            mission_to_upload
                        )
                    else:
                        detail = getattr(
                            self.drone_manager.drone,
                            "last_mission_upload_status",
                            "",
                        )
                        if detail:
                            detail = f" {detail}"
                        self.log_signal.emit(
                            "GÖREV YÜKLENMEDİ/DOĞRULANMADI!"
                            f"{detail} AUTO/kalkış yapmayın."
                        )

                except Exception as e:
                    self.log_signal.emit(f"GÖREV YÜKLEME HATASI: {e}")
                finally:
                    self.mission_upload_state.emit(False)

            t = threading.Thread(target=upload_worker)
            t.daemon = True
            t.start()

        else:
            self.log_signal.emit("Görev haritaya çizildi (Uyarı: Drone bağlı değil!)")
    def set_center(self, lat, lon):
        if not self.map_ready:
            return

        try:
            lat = float(lat)
            lon = float(lon)
        except (TypeError, ValueError):
            return

        self.map_view.page().runJavaScript(
            f"""
            if (typeof setCenter === "function") {{
                setCenter({lat}, {lon});
            }}
            """
        )
    def reset_center(self):
        self.map_view.page().runJavaScript("resetCenter()")

class SettingPage(QWidget):
    log_signal=pyqtSignal(str)
    def __init__(self,drone_manager):
        super().__init__()
        self.drone_manager=drone_manager

        #MOTOR TEST DEĞİŞKENLERİ
        self.ripple_timer = QTimer()
        self.ripple_timer.timeout.connect(self.fire_next_motor)
        self.test_queue = [] 
        self.current_throttle = 0
        self.current_duration = 0

        #KALİBRASYON DEĞİŞKENLERİ
        self.is_calibrating = False
        self.calibration_data = {
            1: {'min': 3000, 'max': 0},
            2: {'min': 3000, 'max': 0},
            3: {'min': 3000, 'max': 0},
            4: {'min': 3000, 'max': 0}
        }

        # SERVO OUTPUT MONITOR DEĞİŞKENLERİ
        self.servo_pwm_bars = {}
        self.servo_pwm_labels = {}
        self.servo_function_combos = {}
        self.servo_reverse_checks = {}

        self.servo_params_loaded = False
        self._updating_servo_ui = False

        # Mission Planner'daki Function combobox mantığı.
        # Listeyi zamanla daha da büyütebiliriz; yarışma için kritik olanlar burada var.
        self.servo_function_options = [
            (-1, "GPIO"),
            (0, "Disabled"),
            (1, "RCPassThru"),
            (2, "Flap"),
            (3, "Flap_auto"),
            (4, "Aileron"),
            (6, "MountPan"),
            (7, "MountTilt"),
            (8, "MountRoll"),
            (10, "CameraTrigger"),
            (16, "DifferentialSpoilerLeft1"),
            (17, "DifferentialSpoilerRight1"),
            (19, "Elevator"),
            (21, "Rudder"),
            (24, "FlaperonLeft"),
            (25, "FlaperonRight"),
            (26, "GroundSteering"),
            (27, "ParachuteRelease"),
            (33, "Motor1"),
            (34, "Motor2"),
            (35, "Motor3"),
            (36, "Motor4"),
            (37, "Motor5"),
            (38, "Motor6"),
            (39, "Motor7"),
            (40, "Motor8"),
            (41, "MotorTilt"),
        ] + [(i, f"RCPassThru{i-50}") for i in range(51, 67)] + [
            (67, "Ignition"),
            (69, "Starter"),
            (70, "Throttle"),
            (73, "ThrottleLeft"),
            (74, "ThrottleRight"),
            (75, "TiltMotorLeft"),
            (76, "TiltMotorRight"),
            (77, "ElevonLeft"),
            (78, "ElevonRight"),
            (79, "VTailLeft"),
            (80, "VTailRight"),
            (86, "DifferentialSpoilerLeft2"),
            (87, "DifferentialSpoilerRight2"),
        ]

        self.main_layout=QVBoxLayout()

        #MOTOR TEST BÖLÜMÜ
        self.label=QLabel("MOTOR TESTİ")
        self.label.setFont(QFont("Arial",14, QFont.Bold))
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("color:black; margin-top:10px;")
        self.main_layout.addWidget(self.label)

        self.warning = QLabel("DİKKAT: Testten önce pervaneleri sökün!")
        self.warning.setStyleSheet("color: red; font-weight: bold;")
        self.warning.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.warning)

        self.current_label=QLabel("Anlık çekilen akım: 0.00 A")
        self.current_label.setFont(QFont("Arial",14))
        self.current_label.setStyleSheet("color: #f1c40f; background-color: #2c313c; padding: 15px; border-radius: 8px;")
        self.current_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.current_label)

        self.settings_layout=QGridLayout()
        self.settings_layout.addWidget(QLabel("Test Edilecek Motor:"), 0, 0)
        
        self.motor_number=QComboBox()
        self.motor_number.addItems(["Sağ Ön (Motor A)", "Sağ Arka (Motor B)", "Sol Arka (Motor C)", "Sol Ön (Motor D)","İleri İtici (Motor 5 - Sadece VTOL)","HEPSİNİ TEST ET (Sırayla)"])
        self.settings_layout.addWidget(self.motor_number, 0, 1)

        self.settings_layout.addWidget(QLabel("Motor Gücü (%):"), 1, 0)
        self.throttle_spin = QSpinBox()
        self.throttle_spin.setRange(1, 100)
        self.throttle_spin.setValue(5)
        self.settings_layout.addWidget(self.throttle_spin, 1, 1)

        self.settings_layout.addWidget(QLabel("Test Süresi (Saniye):"), 2, 0)
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(1, 15) 
        self.duration_spin.setValue(3) 
        self.settings_layout.addWidget(self.duration_spin, 2, 1)

        self.main_layout.addLayout(self.settings_layout)

        self.button_layout=QHBoxLayout()
        self.start_bttn=QPushButton("START MOTOR")
        self.start_bttn.setFont(QFont("Arial",14))
        self.start_bttn.setStyleSheet("background-color: #27ae60; color: white; padding: 10px;")

        self.stop_bttn=QPushButton("STOP MOTOR")
        self.stop_bttn.setFont(QFont("Arial",14))
        self.stop_bttn.setStyleSheet("background-color: #c0392b; color: white; padding: 10px;")

        self.button_layout.addWidget(self.start_bttn)
        self.button_layout.addWidget(self.stop_bttn)
        self.main_layout.addLayout(self.button_layout)

        # Araya ince bir çizgi çekelim
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        self.main_layout.addWidget(line)

        #RADYO KALİBRASYON BÖLÜMÜ
        self.calib_label = QLabel("RADYO (KUMANDA) KALİBRASYONU")
        self.calib_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.calib_label.setAlignment(Qt.AlignCenter)
        self.calib_label.setStyleSheet("color: #2980b9; margin-top: 10px;")
        self.main_layout.addWidget(self.calib_label)

        self.calib_info = QLabel("Başlat'a basın ve kumanda çubuklarını her yöne sonuna kadar ittirin.")
        self.calib_info.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.calib_info)

        self.calib_grid = QGridLayout()
        self.calib_grid.setSpacing(10)
        
        # Sütun Başlıkları
        basliklar = ["Kanal", "Anlık (Live)", "Min", "Max"]
        for i, b in enumerate(basliklar):
            lbl = QLabel(b)
            lbl.setFont(QFont("Arial", 10, QFont.Bold))
            self.calib_grid.addWidget(lbl, 0, i)

        self.ch_labels = {}
        channels = ["CH1 (Roll)", "CH2 (Pitch)", "CH3 (Gaz)", "CH4 (Yaw)"]

        # Tabloyu oluştur ve sözlüğe kaydet
        for i, name in enumerate(channels, start=1):
            self.calib_grid.addWidget(QLabel(name), i, 0)
            self.ch_labels[i] = {
                'live': QLabel("0"),
                'min': QLabel("0"),
                'max': QLabel("0")
            }
            self.ch_labels[i]['live'].setStyleSheet("color: blue; font-weight: bold;")
            self.ch_labels[i]['min'].setStyleSheet("color: red; font-weight: bold;")
            self.ch_labels[i]['max'].setStyleSheet("color: green; font-weight: bold;")
            
            self.calib_grid.addWidget(self.ch_labels[i]['live'], i, 1)
            self.calib_grid.addWidget(self.ch_labels[i]['min'], i, 2)
            self.calib_grid.addWidget(self.ch_labels[i]['max'], i, 3)

        self.main_layout.addLayout(self.calib_grid)

        self.calib_btn_layout = QHBoxLayout()
        self.start_calib_btn = QPushButton("Kalibrasyonu Başlat")
        self.start_calib_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 10px; font-weight: bold;")

        self.save_calib_btn = QPushButton("Uçağa Kaydet")
        self.save_calib_btn.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; font-weight: bold;")
        self.save_calib_btn.setEnabled(False) # Başlangıçta pasif

        self.calib_btn_layout.addWidget(self.start_calib_btn)
        self.calib_btn_layout.addWidget(self.save_calib_btn)
        self.main_layout.addLayout(self.calib_btn_layout)
 
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        self.main_layout.addWidget(line2)

        self.calib_drone_label = QLabel("DRONE KALİBRASYONU (LEVEL + COMPASS)")
        self.calib_drone_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.calib_drone_label.setAlignment(Qt.AlignCenter)
        self.calib_drone_label.setStyleSheet("color: #27ae60; margin-top: 10px;")
        self.main_layout.addWidget(self.calib_drone_label)

        # HTML görselleştirme
        self.calib_web = QWebEngineView()
        base_path = os.path.dirname(os.path.abspath(__file__))
        calib_path = os.path.join(base_path, "calibration.html")
        self.calib_web.load(QUrl.fromLocalFile(os.path.abspath(calib_path)))
        self.calib_web.setFixedHeight(420)
        self.main_layout.addWidget(self.calib_web)

        # Kalibrasyon butonları ve Düzenleri
        calib_drone_btn_layout = QHBoxLayout()

        self.level_calib_btn = QPushButton("📐 Level Kalibrasyonu Başlat")
        self.level_calib_btn.setStyleSheet("background-color:#27ae60; color:white; padding:10px; font-weight:bold;")

        self.level_next_btn = QPushButton("✅ Pozisyonu Kaydet (İleri)")
        self.level_next_btn.setStyleSheet("background-color:#2980b9; color:white; padding:10px; font-weight:bold;")
        self.level_next_btn.hide() # Başlangıçta gizli

        self.compass_calib_btn = QPushButton("🧭 Compass Kalibrasyonu Başlat")
        self.compass_calib_btn.setStyleSheet("background-color:#e67e22; color:white; padding:10px; font-weight:bold;")

        self.compass_cancel_btn = QPushButton("⛔ Compass İptal")
        self.compass_cancel_btn.setStyleSheet("background-color:#c0392b; color:white; padding:10px; font-weight:bold;")

        calib_drone_btn_layout.addWidget(self.level_calib_btn)
        calib_drone_btn_layout.addWidget(self.level_next_btn)
        calib_drone_btn_layout.addWidget(self.compass_calib_btn)
        calib_drone_btn_layout.addWidget(self.compass_cancel_btn)
        self.main_layout.addLayout(calib_drone_btn_layout)

        # Başlangıçta gizle (sadece DRONE sekmesinde gösterilecek)
        self.calib_drone_label.hide()
        self.calib_web.hide()
        self.level_calib_btn.hide()
        self.compass_calib_btn.hide()
        self.compass_cancel_btn.hide()

        # Compass kalibrasyon durumu
        self.compass_calib_active = False
        self.compass_calib_progress = 0.0
        self.compass_calib_timer = QTimer()
        self.compass_calib_timer.timeout.connect(self._compass_calib_tick)

        self.main_layout.addSpacing(25)
        
        self.reboot_btn=QPushButton("UÇAĞI YENİDEN BAŞLAT (REBOOT)")
        self.reboot_btn.setStyleSheet("background-color: #c0392b; color: white; padding: 15px; font-weight: bold; font-size: 14px;")
        self.main_layout.addWidget(self.reboot_btn)

        # Mission Planner benzeri Servo Output paneli
        self.add_servo_output_panel()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        
        container = QWidget()
        container.setLayout(self.main_layout)
        scroll.setWidget(container)
        
        # Dış layout — sadece scroll var
        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)
        self.setLayout(outer)
        
        self.setup_connections()

    def add_servo_output_panel(self):
        """
        Mission Planner > Servo Output benzeri panel.
        Sol satır numarası SERVO numarasıdır:
        1 = SERVO1
        2 = SERVO2
        ...
        16 = SERVO16
        """

        self.main_layout.addSpacing(35)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        self.main_layout.addWidget(line)

        title = QLabel("SERVO OUTPUT")
        title.setFont(QFont("Arial", 15, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color:#16a085; margin-top:10px;")
        self.main_layout.addWidget(title)

        info = QLabel(
            "Mission Planner mantığı: Position bar gerçek PWM çıkışıdır. "
            "Function combobox ise SERVOx_FUNCTION parametresidir."
        )
        info.setWordWrap(True)
        info.setAlignment(Qt.AlignCenter)
        info.setStyleSheet("color:#34495e; margin-bottom:8px;")
        self.main_layout.addWidget(info)

        panel = QFrame()
        panel.setStyleSheet("""
            QFrame {
                background-color:#2c313c;
                border:1px solid #555;
                border-radius:6px;
            }
            QLabel {
                color:white;
            }
            QComboBox {
                background-color:#3a3f4b;
                color:white;
                border:1px solid #9aa0aa;
                padding:3px;
                min-height:22px;
            }
            QCheckBox {
                background-color:transparent;
            }
        """)

        grid = QGridLayout(panel)
        grid.setSpacing(6)
        grid.setContentsMargins(10, 10, 10, 10)

        headers = ["#", "Position", "Reverse", "Function"]
        for col, text in enumerate(headers):
            label = QLabel(text)
            label.setFont(QFont("Arial", 10, QFont.Bold))
            label.setStyleSheet("color:white;")
            grid.addWidget(label, 0, col)

        for servo_no in range(1, 17):
            row = servo_no

            number_label = QLabel(str(servo_no))
            number_label.setMinimumWidth(25)
            number_label.setAlignment(Qt.AlignCenter)

            pwm_bar = QProgressBar()
            pwm_bar.setRange(0, 2200)
            pwm_bar.setValue(0)
            pwm_bar.setFormat("0")
            pwm_bar.setTextVisible(True)
            pwm_bar.setMinimumHeight(26)
            pwm_bar.setStyleSheet("""
                QProgressBar {
                    border:1px solid #808080;
                    border-radius:4px;
                    background-color:qlineargradient(
                        x1:0, y1:0, x2:0, y2:1,
                        stop:0 #888,
                        stop:0.45 #444,
                        stop:1 #222
                    );
                    color:white;
                    text-align:center;
                    font-weight:bold;
                }
                QProgressBar::chunk {
                    background-color:qlineargradient(
                        x1:0, y1:0, x2:0, y2:1,
                        stop:0 #b6ff45,
                        stop:1 #7fbf20
                    );
                    border-radius:3px;
                }
            """)

            reverse_check = QCheckBox()
            reverse_check.setProperty("servo_no", servo_no)
            reverse_check.stateChanged.connect(self.servo_reverse_changed)

            function_combo = QComboBox()
            function_combo.setProperty("servo_no", servo_no)
            function_combo.setMinimumWidth(210)

            for function_id, function_name in self.servo_function_options:
                function_combo.addItem(function_name, function_id)

            function_combo.currentIndexChanged.connect(self.servo_function_changed)

            grid.addWidget(number_label, row, 0)
            grid.addWidget(pwm_bar, row, 1)
            grid.addWidget(reverse_check, row, 2, alignment=Qt.AlignCenter)
            grid.addWidget(function_combo, row, 3)

            self.servo_pwm_bars[servo_no] = pwm_bar
            self.servo_reverse_checks[servo_no] = reverse_check
            self.servo_function_combos[servo_no] = function_combo

        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        self.main_layout.addWidget(panel)

        note = QLabel(
            "Güvenlik: Function veya Reverse değişikliği ARM durumunda yazılmaz. "
            "Sadece DISARM iken SERVOx_FUNCTION / SERVOx_REVERSED parametresine yazılır."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#7f8c8d; font-size:11px; margin-top:6px;")
        self.main_layout.addWidget(note)

    def set_combo_to_function_id(self, combo, function_id):
        idx = combo.findData(function_id)

        if idx < 0:
            combo.addItem(f"Unknown({function_id})", function_id)
            idx = combo.findData(function_id)

        combo.setCurrentIndex(idx)

    def refresh_servo_params_from_vehicle(self, force=False):
        """
        Bağlanınca SERVOx_FUNCTION ve SERVOx_REVERSED değerlerini Pixhawk'tan okuyup combobox'lara basar.
        """
        if not force and self.servo_params_loaded:
            return

        if not self.drone_manager.is_connected():
            return

        try:
            functions = self.drone_manager.drone.get_servo_function_params(count=16)
            reverses = self.drone_manager.drone.get_servo_reverse_params(count=16)

            self._updating_servo_ui = True

            for servo_no in range(1, 17):
                function_id = functions.get(servo_no, 0)
                reversed_state = reverses.get(servo_no, False)

                combo = self.servo_function_combos.get(servo_no)
                check = self.servo_reverse_checks.get(servo_no)

                if combo:
                    self.set_combo_to_function_id(combo, function_id)

                if check:
                    check.setChecked(bool(reversed_state))

            self._updating_servo_ui = False
            self.servo_params_loaded = True

            self.log_signal.emit("Servo Output parametreleri okundu: SERVO1_FUNCTION ... SERVO16_FUNCTION")

        except Exception as e:
            self._updating_servo_ui = False
            self.log_signal.emit(f"Servo parametreleri okunamadı: {e}")

    def servo_function_changed(self):
        if self._updating_servo_ui:
            return

        combo = self.sender()
        if combo is None:
            return

        servo_no = combo.property("servo_no")
        function_id = combo.currentData()

        if servo_no is None or function_id is None:
            return

        if not self.drone_manager.is_connected():
            self.log_signal.emit("Servo function değiştirilemedi: araç bağlı değil.")
            return

        try:
            if self.drone_manager.drone.is_arm():
                self.log_signal.emit(f"GÜVENLİK: SERVO{servo_no}_FUNCTION ARM durumunda değiştirilemez.")
                self.refresh_servo_params_from_vehicle(force=True)
                return
        except Exception:
            pass

        ok = self.drone_manager.drone.set_servo_function_param(servo_no, function_id)

        if ok:
            self.log_signal.emit(f"SERVO{servo_no}_FUNCTION = {function_id} ({combo.currentText()}) yazıldı.")
        else:
            self.log_signal.emit(f"SERVO{servo_no}_FUNCTION yazılamadı.")
            self.refresh_servo_params_from_vehicle(force=True)

    def servo_reverse_changed(self, state):
        if self._updating_servo_ui:
            return

        check = self.sender()
        if check is None:
            return

        servo_no = check.property("servo_no")
        reversed_state = state == Qt.Checked

        if servo_no is None:
            return

        if not self.drone_manager.is_connected():
            self.log_signal.emit("Servo reverse değiştirilemedi: araç bağlı değil.")
            return

        try:
            if self.drone_manager.drone.is_arm():
                self.log_signal.emit(f"GÜVENLİK: SERVO{servo_no}_REVERSED ARM durumunda değiştirilemez.")
                self.refresh_servo_params_from_vehicle(force=True)
                return
        except Exception:
            pass

        ok = self.drone_manager.drone.set_servo_reverse_param(servo_no, reversed_state)

        if ok:
            self.log_signal.emit(f"SERVO{servo_no}_REVERSED = {1 if reversed_state else 0} yazıldı.")
        else:
            self.log_signal.emit(f"SERVO{servo_no}_REVERSED yazılamadı.")
            self.refresh_servo_params_from_vehicle(force=True)

    def update_servo_output_display(self, data):
        """
        Position barlarını canlı PWM verisiyle günceller.
        """
        if self.drone_manager.is_connected() and not self.servo_params_loaded:
            self.refresh_servo_params_from_vehicle(force=True)

        outputs = data.get("servo_outputs", {}) or {}

        for servo_no in range(1, 17):
            pwm = outputs.get(servo_no, outputs.get(str(servo_no), 0)) or 0

            bar = self.servo_pwm_bars.get(servo_no)
            if not bar:
                continue

            try:
                pwm = int(pwm)
            except Exception:
                pwm = 0

            if pwm <= 0:
                bar.setValue(0)
                bar.setFormat("0")
            else:
                bar.setValue(max(0, min(2200, pwm)))
                bar.setFormat(str(pwm))

    def setup_connections(self):
        # Motor bağlantıları
        self.start_bttn.clicked.connect(self.start_test)
        self.stop_bttn.clicked.connect(self.stop_test)
        # Kalibrasyon bağlantıları
        self.start_calib_btn.clicked.connect(self.toggle_calibration)
        self.save_calib_btn.clicked.connect(self.save_calibration)
        self.level_calib_btn.clicked.connect(self.run_level_calibration)
        self.level_next_btn.clicked.connect(self.next_level_step) # İŞTE UNUTULAN/HATALI YERE KONAN SATIR BURADA
        self.compass_calib_btn.clicked.connect(self.start_compass_calibration)
        self.compass_cancel_btn.clicked.connect(self.cancel_compass_calibration)
        self.reboot_btn.clicked.connect(self.reboot_vehicle)

    def reboot_vehicle(self):
        if self.drone_manager.is_connected():
            self.log_signal.emit("Uçağa reboot gönderildi bağlantı koparılıyor")
            self.drone_manager.drone.reboot_vehicle()
        else:
            self.log_signal.emit("HATA: Reboot işlemi için uçağa bağlı olmalısınız!")

    #MOTOR TEST FONKSİYONLARI
    def start_test(self):
        secilen_yazi = self.motor_number.currentText()
        self.current_throttle = self.throttle_spin.value()
        self.current_duration = self.duration_spin.value()

        if not self.drone_manager.is_connected():
            self.log_signal.emit("Motor testi başarısız: Drone bağlı değil!")
            return
            
        is_vtol = self.motor_number.count() > 5 
        gercek_motor_sirasi = [1, 3, 2, 4, 5] if is_vtol else [1, 3, 2, 4] 
        if secilen_yazi=="HEPSİNİ TEST ET (Sırayla)": 
            self.test_queue=gercek_motor_sirasi.copy()
            self.fire_next_motor()
            self.ripple_timer.start(300) 
            self.log_signal.emit(f"Dalga Etkisi (Ripple) başlatıldı! Güç: %{self.current_throttle}")
        else:
            self.ripple_timer.stop() 
            index = self.motor_number.currentIndex()
            gercek_motor_no = gercek_motor_sirasi[index]
            self.drone_manager.drone.run_motor_test(gercek_motor_no, self.current_throttle, self.current_duration, motor_count=1)
            self.log_signal.emit(f"Motor {index+1} (Id:{gercek_motor_no}) başlatıldı. Güç: %{self.current_throttle}")

    def fire_next_motor(self):
        if self.test_queue:
            motor_id=self.test_queue.pop(0) 
            self.drone_manager.drone.run_motor_test(motor_id, self.current_throttle, self.current_duration, motor_count=1)
        else:
            self.ripple_timer.stop() 
            self.log_signal.emit("Tüm motorların dalga testi başarıyla tamamlandı.")

    def stop_test(self):
        if not self.drone_manager.is_connected():
            return
        self.ripple_timer.stop() 
        self.drone_manager.drone.stop_motor_test()
        self.log_signal.emit("TEST İPTAL EDİLDİ! Güç kesildi.")

    # --- RADYO KALİBRASYON FONKSİYONLARI ---
    def toggle_calibration(self):
        if not self.drone_manager.is_connected():
            self.log_signal.emit("HATA: Uçak bağlı değil, kalibrasyon yapılamaz!")
            return

        if not self.is_calibrating:
            # KALİBRASYONU BAŞLAT
            self.is_calibrating = True
            self.start_calib_btn.setText("Kalibrasyonu Bitir")
            self.start_calib_btn.setStyleSheet("background-color: #d35400; color: white; padding: 10px; font-weight: bold;")
            self.save_calib_btn.setEnabled(False)
            
            # Verileri sıfırla (Çok yüksek ve çok düşük değerler atıyoruz ki ezilsinler)
            for i in range(1, 5):
                self.calibration_data[i] = {'min': 3000, 'max': 0}
                self.ch_labels[i]['min'].setText("Bekleniyor...")
                self.ch_labels[i]['max'].setText("Bekleniyor...")
                
            self.log_signal.emit("Kalibrasyon Başladı: Lütfen tüm çubukları sınırlarına kadar ittirin!")
        else:
            # KALİBRASYONU BİTİR
            self.is_calibrating = False
            self.start_calib_btn.setText("Kalibrasyonu Tekrar Başlat")
            self.start_calib_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 10px; font-weight: bold;")
            self.save_calib_btn.setEnabled(True) # Kaydet butonunu artık açabiliriz
            self.log_signal.emit("Okuma tamamlandı. Değerleri uçağa yazmak için KAYDET butonuna basın.")

    def save_calibration(self):
        if not self.drone_manager.is_connected():
            return
        try:
            self.log_signal.emit("kalibrasyon başladı")
            for i in range(1, 5):
                c_min = self.calibration_data[i]['min']
                c_max = self.calibration_data[i]['max']
                
                # SİSTEM GÜVENLİĞİ: Eğer değerler mantıksızsa kaydı engelle
                if c_min < 800 or c_max > 2200 or c_min >= c_max:
                    self.log_signal.emit(f"HATA: Kanal {i} verileri bozuk (Min:{c_min}, Max:{c_max}). Lütfen kalibrasyonu baştan yapın!")
                    return

                # Pixhawk parametrelerine kaydet
                self.drone_manager.drone.vehicle.parameters[f'RC{i}_MIN'] = c_min
                self.drone_manager.drone.vehicle.parameters[f'RC{i}_MAX'] = c_max
                
                # TRIM (Merkez) Değerlerini ArduPilot standartlarına göre ayarla
                if i == 3:
                    self.drone_manager.drone.vehicle.parameters[f'RC{i}_TRIM'] = c_min
                else:
                    trim_val = (c_min + c_max) // 2
                    self.drone_manager.drone.vehicle.parameters[f'RC{i}_TRIM'] = trim_val
            kanal_isimleri = {1: "Roll (Sağ/Sol)", 2: "Pitch (İleri/Geri)", 3: "Throttle (Gaz)", 4: "Yaw (Dönüş)"}
            self.log_signal.emit(f"[+] {kanal_isimleri[i]} -> MIN: {c_min} | MAX: {c_max} | MERKEZ: {trim_val} başarıyla yazıldı.")

            self.log_signal.emit("BAŞARILI: Yeni radyo sınırları Pixhawk'a kalıcı olarak kaydedildi!")
            self.save_calib_btn.setEnabled(False) # Kaydettikten sonra tekrar kilitliyoruz
        except Exception as e:
            self.log_signal.emit(f"Kayıt Hatası: {e}")

    #TELEMETRİDEN GELEN VERİLERİ EKRANA BASAN FONKSİYON
    def update_current_display(self, data):
        # 1. Anlık Akım
        amps = data.get("servo_current", 0.0)
        self.current_label.setText(f"Anlık Çekilen Akım: {amps:.2f} A")

        # 2. Kalibrasyon Verileri (Canlı Okuma)
        ch_data = {
            1: data.get("ch1", 0),
            2: data.get("ch2", 0),
            3: data.get("ch3", 0),
            4: data.get("ch4", 0)
        }

        for i in range(1, 5):
            val = ch_data[i]
            if val is not None and val > 0: 
                # Anlık sütununu güncelle
                self.ch_labels[i]['live'].setText(str(val))

                # Eğer kalibrasyon tuşuna basıldıysa Min ve Max rekorlarını kır
                if self.is_calibrating:
                    if val < self.calibration_data[i]['min']:
                        self.calibration_data[i]['min'] = val
                        self.ch_labels[i]['min'].setText(str(val))
                        
                    if val > self.calibration_data[i]['max']:
                        self.calibration_data[i]['max'] = val
                        self.ch_labels[i]['max'].setText(str(val))

        self.update_servo_output_display(data)

        if self.calib_web.isVisible():
            pitch_val = data.get("pitch", 0) or 0.0
            roll_val = data.get("roll", 0) or 0.0
            heading_val = data.get("heading", 0) or 0.0
            self.update_calibration_display(pitch_val, roll_val, heading_val)

    def show_drone_calibration(self, visible):
        """DRONE sekmesine geçildiğinde çağrılır"""
        for w in [self.calib_drone_label, self.calib_web,
            self.level_calib_btn, self.compass_calib_btn, self.compass_cancel_btn]:
            w.setVisible(visible)

    def run_level_calibration(self):
        if not self.drone_manager.is_connected():
            self.log_signal.emit("HATA: Level kalibrasyonu için drone bağlı olmalı!")
            return
        
        self.accel_step = 0 # Sıfırıncı adımdan başla
        self.calib_web.page().runJavaScript("setStatus('running','⏳ Uçağı DÜZ koyun ve Kaydet e basın.');setAccelStep(0);")
        self.log_signal.emit("Level kalibrasyonu başladı. Ekranda istenen pozisyonlara getirip 'Kaydet'e basın.")
        
        # Kalibrasyon start emri (param5=1)
        self.drone_manager.drone.start_accel_calibration_step(0)
        
        self.level_calib_btn.hide()
        self.level_next_btn.show() # Kaydet butonunu aç
    
    def next_level_step(self):
        self.accel_step += 1
        self.drone_manager.drone.start_accel_calibration_step(self.accel_step) # 1,2,3,4,5,6 yolla
        self.calib_web.page().runJavaScript(f"setAccelStep({self.accel_step});")
        
        if self.accel_step >= 6:
            self.level_next_btn.hide()
            self.level_calib_btn.show()
            self.log_signal.emit("Level Kalibrasyonu Tamamlandı!")

    def _level_calib_done(self):
        self.calib_web.page().runJavaScript("setStatus('success','✅ Level kalibrasyonu tamamlandı!');setStep(2);")
        self.log_signal.emit("Level kalibrasyonu başarıyla tamamlandı! Şimdi compass kalibrasyonuna geçebilirsiniz.")

    def start_compass_calibration(self):
        if not self.drone_manager.is_connected():
            return
        try:
            self.compass_calib_active = True
            self.compass_calib_progress = 0.0
            self.calib_web.page().runJavaScript("setStatus('running','🧭 Drone u her eksende çevirin!');setStep(3);")
            
            # Gerçek veriyi drone.py'den arayüze bağlayan köprü (Callback)
            self.drone_manager.drone.compass_cb = self._real_compass_update

            msg = self.drone_manager.drone.vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 1, 0, 0, 0, 0, 0
            )
            self.drone_manager.drone.vehicle.send_mavlink(msg)
        except Exception as e:
            pass

    def _real_compass_update(self, pct):
        """Drone.py'den gelen GERÇEK yüzdeyi arayüze basar"""
        if not self.compass_calib_active:
            return
        self.compass_calib_progress = pct
        self.calib_web.page().runJavaScript(
            f"document.getElementById('compassProgress').style.width='{pct:.0f}%';"
            f"document.getElementById('compassProgressLabel').textContent='Kalibrasyon: %{pct:.0f}';"
        )
        if pct >= 99.0:
            self.compass_calib_active = False
            self.log_signal.emit("Pusula Kalibrasyonu Başarıyla Tamamlandı!")
            self.calib_web.page().runJavaScript("setStatus('success','✅ Kalibrasyon Bitti!');setStep(4);")

    def _compass_calib_tick(self):
        """Compass kalibrasyon progress güncelleyici"""
        if not self.compass_calib_active:
            self.compass_calib_timer.stop()
            return
        self.compass_calib_progress = min(95.0, self.compass_calib_progress + 2.5)
        self.calib_web.page().runJavaScript(
            f"document.getElementById('compassProgress').style.width='{self.compass_calib_progress:.0f}%';"
            f"document.getElementById('compassProgressLabel').textContent='Kalibrasyon: %{self.compass_calib_progress:.0f}';"
        )

    def cancel_compass_calibration(self):
        """Compass kalibrasyonunu iptal et"""
        self.compass_calib_active = False
        self.compass_calib_timer.stop()
        self.compass_calib_progress = 0.0

        if self.drone_manager.is_connected():
            try:
                # Kalibrasyonu durdur: tüm param'lar 0
                msg = self.drone_manager.drone.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                self.drone_manager.drone.vehicle.send_mavlink(msg)
            except:
                pass

        self.calib_web.page().runJavaScript(
            "setStatus('idle','⛔ Compass kalibrasyonu iptal edildi.');setStep(2);"
            "document.getElementById('compassProgress').style.width='0%';"
            "document.getElementById('compassProgressLabel').textContent='Kalibrasyon başlatılmadı';"
        )
        self.log_signal.emit("Compass kalibrasyonu iptal edildi.")

    def update_calibration_display(self, pitch, roll, heading):
        """Telemetri verisinden kalibrasyon ekranını güncelle"""
        compass_status = "Kalibrasyon aktif — Döndürmeye devam edin" if self.compass_calib_active else "Hazır"
        pct = self.compass_calib_progress
        self.calib_web.page().runJavaScript(
            f"updateCalibration({pitch:.2f},{roll:.2f},{heading:.1f},'{compass_status}',{pct:.1f});"
        )  

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)

        self.drone_manager=DroneManager(Drone)
        self.vtol_manager=DroneManager(VTOL)
        
        self.drone_telemetry=TelemetryManager(self.drone_manager)
        self.vtol_telemetry=TelemetryManager(self.vtol_manager)

        self.active_vehicle="VTOL"

        self.dummy_data = {
            "altitude": None,
            "velocity": None,
            "mode": "DISCONNECTED",
            "arm": None,

            "lat": None,
            "lon": None,

            "pitch": 0.0,
            "roll": 0.0,

            "voltage": 0.0,
            "level": 0,

            "satellites": 0,
            "fix": 0,
            "heading": 0.0,

            "servo_current": 0.0,

            "ch1": 0,
            "ch2": 0,
            "ch3": 0,
            "ch4": 0,
            "ch5": 0,
            "ch6": 0,

            "mission_next": -1,
            "servo_outputs": {},
            "statustext": "",

            "heartbeat_age": None,
            "link_alive": False,
        }
        # Her uçağın son bilinen durumunu tutacak hafızalar
        self.last_vtol_data = self.dummy_data.copy()
        self.last_drone_data = self.dummy_data.copy()

        self.auto_drone_start_enabled = False
        self.auto_drone_started = False
        self.auto_qrtl_started = False

        # QWebEngine haritalarını gereksiz JS kuyruğuyla doldurmamak için.
        self._last_map_update_s = {"VTOL": 0.0, "DRONE": 0.0}
        self._map_update_interval_s = 0.8

        self.setup_top_tabs()
        self.current_manager=self.vtol_manager

        self.setup_pages()

        self.scr_test_btn = QPushButton("SCR KÖPRÜ TESTİ")
        self.message_page.main_layout.addWidget(
            self.scr_test_btn
        )

        self.scr_test_btn.clicked.connect(
            self.test_scr_bridge
        )

        self.radio_bridge = RadioBridgeThread(self.vtol_manager, self.drone_manager)
        self.radio_bridge.log_signal.connect(self.message_page.append_text)

        self.setup_connections()
        self.radio_bridge.start_bridge()
    
        self.ui.menuBar.clicked.connect(self.toggle_menu)
        self.ui.stackedWidget_2.setCurrentWidget(self.get_active_home_page())
        self.ui.stackedWidget.setCurrentWidget(self.connect_page)

        self.refresh_ui_for_active_vehicle()
    def setup_top_tabs(self):
        self.tab_layout=QHBoxLayout()

        self.drone_bttn=QPushButton("KAMİKAZE (DRONE) KONTROLÜ")
        self.drone_bttn.setFont(QFont("Arial",14))
        self.drone_bttn.setStyleSheet("background-color: #7f8c8d; color: white; padding: 10px;")

        self.vtol_bttn=QPushButton("ANA UÇAK (VTOL) KONTROLÜ")
        self.vtol_bttn.setFont(QFont("Arial",14))
        self.vtol_bttn.setStyleSheet("background-color: #2980b9; color: white; padding: 10px;")

        self.tab_layout.addWidget(self.drone_bttn)
        self.tab_layout.addWidget(self.vtol_bttn)

        self.ui.mainBodyContent.layout().insertLayout(0, self.tab_layout)

        self.vtol_bttn.clicked.connect(self.switch_to_vtol)
        self.drone_bttn.clicked.connect(self.switch_to_drone)
    
    def switch_to_vtol(self):
        self.active_vehicle = "VTOL"
        self.vtol_bttn.setStyleSheet("background-color: #2980b9; color: white; padding: 10px; border: 3px solid #f1c40f;")
        self.drone_bttn.setStyleSheet("background-color: #7f8c8d; color: white; padding: 10px; border: none;")
        
        self.ui.camera.setVisible(True)
        
        self.message_page.append_text("SİSTEM:ANA UÇAK (VTOL) EKRANINA GEÇİLDİ")
        self.refresh_ui_for_active_vehicle()
    def switch_to_drone(self):
        self.active_vehicle = "DRONE"
        self.drone_bttn.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; border: 3px solid #f1c40f;")
        self.vtol_bttn.setStyleSheet("background-color: #7f8c8d; color: white; padding: 10px; border: none;")
        
        self.ui.camera.setVisible(False)
        
        if self.ui.stackedWidget_2.currentWidget() in [self.vtol_home_page, self.drone_home_page]:
            self.ui.stackedWidget_2.setCurrentWidget(self.get_active_home_page())
            
        self.message_page.append_text("SİSTEM:KAMİKAZE (DRONE) EKRANINA GEÇİLDİ")
        self.refresh_ui_for_active_vehicle()
    
    def refresh_ui_for_active_vehicle(self):
        if self.active_vehicle == "VTOL":
            active_mgr = self.vtol_manager
            status = "🟢 Connected (VTOL)" if self.vtol_manager.is_connected() else "🔴 Disconnected (VTOL)"
            color = "#00FF00" if self.vtol_manager.is_connected() else "#FF0000"
            
            # VTOL modundaysak ekstra butonları göster
            self.telemetry_page.vtol_quad_bttn.show()
            self.telemetry_page.vtol_plane_bttn.show()
            self.telemetry_page.attack_btn.hide()
        else:
            active_mgr = self.drone_manager
            status = "🟢 Connected (DRONE)" if self.drone_manager.is_connected() else "🔴 Disconnected (DRONE)"
            color = "#00FF00" if self.drone_manager.is_connected() else "#FF0000"
            
            # DRONE modundaysak karmaşa olmasın diye o butonları sakla
            self.telemetry_page.vtol_quad_bttn.hide()
            self.telemetry_page.vtol_plane_bttn.hide()
            self.telemetry_page.attack_btn.hide()
        
        self.settings_page.motor_number.blockSignals(True)
        self.settings_page.motor_number.clear()
        
        if self.active_vehicle == "VTOL":
            # VTOL sekmesindeyken 5. Motoru göster
            self.settings_page.motor_number.addItems([
                "Sağ Ön (Motor A)", "Sol Arka (Motor B)", "Sağ Arka (Motor C)", "Sol Ön (Motor D)", 
                "İleri İtici (Motor 5)", "HEPSİNİ TEST ET (Sırayla)"
            ])
        else:
            # DRONE sekmesindeyken tertemiz 4 motor kalsın
            self.settings_page.motor_number.addItems([
                "Sağ Ön (Motor A)", "Sol Arka (Motor B)", "Sağ Arka (Motor C)", "Sol Ön (Motor D)", 
                "HEPSİNİ TEST ET (Sırayla)"
            ])
        self.settings_page.motor_number.blockSignals(False)

        # Bütün sayfaların "Beynini (Manager)" anında aktif olan uçakla değiştiriyoruz!
        self.connect_page.drone_manager = active_mgr
        self.telemetry_page.drone_manager = active_mgr
        self.settings_page.drone_manager = active_mgr
        self.settings_page.servo_params_loaded = False

        # CONNECT SAYFASININ KABLOLARINI SÖKÜP YENİ UÇAĞA TAKIYORUZ
        try:
            # Önce eski bağlantı komutunu temizle ki iki uçağa birden bağlanmasın
            self.connect_page.connect_button.clicked.disconnect()
            self.connect_page.disconnect_button.clicked.disconnect()
        except TypeError:
            pass # Eğer henüz hiçbir şeye bağlı değilse hatayı yoksay
            
        # Yeni uçağın bağlantı komutlarını mühürle
        self.connect_page.connect_button.clicked.connect(self.connect_page.handle_connect)
        self.connect_page.disconnect_button.clicked.connect(active_mgr.disconnect)

        # Durum yazısını güncelle
        self.connect_page.status_label.setText(status)
        self.connect_page.status_label.setStyleSheet(f"color: {color};")

        if self.active_vehicle == "VTOL":
            self.settings_page.show_drone_calibration(True)

            if self.vtol_manager.is_connected():
                # VTOL bağlıysa, son bilinen VTOL verisini ekrana yapıştır
                self.telemetry_page.telemetry_update(self.last_vtol_data)
                self.settings_page.update_current_display(self.last_vtol_data)
                self.update_map(self.last_vtol_data, "VTOL")
            else:
                # VTOL bağlı değilse sadece VTOL ekranını sıfırla
                self.telemetry_page.telemetry_update(self.dummy_data)
                self.settings_page.update_current_display(self.dummy_data)
                self.vtol_home_page.set_disconnect()

        else:
            self.settings_page.show_drone_calibration(True)

            if self.drone_manager.is_connected():
                # DRONE bağlıysa, son bilinen DRONE verisini ekrana yapıştır
                self.telemetry_page.telemetry_update(self.last_drone_data)
                self.settings_page.update_current_display(self.last_drone_data)
                self.update_map(self.last_drone_data, "DRONE")
            else:
                # DRONE bağlı değilse sadece DRONE ekranını sıfırla
                self.telemetry_page.telemetry_update(self.dummy_data)
                self.settings_page.update_current_display(self.dummy_data)
                self.drone_home_page.set_disconnect()

        # Home ekranındayken VTOL/DRONE sekmesi değişirse doğru home'a geç
        if self.ui.stackedWidget_2.currentWidget() in [self.vtol_home_page, self.drone_home_page]:
            self.ui.stackedWidget_2.setCurrentWidget(self.get_active_home_page())

        # Mission ekranındayken VTOL/DRONE sekmesi değişirse doğru mission'a geç
        if self.ui.stackedWidget_2.currentWidget() in [self.vtol_mission_page, self.drone_mission_page]:
            self.ui.stackedWidget_2.setCurrentWidget(self.get_active_mission_page())

    def setup_pages(self):
        self.connect_page = ConnectPage(self.current_manager)
        self.telemetry_page = TelemetryPage(self.current_manager)
        self.message_page = MessagePage()
        self.camera_page = CameraPage()

        # VTOL ve DRONE için iki ayrı HOME sayfası
        self.vtol_home_page = HomePage()
        self.drone_home_page = HomePage()

        # Eski kodlarda self.home_page geçen yer kalırsa patlamasın diye
        # varsayılan olarak VTOL home'u tutuyoruz.
        self.home_page = self.vtol_home_page

        # VTOL ve DRONE için iki ayrı MISSION sayfası
        # Her mission sayfası kendi home sayfasına çizim yapacak.
        self.vtol_mission_page = MissionPage(self.vtol_home_page, self.vtol_manager)
        self.drone_mission_page = MissionPage(self.drone_home_page, self.drone_manager)

        # Eski kodlar için default mission
        self.mission_page = self.vtol_mission_page

        self.settings_page = SettingPage(self.current_manager)

        self.ui.stackedWidget.addWidget(self.connect_page)
        self.ui.stackedWidget.addWidget(self.settings_page)
        self.ui.stackedWidget.addWidget(self.telemetry_page)
        self.ui.stackedWidget.addWidget(self.message_page)

        self.ui.stackedWidget_2.addWidget(self.vtol_home_page)
        self.ui.stackedWidget_2.addWidget(self.drone_home_page)
        self.ui.stackedWidget_2.addWidget(self.vtol_mission_page)
        self.ui.stackedWidget_2.addWidget(self.drone_mission_page)

    def get_home_page_for(self, vehicle_name):
        if vehicle_name == "VTOL":
            return self.vtol_home_page
        else:
            return self.drone_home_page

    def get_mission_page_for(self, vehicle_name):
        if vehicle_name == "VTOL":
            return self.vtol_mission_page
        else:
            return self.drone_mission_page

    def get_active_home_page(self):
        return self.get_home_page_for(self.active_vehicle)

    def get_active_mission_page(self):
        return self.get_mission_page_for(self.active_vehicle)

    def setup_connections(self):
        self.ui.connect_btn.clicked.connect(lambda: self.ui.stackedWidget.setCurrentWidget(self.connect_page))
        self.ui.telemetry_btn.clicked.connect(lambda: self.ui.stackedWidget.setCurrentWidget(self.telemetry_page))
        self.ui.message_btn.clicked.connect(lambda: self.ui.stackedWidget.setCurrentWidget(self.message_page))
        self.ui.settings_btn.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.settings_page))

        self.ui.camera.clicked.connect(self.camera_page.show)

        self.ui.home_btn.clicked.connect(
            lambda: self.ui.stackedWidget_2.setCurrentWidget(self.get_active_home_page())
        )

        self.ui.mission_btn.clicked.connect(
            lambda: self.ui.stackedWidget_2.setCurrentWidget(self.get_active_mission_page())
        )

        self.vtol_manager.log_signal.connect(lambda msg: self.message_page.append_text(f"[VTOL] {msg}"))
        self.drone_manager.log_signal.connect(lambda msg: self.message_page.append_text(f"[DRONE] {msg}"))
        self.settings_page.log_signal.connect(self.message_page.append_text)
        self.vtol_mission_page.log_signal.connect(
            lambda msg: self.message_page.append_text(f"[VTOL MISSION] {msg}")
        )

        self.drone_mission_page.log_signal.connect(
            lambda msg: self.message_page.append_text(f"[DRONE MISSION] {msg}")
        )

        self.vtol_mission_page.mission_uploaded_signal.connect(
            self.prepare_auto_drone_start_trigger
        )

        # Bağlantı sinyalleri yönlendiriciye gider
        self.vtol_manager.log_signal.connect(lambda msg: self.handle_connection_state(msg, "VTOL"))
        self.drone_manager.log_signal.connect(lambda msg: self.handle_connection_state(msg, "DRONE"))

        # TELEMETRİ VERİLERİ YÖNLENDİRİCİSİ (Sadece o an seçili sekmenin verisi ekrana akar)
        self.vtol_telemetry.telemetry_signal.connect(lambda data: self.route_telemetry(data, "VTOL"))
        self.drone_telemetry.telemetry_signal.connect(lambda data: self.route_telemetry(data, "DRONE"))
    
    def route_telemetry(
        self,
        data,
        vehicle_name
    ):
        # Her durumda son snapshot saklanır.
        if vehicle_name == "VTOL":
            self.last_vtol_data = data
        else:
            self.last_drone_data = data

        link_alive = bool(
            data.get(
                "link_alive",
                False
            )
        )

        # ============================================================
        # HEARTBEAT YOK
        # ============================================================
        if not link_alive:

            # DİKKAT:
            # Burada manager.disconnect() YOK.
            # vehicle.close() YOK.
            # mode/ARM/DISARM komutu YOK.

            if (
                self.active_vehicle
                == vehicle_name
            ):
                self.connect_page.status_label.setText(
                    f"🟠 TELEMETRY LOST "
                    f"({vehicle_name})"
                )

                self.connect_page.status_label.setStyleSheet(
                    "color: #f39c12;"
                )

                # Ekran stale değer göstermesin.
                self.telemetry_page.telemetry_update(
                    data
                )

                self.settings_page.update_current_display(
                    data
                )

            # Servo / drone start / QRTL gibi hiçbir
            # otomasyon stale telemetriyle çalışmayacak.
            return

        # ============================================================
        # HEARTBEAT VAR
        # ============================================================

        if (
            self.active_vehicle
            == vehicle_name
        ):
            self.connect_page.status_label.setText(
                f"🟢 Connected "
                f"({vehicle_name})"
            )

            self.connect_page.status_label.setStyleSheet(
                "color: #00FF00;"
            )

        # Servo açıldı / mission seq alarmı
        self.check_servo_alert(
            data,
            vehicle_name
        )

        # VTOL mission otomasyonları
        if vehicle_name == "VTOL":

            self.check_auto_drone_start_from_vtol_mission(
                data
            )

            self.check_auto_qrtl_from_vtol_mission()

        # Harita sadece canlı GPS ile ve aktif araç için güncellensin.
        # QWebEngine'e her 500 ms'de iki ayrı JS zinciri göndermek yerine
        # update_map kendi içinde de throttle uygular.
        if vehicle_name == self.active_vehicle:
            self.update_map(
                data,
                vehicle_name
            )

        # Aktif ekrana telemetri bas.
        if (
            self.active_vehicle
            == vehicle_name
        ):
            self.telemetry_page.telemetry_update(
                data
            )

            self.settings_page.update_current_display(
                data
            )

    def schedule_drone_start_after_release(self, reason=""):
        if not self.auto_drone_start_enabled:
            return

        if self.auto_drone_started:
            return

        if getattr(self, "auto_drone_start_scheduled", False):
            return

        if not self.drone_manager.is_connected():
            self.message_page.append_text(
                f"SİSTEM: Drone başlatma planlanamadı, drone bağlı değil. Sebep: {reason}"
            )
            return

        self.auto_drone_start_scheduled = True

        delay_ms = 500

        self.message_page.append_text(
            f"SİSTEM: Drone başlatma planlandı. Sebep: {reason}. {delay_ms/1000:.1f} saniye sonra komut gönderilecek."
        )

        QTimer.singleShot(delay_ms, self.start_drone_after_release_timer)

    def start_drone_after_release_timer(self):
        if self.auto_drone_started:
            return

        if not self.drone_manager.is_connected():
            self.message_page.append_text(
                "SİSTEM: Drone başlatılamadı, drone bağlantısı yok."
            )
            self.auto_drone_start_scheduled = False
            return

        self.auto_drone_started = True

        self.message_page.append_text(
            "SİSTEM: Drone'a GUIDED → ARM → AUTO komutları gönderiliyor."
        )

        t = threading.Thread(
            target=self._start_drone_mission_worker,
            daemon=True
        )
        t.start()

    def check_servo_alert(self, data, vehicle_name):
        # Sadece VTOL görevi için takip
        if vehicle_name != "VTOL":
            return

        # VTOL gerçekten AUTO görevde değilse tetikleme yapma
        if str(data.get("mode", "")).upper() != "AUTO":
            return

        page = self.vtol_mission_page

        servo_seq = getattr(page, "servo_alert_seq", None)
        already_sent = getattr(page, "servo_alert_sent", False)

        if servo_seq is None or already_sent:
            return

        try:
            mission_next = int(data.get("mission_next", -1))
        except Exception:
            mission_next = -1

        statustext = str(data.get("statustext", ""))

        # 1) Normal yöntem: mission_next UST_SERVO_AC satırına geldiyse/geçtiyse
        seq_trigger = mission_next >= servo_seq

        # 2) Yedek yöntem: Pixhawk doğru SetServo mission satırını yazdıysa
        # DİKKAT: Sadece kendi servo_seq satırını yakalıyoruz.
        # Herhangi bir SetServo yazısını yakalamıyoruz, çünkü görevde iki servo olabilir.
        text_trigger = f"Mission: {servo_seq} SetServo" in statustext

        if seq_trigger or text_trigger:
            page.servo_alert_sent = True
            self.message_page.append_text(
                f"🚨 ÜST SERVO AÇILDI! mission_next={mission_next}, servo_seq={servo_seq}, text={statustext}"
            )
            QApplication.beep()
            # Servo açıldıktan kısa süre sonra drone görevini başlat
            self.schedule_drone_start_after_release("UST_SERVO_AC / doğru SetServo yakalandı")

    def prepare_auto_drone_start_trigger(self, mission_data):
        """
        VTOL görevi yüklendiğinde otomatik drone başlatma sistemi hazırlanır.
        """

        self.auto_drone_started = False
        self.auto_qrtl_started = False
        self.auto_drone_start_scheduled = False

        self.vtol_mission_page.servo_alert_sent = False

        seq = self.get_vtol_drone_start_seq()

        if seq is None:
            self.message_page.append_text(
                "SİSTEM: VTOL görevinde UST_SERVO_AC sonrası UZUN_BEKLE bulunamadı. "
                "Ana tetik yine UST_SERVO_AC olacak, UZUN_BEKLE yedek tetik pasif kalabilir."
            )
        else:
            self.message_page.append_text(
                f"SİSTEM: Drone otomatik başlatma hazır. "
                f"UST_SERVO_AC ana tetik, VTOL seq={seq} UZUN_BEKLE yedek tetik."
            )

    def get_vtol_drone_start_seq(self):
        """
        VTOL görev listesinde:
        UST_SERVO_AC'tan sonra gelen ilk UZUN_BEKLE satırını bulur.

        Not:
        HOME ArduPilot tarafından ayrı tutulur.
        YKI mission listesi Pixhawk/DroneKit tarafında 0-based sequence kullanır.
        """

        mission = getattr(self.vtol_mission_page, "mission_data", [])

        if not mission:
            return None

        modes = [
            str(wp.get("mode", "")).strip().upper()
            for wp in mission
        ]

        if "UST_SERVO_AC" not in modes:
            return None

        servo_index = modes.index("UST_SERVO_AC")

        for idx in range(servo_index + 1, len(modes)):
            if modes[idx] == "UZUN_BEKLE":
                return idx+1

        return None

    def check_auto_drone_start_from_vtol_mission(self, data=None):
        """
        VTOL telemetrisi geldikçe çalışır.
        VTOL, UST_SERVO_AC sonrası UZUN_BEKLE satırına geldiyse
        drone'un önceden yüklenmiş görevini başlatır.

        ÖNEMLİ:
        Eski sürüm sadece current_seq == target_seq iken çalışıyordu.
        Bu sürüm current_seq >= target_seq olunca da çalışır.
        Böylece telemetri o anı kaçırsa bile drone başlatılır.
        """

        if not self.auto_drone_start_enabled:
            return

        if self.auto_drone_started:
            return

        if not self.vtol_manager.is_connected():
            return

        if not self.drone_manager.is_connected():
            return

        target_seq = self.get_vtol_drone_start_seq()

        if target_seq is None:
            return

        try:
            vtol = self.vtol_manager.drone
            current_mode = vtol.get_mode()

            if current_mode != "AUTO":
                return

            current_seq = -1

            # Öncelik: telemetri datasındaki mission_next
            if data is not None:
                try:
                    current_seq = int(data.get("mission_next", -1))
                except Exception:
                    current_seq = -1

            # Yedek: MISSION_CURRENT listener
            if current_seq < 0:
                current_seq = int(getattr(vtol, "current_mission_seq", -1))

            # Son yedek: DroneKit commands.next
            if current_seq < 0 and vtol.vehicle is not None:
                current_seq = int(vtol.vehicle.commands.next)

        except Exception as e:
            self.message_page.append_text(f"SİSTEM: VTOL görev sırası okunamadı: {e}")
            return

        # Eski kodda sadece == vardı.
        # Bu yüzden UZUN_BEKLE anı kaçarsa drone başlamıyordu.
        if current_seq < target_seq:
            return

        self.message_page.append_text(
            f"SİSTEM: VTOL drone başlatma noktasına geldi/geçti. "
            f"current_seq={current_seq}, target_seq={target_seq}."
        )

        self.schedule_drone_start_after_release("UZUN_BEKLE yakalandı/geçildi")

    def _start_drone_mission_worker(self):
        """
        Drone görev başlatma işlemini arayüz donmasın diye arka planda yapar.
        """

        try:
            self.drone_manager.log_signal.emit(
                "Drone otomatik başlatma işçisi başladı. Komutlar gönderiliyor..."
            )

            if not self.drone_manager.is_connected():
                self.drone_manager.log_signal.emit(
                    "Drone otomatik başlatılamadı: Drone bağlantısı yok."
                )
                self.auto_drone_started = False
                self.auto_drone_start_scheduled = False
                return

            ok = self.drone_manager.drone.start_carrier_mission(
                start_seq=0,
                force_disable_arming_check=False
            )

            if ok:
                self.drone_manager.log_signal.emit(
                    "Drone görevi otomatik başlatıldı. AUTO moda geçmesi gerekiyor."
                )
            else:
                self.drone_manager.log_signal.emit(
                    "Drone otomatik görev başlatma başarısız oldu."
                )
                self.auto_drone_started = False
                self.auto_drone_start_scheduled = False

        except Exception as e:
            self.drone_manager.log_signal.emit(
                f"Drone otomatik görev başlatma hatası: {e}"
            )
            self.auto_drone_started = False
            self.auto_drone_start_scheduled = False

    def handle_connection_state(self,message,vehicle_name):
        msg = message.lower()
        if "drone connected successfully" in msg:
            # Yeni bağlantıda servo function/reverse bilgileri yeniden okunsun.
            # SITL'den gerçek uçağa geçerken eski SERVOx_FUNCTION bilgisi ekranda kalmasın.
            if self.active_vehicle == vehicle_name:
                self.settings_page.servo_params_loaded = False
                self.settings_page.refresh_servo_params_from_vehicle(force=True)
            if vehicle_name == "VTOL":
                self.vtol_telemetry.start()
            else:
                self.drone_telemetry.start()
            # Arayüz güncel uçağa aitse rengini değiştir
            if self.active_vehicle == vehicle_name:
                self.connect_page.status_label.setText(f"🟢 Connected ({vehicle_name})")
                self.connect_page.status_label.setStyleSheet("color: #00FF00;")
                
        elif "disconnected successfully" in msg:
            # Manuel disconnect ise telemetry thread durabilir.
            # Auto-reconnect/COM dead denemesinde thread yaşamaya devam eder.
            if vehicle_name == "VTOL":
                self.vtol_telemetry.stop()
            else:
                self.drone_telemetry.stop()

            if self.active_vehicle == vehicle_name:
                self.connect_page.status_label.setText(
                    f"🔴 Disconnected ({vehicle_name})"
                )
                self.connect_page.status_label.setStyleSheet(
                    "color: #FF0000;"
                )
                self.get_home_page_for(vehicle_name).set_disconnect()
                self.settings_page.servo_params_loaded = False

        elif "error connecting to drone" in msg:
            # Bu mesaj auto-reconnect denemesinin de sonucu olabilir.
            # TelemetryManager durdurulmaz; heartbeat kaybını izlemeye ve
            # sonraki reconnect denemesini başlatmaya devam eder.
            if self.active_vehicle == vehicle_name:
                self.connect_page.status_label.setText(
                    f"🟠 Reconnecting ({vehicle_name})"
                )
                self.connect_page.status_label.setStyleSheet(
                    "color: #f39c12;"
                )

    def test_scr_bridge(self):
        if not self.vtol_manager.is_connected():
            self.message_page.append_text(
                "SCR TEST HATA: VTOL bağlı değil."
            )
            return

        vehicle = self.vtol_manager.drone.vehicle

        if vehicle is None:
            self.message_page.append_text(
                "SCR TEST HATA: VTOL vehicle yok."
            )
            return

        # TEST HEDEFİ
        lat_fraction = 0.8041738
        lon_fraction = 0.3565212

        try:
            # ÖNCE koordinatlar
            vehicle.parameters["SCR_USER1"] = lat_fraction
            vehicle.parameters["SCR_USER2"] = lon_fraction
            vehicle.flush()

            self.message_page.append_text(
                f"SCR TEST: VTOL SCR1={lat_fraction}, SCR2={lon_fraction} yazıldı."
            )

            # EN SON handshake
            vehicle.parameters["SCR_USER3"] = 1.0
            vehicle.flush()

            self.message_page.append_text(
                "SCR TEST: VTOL SCR_USER3=1 gönderildi."
            )

        except Exception as e:
            self.message_page.append_text(
                f"SCR TEST HATA: {e}"
            )
    
    def get_vtol_qrtl_seq(self):
        """
        VTOL mission_data içinde QRTL satırını bulur.
        HOME mission listesine eklenmez; sequence 0-based'tir.
        """

        mission = getattr(self.vtol_mission_page, "mission_data", [])

        if not mission:
            return None

        for idx, wp in enumerate(mission):
            mode = str(wp.get("mode", "")).strip().upper()

            if mode == "QRTL":
                return idx+1

        return None

    def check_auto_qrtl_from_vtol_mission(self):
        """
        VTOL telemetrisi geldikçe çalışır.
        VTOL mission içindeki QRTL satırına gelince gerçek VehicleMode("QRTL") gönderir.
        """

        if self.auto_qrtl_started:
            return

        if not self.vtol_manager.is_connected():
            return

        target_seq = self.get_vtol_qrtl_seq()

        if target_seq is None:
            return

        try:
            vtol = self.vtol_manager.drone

            current_seq = int(getattr(vtol, "current_mission_seq", -1))

            if current_seq < 0 and vtol.vehicle is not None:
                current_seq = int(vtol.vehicle.commands.next)

            current_mode = vtol.get_mode()

        except Exception as e:
            self.message_page.append_text(f"SİSTEM: QRTL için VTOL görev sırası okunamadı: {e}")
            return

        if current_mode != "AUTO":
            return

        # QRTL SADECE GERÇEK QRTL MISSION ITEM'INA GELİNCE TETİKLENSİN.
        if current_seq != target_seq:
            return

        self.auto_qrtl_started = True

        self.message_page.append_text(
            f"SİSTEM: VTOL QRTL seq={target_seq} noktasına geldi. "
            "Gerçek QRTL mode gönderiliyor."
        )

        t = threading.Thread(
            target=self._start_vtol_qrtl_worker,
            daemon=True
        )
        t.start()


    def _start_vtol_qrtl_worker(self):
        """
        Gerçek QRTL mode geçişini arka planda yapar.
        """

        try:
            ok = self.vtol_manager.drone.qrtl()

            if ok:
                self.vtol_manager.log_signal.emit(
                    "VTOL gerçek QRTL moda geçti: 4 dikey motorla home dönüş/iniş başladı."
                )
            else:
                self.vtol_manager.log_signal.emit(
                    "VTOL QRTL mode geçişi başarısız."
                )
                self.auto_qrtl_started = False

        except Exception as e:
            self.vtol_manager.log_signal.emit(
                f"VTOL QRTL otomatik geçiş hatası: {e}"
            )
            self.auto_qrtl_started = False

    def toggle_menu(self):
        width=self.ui.leftmenuSubcontainer.width()
        if width==127:
            self.ui.leftmenuSubcontainer.setFixedWidth(50)
        else:
            self.ui.leftmenuSubcontainer.setFixedWidth(127)
    
    def update_map(self, data, vehicle_name=None):
        lat = data.get("lat")
        lon = data.get("lon")
        heading = data.get("heading", 0)

        if lat is None or lon is None:
            return

        if vehicle_name is None:
            vehicle_name = self.active_vehicle

        # Sadece aktif aracın görünür haritalarını güncelle.
        # Son telemetri zaten last_*_data içinde tutuluyor; sekme değişince
        # refresh_ui_for_active_vehicle() görünür haritayı yeniden konumlandırır.
        if vehicle_name != self.active_vehicle:
            return

        now = time.monotonic()
        last = self._last_map_update_s.get(
            vehicle_name,
            0.0
        )
        if now - last < self._map_update_interval_s:
            return

        self._last_map_update_s[vehicle_name] = now

        home_page = self.get_home_page_for(vehicle_name)
        mission_page = self.get_mission_page_for(vehicle_name)

        home_page.set_connected(lat, lon, heading)
        mission_page.set_center(lat, lon)

    def closeEvent(self, event):
        if hasattr(self, "radio_bridge"):
            self.radio_bridge.stop_bridge()

        if hasattr(self, "vtol_telemetry"):
            self.vtol_telemetry.stop()

        if hasattr(self, "drone_telemetry"):
            self.drone_telemetry.stop()

        event.accept()

def main():
    app=QApplication(sys.argv)
    base_path=os.path.dirname(__file__)
    cach_dir=os.path.join(base_path,"offline_maps")

    default=QWebEngineProfile.defaultProfile()
    default.setHttpCacheType(QWebEngineProfile.DiskHttpCache)
    default.setCachePath(cach_dir)
    default.setHttpCacheMaximumSize(1024 * 1024 * 500)
    window=MainWindow()
    window.show()
    sys.exit(app.exec_())
main()
