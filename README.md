# Çift İHA Destekli Yer Kontrol İstasyonu (GCS)

Bu proje, özel bir görev/yarışma konsepti için geliştirilmiş, eşzamanlı olarak **iki farklı İnsansız Hava Aracını (İHA)** takip ve kontrol edebilen bir Yer Kontrol İstasyonu (Ground Control Station) yazılımıdır. 

Sistem, mevcut görev gereksinimlerini karşılamak üzere tasarlanmış olup, gelecekteki ihtiyaçlara göre **sürekli geliştirilmeye açık ve modüler** bir yapıda kurgulanmıştır.

## 🚀 Proje Hakkında
Bu yer kontrol istasyonu, sahada internetin olmadığı durumlarda bile operasyonel kalabilmek ve birden fazla hava aracını aynı arayüz üzerinden güvenle yönetebilmek amacıyla geliştirilmiştir. Yalnızca telemetri takibi yapmakla kalmaz, aynı zamanda donanım seviyesinde otopilot konfigürasyonlarına da imkan tanır.

### Temel Özellikler
* **Çift İHA Yönetimi:** İki farklı İHA'nın telemetri verilerini aynı anda alabilme, harita üzerinde eşzamanlı konum takibi yapabilme.
* **Canlı Görüntü Aktarımı (GStreamer):** Hava araçlarından Gst-launch entegrasyonu ile düşük gecikmeli (low-latency) anlık kamera görüntüsü alabilme.
* **Çevrimdışı Harita Desteği:** Arazi şartlarında internet bağlantısı olmadan harita verilerini görüntüleyebilme (Offline Maps).
* **Anlık Görev (Mission) Yönetimi:** İHA'lar için rota/görev oluşturma, uçuş öncesi veya uçuş esnasında anlık olarak araca görev yükleme ve bu görevleri kaydetme.
* **Bağımsız Çalıştırılabilir Sürüm (EXE):** Sistemde Python kurulu olmasa dahi Windows ortamında doğrudan çalıştırılabilen derlenmiş versiyon desteği.

### Donanım ve Kalibrasyon Yetenekleri
Uygulama, standart uçuş takibinin ötesine geçerek aşağıdaki donanım kontrollerini de arayüz üzerinden sağlar:
* **Sensör Kalibrasyonları:** İvmeölçer (Level), Pusula (Compass) ve Radyo (Kumanda) kalibrasyonlarını doğrudan GCS üzerinden yapabilme.
* **Aktüatör Kontrolü:** Araç üzerindeki servoların anlık kontrolü ve test edilmesi.
* **Detaylı Telemetri Analizi:** Araç durumunun ve kritik uçuş verilerinin anlık, kesintisiz takibi.

## 📈 Geliştirme ve İleriye Dönük Vizyon
Bu proje tek seferlik bir ürün değil, geliştirilmeye devam edilebilir bir altyapı olarak tasarlanmıştır. Gelecekte otonom sürü formasyon algoritmalarının arayüze eklenmesi veya görüntü işleme yeteneklerinin entegre edilmesi gibi geliştirmelere açıktır.