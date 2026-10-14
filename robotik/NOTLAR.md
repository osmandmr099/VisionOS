# Robotik Dersi – Proje Hafızası

> Bu dosyada yalnızca kullanıcının teyit ettiği bilgiler bulunur.
> Claude'un analiz ve önerileri, onaylanana kadar buraya "karar" olarak yazılmaz.
> Kullanıcı tahmin istemiyor: yalnızca kaynaklı bilgi verilecek, bilinmeyen değerler "ölçülmeli" diye belirtilecek.

## Projenin amacı
GPS ile noktadan noktaya giden bir otonomi **değil**. Görüntüye dayalı, kişiye göre konum alan bir otonom drone:

1. Önceden verilen birkaç kişinin görselleriyle o kişileri tanır (re-ID benzeri).
2. Tanıdığı kişiler dışında gördüğü herkesi **yabancı** olarak işaretler.
3. Tanıdığı kişilerden biri önceden belirlenmiş hareketi (iki eli kafa hizasına kaldırmak) yaparsa o kişiyi **hedef** seçer ve takip eder.
4. Hedefin yüzünü ve baktığı yönü algılar. Engellerden kaçınarak sürekli kişinin **baktığı yönde, yüzünün karşısında** konum alır.

## Donanım (teyitli)
| Parça | Not |
|---|---|
| Emax Eco 2207 2400KV motor seti | **İlk nesil** Eco (Eco II değil) |
| 5030 üç kanatlı pervane | 5 inç |
| 4S1P Tenpower 50XG 21700 Li-ion (5 Ah) | Dengeleme kablolu, punta kaynaklı paket |
| SpeedyBee F405 AIO 40A Bluejay | Uçuş kontrolcüsü + 4'ü 1 arada ESC |
| Raspberry Pi 5 4GB | Görev bilgisayarı. **USB-C'den** beslenecek |
| Pi Camera Module 3 Wide | Otonomi (algılama) için |
| Mini560 + soğutucu blok | Pi5 için 5 V düşürücü |
| VL53L0X × 5 | **Çevre** algılama için: ön (kamerayla aynı hizada), arka, sol, sağ, yukarı. Yükseklik için kullanılmayacak, L1X'e gerek görülmedi |
| TCA9548A | VL53L0X'ler için I2C çoklayıcı |
| MicoAir MTF-02P | Optik akış + aşağı bakan lidar (yükseklik) |
| Gövde | 3D baskı, **organik tasarım** tercih ediliyor. Karbon gövde **mümkün değil** |
| (Opsiyonel) Raspberry Pi AI HAT+ 13 TOPS | Gerekirse eklenecek |

## Kısıtlar ve kararlar (teyitli)
- Kumanda/alıcı **yok**
- Pusula **yok**
- GPS **yok**. Tasarım GPS'siz yapılacak
- Nesne takip algoritmaları (ör. ByteTrack) **kullanılmayacak**
- Yere optik akış (MTF-02P) bakıyor
- Kablolar, montaj malzemesi, şarj cihazı gibi yardımcı parçalar mevcut. Bunlar için öneri yapmaya gerek yok.

## Drive kaynakları
- `Intelligence Robotics` klasörü: ArduPilot SITL uçuş logları (00000001–07.BIN), `flights.json`, `README.md`
  - Rate PID deneyleri, GPS yok, optik akış + mesafe sensörü, ALT_HOLD
  - README'de adı geçen `analiz.py` ve `uret_sitl_log.py` klasörde yok
- Okunamayan Samsung Notes dosyaları: `Robotics project_2026-09-30.note`, `Ders Programı_2026-09-25.note`

## Gövde
- Şartlar: **ücretsiz**, **organik tasarım**, 3D baskı, 5 inç
- Beğenilen aday: [3D Printable 5-inch Drone Frame – PecaJosef (MakerWorld 411258)](https://makerworld.com/en/models/411258-3d-printable-5-inch-drone-frame)
  - Ücretsiz, "generative design" etiketli
  - SLS için tasarlanmış, FDM'de de basılabiliyor
  - İlandaki profil: 0,2 mm katman, 3 duvar, %60 gyroid, PLA'da yaklaşık 6,7 saat / 127 g
- Aether 4'ün 5 inç sürümü bulundu: [BM Aether 4 XL – srhzrl (MakerWorld 593258)](https://makerworld.com/en/models/593258-bm-aether-4-xl-the-5-6-inch-fpv-drone-frame)
  - Ücretsiz, 5–6 inç, BM Aether 4'ün remix'i
  - İlana göre uzatılmış 16×16 motor yuvaları var, FC yuvası 2 adet 30,5×30,5
  - Yorumlarda PLA, ASA ve PA6-GF ile basıldığı yazıyor
  - AIO'muz 25,5×25,5 olduğu için FC yuvası uymuyor; adaptör ya da yuva değişikliği gerekiyor
  - Lisans (srhzrl sayfası): dosyaların ve türev çalışmaların (remix dahil) paylaşılması/dağıtılması yasak
  - **SEÇİLDİ.** Üzerine Pi5 ve AIO yerleşimi için değişiklik yapılacak
  - STEP durumu: XL için STEP yok; remix sahibi bir yorumda STL üzerinden yeniden çalıştığını söylüyor. Orijinal Aether 4'ün (4") kaynağı Onshape'te, ama kullanıcı kontrol etti: **o da üçgenlerden oluşuyor**
  - Kullanıcı şartı: düzenlenecek modelde üçgen yüzeyler olmamalı
- Proje **ticari amaçlı değil**
- Aether ailesinin diğer sürümleri: orijinal BM Aether 4 (4–4,5"), Aether 2.5/3 (hashi), Thingiverse "aether 4" (sdamir2, O4 Pro revizyonu), ayrıca alıcı/GPS adaptörleri (madcatcad, user_1936875010)
- Beğenilmeyenler: Aether 4, Grasshopper5, katlanır modüler 5", Elecrow ModuFrame-5, Drone Frame V1 ve Cults3D/Printables/Thingiverse/Thangs'tan önerilen diğer 9 aday

## STL -> STEP dönüştürücü
- `tools/stl2step/` (Python, OpenCASCADE). Kullanım ve yöntem için README'ye bakın.
- Aether 4 XL STL'si: 166.049 üçgen, birimi **0,1 mm** (16×16 motor deseni 160,0 birim ölçüldü)
- Komut: `stl2step.py govde.stl govde.step --olcek 0.1 --hedef-ucgen 40000 --sapma`
- Sonuç (14.10.2026):
  - Tek geçerli katı, 20.604 yüz (1.203 düzlem, 45 silindir, 19.356 B-spline)
  - 39 deliğin 38'i silindir olarak kuruldu
  - Hacim farkı %0,2
  - Sapma ortalama 0,013 mm, %99'u 0,064 mm'nin altında
- STEP dosyası lisans nedeniyle repoya konmadı; konuşma üzerinden kullanıcıya gönderildi

- 20 bin üçgen sürümü (`--hedef-ucgen 20000`):
  - 10.666 yüz, tek geçerli katı, hacim farkı %0,37
  - Sapma ortalama 0,025 mm, %99'u 0,12 mm'nin altında
  - 35 delik silindir olarak kuruldu
  - Kullanıcıya gönderildi
- Büyük yüzey modu (`buyuk_yuzey.py`, deneysel): bölge başına tek B-spline yüzey
  - Son deneme: 2.734 yüz, ama geçersiz katı (16 kabuk, 508 açık kenar), hacim %90
  - Keskin kırıntılardan 754 tane 3 kenarlı yüz çıkıyor
  - Kullanılabilir değil
- Kullanıcı STEP'i Fusion'da açtı: tek gövde olarak yüklendi, yüzler dörtgen yamalar hâlinde görünüyor; kesme işlemi çalıştı (14.10.2026)

## Açık konular
- STEP üzerinde Pi5 ve AIO yuvaları için düzenleme (kesme/ekleme) işlemlerinin Fusion'da denenmesi
