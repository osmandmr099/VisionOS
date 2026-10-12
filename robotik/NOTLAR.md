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

## Açık konular
- Gövde seçimi/tasarımı (organik, baskı, 5 inç)
