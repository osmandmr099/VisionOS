# stl2step

STL dosyasını, üçgen yüz içermeyen düzenlenebilir bir STEP katısına çevirir.
- Düz bölgeler tek **düzlem** yüz olur.
- Delikler gerçek **silindir** olarak yeniden kesilir.
- Organik bölgeler dört kenarlı **bikübik B-spline** yüzlere dönüşür.

## Kurulum
```
python -m venv venv
venv/bin/pip install -r requirements.txt
```
Windows'ta `venv/bin/` yerine `venv\Scripts\` kullanılır.

## Kullanım
```
venv/bin/python stl2step.py girdi.stl cikti.step --olcek 0.1 --hedef-ucgen 40000 --sapma
```

| Seçenek | Ne yapar |
|---|---|
| `--olcek K` | Koordinatları K ile çarpar. 0,1 mm birimli bir STL'yi mm'ye çevirmek için `0.1`. |
| `--hedef-ucgen N` | Dönüştürmeden önce ağı topolojiyi koruyarak N üçgene indirir. Yüz sayısını ve dosya boyutunu belirleyen ana ayar budur. |
| `--keskin-aci 30` | Bu açıdan büyük kenarlar keskin kalır. Bölgeler ve delikler bu kenarlarla ayrılır. |
| `--delik-yok` | Delikleri silindire çevirmez. |
| `--sapma` | Sonucun girdi ağından iki yönlü sapmasını ölçer. |

## Yöntem
1. Ağ onarılır: manifold olmayan küçük kusurlar silinir ve doldurulur.
2. Keskin kenarlarla çevrili, tam çevreli, içe bakan silindir bölgeleri delik olarak bulunur. Ağdan çıkarılıp tıkanır.
3. Ağ, topolojisi korunarak indirgenir (pymeshlab quadric edge collapse).
4. Aynı düzlemdeki komşu üçgenler tek düzlem yüz olur.
5. Organik üçgenler komşularıyla eşlenip dörtgen olur. Eşleme önce açgözlü yapılır, sonra artırıcı yollarla eşsiz kalanlar azaltılır.
   - Her kenar, uç normallerinden kurulan tek bir kübik eğridir ve iki yüz tarafından paylaşılır.
   - Dörtgenin içi ayrık Coons formülüyle doldurulur.
   - Eşlenemeyen üçgenler 3 dörtgene bölünür.
6. Yüzler OpenCASCADE ile dikilir ve katıya çevrilir. Delikler gerçek silindirlerle kesilir, STEP (AP214) yazılır.

## Sınırlar
- Organik bölgedeki yüz sayısı, indirgenmiş üçgen sayısının yaklaşık yarısı kadardır. Yüzlerin dizilişi üçgenlemeyi izler.
- Yalnızca keskin kenarla ayrılmış tam delikler silindire çevrilir. Pahlı ya da yuvarlatılmış ağızlı delikler organik kalır.
