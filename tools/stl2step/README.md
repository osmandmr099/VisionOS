# stl2step

STL dosyasını, üçgen yüz içermeyen, düzgün yüzeyli bir STEP dosyasına çevirir. Çıktı dört kenarlı bikübik B-spline yüzeylerden oluşan geçerli bir katıdır.

## Kurulum
```
python -m venv venv
venv/bin/pip install -r requirements.txt
```

## Kullanım
```
venv/bin/python stl2step.py girdi.stl cikti.step --sapma
```

| Seçenek | Ne yapar |
|---|---|
| `--hedef-ucgen N` | Dönüştürmeden önce ağı N üçgene indirir. Yüz sayısını ve dosya boyutunu düşürür. |
| `--keskin-aci 30` | Bu açıdan keskin kenarları düz bırakır (delik ağızları, düz yüzeyler). |
| `--sapma` | Üretilen yüzeyin girdi ağından ne kadar saptığını ölçer. |

## Yöntem
1. Her üçgen; köşe, kenar ortası ve merkez noktası kullanılarak 3 dörtgene bölünür. Bütün noktalar orijinal yüzeyin üzerinde kalır.
2. Kenar eğrileri, uçlardaki normallerden kübik Bézier olarak kurulur (PN kuralı). Komşu yüzler aynı eğriyi paylaştığı için gövde su geçirmezdir.
3. İç kontrol noktaları ayrık Coons formülüyle bulunur.
4. Yüzler OpenCASCADE ile dikilir, katıya çevrilir ve STEP olarak yazılır.

## Sınırlar
- Her girdi üçgeni 3 yüz üretir. Yüz sayısı fazla olur ve yüzlerin dizilişi orijinal üçgenlemeyi izler.
- Girdi ağı su geçirmez (manifold) olmalıdır.
