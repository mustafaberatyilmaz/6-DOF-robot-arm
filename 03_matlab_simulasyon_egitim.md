# FAZ 2 — MATLAB Simülasyon: Öğretici Rehber

> **Kod dosyası:** [`03_matlab_simulasyon.m`](03_matlab_simulasyon.m)
> **Ön koşul:** MATLAB R2016b+ (Peter Corke RTB gerekmez)
> **Çalıştırma:** `>> robot_arm_sim`

---

## 1. Simülasyon Ne Yapıyor?

```
[6 slider] → eklem açıları (q) → DH ileri kinematik → 3B görsel + TCP koordinatı
```

Her slider hareketi:
1. `q` vektörünü günceller
2. `forwardKin()` çağrılır → 7 frame matrisi hesaplanır
3. Eski `patch/surf` nesneleri silinir, yenileri çizilir

---

## 2. DH Parametreleri

`02_model_DH_dinamik.md`'den alınan gerçek değerler:

| Link | d (mm) | a (mm) | α (rad) | Fiziksel anlam |
|:---:|---:|---:|:---:|---|
| 1 | **100** | 0 | −π/2 | Taban kolonu (base 60 + J1 motor 40) |
| 2 | 0 | **138** | 0 | Üst kol (omuz bracketi uzunluğu) |
| 3 | 0 | 0 | −π/2 | Dirsek bağlantısı (mesafesiz) |
| 4 | **130** | 0 | +π/2 | Önkol tüpü |
| 5 | 0 | 0 | −π/2 | Bilek pitch (küresel bilek) |
| 6 | **50** | 0 | 0 | TCP plakası |

---

## 3. İleri Kinematik — Adım Adım

### 3.1 DH Dönüşüm Matrisi (her link için)

```matlab
function A = dhA(theta, d, a, alpha)
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
A = [ ct, -st*ca,  st*sa, a*ct;
      st,  ct*ca, -ct*sa, a*st;
       0,     sa,     ca,    d;
       0,      0,      0,    1];
end
```

Bu 4×4 matris **rotasyon + öteleme** bilgisini tek seferde taşır:
- Sol üst 3×3: rotasyon (R)
- Sağ sütun 3×1: pozisyon (t)

### 3.2 Zincirleme çarpım

```
T_TCP = A₁(q₁) × A₂(q₂) × A₃(q₃) × A₄(q₄) × A₅(q₅) × A₆(q₆)
```

Kod:
```matlab
T = eye(4);
for i = 1:6
    T = T * dhA(q(i), DH.d(i), DH.a(i), DH.alpha(i));
    origins(i+1,:) = T(1:3,4)';   % o anki eklem konumu
end
```

`T(1:3,4)` → o frame'in dünya koordinatındaki orijini = **eklem konumu**.

### 3.3 Test Doğrulaması

Simülasyonu çalıştırınca panelde test:

| Poz | Beklenen TCP | Düğme |
|---|---|---|
| q = [0 0 0 0 0 0] | [268, 0, 50] mm | "Test 1 (yatay)" |
| q = [0 −90° 0 0 0 0] | [0, 0, 318] mm | "Test 2 (dik)" |

Değerler ekranda **TCP (mm)** satırında görünür.

---

## 4. Görselleştirme Katmanları

### 4.1 Linkler — neden hangi şekil?

| Link | Şekil | Neden |
|---|---|---|
| 1 (taban kolon) | `drawCyl` r=32 | J1 silindirik motor housing |
| 2 (üst kol) | `drawBox` 30×22 | Omuz bracketları dikdörtgen profil |
| 4 (önkol) | `drawCyl` r=16 | Önkol tüpü silindirik |
| 6 (TCP) | `drawCyl` r=12 | Gripper mili |

### 4.2 Eklem küreleri

Her `origins(i)` noktasına küre çizilir → servo/bilek bloklarını temsil eder.

### 4.3 Frame eksenleri

Her DH frame'e `drawFrame()` ile küçük eksen üçlüsü eklenir:
- **Kırmızı = X**, **Yeşil = Y**, **Mavi = Z**

Kolu döndürürken bu vektörlerin nasıl döndüğünü izlemek, DH konvansiyonunu sezgisel anlamak için etkilidir.

---

## 5. `vecToRot()` — "Lokal Z'yi istenen yöne çevir"

Silindir ve kutu nesneleri MATLAB'da varsayılan +Z yönünde çizilir. Ama iki eklem arasındaki vektör herhangi bir yöne bakabilir.

**Çözüm:** Rodrigues rotasyon formülü ile `[0,0,1]`'i `v_yön`'e döndüren R matrisi bul.

```
k   = cross([0,0,1], vN)   (rotasyon ekseni)
ang = acos(dot([0,0,1], vN))
R   = Rodrigues(k, ang)
```

Bu rotasyonu nesnenin köşe noktalarına uygulayınca nesne doğru yöne döner.

---

## 6. Slider ↔ `q` Akışı

```
addlistener(slider, 'Value', 'PostSet', @onSlider)
    └─> onSlider(idx): q(idx) = slider.Value → drawRobot()
```

`addlistener` kullanımı önemli: MATLAB'ın varsayılan `Callback`'i sadece fareyi bırakınca tetiklenir; `PostSet` listener **sürükleme sırasında** sürekli tetikler → gerçek zamanlı güncelleme.

---

## 7. Sınırlar ve Kısıtlar

- **Çakışma kontrolü yok:** Linkler birbirine geçebilir (FAZ 3/4'te eklenebilir).
- **Dinamik yok:** Sadece kinematik (FAZ 3'te torque hesabı).
- **IK yok:** Slider ile FK; tersine IK için FAZ 3'te `ikine` veya kendi analitik çözüm eklenecek.
- **Frame sayısı:** Tüm 7 frame görünür. Karmaşık görünüyorsa `drawFrame` satırlarını `%` ile kapatabilirsin.

---

## 8. Sık Sorulan Sorular

**S: `isgraphics(S.linkH)` neden var?**
A: Her `drawRobot()` çağrısında eski nesneleri temizlemek için — yoksa eski linkler ekranda kalır.

**S: `drawnow limitrate` ne yapar?**
A: Her slider olayında tam `drawnow` yapmak yerine MATLAB'ın kendi kadansında ekranı günceller. Yüksek framerate'te donmayı engeller.

**S: Silindirler her zaman pürüzsüz değil, neden?**
A: `cylinder(r, 24)` → 24 kenarlı çokgen. Daha pürüzsüz için 36-48'e çıkar (hafif yavaşlar).

**S: Robotics Toolbox ile karşılaştırma?**
A: RTB'nin `teach()` fonksiyonu benzer slider UI sunar. FAZ 2 ek adım: RTB kurulumu sonrası `02_model_DH_dinamik.md §MATLAB doğrulama` kodunu çalıştırıp her iki TCP sonucunun eşleştiğini doğrula.

---

## 9. Sonraki Adımlar (FAZ 3 için Hazırlık)

| Adım | Ne eklenecek |
|---|---|
| Ters kinematik | Slider yerine hedef XYZ gir → q hesapla |
| Çalışma uzayı | 30000 rastgele q → scatter3 ile nokta bulutu |
| Animasyon | İki poz arası lineer `q` interpolasyonu + `drawnow` döngüsü |
| RTB entegrasyonu | `SerialLink + teach()` ile bu kodun sonuçlarını doğrula |

---

*FAZ 2 — 2026-04-26*
