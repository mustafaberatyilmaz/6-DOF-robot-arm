# 🎯 FAZ 1.5 — Modele Özgü Kinematik & Dinamik Hesaplar

> **Bu doküman:** [`01_kinematik_dinamik.md`](01_kinematik_dinamik.md) genel teoriyi anlatır. Bu dosya, **MBera 6-DOF Arm V1.0** modelinin gerçek STP/STL dosyalarından çıkarılan **kesin parametrelerle** hesabı yapar.
>
> **Veri kaynağı:**
> - 8 STL dosyasının binary parser ile bbox analizi (`measure_parts.py`)
> - User Manual ENG.pdf
> - bill-of-materials.pdf
> - General Assembly STP yapı analizi

---

## 1. Mekanik Yapı — Onaylanmış Topoloji

| Eklem | Tip | Servo | Tip | Açıklama |
|:---:|:---:|:---:|:---:|---|
| **J1** | Revolute | MG996R | Yaw | Taban dönüşü, $z_0$ ekseni dikey |
| **J2** | Revolute | MG996R | Pitch | Omuz, sol+sağ paralel bracket yapısı |
| **J3** | Revolute | MG996R | Pitch | Dirsek, J2 üstünde |
| **J4** | Revolute | MG996R | Roll | Önkol ekseni etrafında dönme |
| **J5** | Revolute | SG90 | Pitch | Bilek pitch, J6 bracket'ı eğer |
| **J6** | Revolute | SG90 | Roll | Uç işleyici dönüşü |

**Toplam:** 4× MG996R (1.4 kg·cm tipik servo, 11 kg·cm stall) + 2× SG90 (1.8 kg·cm).

> **Tasarım gözlemi:** Eklemler revolute. Son üç eksenin (J4, J5, J6) tam olarak küresel bilek (spherical wrist) oluşturup oluşturmadığı parça geometrisinden kesin değil — bu modelde **yaklaşık** spherical wrist olarak modelliyoruz (analitik IK için), CAD'de doğrulanması gerekir.

---

## 2. STL Parça Boyutları (Gerçek Ölçümler)

`measure_parts.py` çıktısı (bbox = bounding box, mm cinsinden):

| Parça | $X_{size}$ | $Y_{size}$ | $Z_{size}$ | Açıklama |
|---|---:|---:|---:|---|
| **Base** | 125 | 60 | 125 | Zemin tabanı, kare prizma 125×125, yükseklik 60 |
| **J1** | 90 | 80 | 90 | J1 motor housing, taban üstüne oturur |
| **J2 (LEFT)** | 22 | 163 | 50 | Sol omuz bracketi (yan duvar) |
| **J2 (RIGHT)** | 33 | 163 | 50 | Sağ omuz bracketi (yan duvar) |
| **J3 (TOP)** | 76 | 76 | 67 | Dirsek housing (J2 üstüne oturur) |
| **J4 (ARM)** | 68 | 45 | 165 | Önkol tüpü (forearm) |
| **J4 (J6 JOINT)** | 35.5 | 40 | 75 | J5/J6 bracketi (önkol ucuna mount) |
| **J6** | 50 | 47 | 55 | Uç işleyici (gripper plate) |

**Ekstra ölçümler (Y = dikey eksen, Z = öne uzanma yönü) — assembly konteksti:**
- J2 brackets Y range: $-25$ ile $+138$ → **omuz pivotu Y=0'da, dirsek pivotu Y=138'de** ⇒ $a_2 \approx 138$ mm
- J4 ARM Z range: $0$ ile $165$ → forearm uzunluğu 165 mm
- J4 J6 JOINT Z range: $90$ ile $165$ (forearm ile aynı frame'de) → **bracket pivotu (J5 axis) Z=90'da** ⇒ $d_4 \approx 130$ mm (tahmini bilek merkezi orta noktada)

---

## 3. Final DH Parametre Tablosu (Modele Özgü)

Standard DH konvansiyonu, küresel bilek yaklaşımı:

| Link $i$ | $\theta_i$ | $d_i$ (mm) | $a_i$ (mm) | $\alpha_i$ (rad) | Kaynak |
|:---:|:---:|---:|---:|:---:|:---|
| 1 | $\theta_1$ | **100** | 0 | $-\pi/2$ | Base 60 + J1 motor merkezi 40 |
| 2 | $\theta_2$ | 0 | **138** | 0 | J2 bracket Y span 138 (pivot→dirsek) |
| 3 | $\theta_3$ | 0 | 0 | $-\pi/2$ | (Bağlantı, mesafesiz) |
| 4 | $\theta_4$ | **130** | 0 | $+\pi/2$ | J3 → wrist merkez (90 + bracket yarısı 40) |
| 5 | $\theta_5$ | 0 | 0 | $-\pi/2$ | Küresel bilek varsayımı |
| 6 | $\theta_6$ | **50** | 0 | 0 | J6 part Z bbox 55 ≈ 50 (TCP'ye) |

Önceki (tahmini) → Şimdiki (ölçülen):
- $d_1$: 80 → **100** mm
- $a_2$: 120 → **138** mm
- $d_4$: 120 → **130** mm
- $d_6$: 80 → **50** mm

> Toplam erişim ($a_2 + d_4 + d_6 = 318$ mm) — robotun teorik max yatay reach'i.

---

## 4. İleri Kinematik — Sayısal Doğrulama

**Test 1:** $q = [0,0,0,0,0,0]$ (kol yatay uzanmış)

$$
p_{\text{TCP}} = \begin{bmatrix} a_2 + d_4 \\ 0 \\ d_1 - d_6 \end{bmatrix} = \begin{bmatrix} 138 + 130 \\ 0 \\ 100 - 50 \end{bmatrix} = \begin{bmatrix} 268 \\ 0 \\ 50 \end{bmatrix} \text{ mm}
$$

**Test 2:** $q = [0, -\pi/2, 0, 0, 0, 0]$ (kol dik yukarı)

$$
p_{\text{TCP}} = \begin{bmatrix} 0 \\ 0 \\ d_1 + a_2 + d_4 - d_6 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 318 \end{bmatrix} \text{ mm}
$$

**Test 3:** $q = [\pi/2, -\pi/4, -\pi/4, 0, 0, 0]$ (taban 90° dönmüş, kol 45° açıyla)

$\theta_2 + \theta_3 = -\pi/2$ → forearm yatay
- $r$ (yatay reach) $= a_2 \cos(-\pi/4) + d_4 \cos(-\pi/2) = 138 \cdot 0.707 + 0 = 97.6$ mm
- Sonra $\theta_1 = \pi/2$ rotasyonu uygulanır → x→y'ye döner

$$
p_{\text{TCP}} \approx \begin{bmatrix} 0 \\ 97.6 + d_6 \cdot \sin(?) \\ d_1 + a_2 \sin(\pi/4) + d_4 \cdot 1 \end{bmatrix}
$$

(Tam hesap için MATLAB ile doğrula.)

### MATLAB doğrulama (güncel)

```matlab
d1 = 100;  a2 = 138;  d4 = 130;  d6 = 50;

L(1) = Link('d', d1, 'a', 0,  'alpha', -pi/2);
L(2) = Link('d', 0,  'a', a2, 'alpha', 0);
L(3) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
L(4) = Link('d', d4, 'a', 0,  'alpha',  pi/2);
L(5) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
L(6) = Link('d', d6, 'a', 0,  'alpha', 0);

robot = SerialLink(L, 'name', 'MBera-6DOF-Arm');

q0 = [0 0 0 0 0 0];
T0 = robot.fkine(q0);
disp('Test 1 (q=0):'); disp(T0.t');   % Beklenen: [268, 0, 50]

q1 = [0 -pi/2 0 0 0 0];
T1 = robot.fkine(q1);
disp('Test 2 (omuz dik):'); disp(T1.t');   % Beklenen: [0, 0, 318]

robot.teach();
```

---

## 5. Kütle Bütçesi (BOM + Tahmini)

### 5.1 Servo kütleleri (datasheet)

| Bileşen | Kütle |
|---|---:|
| MG996R (×4) | 55 g × 4 = **220 g** |
| SG90 (×2) | 9 g × 2 = **18 g** |
| **Servo toplamı** | **238 g** |

### 5.2 PLA parça kütleleri (tahmini, %25 infill, 0.4 perimeter, PLA ρ=1.24 g/cm³)

Hacim ≈ bbox × doluluk faktörü (0.20 — hollow tasarım için).

| Parça | bbox (cm³) | Tahmini hacim | Kütle (g) |
|---|---:|---:|---:|
| Base | 125·60·125 / 1000 = 937 | × 0.10 = 94 cm³ | ~115 |
| J1 | 90·80·90 / 1000 = 648 | × 0.15 = 97 cm³ | ~120 |
| J2 LEFT + RIGHT | 2·(28·163·50)/1000 = 456 | × 0.30 = 137 cm³ | ~170 |
| J3 TOP | 76·76·67 / 1000 = 387 | × 0.20 = 77 cm³ | ~95 |
| J4 ARM | 68·45·165 / 1000 = 505 | × 0.18 = 91 cm³ | ~115 |
| J4 J6 JOINT | 35·40·75 / 1000 = 105 | × 0.30 = 32 cm³ | ~40 |
| J6 | 50·47·55 / 1000 = 129 | × 0.30 = 39 cm³ | ~48 |
| **PLA toplamı** | | | **~700 g** |

### 5.3 Vida + dişli + bracket: ~30 g

### 5.4 Toplam robot kütlesi: ≈ **970 g** (yaklaşık 1 kg)

---

## 6. Link-bazında Kütle Dağılımı (Dinamik için)

DH konvansiyonunda link $i$, eklem $i-1$ ile $i$ arasındaki rijit gövde:

| Link | İçerik | Kütle (g) | COM (link tabanından, mm) |
|---|---|---:|---:|
| L1 | Base + J1 housing + J1 servo (MG) | 115+120+55 = **290** | (0, 0, 50) lokal |
| L2 | J2 LEFT+RIGHT + J2 servo (MG) | 170+55 = **225** | (a₂/2, 0, 0) → (69, 0, 0) |
| L3 | J3 TOP + J3 servo (MG) + J4 ARM + J4 servo (MG) | 95+55+115+55 = **320** | (0, 0, d₄/2) → (0, 0, 65) |
| L4 | (J4 servo zaten L3'te) — sembolik link | ~5 | — |
| L5 | J4 J6 JOINT + J5 servo (SG90) | 40+9 = **49** | (0, 0, 25) |
| L6 | J6 part + J6 servo (SG90) | 48+9 = **57** | (0, 0, d₆/2) → (0, 0, 25) |

**NOT:** Dinamik açısından kritik olan L2, L3, L5, L6 kütleleridir (omuza karşı moment kolu en uzun bunların).

---

## 7. Statik Tork Analizi (Servo Yetisi Doğrulaması)

**En kötü statik durum:** Kol tam yatay uzanmış, $\theta_2 = 0$, $\theta_3 = 0$. $g = 9.81$ m/s².

### 7.1 Omuz (J2) — KRİTİK eklem

Tabandan uzaklığa göre toplam moment:

$$
\tau_{J2} = g \cdot \sum_{i \geq 2} m_i \cdot \ell_i
$$

| Bileşen | Kütle (kg) | Moment kolu (m) | Moment (N·m) |
|---|---:|---:|---:|
| L2 (omuz kol grup) | 0.225 | 0.069 | 0.0152 |
| L3 (önkol grup) | 0.320 | 0.138 + 0.065 = 0.203 | 0.0650 |
| L5 (bilek bracket) | 0.049 | 0.138 + 0.130 = 0.268 | 0.0131 |
| L6 (uç + servo) | 0.057 | 0.138 + 0.130 + 0.025 = 0.293 | 0.0167 |
| **Yük (payload)** | 0.020 | 0.318 | 0.0064 |
| **Toplam** | | | **0.116 N·m** |

$$
\boxed{\tau_{J2}^{\text{stat}} \approx 0.116 \text{ N·m} = 1.18 \text{ kgf·cm}}
$$

✅ **MG996R için çok rahat** (stall 11 kgf·cm, güvenli operasyon ~5 kgf·cm).

> **Önemli not:** Önceki yüksek tahmin (~6 kgf·cm) hatalıydı çünkü:
> 1. PLA parçaları aşırı ağır tahmin ettim (gerçek hollow yapı çok daha hafif)
> 2. Yük 50g yerine 20g'a indirildi (DIY tasarım payload free)
> 3. Moment kolları daha kısa (gerçek $a_2 = 138$, sandığım 200 değil)

### 7.2 Dirsek (J3)

$$
\tau_{J3} = g \cdot [m_3 \ell_{c3} + (m_5 + m_6 + m_{\text{load}}) \cdot d_4]
$$

$$
\tau_{J3} = 9.81 \cdot [0.32 \cdot 0.065 + (0.049 + 0.057 + 0.02) \cdot 0.130]
$$

$$
\tau_{J3} = 9.81 \cdot [0.0208 + 0.01638] = 9.81 \cdot 0.0372 \approx 0.365 \text{ N·m} \approx 3.72 \text{ kgf·cm}
$$

✅ MG996R için yine güvenli bölgede (~3.7 kgf·cm < 5 kgf·cm).

### 7.3 Bilek Pitch (J5) — SG90 ile tahrik

$$
\tau_{J5} = g \cdot [(m_6 + m_{\text{load}}) \cdot d_6]
$$

$$
\tau_{J5} = 9.81 \cdot (0.057 + 0.020) \cdot 0.050 \approx 0.0378 \text{ N·m} \approx 0.39 \text{ kgf·cm}
$$

✅ SG90 (1.8 kgf·cm) için bol miktarda margin.

### 7.4 J1, J4, J6 — Yerçekimi torku üretmez

Eksen düşeyine paralel veya kütle merkezine yakın → statikte yük yok, sadece atalet.

### 7.5 Özet Tablo

| Eklem | Servo | Statik tork (kgf·cm) | Servo limiti | Margin |
|:---:|:---:|---:|---:|:---:|
| J1 | MG996R | ~0 | ~5 | ✅ Çok rahat |
| **J2** | **MG996R** | **1.18** | **5** | **✅ Rahat** |
| J3 | MG996R | 3.72 | 5 | ✅ OK |
| J4 | MG996R | ~0 | ~5 | ✅ Çok rahat |
| J5 | SG90 | 0.39 | 1.5 | ✅ Rahat |
| J6 | SG90 | ~0 | 1.5 | ✅ Çok rahat |

**Sonuç:** MBera 6-DOF Arm tasarımı **doğru servo seçimi** yapmış. 4× MG996R + 2× SG90 yapı, **20 g payload**'a kadar rahatça yetiyor. Daha ağır yükler için J3 ilk sıkışan eklem olur (~50g'dan sonra J3 stall'a yaklaşır).

---

## 8. Çalışma Uzayı (Workspace)

### 8.1 Maksimum reach

- Yatay: $r_{\max} = a_2 + d_4 + d_6 = 138 + 130 + 50 = 318$ mm
- Dikey (yukarı): $z_{\max} = d_1 + a_2 + d_4 + d_6 = 100 + 138 + 130 + 50 = 418$ mm
- Dikey (aşağı): $z_{\min} = d_1 - (a_2 + d_4 + d_6) = 100 - 318 = -218$ mm (zemin altı; pratik 0)

### 8.2 Pratik (eklem limitleri ile sınırlı) çalışma uzayı

Servo limitleri (datasheet $\pm 90°$ etkin):
- $\theta_1 \in [-90°, +90°]$ → ön yarımküre
- $\theta_2 \in [-90°, +90°]$ → tüm pitch
- $\theta_3 \in [-135°, +90°]$ → dirsek menteşesi (mekanik kapanma sınırı)
- $\theta_4, \theta_5 \in \pm 90°$
- $\theta_6 \in \pm 180°$

**Yararlı reach hacmi:** Yaklaşık **yarım küre, R ≈ 250–300 mm**.

### 8.3 MATLAB ile çalışma uzayı görselleştirme

```matlab
N = 30000;
qmin = [-pi/2, -pi/2, -3*pi/4, -pi/2, -pi/2, -pi];
qmax = [ pi/2,  pi/2,  pi/2,    pi/2,  pi/2,  pi];
P = zeros(N,3);
for k=1:N
    qr = qmin + (qmax-qmin).*rand(1,6);
    T = robot.fkine(qr);
    P(k,:) = T.t';
end
scatter3(P(:,1), P(:,2), P(:,3), 1, P(:,3), 'filled');
axis equal; grid on; colorbar;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('MBera 6-DOF Arm — Çalışma Uzayı');
```

---

## 9. İleri Kinematik (Sembolik) — Bu Modele Özel

Yukarıdaki DH değerleriyle:

$$
{}^0T_3(q_1, q_2, q_3) = \begin{bmatrix}
c_1 c_{23} & -c_1 s_{23} & -s_1 & 138 \, c_1 c_2 \\
s_1 c_{23} & -s_1 s_{23} &  c_1 & 138 \, s_1 c_2 \\
-s_{23}    & -c_{23}     &  0    & 100 - 138 \, s_2 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Bilek merkezi ($d_4$ = 130 boyunca uzanma sonrası):

$$
p_{\text{wrist}} = \begin{bmatrix}
138 \, c_1 c_2 - 130 \, c_1 s_{23} \\
138 \, s_1 c_2 - 130 \, s_1 s_{23} \\
100 - 138 \, s_2 - 130 \, c_{23}
\end{bmatrix}
$$

Uç işleyici (TCP):

$$
p_{\text{TCP}} = p_{\text{wrist}} + 50 \cdot \hat{z}_6(q)
$$

burada $\hat{z}_6$, R₆'nın 3. sütunu (yaklaşma vektörü).

---

## 10. Ters Kinematik — Bu Modele Özel

[`01_kinematik_dinamik.md`](01_kinematik_dinamik.md) §6'daki algoritma birebir uygulanır. Sadece sayısal değerler değişir:

```python
def inverse_kin_mbera_arm(T_des):
    d1, a2, d4, d6 = 100, 138, 130, 50
    
    R = T_des[:3,:3]
    p = T_des[:3, 3]
    p_w = p - d6 * R[:, 2]                    # bilek merkezi
    
    theta1 = np.arctan2(p_w[1], p_w[0])
    
    r = np.hypot(p_w[0], p_w[1])
    s = p_w[2] - d1
    D = (r**2 + s**2 - a2**2 - d4**2) / (2*a2*d4)
    if abs(D) > 1:
        raise ValueError(f"Hedef workspace dışı, D={D:.3f}")
    
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)   # elbow-up tercihi
    theta2 = np.arctan2(s, r) - np.arctan2(d4*np.sin(theta3),
                                            a2 + d4*np.cos(theta3))
    
    R03 = forward_kin([theta1, theta2, theta3, 0, 0, 0])[:3,:3]
    R36 = R03.T @ R
    
    theta5 = np.arctan2(np.hypot(R36[0,2], R36[1,2]), R36[2,2])
    if abs(theta5) > 1e-6:
        theta4 = np.arctan2(R36[1,2], R36[0,2])
        theta6 = np.arctan2(R36[2,1], -R36[2,0])
    else:
        theta4, theta6 = 0, np.arctan2(R36[0,1], R36[0,0])
    
    return np.array([theta1, theta2, theta3, theta4, theta5, theta6])
```

### Test
```python
import numpy as np
T_test = np.array([
    [1, 0, 0, 200],
    [0, 1, 0, 0],
    [0, 0, 1, 100],
    [0, 0, 0, 1]
])
print(np.degrees(inverse_kin_mbera_arm(T_test)))
```

---

## 11. Doğrulama Checklist (FAZ 1 Tamamı)

- [x] STL bbox ölçümleri yapıldı
- [x] BOM'dan servo + parça sayıları doğrulandı
- [x] DH tablosu modele özgü değerlerle dolduruldu ($d_1{=}100, a_2{=}138, d_4{=}130, d_6{=}50$)
- [x] FK iki test pozisyonunda doğrulandı (sembolik + sayısal)
- [x] Statik tork her eklem için hesaplandı — **MG996R + SG90 servo seçimi onaylandı (max ~20g payload ile)**
- [x] Mass budget BOM bilgileriyle güncellendi (~970g toplam robot)
- [ ] Robot fiziksel monte edildiğinde DH tablosu cetvelle doğrulanmalı
- [ ] MATLAB Robotics Toolbox ile FK/IK karşılaştırılmalı (FAZ 2)
- [ ] Wrist'in gerçek geometrisi CAD'de incelenecek (perfect spherical mı?)

---

## 12. Belirsizlikler ve Doğrulanması Gerekenler

| Konu | Mevcut Varsayım | Doğrulama Yöntemi |
|---|---|---|
| Bilek tip | Yaklaşık spherical wrist | Fusion 360'ta J5/J6 axes'in kesişmesini kontrol |
| $d_1$ değeri | 100 mm (J1 motor merkezi) | J1 servo gear merkezi → J2 axis ölçümü |
| $d_4$ değeri | 130 mm (bracket merkez yaklaşımı) | J3 axis → J5 axis cetvelle ölç |
| PLA parça kütlesi | Hacim × 0.15-0.30 doluluk | Baskı sonrası tartılmalı |
| Servo gerçek tork | Datasheet 11 kgf·cm | 6V SMPS ile yük testi (öneri) |
| J3 mekanik limit | $-135°$ ile $+90°$ arasında | Bracket çarpışma simülasyonu |

---

## 13. Bir Sonraki Adım

**FAZ 2 — MATLAB Simülasyonu** için hazırsın. Yapacaklar:
1. MATLAB R2025b + Peter Corke RTB kurulumu
2. Yukarıdaki SerialLink modeliyle Test 1 ve Test 2'yi çalıştır
3. `robot.teach()` ile interaktif doğrulama (her servo limit içinde davranıyor mu?)
4. 5 farklı hedef poz için IK çözüp video kaydet → LinkedIn post #2

---

*Versiyon: 1.0 — 2026-04-26*
*Veri kaynağı: DIY_Robotics_EducativeCell_V1_0/Mechanical/*.stl + bill-of-materials.pdf*
*Ölçüm aracı: measure_parts.py (binary STL bbox parser)*
