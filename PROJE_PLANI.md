# 🤖 6-DOF Robot Kol Projesi — Kapsamlı Öğrenme ve Uygulama Planı

> **Proje sahibi:** [Adın]
> **Amaç:** Öğrenme + CV/Portfolyo + LinkedIn & GitHub paylaşımı
> **Referans model:** MBera 6-DOF Arm
> **Benzer açık kaynak:** OmArm Zero (omartronics.com) — 3×MG996R + 3×SG90 + Arduino Uno + PCA9685

---

## 📋 Proje Özeti

Bu proje, sıfırdan başlayarak **6 serbestlik dereceli (DOF) robotik bir kol**'un matematiksel modelini kurmayı, MATLAB üzerinde simüle etmeyi, 3D baskı ile fiziksel olarak üretmeyi ve elektronik+yazılım kontrolünü gerçekleştirmeyi hedefler.

**Nihai çıktı:** Çalışan bir robot kol + GitHub reposu + LinkedIn yazı serisi + CV'ye eklenebilir proje.

---

## 🎯 Öğrenme Hedefleri

| Alan | Kazanım |
|------|---------|
| **Robot Kinematiği** | DH parametreleri, ileri/ters kinematik, homojen dönüşüm matrisleri |
| **Robot Dinamiği** | Jakobiyen matrisi, Lagrange/Newton-Euler yaklaşımı, tork hesabı |
| **Simülasyon** | MATLAB Robotics Toolbox (Peter Corke), Simulink, Simscape Multibody |
| **Mekatronik** | Servo kontrol (PWM), I²C, PCA9685 sürücü, güç yönetimi |
| **Gömülü Yazılım** | Arduino C++, gerçek zamanlı kontrol, trajektori planlama |
| **CAD & 3D Baskı** | Fusion 360 / Inventor, FDM baskı, tolerans tasarımı |
| **Proje Yönetimi** | Git versiyon kontrolü, teknik dokümantasyon, içerik üretimi |

---

## 🗂️ Proje Fazları (Yol Haritası)

```
FAZ 1: Teorik Temel (2-3 hafta)     ──► Kinematik & Dinamik öğrenimi
FAZ 2: MATLAB Simülasyonu (2 hafta) ──► Robotics Toolbox ile doğrulama
FAZ 3: Mekanik Tasarım (1-2 hafta)  ──► STL dosyaları + 3D baskı
FAZ 4: Elektronik & Montaj (1 hafta)──► Kablolama + ilk hareket
FAZ 5: Kontrol Yazılımı (2 hafta)   ──► Arduino + trajektori kontrolü
FAZ 6: İleri Özellikler (opsiyonel) ──► Ters kinematik gömülü, ROS, CV
FAZ 7: Dokümantasyon & Paylaşım     ──► GitHub, LinkedIn, Video
```

---

## 🔧 FAZ 1 — Teorik Temel: Kinematik ve Dinamik

### 1.1 Ön Gereksinimler (hatırlatma)
- Lineer cebir: matris çarpımı, dönme matrisleri, özdeğer
- Trigonometri: atan2 fonksiyonu kritik
- Diferansiyel denklemler: dinamik için
- Newton mekaniği ve Lagrange yaklaşımı

### 1.2 Denavit-Hartenberg (DH) Parametreleri

Her eksen için 4 parametre tanımlanır:

| Sembol | İsim | Anlamı |
|--------|------|--------|
| $\theta_i$ | Eklem açısı | $z_{i-1}$ etrafında dönme (revolute eklemde değişken) |
| $d_i$ | Link offset | $z_{i-1}$ boyunca öteleme |
| $a_i$ | Link uzunluğu | $x_i$ boyunca öteleme (ortak dikme) |
| $\alpha_i$ | Link bükümü | $x_i$ etrafında dönme (iki $z$ ekseni arası açı) |

**Robot kolun için tahmini DH tablosu** (ölçümden sonra güncellenecek):

```
Link |   θ_i    | d_i (mm) | a_i (mm) | α_i (°)
-----+----------+----------+----------+---------
  1  |   θ_1    |   d_1    |    0     |   -90
  2  |   θ_2    |    0     |   a_2    |    0
  3  |   θ_3    |    0     |    0     |   -90
  4  |   θ_4    |   d_4    |    0     |   +90
  5  |   θ_5    |    0     |    0     |   -90
  6  |   θ_6    |   d_6    |    0     |    0
```
*(Taban dönüşü → omuz → dirsek → bilek pitch → bilek roll → gripper/yaw)*

> ⚠️ `d_1, a_2, a_3, d_4, d_6` değerleri senin 3D baskı parçalarının gerçek ölçüleri olacak. CAD modelden veya cetvelle ölçüp bu tabloyu MUTLAKA doldur.

### 1.3 Homojen Dönüşüm Matrisi

Her link için standart DH matrisi:

$$
{}^{i-1}T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Tabandan uç işleyiciye toplam dönüşüm:

$$
{}^0T_6 = {}^0T_1 \cdot {}^1T_2 \cdot {}^2T_3 \cdot {}^3T_4 \cdot {}^4T_5 \cdot {}^5T_6
$$

### 1.4 İleri Kinematik (Forward Kinematics)
- **Girdi:** 6 eklem açısı $(\theta_1, \ldots, \theta_6)$
- **Çıktı:** Uç işleyicinin konumu $(x,y,z)$ + yönelimi (Euler açıları veya kuaterniyon)
- Çözüm her zaman **tek**tir, kapalı formda doğrudan matris çarpımı ile bulunur.

### 1.5 Ters Kinematik (Inverse Kinematics)
- **Girdi:** İstenen uç işleyici pozu
- **Çıktı:** 6 eklem açısı
- **Zorluk:** Birden fazla çözüm (genelde 8 tane), tekillik noktaları (singularity).

**Senin kolun için kapalı-form çözüm VAR mı?** Kolun son 3 ekseni tek noktada kesişiyorsa (**küresel bilek** — spherical wrist), pozisyon ve yönelim ayrıştırılabilir. Bu çok büyük avantaj. Tasarımı buna göre gözden geçir.

- Pozisyon problemi: $\theta_1, \theta_2, \theta_3$ (geometrik/algebraik çözüm)
- Yönelim problemi: $\theta_4, \theta_5, \theta_6$ (Euler açılarından çözüm)

### 1.6 Diferansiyel Kinematik — Jakobiyen

$$
\dot{x} = J(\theta)\dot{\theta}
$$

- 6×6 matris; eklem hızlarını uç işleyici hızına dönüştürür
- Determinantının sıfır olduğu noktalar **tekillikler**dir (robot o yönde hareket edemez)
- Ters Jakobiyen, sayısal ters kinematik için kullanılır (Newton-Raphson iterasyonu)

### 1.7 Dinamik Modelleme

**Amaç:** Her eklemde gerekli torku hesaplayarak servo seçimini doğrulamak.

$$
\tau = M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + G(\theta)
$$

- $M$: Kütle-atalet matrisi
- $C$: Coriolis ve merkezkaç terimleri
- $G$: Yerçekimi terimi (servo seçiminde **en kritik** olan)

> 💡 **MG996R torku ≈ 11 kg·cm**. Her linkin kütle merkezini ve ağırlığını hesaplayıp omuz/dirsek eklemlerinin statik torkunu mutlaka kontrol et. Yoksa kol kaldıramayacak.

### 📚 FAZ 1 için önerilen kaynaklar
- **Kitap:** *Robotics: Modelling, Planning and Control* — Siciliano et al.
- **Kitap:** *Robotics, Vision and Control* — Peter Corke (MATLAB Toolbox ile birlikte)
- **YouTube:** Angela Sodemann — Robot Mechanics playlist (DH ve IK için mükemmel)
- **Ücretsiz PDF:** Peter Corke DH handout
- **Video seri:** "Robot Academy" — Peter Corke (queenslandrobotics)

### ✅ FAZ 1 çıktıları
- [ ] Robotun çizimi ile DH çerçeve ataması yapılmış kroki (el çizimi yeterli)
- [ ] Doldurulmuş DH tablosu (gerçek mm ölçülerle)
- [ ] Kağıt üzerinde sembolik $T_6^0$ matrisi türetimi
- [ ] İleri kinematik için Python/MATLAB fonksiyonu
- [ ] Test vektörü: $\theta = [0,0,0,0,0,0]$ için uç pozu doğrulama

---

## 💻 FAZ 2 — MATLAB Simülasyonu

### 2.1 Kurulum
1. MATLAB R2023b+ (üniversiten varsa ücretsiz lisans)
2. **Robotics System Toolbox** (MathWorks resmi)
3. **Peter Corke Robotics Toolbox** (RTB) — Dosya değişim merkezinden indir (ücretsiz)
4. Simscape Multibody (dinamik simülasyon için)

### 2.2 İlk Test: Robot Modeli

Peter Corke RTB ile iskeleti oluştur:

```matlab
% DH parametreleri (örnek — kendi değerlerinle doldur)
L(1) = Link('d', 0.08, 'a', 0,    'alpha', -pi/2);
L(2) = Link('d', 0,    'a', 0.12, 'alpha', 0);
L(3) = Link('d', 0,    'a', 0,    'alpha', -pi/2);
L(4) = Link('d', 0.10, 'a', 0,    'alpha', pi/2);
L(5) = Link('d', 0,    'a', 0,    'alpha', -pi/2);
L(6) = Link('d', 0.05, 'a', 0,    'alpha', 0);

robot = SerialLink(L, 'name', 'MyArm6DOF');
robot.teach();    % İnteraktif slider ile oynat
```

### 2.3 İleri Kinematik Doğrulaması

```matlab
q = [0 pi/4 -pi/4 0 pi/2 0];
T = robot.fkine(q);      % Homojen matris
disp(T.t)                % Pozisyon [x;y;z]
```

Kendi yazdığın Python/MATLAB fonksiyonunun sonucu ile **birebir aynı** çıkmalı. Çıkmıyorsa DH çerçevelerin hatalı.

### 2.4 Ters Kinematik

Küresel bileğin varsa kapalı-form:

```matlab
T_goal = SE3(0.15, 0.10, 0.20) * SE3.Rx(pi/6);
q_sol = robot.ikine6s(T_goal);   % Sadece küresel bilekli 6-DOF için
```

Genel durumda sayısal:

```matlab
q_sol = robot.ikine(T_goal, 'q0', [0 0 0 0 0 0]);
```

### 2.5 Trajektori Planlama

```matlab
q_start = [0 0 0 0 0 0];
q_end   = [pi/2 -pi/3 pi/4 0 pi/6 0];
t       = 0:0.05:3;
[q, qd, qdd] = jtraj(q_start, q_end, t);   % Kübik spline
robot.plot(q);
```

### 2.6 Simscape Multibody (Gelişmiş)
- Inventor/Fusion'dan CAD'i **URDF** veya **XML** olarak export et
- Simscape'te yerçekimi altında dinamik simülasyon
- PID kontrolcü ekle, basamak cevabını ölç
- Bu kısım CV için **çok değerli görsel** üretir (GIF/video)

### ✅ FAZ 2 çıktıları
- [ ] `robot.teach()` GUI'sinden tüm eklemler doğru yönde hareket ediyor
- [ ] FK kodunun RTB sonucuyla eşleşmesi (hata < 1e-9)
- [ ] 3 farklı hedef poz için IK çözümü
- [ ] Animasyonlu trajektori videosu (MP4/GIF) — LinkedIn paylaşımı için altın değerinde
- [ ] Çalışma uzayı (workspace) görselleştirmesi

---

## 🔩 FAZ 3 — Mekanik Tasarım ve 3D Baskı

### 3.1 Tasarım Kararları

| Parametre | Öneri |
|-----------|-------|
| CAD yazılımı | Fusion 360 (öğrenci ücretsiz) veya Inventor |
| Filament | **PLA+** — kolay, boyutsal kararlı (PLA yeterli; PETG dayanımı artırır) |
| Katman yüksekliği | 0.2 mm |
| İnfill | Yapısal parçalar için **%40-60**, gövde için %25 |
| Duvar sayısı | Minimum 4 perimetre (servo yuvalarında stres var) |
| Tolerans | Servo yuvaları için ±0.2 mm — test baskısı yap |

### 3.2 Servo Seçimi

| Eksen | Servo | Gerekçe |
|-------|-------|---------|
| 1 — Taban dönüşü | **MG996R** | Üst kolun tüm torkunu taşıyor |
| 2 — Omuz | **MG996R** | En yüksek statik tork burada |
| 3 — Dirsek | **MG996R** | Önkol + yükü kaldırıyor |
| 4 — Bilek pitch | **SG90** veya MG90S | Hafif, yeterli tork |
| 5 — Bilek roll | **SG90** | Uç rotasyonu, hafif yük |
| 6 — Gripper | **SG90** | Parmak tutma mekanizması |

### 3.3 STL Kaynakları
- **Hazır başlangıç:** OmArm Zero (cults3d.com) — ücretsiz STL seti, MG996R+SG90 kombinasyonu
- **Thingiverse:** "EEZYbotARM MK2" benzerleri (basit), "BCN3D MOVEO" (karmaşık, tam metal takviyeli)
- **Kendin tasarla:** Öğrenmek için EN değerli yol — servoların datasheet ölçüleriyle başla

### 3.4 Montaj Sırası
1. Taban + MG996R #1 (yatay rulmanla desteklenmiş olmalı — titreşimi azaltır)
2. Omuz bağlantısı + MG996R #2 (çift taraflı destek kritik)
3. Üst kol linki + dirsek MG996R #3
4. Önkol + SG90 bilek pitch
5. Bilek roll + gripper
6. Her eklemde **cıvata loktite** ile sabitle (servo titreşimi vida gevşetir)

### ✅ FAZ 3 çıktıları
- [ ] 3D model (STEP + STL) — GitHub'a yüklenecek
- [ ] Baskı fotoğrafları (tüm parçalar)
- [ ] Montaj aşama fotoğrafları (LinkedIn serisi için!)
- [ ] Servo kalibrasyonu yapılmış (her servo 0°'de sıfırlı)

---

## ⚡ FAZ 4 — Elektronik ve Güç

### 4.1 Bileşen Listesi

| Bileşen | Model | Adet | Not |
|---------|-------|------|-----|
| Mikrokontrolcü | Arduino Uno / Mega | 1 | Mega ileride sensör eklemek için daha iyi |
| Servo sürücü | **PCA9685** 16-kanal I²C | 1 | Arduino'nun timer'larını bloke etmez |
| Güç kaynağı | 5V **5A** anahtarlamalı SMPS | 1 | MG996R'ler aç durumda 2A+ çekebilir |
| Kondansatör | 1000µF elektrolitik | 2-3 | Servo pinlerinin yakınına koy (tekme akımı) |
| DC-DC | Buck konvertör (opsiyonel) | 1 | 12V→5V dönüşümü için |
| Jack + kablolar | XT60 + 22AWG | - | İnce kablo MG996R'de ısınır! |

### 4.2 Kablolama Şeması (kritik noktalar)

```
Güç yolu:
  SMPS 5V ──┬── PCA9685 V+ terminali (servo gücü)
            └── 1000µF kondansatör (GND'ye)

Sinyal yolu:
  Arduino 5V  ──► PCA9685 VCC  (mantık gücü, servo değil!)
  Arduino GND ──► PCA9685 GND  ──► SMPS GND  (ORTAK TOPRAK ŞART!)
  Arduino SDA ──► PCA9685 SDA
  Arduino SCL ──► PCA9685 SCL

Servolar: PCA9685 kanal 0-5'e tak (her servo 3 pin: sinyal/V+/GND)
```

> ⚠️ **En sık yapılan 3 hata:**
> 1. Servoları doğrudan Arduino 5V pininden beslemek → kart yanar
> 2. Ortak toprak (common GND) unutulması → servo çılgınca titrer
> 3. Zayıf güç kaynağı → kol hareket ettikçe Arduino resetlenir

### 4.3 İlk Test Kodu

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_MIN 150   // 0°   için PWM değeri (kalibre et)
#define SERVO_MAX 600   // 180° için PWM değeri

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);   // Servolar 50 Hz ister
}

int angleToPulse(int ang) {
  return map(ang, 0, 180, SERVO_MIN, SERVO_MAX);
}

void loop() {
  for (int ch = 0; ch < 6; ch++) {
    pwm.setPWM(ch, 0, angleToPulse(90));   // Tümünü orta konuma al
  }
  delay(2000);
}
```

### ✅ FAZ 4 çıktıları
- [ ] Tüm 6 servo PCA9685 üzerinden 0°-180° arası süpürüyor
- [ ] Fritzing veya KiCad'da devre şeması çizildi
- [ ] Her servo için min/max PWM değerleri kalibre edildi (mekanik limitler!)

---

## 🎛️ FAZ 5 — Kontrol Yazılımı

### 5.1 Kod Mimarisi

```
/firmware
├── main.ino              — Ana döngü, durum makinesi
├── Kinematics.h/.cpp     — FK + sayısal IK (ters Jakobiyen iterasyon)
├── Trajectory.h/.cpp     — Kübik/Bezier interpolasyon
├── ServoManager.h/.cpp   — PCA9685 soyutlama + yumuşak hareket
└── SerialProtocol.h/.cpp — PC'den komut parse (JSON veya custom)

/gui (opsiyonel, PC tarafı)
├── controller.py         — PySerial + tkinter GUI
└── camera_pickplace.py   — OpenCV + ArUco marker
```

### 5.2 Kontrol Modları (artan zorlukta)

1. **Manuel mod** — 6 potansiyometre, her biri bir servoyu sürer
2. **Seri komut modu** — PC'den "J1 90 J2 45 ..." gönder
3. **Joint-space trajektori** — başlangıç/bitiş açı verip kübik spline ile ara
4. **Cartesian mod** — (x,y,z,rx,ry,rz) iste → gömülü IK çöz → eklemlere uygula
5. **Kayıt & oynatma** — Kolu elle hareket ettir (servo kapalı), açıları kaydet, oynat
6. **Görüntü işleme** — OpenCV ile nesne tespit → kolu oraya yönlendir (pick & place)

### 5.3 Yumuşak Hareket (kritik!)

Servoları doğrudan hedef açıya yazma — servo tüm hızıyla çılgınca hareket eder. Yerine:

```cpp
// Her 20ms'de bir küçük adım at (50 Hz kontrol döngüsü)
for (int i = 0; i < 6; i++) {
  if (abs(target[i] - current[i]) > STEP) {
    current[i] += (target[i] > current[i]) ? STEP : -STEP;
    servoWrite(i, current[i]);
  }
}
```

### ✅ FAZ 5 çıktıları
- [ ] Çalışan seri kontrol arayüzü
- [ ] Kaydet/oynat fonksiyonu (en az 1 pick-and-place demo)
- [ ] Video: kol istenen trajektoriyi takip ediyor

---

## 🚀 FAZ 6 — İleri Özellikler (Opsiyonel — CV'yi bir üst seviyeye taşır)

- **ROS 2 entegrasyonu:** `moveit2` ile trajektori planlama
- **Sensör füzyonu:** IMU ile gerçek eklem açılarını ölç, servo geri besleme hatasını düzelt
- **Görüntü işleme:** OpenCV + ArUco ile otonom pick & place
- **Reinforcement Learning:** MATLAB RL Toolbox veya Python (gymnasium + stable-baselines3) ile öğrenmeli kontrol
- **Web kontrol paneli:** Flask + WebSocket ile tarayıcıdan kontrol
- **Mobil uygulama:** Bluetooth (HC-05) ile telefondan kontrol

---

## 📢 FAZ 7 — Paylaşım ve Dokümantasyon

### 7.1 GitHub Reposu Yapısı

```
6DOF-Robot-Arm/
├── README.md              ★ Ana sayfa — GIF, özellikler, kurulum
├── LICENSE                (MIT öneririm)
├── docs/
│   ├── 01_kinematics.md   Teorik türetim
│   ├── 02_simulation.md   MATLAB çıktıları
│   ├── 03_hardware.md     BOM + şema
│   └── assets/            Fotoğraflar, GIF'ler
├── cad/
│   ├── STL/
│   └── STEP/
├── matlab/
│   ├── forward_kinematics.m
│   ├── inverse_kinematics.m
│   └── simulate_trajectory.m
├── firmware/
│   └── arm_controller/
└── gui/
    └── controller.py
```

### 7.2 README'de bulunması şart:
- Proje özeti (1 paragraf)
- **Demo GIF'i en üstte** (LinkedIn'den gelen insan ilk saniye bunu görmeli)
- Özellikler listesi
- Donanım listesi + maliyet
- Hızlı kurulum (3 komut)
- Kinematik şema (ASCII veya resim)
- Video linki (YouTube/Vimeo)

### 7.3 LinkedIn Yazı Serisi Önerisi

| # | Başlık | Ne paylaşırsın |
|---|--------|----------------|
| 1 | "Sıfırdan 6-DOF robot kol yapıyorum #1: Teorik Başlangıç" | El yazısı DH tablosu + motivasyon |
| 2 | "MATLAB simülasyonu hazır" | FAZ 2 çıktısı — animasyon GIF'i |
| 3 | "3D baskı aşamasındayız" | Parça fotoğrafları |
| 4 | "İlk hareket!" | Kısa video, servolar süpürüyor |
| 5 | "Pick & place demosu" | Final video (60 saniye altı) |
| 6 | "Öğrendiklerim ve GitHub repo" | Tam bağlantı + özet |

### 7.4 CV Girişi Formatı (öneri)

> **6-DOF Robotic Manipulator — Personal Project** *(6 months)*
> Designed and built a 3D-printed 6-axis robotic arm from scratch. Derived full forward/inverse kinematics using DH convention, validated in MATLAB Robotics Toolbox. Implemented real-time joint-space and Cartesian control on Arduino with PCA9685 driver. Achieved ±1 cm repeatability in pick-and-place tasks.
> *Skills: Robot Kinematics, MATLAB/Simulink, Arduino (C++), Fusion 360, 3D Printing, I²C, PWM Control*
> GitHub: `github.com/username/6DOF-Robot-Arm`

---

## 📅 Zaman Çizelgesi (Önerilen)

| Hafta | Odak | Saat/hafta |
|-------|------|------------|
| 1-2 | FAZ 1: Kinematik teorisi, DH tablosu | 10-15 |
| 3-4 | FAZ 2: MATLAB simülasyonu | 10-15 |
| 5-6 | FAZ 3: CAD + 3D baskı | 15-20 |
| 7 | FAZ 4: Elektronik montaj | 10 |
| 8-9 | FAZ 5: Kontrol yazılımı | 15-20 |
| 10-12 | FAZ 6 + FAZ 7: İleri özellikler + paylaşım | 10-15 |

**Toplam: ~12 hafta (3 ay) yoğun tempoda, 5-6 ay rahat tempoda.**

---

## 🧰 Araç ve Kaynak Özeti

**Yazılım**
- MATLAB + Robotics System Toolbox + Peter Corke RTB
- Fusion 360 / Inventor (CAD)
- PrusaSlicer / Cura (3D baskı dilimleyici)
- Arduino IDE veya PlatformIO (öneririm)
- VS Code + GitHub Desktop
- Fritzing veya KiCad (şema)

**Donanım (tahmini bütçe ~₺3000-5000)**
- 3D yazıcı erişimi (kendi veya Fab Lab)
- PLA+ filament ~1 kg
- 3× MG996R + 3× SG90 servo
- Arduino Uno/Mega
- PCA9685 modül
- 5V 5A SMPS
- Kablolar, konektörler, cıvatalar M3

---

## ⚠️ Sık Yapılan Hatalar ve Tuzaklar

1. **DH çerçeve ataması yanlış** → FK sonuçları MATLAB ile eşleşmez. Dikkatli çiz.
2. **Servo statik tork yetersiz** → Omuz eklemi çöker. Dinamik hesap yap, gerekirse servo büyüt (MG996R → DS3218).
3. **Zayıf güç kaynağı** → Arduino resetlenir. 5V 5A minimum.
4. **Mekanik toleranslar** → Baskı parçaları servo yuvasına oturmaz. Test baskıları şart.
5. **IK tekilliklerde patlar** → Sayısal çözücünün iterasyon limitini ve başlangıç tahminini düşün.
6. **Kabloları döner eksenlerden geçirmemek** → Kablo birkaç hareketle kopar. Kablo kanalı tasarla.
7. **Lokaltite unutmak** → Servo vidaları 100 çalışmada gevşer, kol gevşek olur.
8. **Ortak toprak yok** → En sinir bozucu hata; servolar rastgele titrer.

---

## 🎯 Sonraki Adım (HEMEN yapılacak)

1. **Bu .md dosyasını GitHub'da yeni bir repoya koy** — repo adı: `6DOF-Robot-Arm`
2. **Kendi DH çerçevelerini çiz** (kağıt kalemle, sonra fotoğraflayıp repoya ekle)
3. **Servoları ve 3D baskı parçalarını siparişe ver** (teslimat süresi ile teori çalış)
4. **MATLAB kurulumunu hallet** ve Peter Corke RTB'yi indir
5. **İlk LinkedIn yazısını yaz** — "bir proje başlatıyorum" girişi, insanlar takip eder

---

*Son güncelleme: 2026-04-22*
*Versiyon: 1.0 — Başlangıç planı*
