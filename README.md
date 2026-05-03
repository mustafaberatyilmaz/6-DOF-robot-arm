# MBera 6-DOF Robot Kol

Sıfırdan tasarladığım 6 eksenli (6-DOF) robotik kol projesi: kinematik–dinamik teorisi, MATLAB simülasyonu ve ileride 3D baskı + servo kontrol fazları.

Bu repo, projenin **yazılım/teori** tarafını içerir. Donanım fazı (CAD, 3D baskı, Arduino + PCA9685 + MG996R/SG90) ilerleyen sürümlerde eklenecek.

![Simülasyon Görüntüsü](sim_photo.png)

---

## 🎯 Amaç

- Robot kinematiği & dinamiği teorisini uygulamalı öğrenmek
- DH parametreleri, ileri/ters kinematik, Jakobiyen, statik tork hesabı
- MATLAB'da gerçek mm ölçülerinde interaktif simülasyon
- İlerleyen fazda fiziksel kolu kurup pick & place demosu

---

## 📂 Repo İçeriği

| Dosya | Açıklama |
|---|---|
| [`01_kinematik_dinamik.md`](01_kinematik_dinamik.md) | FAZ 1 — DH konvansiyonu, dönüşüm matrisleri, FK/IK, Jakobiyen, dinamik teorisi |
| [`02_model_DH_dinamik.md`](02_model_DH_dinamik.md) | Modelin gerçek STL/STP ölçülerinden çıkarılan **kesin DH parametreleri**, çalışma uzayı ve servo tork doğrulaması |
| [`03_matlab_simulasyon_egitim.md`](03_matlab_simulasyon_egitim.md) | MATLAB simülasyonunun adım adım açıklaması |
| [`matlab_simulasyon.m`](matlab_simulasyon.m) | Kendi başına çalışan MATLAB simülasyonu (Peter Corke RTB **gerekmez**) |
| `sim_photo.png` | Simülasyondan ekran görüntüsü |
| `proje dosyalari/measure_parts.py` | STL bbox ölçümü için yardımcı script |
| `proje dosyalari/parse_assembly.py` | STP assembly analizi için yardımcı script |

---

## 📐 Robot Geometrisi

Standard DH (mm cinsinden):

| Link | θ | d | a | α |
|:---:|:---:|---:|---:|:---:|
| 1 | q₁ | 100 | 0 | −π/2 |
| 2 | q₂ | 0 | 138 | 0 |
| 3 | q₃ | 0 | 0 | −π/2 |
| 4 | q₄ | 130 | 0 | +π/2 |
| 5 | q₅ | 0 | 0 | −π/2 |
| 6 | q₆ | 50 | 0 | 0 |

**Konfigürasyon:** Antropomorfik (insan koluna benzer) + küresel bilek → ters kinematik kapalı-form çözülür.

**Doğrulama testleri:**
- `q = 0` → TCP = `[268, 0, 50]` mm ✅
- `q₂ = −π/2` (omuz dik) → TCP = `[0, 0, 318]` mm ✅

---

## 🖥️ Simülasyonu Çalıştırma

MATLAB R2016b+ yeterli, ek toolbox gerekmiyor:

```matlab
>> robot_arm_sim
```

**Özellikler:**
- 6 slider ile her eklemi bağımsız kontrol
- Gerçek mm ölçülerinde 3B görsel
- Anlık TCP (uç nokta) koordinatı
- Hazır pozlar: `Sıfırla (q=0)`, `Test 1 (yatay)`, `Test 2 (dik)`
- Karanlık tema arayüz

---

## ⚙️ Donanım Hedefleri

| Eklem | Servo | Tork |
|---|---|---|
| J1, J2, J3, J4 | MG996R | ~10 kg·cm |
| J5, J6 | SG90 | ~1.8 kg·cm |

Statik tork analizine göre **20 g payload**'a kadar güvenli. Kontrolcü: Arduino Uno + PCA9685 16-kanal PWM sürücü.

---

## 🗺️ Yol Haritası

- [x] FAZ 1 — Kinematik/dinamik teori
- [x] FAZ 1.5 — Modele özgü DH + servo doğrulama
- [x] FAZ 2 — MATLAB simülasyon
- [ ] FAZ 3 — CAD modelleme (Fusion 360)
- [ ] FAZ 4 — 3D baskı & montaj
- [ ] FAZ 5 — Arduino firmware
- [ ] FAZ 6 — Pick & place demo
- [ ] FAZ 7 — Dokümantasyon & video

---

## 📝 Lisans

Kişisel öğrenme & portfolyo projesi. Kod ve dokümanlar serbestçe incelenebilir.

---

## 🇬🇧 English

A 6-axis (6-DOF) robotic arm I'm designing from scratch: kinematic–dynamic theory, MATLAB simulation, and upcoming 3D-print + servo control phases.

This repo contains the **software/theory** side of the project. The hardware phase (CAD, 3D printing, Arduino + PCA9685 + MG996R/SG90) will be added in later releases.

### 🎯 Goals

- Learn robot kinematics & dynamics hands-on
- DH parameters, forward/inverse kinematics, Jacobian, static torque analysis
- Interactive MATLAB simulation in real mm dimensions
- Eventually build the physical arm and run a pick & place demo

### 📂 Repo Contents

| File | Description |
|---|---|
| [`01_kinematik_dinamik.md`](01_kinematik_dinamik.md) | PHASE 1 — DH convention, transforms, FK/IK, Jacobian, dynamics theory |
| [`02_model_DH_dinamik.md`](02_model_DH_dinamik.md) | **Exact DH parameters** derived from real STL/STP geometry, workspace, servo torque verification |
| [`03_matlab_simulasyon_egitim.md`](03_matlab_simulasyon_egitim.md) | Step-by-step explanation of the MATLAB simulation |
| [`matlab_simulasyon.m`](matlab_simulasyon.m) | Self-contained MATLAB simulation (Peter Corke RTB **not required**) |
| `sim_photo.png` | Screenshot from the simulation |
| `proje dosyalari/measure_parts.py` | Helper script for STL bbox measurement |
| `proje dosyalari/parse_assembly.py` | Helper script for STP assembly analysis |

### 📐 Robot Geometry

Standard DH (in mm):

| Link | θ | d | a | α |
|:---:|:---:|---:|---:|:---:|
| 1 | q₁ | 100 | 0 | −π/2 |
| 2 | q₂ | 0 | 138 | 0 |
| 3 | q₃ | 0 | 0 | −π/2 |
| 4 | q₄ | 130 | 0 | +π/2 |
| 5 | q₅ | 0 | 0 | −π/2 |
| 6 | q₆ | 50 | 0 | 0 |

**Configuration:** Anthropomorphic (human-arm-like) + spherical wrist → closed-form inverse kinematics.

**Verification tests:**
- `q = 0` → TCP = `[268, 0, 50]` mm ✅
- `q₂ = −π/2` (shoulder vertical) → TCP = `[0, 0, 318]` mm ✅

### 🖥️ Running the Simulation

MATLAB R2016b+ is enough, no extra toolbox required:

```matlab
>> robot_arm_sim
```

**Features:**
- 6 sliders for independent joint control
- 3D view in real mm dimensions
- Live TCP (end-effector) coordinates
- Preset poses: `Reset (q=0)`, `Test 1 (horizontal)`, `Test 2 (vertical)`
- Dark-theme UI

### ⚙️ Hardware Targets

| Joint | Servo | Torque |
|---|---|---|
| J1, J2, J3, J4 | MG996R | ~10 kg·cm |
| J5, J6 | SG90 | ~1.8 kg·cm |

Static torque analysis confirms safe operation up to **20 g payload**. Controller: Arduino Uno + PCA9685 16-channel PWM driver.

### 🗺️ Roadmap

- [x] PHASE 1 — Kinematics/dynamics theory
- [x] PHASE 1.5 — Model-specific DH + servo verification
- [x] PHASE 2 — MATLAB simulation
- [ ] PHASE 3 — CAD modeling (Fusion 360)
- [ ] PHASE 4 — 3D printing & assembly
- [ ] PHASE 5 — Arduino firmware
- [ ] PHASE 6 — Pick & place demo
- [ ] PHASE 7 — Documentation & video

### 📝 License

Personal learning & portfolio project. Code and docs are free to browse.

---

**Mustafa Berat Yılmaz**
