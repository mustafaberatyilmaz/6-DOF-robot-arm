# 🔧 FAZ 1 — Kinematik ve Dinamik Hesaplamalar

> **Amaç:** Robot kolun matematiksel modelini sıfırdan kurmak. DH parametreleri → dönüşüm matrisleri → ileri/ters kinematik → Jakobiyen → dinamik → servo tork doğrulaması.
>
> **Referans model:** MBera 6-DOF Arm + OmArm Zero benzeri yapı (3×MG996R + 3×SG90).
>
> **Konfigürasyon:** Antropomorfik (insan koluna benzer) 6-DOF + **küresel bilek** (son 3 ekseni tek noktada kesişir → ters kinematik kapalı-form çözülür).

---

## 0. Notasyon

| Sembol | Anlamı |
|--------|--------|
| $q_i = \theta_i$ | $i.$ eklem açısı (revolute eklem değişkeni) |
| $\dot q, \ddot q$ | Eklem hızı, ivmesi |
| $c_i = \cos\theta_i$, $s_i = \sin\theta_i$ | Kısaltma |
| $c_{ij} = \cos(\theta_i+\theta_j)$ | Toplam açı kısaltması |
| ${}^{i-1}T_i$ | $i-1$ çerçevesinden $i$ çerçevesine homojen dönüşüm |
| $R \in SO(3)$ | 3×3 dönme matrisi |
| $p \in \mathbb{R}^3$ | 3×1 pozisyon vektörü |
| $J \in \mathbb{R}^{6\times 6}$ | Geometrik Jakobiyen |
| $\tau$ | Eklem torku (Nm) |

---

## 1. Eksen Konfigürasyonu

Tabandan uç-işleyiciye doğru eklemler:

```
Eklem  Tip          Yön           Servo      Açıklama
──────────────────────────────────────────────────────────
J1     Revolute     z₀ ekseni     MG996R    Taban dönüşü (yaw)
J2     Revolute     y₁ ekseni     MG996R    Omuz pitch
J3     Revolute     y₂ ekseni     MG996R    Dirsek pitch
J4     Revolute     z₃ ekseni     SG90      Bilek roll  ┐
J5     Revolute     y₄ ekseni     SG90      Bilek pitch ├ küresel bilek
J6     Revolute     z₅ ekseni     SG90      Uç roll     ┘
```

Son üç eksenin **tek noktada kesiştiğine** dikkat — bu "spherical wrist" özelliği IK'yı çözülebilir yapar (Pieper kriteri).

### Çerçeve atama şeması (Standard DH konvansiyonu)

```
         z₆  ← uç işleyici
         ↑
        ┌─┐
        │J6│ z₅ ←┐
        ├─┤      │
        │J5│ y₄  │ küresel bilek
        ├─┤      │ (z₃, z₄, z₅ tek noktada)
        │J4│ z₃ ←┘
        ╞═╡
         │       d₄
         │       (önkol)
        ╞═╡
        │J3│ y₂      dirsek
        ├─┤
         │       a₂
         │       (üst kol)
        ╞═╡
        │J2│ y₁      omuz
        ├─┤
         │       d₁
         │       (taban kolonu)
        ┌─┴─┐
        │J1 │ z₀     taban dönüşü
        └───┘
        ═════════ zemin (çerçeve {0})
```

> Çerçeve {0} taban yere oturur, $z_0$ yukarı bakar. {6} uç işleyicidir.

---

## 2. Denavit-Hartenberg Parametre Tablosu

Standard DH konvansiyonu (Khalil-Dombre / Spong / Peter Corke RTB ile uyumlu):

| Link $i$ | $\theta_i$ | $d_i$ (mm) | $a_i$ (mm) | $\alpha_i$ (rad) | Aralık |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | $\theta_1$ | $d_1 = 80$ | 0 | $-\pi/2$ | $\pm 180°$ |
| 2 | $\theta_2$ | 0 | $a_2 = 120$ | 0 | $\pm 90°$ |
| 3 | $\theta_3$ | 0 | 0 | $-\pi/2$ | $\pm 135°$ |
| 4 | $\theta_4$ | $d_4 = 120$ | 0 | $+\pi/2$ | $\pm 180°$ |
| 5 | $\theta_5$ | 0 | 0 | $-\pi/2$ | $\pm 90°$ |
| 6 | $\theta_6$ | $d_6 = 80$ | 0 | 0 | $\pm 180°$ |

> ⚠️ **Önemli:** Yukarıdaki mm değerleri OmArm/EEZYbot benzeri tipik bir 3D-baskılı kol için **başlangıç tahminidir**. CAD modelden veya gerçek baskıdan ölçtüğünde MUTLAKA güncelle.

### Geometrik anlamı
- **$d_1 = 80$ mm:** Tabandan omuz eksenine kadar dikey yükseklik (taban kolonu)
- **$a_2 = 120$ mm:** Üst kol uzunluğu (omuz → dirsek)
- **$d_4 = 120$ mm:** Önkol uzunluğu (dirsek → bilek küresel merkez)
- **$d_6 = 80$ mm:** Bilek merkezinden uç işleyicinin kavrama noktasına

---

## 3. Tek Tek Dönüşüm Matrisleri

Standard DH dönüşümü:

$$
{}^{i-1}T_i =
\begin{bmatrix}
c_i & -s_i c_{\alpha_i} & s_i s_{\alpha_i} & a_i c_i \\
s_i & c_i c_{\alpha_i} & -c_i s_{\alpha_i} & a_i s_i \\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

DH tablosundaki sabitleri yerine koyarak her link için:

### ${}^0T_1$ ($\alpha_1 = -\pi/2$, $a_1=0$, $d_1$)

$$
{}^0T_1 = \begin{bmatrix} c_1 & 0 & -s_1 & 0 \\ s_1 & 0 & c_1 & 0 \\ 0 & -1 & 0 & d_1 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### ${}^1T_2$ ($\alpha_2 = 0$, $a_2$, $d_2=0$)

$$
{}^1T_2 = \begin{bmatrix} c_2 & -s_2 & 0 & a_2 c_2 \\ s_2 & c_2 & 0 & a_2 s_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### ${}^2T_3$ ($\alpha_3 = -\pi/2$, $a_3=0$, $d_3=0$)

$$
{}^2T_3 = \begin{bmatrix} c_3 & 0 & -s_3 & 0 \\ s_3 & 0 & c_3 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### ${}^3T_4$ ($\alpha_4 = +\pi/2$, $a_4=0$, $d_4$)

$$
{}^3T_4 = \begin{bmatrix} c_4 & 0 & s_4 & 0 \\ s_4 & 0 & -c_4 & 0 \\ 0 & 1 & 0 & d_4 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### ${}^4T_5$ ($\alpha_5 = -\pi/2$, $a_5=0$, $d_5=0$)

$$
{}^4T_5 = \begin{bmatrix} c_5 & 0 & -s_5 & 0 \\ s_5 & 0 & c_5 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### ${}^5T_6$ ($\alpha_6 = 0$, $a_6=0$, $d_6$)

$$
{}^5T_6 = \begin{bmatrix} c_6 & -s_6 & 0 & 0 \\ s_6 & c_6 & 0 & 0 \\ 0 & 0 & 1 & d_6 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

---

## 4. Toplam Dönüşüm Matrisi (İleri Kinematik — Symbolic)

$$
{}^0T_6 = {}^0T_1 \cdot {}^1T_2 \cdot {}^2T_3 \cdot {}^3T_4 \cdot {}^4T_5 \cdot {}^5T_6
$$

Bu çarpımı parça parça toplamak için ara matrisleri tanımla:

$$
{}^0T_3 = {}^0T_1 \cdot {}^1T_2 \cdot {}^2T_3 \quad\Rightarrow\quad \text{(omuz+dirsek alt-zinciri)}
$$

$$
{}^3T_6 = {}^3T_4 \cdot {}^4T_5 \cdot {}^5T_6 \quad\Rightarrow\quad \text{(küresel bilek alt-zinciri)}
$$

### 4.1 ${}^0T_3$ kapalı formu

Üç matrisin çarpımı (yorucu ama mekanik):

$$
{}^0T_3 = \begin{bmatrix}
c_1 c_{23} & -c_1 s_{23} & -s_1 & a_2 c_1 c_2 \\
s_1 c_{23} & -s_1 s_{23} &  c_1 & a_2 s_1 c_2 \\
-s_{23}    & -c_{23}     &  0    & d_1 - a_2 s_2 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Burada $c_{23} = \cos(\theta_2+\theta_3)$, $s_{23}=\sin(\theta_2+\theta_3)$.

### 4.2 ${}^3T_6$ kapalı formu (küresel bilek = ZYZ benzeri Euler)

$$
{}^3T_6 = \begin{bmatrix}
c_4 c_5 c_6 - s_4 s_6 & -c_4 c_5 s_6 - s_4 c_6 & c_4 s_5 & c_4 s_5 d_6 \\
s_4 c_5 c_6 + c_4 s_6 & -s_4 c_5 s_6 + c_4 c_6 & s_4 s_5 & s_4 s_5 d_6 \\
-s_5 c_6 & s_5 s_6 & c_5 & c_5 d_6 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

> Bu yapı klasik **ZYZ Euler** açılarına eşdeğer; bilek "yaw-pitch-yaw" gibi davranır.

### 4.3 Bilek merkezi pozisyonu (kritik!)

Küresel bilek merkezi $p_w$ — son üç eksenin kesiştiği nokta:

$$
p_w = p_{\text{tcp}} - d_6 \cdot \hat{z}_6
$$

burada $\hat{z}_6$ uç işleyicinin yaklaşma yönü ($R_6$ matrisinin 3. sütunu).

Bu noktayı bilmek IK'yı **iki bağımsız probleme** ayırır — büyük zafer.

---

## 5. İleri Kinematik — Sayısal Doğrulama

**Test 1:** Tüm açılar sıfır $q = [0,0,0,0,0,0]$.

DH'ten elle hesap:
- ${}^0T_1$: $c_1=1, s_1=0$ → uç {1}'in pozisyonu $(0,0,d_1)$, yönelim $z_1$ orijinal $-y_0$ yönünde.
- ${}^0T_2$: $a_2$ kadar $x_2$ ekseninde öteleme, vs.

Birikimli olarak $q=0$'da uç işleyici pozisyonu **kapalı form**:

$$
p_{0\to 6}\big|_{q=0} = \begin{bmatrix} a_2 + d_4 \\ 0 \\ d_1 - d_6 \end{bmatrix} = \begin{bmatrix} 120 + 120 \\ 0 \\ 80 - 80 \end{bmatrix} = \begin{bmatrix} 240 \\ 0 \\ 0 \end{bmatrix} \text{ mm}
$$

> Bu, yatay uzanmış bir kol ile uyumlu. Eğer fiziksel ölçümün farklıysa DH tablonu güncelle.

**Test 2:** $q = [0, -\pi/2, 0, 0, 0, 0]$ (omuz dik yukarı):

$$
p \approx \begin{bmatrix} 0 \\ 0 \\ d_1 + a_2 + d_4 - d_6 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 240 \end{bmatrix} \text{ mm}
$$

(Tam dik kol — maksimum yükseklik.)

### 5.1 MATLAB doğrulama kodu

```matlab
% DH parametreleri (mm cinsinden)
d1 = 80;  a2 = 120;  d4 = 120;  d6 = 80;

L(1) = Link('d', d1, 'a', 0,  'alpha', -pi/2);
L(2) = Link('d', 0,  'a', a2, 'alpha', 0);
L(3) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
L(4) = Link('d', d4, 'a', 0,  'alpha',  pi/2);
L(5) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
L(6) = Link('d', d6, 'a', 0,  'alpha', 0);

robot = SerialLink(L, 'name', 'MyArm6DOF');

% Test 1: tüm sıfır
q0 = [0 0 0 0 0 0];
T0 = robot.fkine(q0);
disp('Test 1 (q=0):'); disp(T0.t');   % Beklenen: [240, 0, 0]

% Test 2: omuz -90°
q1 = [0 -pi/2 0 0 0 0];
T1 = robot.fkine(q1);
disp('Test 2 (omuz dik):'); disp(T1.t');   % Beklenen: [0, 0, 240]

robot.teach();   % İnteraktif
```

### 5.2 Python doğrulama (NumPy, RTB'siz)

```python
import numpy as np

def dh_T(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d  ],
        [0,   0,      0,     1  ]
    ])

def forward_kin(q, params):
    d1, a2, d4, d6 = params
    T = np.eye(4)
    T = T @ dh_T(q[0], d1, 0,  -np.pi/2)
    T = T @ dh_T(q[1], 0,  a2, 0)
    T = T @ dh_T(q[2], 0,  0,  -np.pi/2)
    T = T @ dh_T(q[3], d4, 0,  np.pi/2)
    T = T @ dh_T(q[4], 0,  0,  -np.pi/2)
    T = T @ dh_T(q[5], d6, 0,  0)
    return T

params = (80, 120, 120, 80)
print(forward_kin([0,0,0,0,0,0], params)[:3, 3])   # [240, 0, 0]
print(forward_kin([0,-np.pi/2,0,0,0,0], params)[:3, 3])  # [0, 0, 240]
```

---

## 6. Ters Kinematik (Inverse Kinematics)

**Girdi:** Hedef poz $T^{\text{des}} = (R^{\text{des}}, p^{\text{des}})$
**Çıktı:** $q_1, \ldots, q_6$ (genelde 8 olası çözüm — sağ/sol omuz × yukarı/aşağı dirsek × bilek flip)

Küresel bilek özelliği sayesinde **iki bağımsız problem**:

### 6.1 Pozisyon problemi (q1, q2, q3) — geometrik yaklaşım

#### Adım 1: Bilek merkezini bul

$$
p_w = p^{\text{des}} - d_6 \cdot R^{\text{des}} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

#### Adım 2: $\theta_1$ — taban dönüşü

$$
\boxed{\theta_1 = \text{atan2}(p_{w,y},\; p_{w,x})}
$$

(Diğer çözüm: $\theta_1 + \pi$ → "kol arkadan uzanır" konfigürasyonu.)

#### Adım 3: $\theta_2, \theta_3$ — düzlemsel 2-link problemi

Tabandan ($z_0$ ekseni) bilek merkezine olan **yatay** ve **dikey** mesafeleri hesapla:

$$
r = \sqrt{p_{w,x}^2 + p_{w,y}^2}, \qquad s = p_{w,z} - d_1
$$

Şimdi $r$-$s$ düzleminde uzunlukları $a_2$ ve $d_4$ olan iki bağlantılı kol problemi (**kosinüs teoremi**):

$$
D = \frac{r^2 + s^2 - a_2^2 - d_4^2}{2 a_2 d_4}
$$

$D$ değeri $[-1,+1]$ aralığında değilse hedef **erişilemez** (workspace dışı).

$$
\boxed{\theta_3 = \text{atan2}\!\left(\pm\sqrt{1-D^2},\; D\right)}
$$

İki çözüm: "+" → dirsek aşağı (elbow-down), "−" → dirsek yukarı (elbow-up).

$$
\boxed{\theta_2 = \text{atan2}(s, r) - \text{atan2}\!\left(d_4 \sin\theta_3,\; a_2 + d_4 \cos\theta_3\right)}
$$

### 6.2 Yönelim problemi (q4, q5, q6) — Euler decomposition

Bilek merkezine kadar olan dönmeyi hesapla:

$$
{}^0R_3 = {}^0R_3(\theta_1, \theta_2, \theta_3) \quad \text{(yukarıdaki }{}^0T_3\text{'ten)}
$$

Sonra son üç eksene düşen dönme:

$$
{}^3R_6 = ({}^0R_3)^T \cdot R^{\text{des}}
$$

${}^3R_6$ ZYZ Euler açılarına ayrıştırılır:

$$
\boxed{
\theta_5 = \text{atan2}\!\left(\sqrt{r_{13}^2 + r_{23}^2},\; r_{33}\right)
}
$$

$\theta_5 \neq 0$ ise (tekillik dışı):

$$
\boxed{
\theta_4 = \text{atan2}(r_{23},\; r_{13}), \qquad
\theta_6 = \text{atan2}(r_{32},\; -r_{31})
}
$$

burada $r_{ij}$ matrisi ${}^3R_6$'nın elemanlarıdır.

> **Tekillik:** $\theta_5 = 0$ veya $\theta_5 = \pi$ → bilek tekilliği ("gimbal lock"). $\theta_4$ ve $\theta_6$ tek tek belirlenemez, sadece toplamları $\theta_4 + \theta_6$ belirlidir. Pratikte: $\theta_4 = 0$ atayıp $\theta_6 = \text{atan2}(r_{12}, r_{11})$ alınır.

### 6.3 Çözüm seçimi

8 olası çözüm arasından **eklem limitlerine giren** ve **mevcut konfigürasyona en yakın** olan seçilir. Ölçüt:

$$
\arg\min_k \sum_{i=1}^{6} (q_i^{(k)} - q_i^{\text{current}})^2
$$

### 6.4 Pseudokod

```python
def inverse_kin(T_des, params):
    d1, a2, d4, d6 = params
    R_des = T_des[:3,:3]
    p_des = T_des[:3, 3]
    
    # 1. Bilek merkezi
    p_w = p_des - d6 * R_des[:, 2]
    
    # 2. theta1
    theta1 = np.arctan2(p_w[1], p_w[0])
    
    # 3. theta2, theta3 (kosinüs teoremi)
    r = np.hypot(p_w[0], p_w[1])
    s = p_w[2] - d1
    D = (r**2 + s**2 - a2**2 - d4**2) / (2*a2*d4)
    if abs(D) > 1: raise ValueError("Hedef workspace dışı")
    theta3 = np.arctan2(-np.sqrt(1-D**2), D)   # elbow-up
    theta2 = np.arctan2(s, r) - np.arctan2(d4*np.sin(theta3),
                                            a2 + d4*np.cos(theta3))
    
    # 4. R_3^0 (FK'den)
    R03 = ...  # theta1, theta2, theta3'ten yukarıdaki ${}^0R_3$
    R36 = R03.T @ R_des
    
    # 5. ZYZ Euler ayrıştırma
    theta5 = np.arctan2(np.hypot(R36[0,2], R36[1,2]), R36[2,2])
    if abs(theta5) > 1e-6:
        theta4 = np.arctan2(R36[1,2], R36[0,2])
        theta6 = np.arctan2(R36[2,1], -R36[2,0])
    else:  # tekillik
        theta4 = 0
        theta6 = np.arctan2(R36[0,1], R36[0,0])
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]
```

---

## 7. Diferansiyel Kinematik — Geometrik Jakobiyen

Eklem hızlarını uç-işleyici hızına çevirir:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} = J(q) \dot q, \qquad J \in \mathbb{R}^{6\times 6}
$$

Her sütun şu yapıda (revolute eklemler için):

$$
J_i = \begin{bmatrix} z_{i-1} \times (p_n - p_{i-1}) \\ z_{i-1} \end{bmatrix}
$$

burada:
- $z_{i-1}$: ${}^0T_{i-1}$ matrisinin 3. sütununun ilk 3 elemanı (eklem ekseni dünya çerçevesinde)
- $p_{i-1}$: ${}^0T_{i-1}$ matrisinin 4. sütununun ilk 3 elemanı
- $p_n$: uç-işleyici pozisyonu (${}^0T_6$'dan)

### 7.1 Tekillikler

$\det J(q) = 0$ olan konfigürasyonlar. Bu robot için **3 tipik tekillik:**

1. **Omuz tekilliği:** $p_w$ tabanın $z_0$ ekseni üzerinde → $\theta_1$ belirsiz.
2. **Dirsek tekilliği:** Kol tam uzanmış ($\theta_3 = 0$ veya $\pi$) → workspace sınırı.
3. **Bilek tekilliği:** $\theta_5 = 0$ → $\theta_4$ ve $\theta_6$ aynı işi yapar.

Trajektori planlamada bu noktalardan **kaçın** (path planner'a yumuşak büküm ekle).

### 7.2 Sayısal IK (Newton-Raphson)

Kapalı-form çözüm yoksa veya hedef tekilliğe yakınsa:

$$
q_{k+1} = q_k + J^{\dagger}(q_k) \cdot e_k
$$

burada $e_k$ poz hatası (pozisyon + yönelim), $J^{\dagger}$ pseudo-inverse veya damped least-squares (DLS):

$$
J^{\dagger}_{\text{DLS}} = J^T (JJ^T + \lambda^2 I)^{-1}
$$

$\lambda \approx 0.01$ tekilliklerde patlamayı önler.

---

## 8. Dinamik Modelleme

Robot dinamiğinin standart formu:

$$
\boxed{\tau = M(q)\ddot q + C(q,\dot q)\dot q + g(q) + \tau_{\text{fric}}}
$$

| Terim | Anlamı | Servo seçimi için önemi |
|-------|--------|-------------------------|
| $M(q)$ | Eylemsizlik (kütle) matrisi 6×6 | Yüksek ivmeli hareketlerde |
| $C(q,\dot q)\dot q$ | Coriolis + merkezkaç | Hızlı dönüşlerde |
| $g(q)$ | Yerçekimi torku | **Statik tutmada en kritik** |
| $\tau_{\text{fric}}$ | Sürtünme | Düşük hızda büyük |

### 8.1 Statik tork analizi (yerçekimi terimi $g(q)$)

3D-baskılı kolun yavaş hareket ettiğini varsayarsak $\ddot q \approx 0$ ve $\dot q$ küçük → sadece $g(q)$ önemli. Bu **servo seçimi için zorunlu hesap**.

#### Tahmini kütle ve geometri

| Link | Kütle (g) | Uzunluk (mm) | Kütle merkezi (link tabanından) |
|------|----------|---------------|-------------------------------|
| Üst kol (link 2) | $m_2 \approx 80$ | $a_2 = 120$ | $\ell_{c2} \approx 60$ mm |
| Önkol (link 3-4) | $m_3 \approx 100$ | $d_4 = 120$ | $\ell_{c3} \approx 60$ mm |
| Bilek + gripper | $m_w \approx 120$ | — | bilek merkezinde |
| Ek yük (cisim) | $m_{\text{load}} \approx 50$ | — | gripper ucunda |

> Bu değerler 3D baskılı PLA + servoların kütleleri için **tipik**. Gerçek baskıdan tartım yapıp güncellemen lazım. Servoların kendisi 55g (MG996R) ve 9g (SG90).

#### En kötü statik durum: kol yatay uzanmış

$\theta_2 = 0$ (üst kol yatay), $\theta_3 = 0$ (dirsek düz):

**Omuz eklemi (J2) için statik tork:**

$$
\tau_2^{\text{stat}} = g \cdot \left[ m_2 \ell_{c2} + m_3 (a_2 + \ell_{c3}) + (m_w + m_{\text{load}})(a_2 + d_4) \right]
$$

Sayısal (g = 9.81 m/s², kütleler kg, uzunluklar m):

$$
\tau_2 = 9.81 \cdot [0.08 \cdot 0.06 + 0.10 \cdot (0.12 + 0.06) + (0.12+0.05) \cdot (0.12 + 0.12)]
$$

$$
\tau_2 = 9.81 \cdot [0.0048 + 0.018 + 0.0408] = 9.81 \cdot 0.0636 \approx 0.624 \text{ Nm}
$$

Birime çevir: $1 \text{ Nm} \approx 10.2 \text{ kgf·cm}$:

$$
\tau_2 \approx 6.36 \text{ kgf·cm}
$$

#### MG996R kapasitesi

- **MG996R nominal stall tork:** ~10–11 kgf·cm @ 6V
- **Güvenlik faktörü:** 2× → kullanılabilir tork ~5–5.5 kgf·cm

**SONUÇ:** $\tau_2 \approx 6.36$ kgf·cm > 5.5 kgf·cm → MG996R **sınırda kalıyor**, yük 50g'dan ağırsa yetmez.

#### ⚠️ Aksiyon listesi

1. **Yük kapasitesini düşür** (max 30g uç-yükü) → güvenli aralığa gel
2. **VEYA** omuz servosunu yükselt: **DS3218** (~20 kgf·cm) çok rahat
3. **VEYA** mekanik avantaj ekle: omuza **karşıt yay** veya **sayaç-ağırlık** koy → servo yükü %40-50 düşer
4. Önkol ağırlığını azalt: infill %25, daha hafif PLA

**Dirsek eklemi (J3) için:**

$$
\tau_3^{\text{stat}} = g \cdot [m_3 \ell_{c3} + (m_w + m_{\text{load}}) d_4]
$$

$$
\tau_3 = 9.81 \cdot [0.10 \cdot 0.06 + 0.17 \cdot 0.12] = 9.81 \cdot 0.0264 \approx 0.259 \text{ Nm} \approx 2.64 \text{ kgf·cm}
$$

Bu MG996R için rahat. ✅

**Taban (J1):** Yerçekimi torku üretmez (eksen dikey) — sadece atalet ve sürtünme. ✅

**Bilek (J4, J5, J6):** $\tau \approx g \cdot m_{\text{gripper}} \cdot \ell \approx 0.05 \cdot 9.81 \cdot 0.05 = 0.025$ Nm $\approx 0.25$ kgf·cm. SG90 (~1.8 kgf·cm) çok rahat. ✅

### 8.2 Lagrange yöntemiyle tam dinamik

Tam $M(q), C(q,\dot q), g(q)$ türetmek elle çok yorucu (6 eklemde 6×6=36 terim). Pratikte:
- **MATLAB Symbolic + Robotics Toolbox:** `robot.gravity_load(q)`, `robot.inertia(q)`, `robot.coriolis(q,qd)`
- **Otomatik üretim:** `robot.rne(q, qd, qdd)` → Newton-Euler ters dinamik
- **Simscape:** CAD'den URDF export → tüm kütle/atalet matrisleri otomatik

### 8.3 MATLAB ile tam dinamik kontrolü

```matlab
% Linklere kütle ve eylemsizlik ata (PLA + servo)
robot.links(1).m = 0.150;   robot.links(1).r = [0; 0; -0.04];
robot.links(2).m = 0.080;   robot.links(2).r = [-0.06; 0; 0];
robot.links(3).m = 0.100;   robot.links(3).r = [-0.06; 0; 0];
robot.links(4).m = 0.060;   robot.links(4).r = [0; 0; -0.06];
robot.links(5).m = 0.030;
robot.links(6).m = 0.030;
% Eylemsizlik tensörü (homojen prizma yaklaşımı, kg.m^2)
for i=1:6
    robot.links(i).I = diag([1e-4, 1e-4, 1e-4]);
end

% Tüm konfigürasyonlarda yerçekimi torku tarayışı
q_test = [0, 0, 0, 0, 0, 0];
tau_g = robot.gravload(q_test);   % Nm cinsinden, 1x6 vektör
tau_g_kgcm = tau_g * 10.2;         % kgf·cm'e çevir
disp('Yerçekimi torku (kgf·cm):'); disp(tau_g_kgcm);

% En kötü durum tarayışı
q2_range = -pi/2:0.1:pi/2;
q3_range = -pi/2:0.1:pi/2;
max_tau2 = 0;
for q2 = q2_range
    for q3 = q3_range
        tau = robot.gravload([0 q2 q3 0 0 0]);
        if abs(tau(2)) > max_tau2
            max_tau2 = abs(tau(2));
            worst_q = [q2 q3];
        end
    end
end
fprintf('Max omuz torku: %.2f kgf·cm @ q2=%.2f, q3=%.2f\n', ...
        max_tau2*10.2, worst_q(1), worst_q(2));
```

---

## 9. Trajektori Planlama (Dinamiğe köprü)

Eklem uzayında kübik spline interpolasyon (FAZ 5'in zemini):

$$
q_i(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$

Sınır koşulları: $q_i(0)=q_i^{\text{start}}$, $q_i(T)=q_i^{\text{end}}$, $\dot q_i(0)=\dot q_i(T)=0$.

```matlab
[q, qd, qdd] = jtraj([0 0 0 0 0 0], [pi/2 -pi/3 pi/4 0 pi/6 0], 0:0.05:3);
robot.plot(q);
% Tork hesabı (her zaman adımında)
tau_traj = robot.rne(q, qd, qdd);
plot(0:0.05:3, tau_traj * 10.2);   % kgf·cm cinsinden
legend('J1','J2','J3','J4','J5','J6');
title('Trajektori boyunca eklem torkları');
```

Bu grafik servoların **dinamik** yük altında nasıl davrandığını gösterir — statik analizden daha gerçekçi.

---

## 10. Workspace (Çalışma Uzayı) Görselleştirme

Eklem aralıkları içinde rastgele örnekleme + uç-işleyici noktasını çiz:

```matlab
N = 20000;
qmin = [-pi, -pi/2, -3*pi/4, -pi, -pi/2, -pi];
qmax = [ pi,  pi/2,  3*pi/4,  pi,  pi/2,  pi];
P = zeros(N,3);
for k=1:N
    q_rand = qmin + (qmax-qmin).*rand(1,6);
    T = robot.fkine(q_rand);
    P(k,:) = T.t';
end
scatter3(P(:,1), P(:,2), P(:,3), 1, '.');
axis equal; grid on;
title('Erişilebilir Çalışma Uzayı');
```

Bu grafik LinkedIn paylaşımı için **mükemmel görsel**.

---

## 11. Bu Aşamanın Çıktıları (Checklist)

- [ ] Bu doküman okundu ve çerçeve atama el çizimi yapıldı (kağıt + kalem)
- [ ] Robot fiziksel parçaları üzerinde DH ölçümü yapıldı, mm değerleri güncellendi
- [ ] MATLAB'da `forward_kin` çağrısı Test 1 ve Test 2 ile doğrulandı
- [ ] Python'da bağımsız FK kodu yazıldı, MATLAB ile karşılaştırıldı (hata < 1e-9)
- [ ] IK fonksiyonu yazıldı, en az 5 hedef poz için çözüldü
- [ ] Statik tork hesabı yapıldı, MG996R'nin yetip yetmediğine karar verildi
- [ ] (Opsiyonel) Dinamik tork tarayışı animasyonu hazırlandı

---

## 12. Bir Sonraki Adım: FAZ 2 — MATLAB Simülasyonu

FAZ 1 doğrulandığında:
1. **Robotics Toolbox** kurulumu (Peter Corke RTB)
2. SerialLink modeli + `robot.teach()` GUI
3. 3 farklı hedef poz için IK çözümü (kapalı + sayısal karşılaştırma)
4. Trajektori animasyonu → MP4 export → LinkedIn post #2 hazır
5. Workspace görselleştirme → portfolyo görseli

---

## 13. Hata Ayıklama İpuçları

| Belirti | Olası neden | Çözüm |
|---------|-------------|-------|
| FK sonuçları MATLAB ile uyuşmuyor | DH konvansiyonu (modified vs standard) karışmış | Tek bir konvansiyona bağlı kal — bu doküman **standard** kullanıyor |
| IK saçma açılar veriyor | $\theta_3$ işareti yanlış | "elbow-up" / "elbow-down" seçimini test et |
| Tekillikte IK NaN | Damped least-squares uygulanmamış | $\lambda = 0.01$ ile DLS kullan |
| Servo titriyor | Tork yetersiz veya güç düşüyor | Statik tork tarayışı + 5V 5A SMPS şart |
| MATLAB `robot.plot` çalışmıyor | RTB versiyonu eski | Peter Corke RTB v10+ indir |

---

## 14. Kaynaklar (Faz 1 derinleştirme)

- 📘 Siciliano, Sciavicco — *Robotics: Modelling, Planning and Control* — Bölüm 2 (Kinematics), Bölüm 7 (Dynamics)
- 📘 Spong, Hutchinson, Vidyasagar — *Robot Modeling and Control* — Bölüm 3-4
- 📘 Peter Corke — *Robotics, Vision and Control* (MATLAB ile birlikte) — Bölüm 7-9
- 🎥 Angela Sodemann YouTube — *Introduction to Robotics* serisi (DH ve IK için)
- 🎥 Robot Academy (queenslandrobotics.com) — Peter Corke video dersleri
- 📄 [Pieper Criterion proof](https://en.wikipedia.org/wiki/Inverse_kinematics) — küresel bilek IK kapalı-formu için

---

*Versiyon: 1.0 — 2026-04-26*
*Kaynak: Standard DH konvansiyonu (Spong/Corke). Modified DH (Khalil) kullanmıyoruz.*
