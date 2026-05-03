function robot_arm_sim()
% =========================================================================
%  6-DOF Robot Kol Simulasyonu  -  FAZ 2
%  Model: MBera 6-DOF Arm V1.0
%
%  Bu dosya kendi kendine yeter (Peter Corke RTB GEREKMEZ).
%  Calistirmak icin: MATLAB komut satirinda  >> robot_arm_sim
%
%  Ozellikler:
%   - 6 slider ile her eklemi bagimsiz kontrol
%   - Patch/surf tabanli basit 3B gorsel (gercek mm olculerinde)
%   - TCP (uc nokta) konumu ve eklem acilarini anlik gosterim
%   - "Sifirla" ve hazir poz dugmeleri (Test 1, Test 2)
%
%  DH parametreleri 02_model_DH_dinamik.md dosyasindan alinmistir:
%      d1 = 100 mm    a2 = 138 mm    d4 = 130 mm    d6 = 50 mm
%      Test 1 (q=0)        -> TCP = [268,   0,  50]  mm
%      Test 2 (q2=-pi/2)   -> TCP = [  0,   0, 318]  mm
% =========================================================================

%% --- 1) DH parametreleri (Standard DH) ----------------------------------
% Sira:    | theta | d    | a    | alpha
% Link 1:  |  q1   | 100  |   0  | -pi/2
% Link 2:  |  q2   |   0  | 138  |    0
% Link 3:  |  q3   |   0  |   0  | -pi/2
% Link 4:  |  q4   | 130  |   0  | +pi/2
% Link 5:  |  q5   |   0  |   0  | -pi/2
% Link 6:  |  q6   |  50  |   0  |    0
DH.d     = [100,   0,   0, 130,   0,  50];
DH.a     = [  0, 138,   0,   0,   0,   0];
DH.alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];

%% --- 2) Eklem limitleri (servo gercekciligi icin) -----------------------
qmin = deg2rad([-90, -90, -135, -90, -90, -180]);
qmax = deg2rad([ 90,  90,   90,  90,  90,  180]);
q    = zeros(1,6);            % baslangic pozu

%% --- 3) Figure ve 3B eksen ---------------------------------------------
bgCol = [0.08 0.08 0.10];   % koyu arka plan
fgCol = [0.92 0.92 0.95];   % acik metin / eksen rengi

fig = figure('Name','6-DOF Robot Kol - FAZ 2 Simulasyon', ...
             'Color',bgCol, ...
             'Position',[100 80 1180 720]);

ax = axes('Parent',fig,'Position',[0.05 0.10 0.62 0.86], ...
          'Color',bgCol, ...
          'XColor',fgCol,'YColor',fgCol,'ZColor',fgCol, ...
          'GridColor',[0.55 0.55 0.60],'GridAlpha',0.4);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,135,22);
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
xlim(ax,[-400 400]); ylim(ax,[-400 400]); zlim(ax,[-50 500]);
title(ax,'MBera 6-DOF Arm  -  Kinematik Simulasyon','Color',fgCol);
camlight(ax,'headlight'); lighting(ax,'gouraud');

% Zemin izgara (referans)
[Xg,Yg] = meshgrid(-300:60:300, -300:60:300);
Zg = zeros(size(Xg));
mesh(ax, Xg, Yg, Zg, 'EdgeColor',[0.45 0.45 0.50], 'FaceAlpha',0);

%% --- 4) UI elemanlari (slider + label + butonlar) -----------------------
S.sliders = gobjects(1,6);
S.qlabels = gobjects(1,6);
S.linkH   = gobjects(1,6);
S.jointH  = gobjects(1,7);
S.frameH  = gobjects(1,7);
S.tcpDot  = gobjects(1);
S.tcpTxt  = gobjects(1);

panelX = 0.70;  panelY0 = 0.18;  panelW = 0.28;  rowH = 0.085;
sliderNames = {'J1  Yaw   (taban)', ...
               'J2  Pitch (omuz)', ...
               'J3  Pitch (dirsek)', ...
               'J4  Roll  (onkol)', ...
               'J5  Pitch (bilek)', ...
               'J6  Roll  (uc)'};

uicontrol(fig,'Style','text','String','Eklem Kontrolleri (deg)', ...
    'Units','normalized','Position',[panelX panelY0+6*rowH+0.04 panelW 0.035], ...
    'FontWeight','bold','FontSize',12, ...
    'BackgroundColor',bgCol,'ForegroundColor',fgCol);

for i = 1:6
    yi = panelY0 + (6-i)*rowH;
    uicontrol(fig,'Style','text','String',sliderNames{i}, ...
        'Units','normalized','Position',[panelX yi+0.045 panelW 0.022], ...
        'BackgroundColor',bgCol,'ForegroundColor',fgCol, ...
        'HorizontalAlignment','left','FontSize',10);

    S.sliders(i) = uicontrol(fig,'Style','slider', ...
        'Units','normalized', ...
        'Min',qmin(i),'Max',qmax(i),'Value',q(i), ...
        'Position',[panelX yi+0.020 panelW*0.74 0.022]);
    % Surekli surukleyince guncelleme:
    addlistener(S.sliders(i),'Value','PostSet', @(~,~) onSlider(i));

    S.qlabels(i) = uicontrol(fig,'Style','text', ...
        'Units','normalized', ...
        'Position',[panelX+panelW*0.76 yi+0.020 panelW*0.24 0.022], ...
        'String',sprintf('%6.1f',rad2deg(q(i))), ...
        'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',fgCol,'FontSize',10);
end

uicontrol(fig,'Style','pushbutton','String','Sifirla (q=0)', ...
    'Units','normalized', ...
    'Position',[panelX panelY0-0.07 panelW*0.32 0.045], ...
    'Callback', @(~,~) onPreset(zeros(1,6)));

uicontrol(fig,'Style','pushbutton','String','Test 1 (yatay)', ...
    'Units','normalized', ...
    'Position',[panelX+panelW*0.34 panelY0-0.07 panelW*0.32 0.045], ...
    'Callback', @(~,~) onPreset(zeros(1,6)));

uicontrol(fig,'Style','pushbutton','String','Test 2 (dik)', ...
    'Units','normalized', ...
    'Position',[panelX+panelW*0.68 panelY0-0.07 panelW*0.32 0.045], ...
    'Callback', @(~,~) onPreset([0 -pi/2 0 0 0 0]));

S.info = uicontrol(fig,'Style','text', ...
    'Units','normalized','Position',[panelX 0.78 panelW 0.18], ...
    'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',fgCol, ...
    'HorizontalAlignment','left','FontSize',10, ...
    'String','TCP: -');

drawRobot();   % ilk cizim

% =========================================================================
%                         IC (NESTED) FONKSIYONLAR
% =========================================================================
    function onSlider(idx)
        q(idx) = S.sliders(idx).Value;
        S.qlabels(idx).String = sprintf('%6.1f',rad2deg(q(idx)));
        drawRobot();
    end

    function onPreset(qp)
        for k = 1:6
            q(k) = max(qmin(k), min(qmax(k), qp(k)));
            S.sliders(k).Value   = q(k);
            S.qlabels(k).String  = sprintf('%6.1f',rad2deg(q(k)));
        end
        drawRobot();
    end

    function drawRobot()
        % Ileri kinematik -> her eklemin global donusumu
        [origins, Ts] = forwardKin(q, DH);

        % Eski grafikleri temizle
        delete(S.linkH(isgraphics(S.linkH)));
        delete(S.jointH(isgraphics(S.jointH)));
        delete(S.frameH(isgraphics(S.frameH)));
        if isgraphics(S.tcpDot), delete(S.tcpDot); end
        if isgraphics(S.tcpTxt), delete(S.tcpTxt); end

        % --- Linkler (fiziksel olarak uzunluga sahip olanlar) ---
        % Link 1: Base + J1 housing  (dikey kolon, d1=100)
        S.linkH(1) = drawCyl(ax, origins(1,:), origins(2,:), 32, [0.28 0.45 0.82]);
        % Link 2: Omuz brackets (a2=138, ust kol)
        S.linkH(2) = drawBox(ax, origins(2,:), origins(3,:), 30, 22, [0.85 0.55 0.20]);
        % Link 4: Onkol tubu (d4=130)
        S.linkH(4) = drawCyl(ax, origins(4,:), origins(5,:), 16, [0.25 0.65 0.45]);
        % Link 6: TCP plakasi (d6=50)
        S.linkH(6) = drawCyl(ax, origins(6,:), origins(7,:), 12, [0.30 0.30 0.30]);

        % --- Eklem kureler ---
        rJ = [28 22 20 18 16 13 0];   % son nokta TCP, sphere yok
        for i = 1:6
            S.jointH(i) = drawSphere(ax, origins(i,:), rJ(i), [0.20 0.20 0.20]);
        end

        % --- Frame eksenleri (kirmizi=X, yesil=Y, mavi=Z) ---
        for i = 1:7
            S.frameH(i) = drawFrame(ax, Ts(:,:,i), 30);
        end

        % --- TCP isaretciisi ---
        tcp = origins(7,:);
        S.tcpDot = plot3(ax, tcp(1), tcp(2), tcp(3), 'ro', ...
                         'MarkerSize',10,'MarkerFaceColor','r');
        S.tcpTxt = text(ax, tcp(1)+20, tcp(2)+20, tcp(3)+20, ...
                        sprintf('TCP\n[%.0f %.0f %.0f]',tcp), ...
                        'FontSize',9,'BackgroundColor','w');

        % --- Bilgi paneli ---
        S.info.String = sprintf([ ...
            'TCP (mm):\n   X = %7.1f\n   Y = %7.1f\n   Z = %7.1f\n\n' ...
            'q (deg):\n   J1 = %6.1f\n   J2 = %6.1f\n   J3 = %6.1f\n' ...
            '   J4 = %6.1f\n   J5 = %6.1f\n   J6 = %6.1f'], ...
            tcp(1), tcp(2), tcp(3), rad2deg(q));

        drawnow limitrate;
    end
end  % ===== robot_arm_sim sonu =============================================


% =========================================================================
%                       YARDIMCI (DIS) FONKSIYONLAR
% =========================================================================
function [origins, Ts] = forwardKin(q, DH)
% Tum eklem orijinlerini ve donusum matrislerini hesaplar.
% origins: 7x3 (taban + 6 eklem)   Ts: 4x4x7
T  = eye(4);
Ts = repmat(eye(4),[1,1,7]);
origins = zeros(7,3);
for i = 1:6
    T = T * dhA(q(i), DH.d(i), DH.a(i), DH.alpha(i));
    Ts(:,:,i+1)    = T;
    origins(i+1,:) = T(1:3,4)';
end
end

function A = dhA(theta, d, a, alpha)
% Standard DH donusum matrisi (4x4).
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
A = [ ct, -st*ca,  st*sa, a*ct;
      st,  ct*ca, -ct*sa, a*st;
       0,     sa,     ca,    d;
       0,      0,      0,    1];
end

function h = drawCyl(ax, p1, p2, r, color)
% p1 -> p2 arasinda yaricapi r olan silindir.
v = p2 - p1;  L = norm(v);
if L < 1e-6
    h = patch('XData',[],'YData',[],'ZData',[],'Parent',ax); return;
end
[X,Y,Z] = cylinder(r, 24);
Z = Z * L;
R = vecToRot(v / L);
P = [X(:) Y(:) Z(:)] * R';
P = P + p1;
X(:) = P(:,1); Y(:) = P(:,2); Z(:) = P(:,3);
h = surf(ax, X, Y, Z, 'FaceColor',color, 'EdgeColor','none', ...
         'FaceLighting','gouraud','AmbientStrength',0.45);
end

function h = drawBox(ax, p1, p2, w, hgt, color)
% p1 -> p2 boyunca, kesiti w x hgt olan dikdortgen prizma.
v = p2 - p1;  L = norm(v);
if L < 1e-6
    h = patch('XData',[],'YData',[],'ZData',[],'Parent',ax); return;
end
xv = w/2; yv = hgt/2;
verts = [-xv -yv 0;  xv -yv 0;  xv  yv 0; -xv  yv 0;
         -xv -yv L;  xv -yv L;  xv  yv L; -xv  yv L];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
R = vecToRot(v / L);
verts = (R * verts')' + p1;
h = patch(ax,'Vertices',verts,'Faces',faces, ...
          'FaceColor',color,'EdgeColor','k', ...
          'FaceLighting','gouraud','AmbientStrength',0.45);
end

function h = drawSphere(ax, p, r, color)
if r <= 0
    h = patch('XData',[],'YData',[],'ZData',[],'Parent',ax); return;
end
[Xs,Ys,Zs] = sphere(14);
h = surf(ax, p(1)+r*Xs, p(2)+r*Ys, p(3)+r*Zs, ...
         'FaceColor',color,'EdgeColor','none', ...
         'FaceLighting','gouraud','AmbientStrength',0.45);
end

function h = drawFrame(ax, T, L)
% Bir DH frame'inin X(R), Y(G), Z(B) eksenlerini cizer.
o  = T(1:3,4)';
xA = (T(1:3,1)' * L) + o;
yA = (T(1:3,2)' * L) + o;
zA = (T(1:3,3)' * L) + o;
h = hggroup('Parent',ax);
plot3(ax,[o(1) xA(1)],[o(2) xA(2)],[o(3) xA(3)],'r','LineWidth',1.4,'Parent',h);
plot3(ax,[o(1) yA(1)],[o(2) yA(2)],[o(3) yA(3)],'g','LineWidth',1.4,'Parent',h);
plot3(ax,[o(1) zA(1)],[o(2) zA(2)],[o(3) zA(3)],'b','LineWidth',1.4,'Parent',h);
end

function R = vecToRot(vN)
% Lokal +Z eksenini birim vektor vN'e tasiyacak rotasyon matrisini uretir
% (Rodrigues formulu).
zAxis = [0;0;1];
vN    = vN(:);
axc   = cross(zAxis, vN);
if norm(axc) < 1e-9
    if dot(zAxis, vN) > 0
        R = eye(3);
    else
        R = diag([1, -1, -1]);     % 180 derece X etrafinda
    end
    return;
end
k   = axc / norm(axc);
ang = acos(max(-1,min(1, dot(zAxis,vN))));
c = cos(ang); s = sin(ang); v = 1 - c;
kx = k(1); ky = k(2); kz = k(3);
R = [c+kx*kx*v,    kx*ky*v-kz*s, kx*kz*v+ky*s;
     ky*kx*v+kz*s, c+ky*ky*v,    ky*kz*v-kx*s;
     kz*kx*v-ky*s, kz*ky*v+kx*s, c+kz*kz*v   ];
end
