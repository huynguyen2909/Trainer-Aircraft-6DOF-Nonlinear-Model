# Seed propeller T-6C / PC-9M và input DATCOM `PROPWR`

**Phạm vi:** seed cánh quạt 4 cánh cho BEMT hiện tại và ánh xạ lực đẩy dự đoán sang `PROPWR` của *USAF Digital DATCOM*. `PROPWR` dự báo **gia số khí động của tổ hợp máy bay có công suất**; DATCOM không tự dự đoán polar blade, công suất trục hay thrust từ đường kính. Vì thế các con số \(T,Q,C_T,C_P\) dưới đây là **BEMT proxy**, không phải kết quả chạy Digital DATCOM. Không cộng `THSTCP` dưới dạng lực đẩy lần thứ hai nếu `PropellerComponent` đã cung cấp lực đẩy.

Nguồn hình học: [Pilatus PC-9 M model-building plan](https://www.pilatus-aircraft.com/assets/files/Model-Building-Plans/PC-9-M-Model-Building-Plan.pdf), trang PDF 5; [Hartzell về hệ bốn cánh 97 inch T-6A](https://hartzellprop.com/hartzell-receives-faa-certification-and-makes-first-shipments-for-jpats-program/); [Hartzell service-bulletin index](https://hartzellprop.com/SERVICE-DOCUMENTS/SB/SB-Index.pdf) ghi họ HC-E4A-2/E9612 lắp trên T-6A; [EASA TCDS IM.P.133](https://www.easa.europa.eu/sl/downloads/7819/en), PDF tr. 9, cho RPM tối đa 2000, D tối đa 246.4 cm và khối lượng xấp xỉ tối đa 70.3 kg cho họ cánh này. T-6C dùng chung họ máy bay cho seed proxy; chưa xác minh part number từng cấu hình T-6C. NASA ghi nhận NACA 16 series trên [những cánh quạt khác](https://ntrs.nasa.gov/citations/19930093315); **không có dữ liệu công khai xác nhận E9612 dùng NACA 16 series**, nên polar trong mã là xấp xỉ giải tích dạng 16-series, không phải polar NACA đã đo.

Quy ước nguồn: hình vẽ có **đường kính PC-9M 96 inch = 2.439 m** và chú thích **4 BLADE PROP**. Chọn **97 inch = 2.4638 m** cho proxy T-6C theo Hartzell. Không đọc bề rộng tại 0.3/0.6/0.9R hoặc góc xoắn từ nét chiếu của bản vẽ mô hình tỷ lệ 1:25. Chúng được ước lượng để khởi tạo, không phải số đo Hartzell. Trong hệ bản vẽ \(X\) dương ra đuôi và \(Z\) dương lên, hub có \(X=-1.9704\) m và \(Z=2.000+0.4468=2.4468\) m. Với CG seed tại \(X=4.3675\), \(Z=2.000\) m, \(\mathbf r_{CG\to hub}^{B}=(+6.3379,0,-0.4468)\) m, hệ body FRD. \(Z_{CG}\) vẫn là giả định; 30% MAC chỉ xác định \(X_{CG}\).

## Input, phép tính và output

| Input và mức tin cậy | Công thức / cách chuyển | Output seed | Nguồn DATCOM / mô hình |
|---|---|---|---|
| PC-9M D=2.439 m; T-6A Hartzell 97 in; 4 cánh (hình/Hartzell) | \(R=D/2,\ A=\pi R^2\); ưu tiên D T-6C proxy | \(D=2.4638\) m, \(R=1.2319\) m, \(A=4.76761\) m²; `PRPRAD=1.2319`, `NOPBPE=4` | Digital DATCOM **Figure 12, PROPWR**; Pilatus PDF tr. 5 |
| X,Z hub, X,Z CG (X đo/CG giả định) | \(\mathbf r_{h/CG}=(X_{CG}-X_h,0,Z_{CG}-Z_h)\) theo FRD | \((6.3379,0,-0.4468)\) m; `PHALOC` từ \(X_h=-1.9704\) m, `PHVLOC` từ \(Z_h=2.4468\) m | Figure 12; `SYNTHS`/`BODY` **phải dùng cùng gốc X,Z** |
| 1 động cơ; shaft trùng FRL (ước lượng), đối xứng (hình) | góc trục lực đẩy so với đường chuẩn | `NENGSP=1`, `AIETLP=0°`, `YP=0`, `CROT=.FALSE.` | Figure 12; chiều quay BEMT \(+1\) là giả định chưa kiểm chứng |
| Chord ước lượng tại 0.3/0.6/0.9R | Nội suy chord các trạm ở bảng dưới | `BWAPR3=0.210`, `BWAPR6=0.220`, `BWAPR9=0.120` m | Figure 12: cung cấp chord nên bỏ `ENGFCT` |
| Pitch tham chiếu 29° tại 0.75R (ước lượng; chưa có governor) | \(\beta(r)=\tan^{-1}[0.75\tan(29^\circ)/(r/R)]\) | `BAPR75=29°`; luật pitch cố định trong seed | Figure 12 cần BAPR75; luật theo r là **proxy BEMT**, không phải luật DATCOM |
| Polar giả định kiểu NACA 16-series (không phải bảng đo) | \(c_l=\mathrm{clip}[5.7(\alpha+2^\circ),-1.15,1.15],\quad c_d=0.012+0.020c_l^2\), bảng −24° đến +24° | 25 điểm \(c_l(\alpha),c_d(\alpha)\) cho `AirfoilPolar` | BEMT của repo; **không phải DATCOM**, không hiệu chỉnh Mach/Re |
| FDR cruise \(\rho=1.10681, V=87.953, M=0.25383,\alpha_B=1.22929^\circ\); NP gần 100%; n=2000 rpm theo giới hạn EASA, **giả định** NP=100% ứng với n này; \(S=16.28\) m² | \(J=V_{axial}/(nD)\), \(q_\infty=\frac12\rho V^2\); BEMT giải uniform induced inflow và tích phân blade elements | \(J=1.07070\), \(q_\infty=4280.99\) Pa; \(T=5872.4\) N, \(Q=2850.05\) Nm, \(P=596.91\) kW | FDR seed wing; BEMT `PropellerModel.cpp`; NASA blade-element theory, **không phải output DATCOM** |
| EASA nêu khối lượng cụm xấp xỉ tối đa 70.3 kg; phân bố khối lượng chưa biết | Giả sử 60% khối lượng ở bốn lá phân bố đều từ 0.2R đến R; 40% còn lại là đĩa bán kính 0.2R: \(I_x=m_b(r_0^2+r_0R+R^2)/3+\tfrac12m_h r_0^2\) | \(I_{x,prop}=27.31\) kg m², **ước lượng**, cho moment con quay khi thân đổi tốc độ góc | EASA TCDS chỉ cho khối lượng, **không cung cấp mômen quán tính**; không phải đầu vào `PROPWR` |
| \(T,Q,P\) từ BEMT | \(C_T=T/(\rho n^2D^4),\ C_Q=Q/(\rho n^2D^5),\ C_P=2\pi C_Q,\ \eta=JC_T/C_P\) | \(C_T=0.129588,\ C_Q=0.0255267,\ C_P=0.160389,\ \eta=0.86528\) | Hệ số performance BEMT; **khác định nghĩa `THSTCP`** |
| Lực đẩy BEMT/FDR hiệu chỉnh và \(S_{ref}=16.28\) m² | `THSTCP` \(=T/(q_\infty S_{ref})=2T/(\rho V^2 S_{ref})\) | `THSTCP=0.0842591` tại điểm FDR dùng T BEMT | Digital DATCOM Figure 12, **§3.4, §5.2**; thay bằng T đo/fit khi có |
| Lực/moment CG do `PropellerComponent`, gồm \(\mathbf r_{h/CG}\times\mathbf F\) | \(C_X=F_x/(q_\infty S),\ C_Z=F_z/(q_\infty S),\ C_m=M_y/(q_\infty S\bar c)\) | \(F_x=5872.4\) N, \(F_z=-139.9\) N, \(M_y=-1737.1\) Nm; \(C_X=0.084259, C_Z=-0.002007, C_m=-0.015106\) | Hệ số **tương đương** từ BEMT/body CG; không phải \(\Delta C_m\) DATCOM |
| Tổ hợp wing + body + tail và `$PROPWR` trong ca DATCOM | Tính power-on trừ power-off ở cùng Mach/\(\alpha\) | **Cần tính sau:** \(\Delta C_L^{power},\Delta C_D^{power},\Delta C_m^{power}\), các đạo hàm theo \(\alpha\) nếu cần | DATCOM §4.6.1, Digital DATCOM §5.2, §6.1.3; **chưa chạy ca máy bay hoàn chỉnh** |

Các tham số khí động từ BEMT vừa nêu **chỉ đúng tại RPM/pitch/flight point này**. Mô hình hiện tại đã tính vector lực và moment tại hub, phản lực mômen trục và \(\mathbf r_{h/CG}\times\mathbf F\), rồi đưa vào `RigidBody6DOF` thông qua `PropellerComponent`. Trục body \(+x\) về trước; lực đẩy dương có xu hướng tạo moment pitch do hub cao hơn CG. Không thêm \(q S C_{m,prop}\) tương đương từ chính tải này vào một component khác.

## Proxy chord/twist có thể thay thế

| r/R | Chord c (m) | Góc β (°), pitch xoắn helix |
|---:|---:|---:|
| 0.20 | 0.170 | 64.309 |
| 0.30 | 0.210 | 54.185 |
| 0.45 | 0.240 | 42.733 |
| 0.60 | 0.220 | 34.718 |
| 0.75 | 0.190 | 29.000 |
| 0.90 | 0.120 | 24.793 |
| 1.00 | 0.045 | 22.574 |

RPM tối đa có chứng cứ ở EASA, nhưng giá trị RPM thực tế của snapshot FDR và các giá trị pitch tập thể, chord, cutout r/R=0.20, shaft incidence 0°, chiều quay, quán tính, polar vẫn cần xác minh/fit theo T-6C. Local Mach cánh tại điểm seed đạt **0.792**; BEMT hiện chỉ cảnh báo khi Mach>0.70, **không áp dụng hiệu chỉnh nén được**, do đó hiệu suất và lực đẩy ở đây có bất định lớn. Re của blade tại 0.75R ước lượng cỡ \(2.4\times10^6\), **khác Re wing \(8.72\times10^6\)**. Chưa có mô hình governor thay pitch khi thay power/airspeed; `propellerSpeedScale` chỉ đổi RPM.

Kiểm tra thêm ở RPM giả định 2000 cho thấy uniform-inflow solver hội tụ tại \(V=0,30,60,87.953\) m/s, nhưng **không hội tụ tại 120 m/s**, nơi seed cố định pitch tạo lực đẩy âm. Ở \(V=0\), một phần polar cũng bị chặn ngoài khoảng bảng; vì thế seed này được xác nhận tại điểm cruise tham chiếu, chưa phù hợp để dùng như hiệu chuẩn toàn bộ bao bay hoặc governor cho takeoff/landing.

## Cách đưa vào ca Digital DATCOM

`makeT6CDatcomPropellerPowerInputs(p,T,rho,V,Sref)` trả về các giá trị Figure 12. Ví dụ với gốc DATCOM trùng **gốc X/Z bản vẽ** và các `SYNTHS`/`BODY` được biến đổi về cùng gốc, trích đoạn input:

```text
DIM M
$PROPWR AIETLP=0., NENGSP=1., THSTCP=0.08425913,
 PHALOC=-1.9704, PHVLOC=2.4468, PRPRAD=1.2319,
 BWAPR3=0.210, BWAPR6=0.220, BWAPR9=0.120,
 NOPBPE=4., BAPR75=29., YP=0., CROT=.FALSE.$
```

Đây là **một namelist để ghép vào ca máy bay hoàn chỉnh**, không phải deck DATCOM độc lập. Khi chọn gốc mũi khác cho `BODY`/`SYNTHS`, cần đổi `PHALOC`/`PHVLOC` theo cùng phép tịnh tiến; không được ghép tọa độ hub theo body-CG với tọa độ gốc bản vẽ. Với `THSTCP` từ nhiều mẫu FDR, có thể xây dựng bảng công suất. Chỉ cộng **gia số khí động do slipstream trên cánh/đuôi** từ kết quả power-on trừ power-off nếu xác định được phần nào chưa được wing–tail flow field đang mô phỏng; tránh cộng gia số lực đẩy, downwash hay moment hai lần. Không gán \(\Delta C_L,\Delta C_D,\Delta C_m\) bừa từ \(T\) khi chưa chạy DATCOM.

Tích hợp: `aircraft.addLoadComponent(makeT6CPropellerWithProxySeed());` sau khi include header của factory. Biên dịch/vận hành: `makeT6CPropellerWithProxySeed()` là factory opt-in; ví dụ số học tại `examples/t6c_propeller_seed_demo.cpp`, hồ sơ tại `reference_data/t6c/propeller_datcom_seed.yaml`. Test tích hợp `tests/test_t6c_propeller_seed.cpp`. Seed không tự đổi cấu hình public mặc định hay mô hình điều khiển governor.
