# Main Wing T-6C/PC-9M: bộ seed DATCOM và các giả thiết sơ bộ

Bộ số khởi tạo có thể chạy và tái lập; **chưa optimize, chưa xác thực T-6C, không phải output Digital DATCOM**.
Mọi đạo hàm góc trong code dùng radian; p_hat=p*b/(2V), q_hat=q*cbar/(2V), r_hat=r*b/(2V).
DATCOM là AFWAL-TR-83-3048. `PDF` là thứ tự trang trong bản 3134 trang; số mục/trang in đứng trước nó.

## Các lựa chọn để khép kín input

- Giữ số đọc PC-9M 1,880/1,140 m; tăng đồng đều chord để diện tích hình học đạt 16,28 m². Đây là hình học hiệu dụng cho seed, không phải sửa số đo.
- Cánh trong R6 có chord/incidence không đổi; twist tuyến tính R6–R25. Dihedral đổi từ 0° sang 7° tại R11. Dầm 35% chord được coi không sweep.
- Điểm gãy là chỗ đổi quy luật hình học: R6 là gãy mặt bằng trong mô hình này; R11 là gãy dihedral, không phải một chord mới độc lập. Dihedral trong bảng là góc của panel ngay phía ngoài trạm.
- Sải proxy 10,124 m chỉ áp dụng factory cánh này. Cấu hình public baseline vẫn lưu sải T-6C 10,20 m. MAC tham chiếu 1,650 m khác nhẹ MAC hình học tích phân.
- CL_alpha dùng công thức DATCOM cho cánh tương đương với AR toàn cánh và sweep đoạn ngoài. Hiệu chỉnh twist dùng trọng số chord; chưa giải bài toán lifting-line đầy đủ.
- NACA 4312, giảm slope tiết diện 5%, e=0,82, roughness, airfoil polar và hiệu quả mặt lái là giả thiết; việc optimize sau này không chứng minh chúng là giá trị đo.
- CG=0,30 MAC; AC tạm ở 0,25 MAC; cùng cao độ. Hệ số mô men trong config ở AC, runtime chuyển về CG đúng một lần.
- AOA-2/3/4 trung bình 1,229293° được tạm đồng nhất alpha_body. Cần xem bias/trục cảm biến là nguồn bất định; không thay âm thầm bằng AOA-1.
- Hệ số ngang hướng dùng strip theory/proxy cánh riêng, đóng băng tại điểm FDR. Không dùng đạo hàm toàn máy bay khác để thay cho wing-only.
- Chỉ snapshot flap Up là mục tiêu hiệu chỉnh hiện tại. Bảng split flap 0–60° là ngoại suy kỹ thuật chưa xác thực; không dùng để tuyên bố đạt bài flap dynamics.

## Toàn bộ input

| Input | Giá trị | Đơn vị | Nguồn / giả thiết |
|---|---:|---|---|
| `S_ref` | 16.28 | m2 | USER: reference wing area |
| `b` | 10.124 | m | DRAWING_PROXY: PC-9M projected span; public T-6C span remains 10.20 m |
| `cbar_ref` | 1.65 | m | DRAWING_PROXY: mean aerodynamic chord |
| `root_chord_raw` | 1.88 | m | USER/DRAWING: chord interpreted at R6 |
| `tip_chord_raw` | 1.14 | m | USER/DRAWING: chord at R25 |
| `y_R6` | 0.65 | m | PROXY: approximate rib station from drawing; center panel constant chord |
| `y_R11` | 1.35 | m | PROXY: approximate station; USER: dihedral starts here |
| `y_R5` | 0.45 | m | PROXY: nominal flap inboard station |
| `y_R17` | 2.8 | m | PROXY: flap outboard / aileron inboard station |
| `incidence_root` | 0.5 | deg | USER: root incidence |
| `twist_tip_minus_root` | -2.5 | deg | USER: linear R6-R25; constant inboard |
| `outer_dihedral` | 7 | deg | USER/DRAWING: zero inboard R11 |
| `airfoil` | "NACA 4312" | - | USER_ASSUMPTION: all stations; not a verified PIL airfoil polar |
| `thickness_ratio` | 0.12 | - | NACA designation |
| `maximum_camber` | 0.04 | - | NACA designation |
| `maximum_camber_x_over_c` | 0.3 | - | NACA designation |
| `maximum_thickness_x_over_c` | 0.3 | - | PROXY: choose DATCOM thickness factor L=1.2 |
| `unswept_spar_chord_fraction` | 0.35 | - | PROXY: straight spar from drawing |
| `Mach` | 0.25383 | - | FDR summary: Relative 13962..14027, 66 one-second rows |
| `rho` | 1.10681 | kg/m3 | FDR-derived: Pt-Qc and TAT, recovery factor 1 |
| `TAS` | 87.953 | m/s | FDR-derived: Mach + TAT |
| `mu` | 1.84e-05 | Pa s | Sutherland at Ts=298.751 K |
| `alpha_body_seed` | 1.229292929 | deg | FDR AOA-2/3/4 average; ASSUMPTION: equals body alpha; bias=0 |
| `alpha_sensor_bias` | 0 | deg | PROXY: no calibration available; freeze initially |
| `beta_seed` | 0 | deg | ASSUMPTION: symmetric near-level condition |
| `rates_seed` | [0.0, 0.0, 0.0] | rad/s | ASSUMPTION: p=q=r=0 for static seed evaluation |
| `flap_seed` | 0 | deg | FDR Up mapped to nominal zero |
| `aileron_seed` | 0 | deg | ASSUMPTION: symmetric seed; FDR mean -0.209 deg not imposed |
| `h_CG` | 0.3 | MAC | USER_ASSUMPTION: CG; positive aft |
| `h_aero` | 0.25 | MAC | PROXY: quarter-MAC aerodynamic center |
| `z_aero_minus_CG` | 0 | m | PROXY: same height; avoids inventing a vertical lever arm |
| `MAC_LE_drawing_x_aft` | 3.8725 | m | DRAWING corrected global X; fuselage station zero is X=3 m; 266 mm is LE offset, not MAC station |
| `section_slope_viscous_ratio` | 0.95 | - | PROXY: 5% reduction from 2*pi; not measured polar |
| `oswald_e` | 0.82 | - | PROXY: moderate AR tapered wing; includes viscous drag-due-to-lift |
| `surface_roughness_height` | 6.35e-06 | m | PROXY: smooth painted surface scenario |
| `lifting_surface_drag_factor` | 1.05 | - | PROXY for R_LS chart, not digitized DATCOM data |
| `transition` | "fully_turbulent" | - | ASSUMPTION: conservative profile drag seed |
| `wing_body_interference_factor` | 1 | - | ASSUMPTION: isolated wing; body loads treated separately |
| `slipstream_dynamic_pressure_ratio` | 1 | - | ASSUMPTION: free stream; propwash coupling not identified |
| `ground_effect_factor` | 1 | - | ASSUMPTION: airborne seed, no ground effect |
| `split_flap_chord_ratio` | 0.25 | - | PROXY: constant cf/c; mechanical hinge loads excluded |
| `split_flap_panel_boundaries` | [0.45, 1.625, 2.8] | m | PROXY: two equal-span panels each side; four total, contiguous, no gap |
| `aileron_absolute_chord` | 0.285 | m | USER: quarter of raw tip chord, held fixed after area closure |
| `aileron_up_down_limits` | [20.0, 11.0] | deg | USER/FCS: physical up/down magnitudes |
| `aileron_effectiveness_ratio` | 0.85 | - | PROXY: reduction of thin-airfoil plain-flap effectiveness |
| `split_flap_effectiveness_ratio` | 0.75 | - | PROXY: split-flap effectiveness; not a DATCOM chart result |
| `split_flap_nonlinearity_angle` | 45 | deg | PROXY: smooth saturation scale |
| `split_flap_table_angles` | [0.0, 10.0, 20.0, 23.0, 30.0, 40.0, 50.0, 60.0] | deg | PROXY: tabulation and max domain 60 deg; only zero-flap seed is current calibration target |
| `split_flap_pressure_drag_factor` | 1.2 | - | PROXY: Cd_plate multiplier on flap frontal projection |
| `split_flap_increment_arm` | 0.25 | local chord | PROXY: lift increment acts 0.25 chord aft of quarter-chord |
| `split_flap_extra_induced_factor` | 0.1 | - | PROXY: additional nonuniform-loading drag, separate from total-CL polar |
| `Cm_twist_factor` | -0.0002 | 1/deg | PROXY: small-sweep estimate inspired by DATCOM Fig 4.1.4.1-5, not digitized |
| `aileron_adverse_yaw_factor` | -0.15 | - | PROXY for DATCOM Fig 6.2.2.1-9, not digitized |
| `CL_alpha_dot` | 0 | - | ASSUMPTION: omitted; triangular-wing DATCOM formula does not apply |
| `Cm_alpha_dot` | 0 | - | ASSUMPTION: omitted; not identifiable from steady segment |
| `alpha_working_range` | [-2.0, 8.0] | deg body | PROXY: local pre-stall model, not a stall model |

## Mặt bằng hiệu dụng

| Trạm | y (m) | eta | chord đọc (m) | chord hiệu dụng (m) | incidence (deg) | dihedral (deg) | x_LE từ gốc, dương sau (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | 0.000000 | 0.000000 | 1.880000 | 1.941016 | 0.500000 | 0.000000 | 0.000000 |
| R5 | 0.450000 | 0.088898 | 1.880000 | 1.941016 | 0.500000 | 0.000000 | 0.000000 |
| R6 | 0.650000 | 0.128408 | 1.880000 | 1.941016 | 0.500000 | 0.000000 | 0.000000 |
| R11 | 1.350000 | 0.266693 | 1.762593 | 1.819798 | 0.103354 | 7.000000 | 0.042426 |
| R17 | 2.800000 | 0.553141 | 1.519393 | 1.568705 | -0.718268 | 7.000000 | 0.130309 |
| R25 | 5.062000 | 1.000000 | 1.140000 | 1.176999 | -2.000000 | 7.000000 | 0.267406 |

## Bảng input – công thức – output – reference

Tên biến trong công thức chỉ về bảng input phía trên. `PROXY` có reference DATCOM theo chủ đề để đối chiếu, không có nghĩa con số đã được tra từ đồ thị đó.

| Input / loại phương pháp | Công thức sử dụng | Output và giá trị | Reference DATCOM / phạm vi |
|---|---|---|---|
| GEOMETRY | `b^2/S` | **aspect_ratio = 6.29578477 -** | 2.2.2-1; PDF 235 |
| GEOMETRY | `2*integral(c_raw dy)` | **raw_planform_area = 15.76824 m2** | 2.2.2-2; PDF 236 |
| ASSUMPTION/CLOSURE | `S_ref/S_raw` | **chord_area_scale = 1.03245511 -** | Engineering geometry closure |
| GEOMETRY | `2*integral(c dy)` | **closed_planform_area = 16.28 m2** | 2.2.2-2; PDF 236 |
| GEOMETRY/CLOSURE | `scale*1.880` | **c_root_effective = 1.94101561 m** | Not a measured dimension |
| GEOMETRY/CLOSURE | `scale*1.140` | **c_tip_effective = 1.17699883 m** | Not a measured dimension |
| GEOMETRY | `2*integral(c^2 dy)/S` | **MAC_geometric = 1.64458213 m** | 2.2.2-1; PDF 235; ref MAC retained as 1.650 |
| GEOMETRY_PROXY | `atan(delta(x_LE+f*c)/delta y)` | **sweep_LE_outer = 3.46838368 deg** | 2.2.2-3; PDF 237; outer panel representative sweep |
| GEOMETRY_PROXY | `atan(delta(x_LE+f*c)/delta y)` | **sweep_quarter_outer = 0.992079835 deg** | 2.2.2-3; PDF 237; outer panel representative sweep |
| GEOMETRY_PROXY | `atan(delta(x_LE+f*c)/delta y)` | **sweep_half_outer = -1.48793391 deg** | 2.2.2-3; PDF 237; outer panel representative sweep |
| PROXY+PG | `0.95*2*pi/sqrt(1-M^2)` | **cl_alpha_section = 6.17113793 1/rad** | 4.1.1.2-1; PDF 471; viscous factor assumed |
| DATCOM+EQUIVALENT_WING | `2*pi*A/[2+sqrt(4+(A*B/kappa)^2*(1+tan(Lambda_half)^2/B^2))]` | **CL_alpha = 4.53794768 1/rad** | Fig 4.1.3.2-49; PDF 549; kappa=0.95, representative outer sweep |
| DATCOM | `0.93*(alpha_i-9.12*c_li); alpha_i=1.6*4/6, c_li=0.8*4/6` | **alpha_zero_section = -3.53152 deg** | Table 4.1.1-D, Eq 4.1.1.1-a; PDF 467-468 |
| STRIP_PROXY | `2*integral(c*twist dy)/S` | **chord_weighted_twist = -0.969983549 deg** | Replaces a full lifting-line/twist-chart calculation |
| STRIP_PROXY | `alpha_zero_section - chord_weighted_twist` | **alpha_zero_wing = -2.56153645 deg** | DATCOM 4.1.3.1-c structure; PDF 496-500; weighting is a proxy |
| DERIVED | `-CL_alpha*alpha_zero_wing_rad` | **CL0_wing = 0.202879139 -** | Linear lift law; incidence not included here |
| DATCOM_THEORETICAL | `-0.124*4/6` | **cm0_section = -0.0826666667 -** | Table 4.1.1-D; 4.1.2.1-1; PDF 467,489 |
| DATCOM+TWIST_PROXY | `A*cos(Lambda25)^2/(A+2*cos(Lambda25))*cm0_section + K_mtheta*twist` | **Cm0_at_quarter_MAC = -0.0622203257 -** | 4.1.4.1-a,c; PDF 654; Fig 4.1.4.1-5, PDF 658-659 |
| AC_ASSUMPTION | `(h_ref-h_AC)*CL_alpha = 0` | **Cm_alpha_at_quarter_MAC = 0 1/rad** | 4.1.4.2-a; PDF 662 |
| AIR_DATA | `rho*V*cbar/mu` | **Re_MAC = 8729509.72 -** | FDR summary + Sutherland; rounded inputs |
| CORRELATION_PROXY | `38.21*(cbar/k)^1.053` | **roughness_Re_cutoff = 19225024.3 -** | Engineering substitute for roughness chart 4.1.5.1-27; not its exact equation |
| CORRELATION_PROXY | `0.455/[log10(min(Re,Re_cut))^2.58*(1+0.144*M^2)^0.65]` | **skin_friction_Cf = 0.00305166721 -** | Analytic approximation replacing Fig 4.1.5.1-26, not a digitized chart |
| GEOMETRY_PROXY | `2*(1+0.25*t/c)*S_exposed` | **S_wet_exposed = 28.3387602 m2** | Two surfaces; center patch excluded from friction to avoid counting fuselage skin |
| DATCOM+DRAG_PROXIES | `Cf*(1+1.2*t/c+100*(t/c)^4)*R_LS*S_wet/S` | **CD0 = 0.00649651455 -** | 4.1.5.1-a; PDF 723 |
| STATIC_SEED | `CL0+CL_alpha*(alpha_body+incidence)` | **CL_at_FDR_seed = 0.339842819 -** | AOA->body interpretation is assumed |
| DATCOM_POLAR+E_PROXY | `CD0+CL^2/(pi*A*e)` | **CD_at_FDR_seed = 0.013617541 -** | 4.1.5.2-e; PDF 757 |
| AIR_DATA | `rho*V^2/2` | **dynamic_pressure = 4280.99178 Pa** | Dynamic pressure definition |
| DATCOM | `(0.5+2*(h_AC-h_CG))*CL_alpha` | **CL_q = 1.81517907 -** | 7.1.1.1-a,b; PDF 2470; rotation about CG |
| FAIRING_PROXY | `0.7+0.2*clip((A-6)/4,0,1)` | **Cmq_empirical_factor = 0.714789238 -** | 7.1.1.2-1; PDF 2488 recommends fairing; interpolation chosen here |
| DATCOM+FAIRING_PROXY | `-k*c_lalpha0*cos(L25)*[A*(0.5+x+2*x^2)/(A+2*cos(L25))+A^3*tan(L25)^2/(24*(A+6*cos(L25)))+1/8], then Mach correction` | **Cm_q_about_CG = -2.07532106 -** | 7.1.1.2-a,b; PDF 2488-2489; x=h_AC-h_CG |
| REFERENCE_CONVENTION | `CL0_wing+CL_alpha*incidence_rad` | **CL_at_zero_body_alpha = 0.242480203 -** | Body alpha=0; wing-axis CL0 excludes incidence |
| REFERENCE_TRANSFER | `(h_CG-h_AC)*[CL_alpha*cos(alpha)-CL*sin(alpha)+CD_alpha*sin(alpha)+CD*cos(alpha)]` | **Cm_alpha_about_CG_at_seed = 0.227365338 1/rad** | Exact local derivative of r cross F; Cm_alpha at AC=0; wing-only, not aircraft stability |
| REFERENCE_TRANSFER | `Cm_q_CG-(h_CG-h_AC)*(cos(alpha)+dCD/dCL*sin(alpha))*CL_q` | **Cm_q_at_aero_reference = -2.16614073 -** | Remove r cross F derivative once; local linearization at alpha_seed |
| STRIP/ENGINEERING_PROXY | `-2*a/S*integral(c*sin(Gamma)^2 dy)` | **CY_beta = -0.0460531463 1/rad or per normalized rate** | 5.1.1.1; PDF 1531; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `-4*a/(S*b)*integral(c*y*sin(Gamma) dy)` | **CY_p = -0.229428563 1/rad or per normalized rate** | 7.1.2.1; PDF 2521-2522; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `0 (omitted wing-only yaw side force)` | **CY_r = 0 1/rad or per normalized rate** | 7.1.3.1; PDF 2578 (no general method); topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `0 (omitted wing-only control side force)` | **CY_da = 0 1/rad or per normalized rate** | 6.2.3.1; PDF 13 (not provided); topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `-2*a/(S*b)*integral(c*y*sin(Gamma) dy)` | **Cl_beta = -0.114714282 1/rad or per normalized rate** | 5.1.2.1; PDF 1538; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `-4*a/(S*b^2)*integral(c*y^2 dy)` | **Cl_p = -0.656624797 1/rad or per normalized rate** | 7.1.2.2; PDF 2531-2533; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `8*CL/(S*b^2)*integral(c*y^2 dy)` | **Cl_r = 0.0983480806 1/rad or per normalized rate** | 7.1.3.2; PDF 2580-2582; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `2*a*0.85/(S*b)*integral_aileron(y*c*tau dy)` | **Cl_da = 0.316937058 1/rad or per normalized rate** | 6.2.1.1; PDF 2238-2240; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `0 (low sweep seed, no claimed directional stability)` | **Cn_beta = 0 1/rad or per normalized rate** | 5.1.3.1; PDF 1576; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `-CL/6 (low-sweep proxy)` | **Cn_p = -0.0566404699 1/rad or per normalized rate** | 7.1.2.3; PDF 2558-2560; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `-8*CD/(S*b^2)*integral(c*y^2 dy)` | **Cn_r = -0.00394081894 1/rad or per normalized rate** | 7.1.3.3; PDF 2592-2593; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| STRIP/ENGINEERING_PROXY | `K_yaw*CL*Cl_da` | **Cn_da = -0.0161563175 1/rad or per normalized rate** | 6.2.2.1-a; PDF 2292; topical reference only; formula here is a proxy, not evaluated DATCOM graphs |
| GEOMETRY | `2*integral_R5^R17(c dy)` | **flapped_wing_area = 8.32230475 m2** | Not flap plate area; nominal inboard R5 included |
| ENGINEERING_PROXY | `a*tau(cf/c)*0.75*S_flapped/S` | **flap_CL_delta_at_zero = 1.05956029 1/rad** | 6.1.4.1-a architecture; PDF 2028-2029; span weighting is proxy |
| CONTROL_CONVENTION | `(20+11)/2` | **aileron_effective_limit = 15.5 deg** | USER/FCS physical limits; runtime input is half-difference, symmetric equivalent |

## Split flap: bảng increment tại điểm khí động

cf/c=0,25. tau=1-(theta_h-sin(theta_h))/pi, theta_h=acos(2*cf/c-1).
delta_CL = CL_alpha*tau*0.75*delta_rad/[1+(delta_deg/45)^2]*S_flapped/S.
delta_CD_profile = 1.2*(S_flapped/S)*(cf/c)*sin(delta)^2.
delta_Cm = -0.25*(chord_flapped_weighted/cbar_ref)*delta_CL.
Đây là closure kỹ thuật, **không phải đồ thị split-flap DATCOM đã số hóa**. §6.1.4.1 (PDF 2028–2029), §6.1.5.1 (2075–2078), §6.1.7 (2208) chỉ ra phương pháp chi tiết để thay thế.
Code dùng CD=CD0+CL_total²/(pi*A*e)+delta_CD_profile+0.10*delta_CL². Phần 0.10 chỉ là giả thiết tổn thất tải không đều, không cộng lại delta(CL²)/(pi*A*e).

| Flap (deg) | delta_CL | delta_CD_profile | delta_Cm tại aero_h |
|---:|---:|---:|---:|
| 0 | 0 | 0 | -0 |
| 10 | 0.176225655 | 0.00462435247 | -0.0474793391 |
| 20 | 0.308849086 | 0.0179396447 | -0.0832112129 |
| 23 | 0.33723684 | 0.0234135082 | -0.0908595418 |
| 30 | 0.384081556 | 0.038339856 | -0.103480611 |
| 40 | 0.413218777 | 0.0633644198 | -0.111330864 |
| 50 | 0.413789521 | 0.0899950043 | -0.111484636 |
| 60 | 0.399444818 | 0.115019568 | -0.107619835 |

## Giá trị thực sự đưa vào C++

| Trường config | Giá trị |
|---|---:|
| `model.derivative_axes` | "body" |
| `wing.area_m2` | 16.28 |
| `wing.aspect_ratio` | 6.295784767 |
| `wing.mean_chord_m` | 1.65 |
| `wing.taper` | 0.6063829787 |
| `wing.sweep_le_deg` | 3.46838368 |
| `wing.quarter_chord_sweep_deg` | 0.992079835 |
| `wing.flapped_area_m2` | 8.322304746 |
| `wing.incidence_deg` | 0.5 |
| `flight.rho_kg_m3` | 1.10681 |
| `flight.speed_m_s` | 87.953 |
| `flight.alpha_deg` | 1.229292929 |
| `controls.aileron_limit_deg` | 15.5 |
| `reference.aero_h` | 0.25 |
| `reference.output_h` | 0.3 |
| `reference.aero_z_m` | 0 |
| `reference.output_z_m` | 0 |
| `reference.stability_angle_deg` | 0 |
| `aero.cl_alpha_per_rad` | 6.171137927 |
| `aero.CL_alpha_per_rad` | 4.537947682 |
| `aero.CL0` | 0.2028791388 |
| `aero.CD0` | 0.006496514545 |
| `aero.oswald_e` | 0.82 |
| `aero.Cm0` | -0.06222032574 |
| `aero.Cm_alpha_per_rad` | 0 |
| `aero.CL_q` | 1.815179073 |
| `aero.Cm_q` | -2.166140727 |
| `flap.eta_in` | 0.08889766891 |
| `flap.eta_out` | 0.553141051 |
| `flap.chord_ratio` | 0.25 |
| `flap.thickness_ratio` | 0.12 |
| `flap.section_effectiveness_per_rad` | 2.818656978 |
| `flap.effectiveness_ratio_actual` | 1 |
| `flap.effectiveness_ratio_reference` | 1 |
| `flap.induced_factor_K` | 0.316227766 |
| `flap.interference_factor` | 0 |
| `tabulated_flap.enabled` | true |
| `tabulated_flap.extra_induced_factor` | 0.1 |
| `lateral.CY_beta` | -0.0460531463 |
| `lateral.CY_p` | -0.2294285631 |
| `lateral.CY_r` | 0 |
| `lateral.CY_da` | 0 |
| `lateral.Cl_beta` | -0.1147142816 |
| `lateral.Cl_p` | -0.6566247969 |
| `lateral.Cl_r` | 0.0983480806 |
| `lateral.Cl_da` | 0.316937058 |
| `lateral.Cn_beta` | 0 |
| `lateral.Cn_p` | -0.05664046988 |
| `lateral.Cn_r` | -0.003940818939 |
| `lateral.Cn_da` | -0.0161563175 |

Các trường flap kiểu Roskam được điền để giữ tương thích cấu trúc, nhưng không được đánh giá khi `tabulated_flap.enabled=true`; chỉ bảng increment ở trên có tác dụng.
Aileron runtime là delta_a=(delta_L-delta_R)/2, dương tạo roll phải. Miền ±15,5° suy từ 11° down và 20° up. Lệnh/potentiometer FDR phải đổi về quy ước này trước khi replay. Mô hình cục bộ chưa biểu diễn lực nâng đối xứng do differential aileron ở biên hành trình.

## Kết nối lực – mô men – RigidBody6DOF

Factory `makeT6CWingCalibrationSeed()` trả cấu hình; truyền vào `MainWing` rồi `TrainerAircraftModel::addLoadComponent`. Không có factory toàn máy bay T-6C hoàn chỉnh trong nhánh hiện hành.
Mỗi bước: alpha_w=alpha_body+incidence; CL=CL0+CL_alpha*alpha_w+CL_q*q_hat+delta_CL; Cm_ref=Cm0+Cm_alpha*alpha_w+Cm_q_ref*q_hat+delta_Cm.
Các đạo hàm động quay quanh CG. Cm_q_ref được trừ phần chuyển mô men của CL_q tại alpha_seed, sau đó runtime cộng r×F đúng một lần. Hệ số đóng băng chỉ đúng cục bộ quanh alpha_seed.
CX=-CD*cos(alpha_body)+CL*sin(alpha_body); CZ=-CL*cos(alpha_body)-CD*sin(alpha_body). FY=qbar*S*CY. Biến đổi dọc này phù hợp beta nhỏ; chưa thay bằng biến đổi wind-to-body đầy đủ.
F_body=qbar*S*[CX,CY,CZ]. M_ref=qbar*S*[b*Cl,cbar*Cm,b*Cn]. M_CG=M_ref+(r_ref-r_CG)×F_body.
LoadAccumulator cộng wing với thân, đuôi, cánh quạt và càng. RigidBody6DOF dùng tổng lực, mô men, mass và inertia để tính đạo hàm Newton–Euler; RK4 tích phân trạng thái.
Với trục body x trước, y phải, z xuống: v_dot_body=F_total_body/m + g_body - omega×v_body; omega_dot=I^(-1)*(M_total_CG - omega×(I*omega)). Trọng lực chỉ cộng một lần ở cấp rigid body; cánh chỉ trả tải khí động.
Wing-only không cần tự tạo mass/inertia. Các input còn thiếu của đuôi/cánh quạt và khối lượng chuyến bay nằm ngoài cập nhật này; `T6CConfig::isSimulationReady()` không bị chuyển thành true.

## Snapshot kiểm tra, không phải nghiệm trim

```json
{
  "CL": 0.3398428192787815,
  "CD": 0.013617540985836445,
  "Cm_aero": -0.06222032574165327,
  "Cm_CG": -0.045217488351091795,
  "force_body_N": [
    -440.71816661930535,
    0,
    -23700.100693187076
  ],
  "moment_CG_Nm": [
    0,
    -5199.830339948117,
    0
  ],
  "note": "Wing-only static snapshot, NOT a whole-aircraft trim or FDR fit."
}
```

Tại alpha này, lực nâng cánh có thể chưa bằng trọng lượng và mô men cánh không bằng 0. Tổng cân bằng cần cả thân/đuôi/lực đẩy và khối lượng chuyến bay.

## Tái lập và sử dụng

```sh
python tools/generate_t6c_wing_seed.py --check
cmake -S . -B build-t6c -DTRAINER_AIRCRAFT_BUILD_TESTS=ON
cmake --build build-t6c --config Release
ctest --test-dir build-t6c -C Release --output-on-failure
# Windows multi-config: build-t6c/Release/trainer_aircraft_t6c_wing_seed_demo.exe
# Linux single-config: build-t6c/trainer_aircraft_t6c_wing_seed_demo
```

Để thay input, sửa generator, chạy lại không có `--check`; YAML, C++ và tài liệu được sinh cùng nguồn. YAML dùng cú pháp JSON hợp lệ theo YAML 1.2, C++ được sinh lúc phát triển, không parse YAML khi chạy.

## Optimize cân bằng dọc

Vòng đầu chỉ mở delta_CL0, delta_CD0, delta_Cm0 với bounds/prior; giữ hình học, slope, e và các đạo hàm động. Kiểm tra lực đẩy, mass, CG, bias AOA và mô hình đuôi trước khi diễn giải nghiệm.
Một điểm gần ổn định không tách duy nhất CL0/CL_alpha, CD0/e hay lực đẩy/cản. Dùng đoạn khác để kiểm tra; không dùng đường elevator replay làm bằng chứng đạt longitudinal trim.
Bounds khởi tạo tham khảo: delta_CL0 ±0,15; delta_CD0 ±0,005 (CD0 cuối phải dương); delta_Cm0 ±0,04. Đây là engineering priors, không phải dung sai CS-FSTD hoặc bảo đảm nghiệm đúng.
