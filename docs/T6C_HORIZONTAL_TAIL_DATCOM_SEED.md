# Seed đuôi ngang T-6C/PC-9M và downwash

Seed sơ bộ tại điểm FDR của Main Wing; chưa optimize và không phải output Digital DATCOM. Góc radian trong code; hệ số đuôi chuẩn hóa bằng S_h và MAC_h, không bằng diện tích cánh chính.
## Tọa độ và giả thiết cao độ

Gốc bản vẽ tổng thể có X dương về đuôi, Y phải, Z lên. Station thân 0 tại X=3 m. Không dùng station thân 0 làm gốc cho số X0=9.7123. Body FRD lấy CG làm gốc: r_B=(X_CG-X, Y-Y_CG, Z_CG-Z). Giả sử FRL song song trục X bản vẽ.
FRP FRONT Z=2.000 m; FRP REAR Z=2.045 m; quarter-MAC tail Z=2.645 m theo người dùng. CG 30% MAC chốt X=4.3675 m nhưng không xác định Z. Giữ Z_CG=2.000 m và wing representative plane Z=2.000 m như proxy của seed trước. Đây KHÔNG phải số đo cao độ MAC thực; chưa đưa phân bố dihedral/twist vào tọa độ AC đại diện. Cần hiệu chỉnh cao độ nếu xác định được từ bản vẽ/loading.
Wing MAC LE X=3.8725 m, quarter-MAC X=4.2850 m; 266 mm là độ lệch mép trước, không phải station MAC. Vị trí body wing=(+0.0825,0,0) m, tail=(-5.3448,0,-0.645) m. MAC span station ±0.8142 m không phải y của hợp lực toàn đuôi: y=0. MAC được dùng như chiều dài chiếu bằng trong chuyển tọa độ proxy; chưa áp dụng offset Z theo incidence.
Không đổi lift/drag seed Wing: với CG vẫn ở 30% MAC, cùng giả thiết cao độ, cánh tay đòn wing không đổi. Dữ liệu datum cũ 0.266/0.761 được thay bằng 3.8725/4.3675 m.
## Input và nguồn

| Input | Giá trị | Nguồn / giả thiết |
|---|---|---|
| span_m | 3.664 | USER: full tail/elevator span, not 3.100 m balance span |
| root_chord_m | 1.3 | USER/DRAWING |
| tip_chord_m | 0.65 | USER/DRAWING |
| reference_MAC_m | 1.011 | USER/DRAWING: retain rounded reference length |
| hinge_fraction | 0.64 | USER: unswept hinge; full-span elevator |
| airfoil | NACA 0012 | USER simplification, including outboard profile |
| incidence_deg | -2.0 | USER: relative FRL; zero twist and dihedral |
| balance_span_m | 3.1 | USER: balance plate, not elevator span; no hinge-moment model |
| wing_MAC_LE_X_m | 3.8725 | DRAWING corrected: global X aft, not fuselage station zero |
| tail_quarter_MAC_X_m | 9.7123 | USER/DRAWING global X |
| FRP_FRONT_Z_m | 2.0 | USER |
| FRP_REAR_Z_m | 2.045 | USER +45 mm |
| tail_quarter_MAC_Z_m | 2.645 | USER +600 mm above FRP REAR |
| wing_representative_Z_m | 2.0 | ASSUMPTION: representative wing plane at FRP FRONT; not measured MAC height |
| CG_Z_m | 2.0 | ASSUMPTION: retain provisional CG height; 30% MAC does not determine Z |
| CG_MAC_fraction | 0.3 | USER longitudinal CG assumption |
| Mach | 0.25383 | same FDR seed as Main Wing; frozen coefficients |
| rho | 1.10681 | same wing seed |
| TAS | 87.953 | same wing seed |
| mu | 1.84e-05 | same wing seed |
| body_alpha_deg | 1.2292929292929293 | same wing seed; sensor/body bias assumed zero |
| section_slope_ratio | 0.95 | ENGINEERING: viscous reduction of 2*pi |
| oswald_e | 0.85 | ENGINEERING prior |
| elevator_effectiveness_ratio | 0.85 | ENGINEERING correction to thin-airfoil flap effectiveness |
| wake_dynamic_pressure_ratio | 0.95 | ENGINEERING prior, not DATCOM wake chart evaluation; no propwash |
| elevator_limits_deg | [-25.0, 20.0] | ENGINEERING mechanical proxy: up negative/down positive, awaiting FCS confirmation |
| elevator_increment_arm_chord | 0.25 | ENGINEERING lift-increment arm behind AC; not hinge moment |

Elevator phủ toàn sải 3.664 m, hinge thẳng 64% chord. Diện tích elevator danh nghĩa =0.36*S_h; bỏ qua phần balance trước hinge, khe, bo tip và che thân. Chord sau hinge gốc/tip =0.468/0.234 m. 3.100 m là sải tấm cân bằng theo người dùng. NACA 0012 toàn sải là giản lược người dùng chọn; bản vẽ còn ghi NACA 0008 ngoài cánh. Không có mô hình hinge moment hoặc actuator.

## Input → công thức → kết quả → reference

| Output | Công thức / input | Giá trị | Reference / phương pháp |
|---|---|---:|---|
| S_h | `b*(cr+ct)/2` | 3.5724 | Geometry: projected trapezoid includes fuselage patch, neutral elevator |
| AR_h | `b^2/S` | 3.75794872 | DATCOM 2.2.2 / PDF 235-237 |
| MAC_geometric | `2*cr/3*(1+lambda+lambda^2)/(1+lambda)` | 1.01111111 | 2.2.2 / PDF 235 |
| y_MAC | `b/6*(1+2*lambda)/(1+lambda)` | 0.814222222 | trapezoid geometry |
| sweep_LE_deg | `atan(hinge*(cr-ct)/(b/2))` | 12.7934515 | unswept hinge geometry |
| sweep_quarter_deg | `atan((hinge-.25)*(cr-ct)/(b/2))` | 7.87818218 | geometry |
| CLalpha_h | `2*pi*A/[2+sqrt(4+(A*B/kappa)^2*(1+tan(L50)^2/B^2))]` | 3.7349732 | DATCOM Fig 4.1.3.2-49 / PDF 549; kappa assumed |
| tau_e | `0.85*[1-(theta-sin(theta))/pi], theta=acos(2*ce/c-1)` | 0.607956567 | Thin-airfoil engineering proxy; not digitized DATCOM elevator charts |
| CLdelta_e | `a_h*tau_e` | 2.27070148 | per rad, tail-area normalization |
| Cmdelta_e_AC | `-increment_arm*CLdelta_e` | -0.567675371 | ENGINEERING proxy about tail AC; CG transfer separate |
| CD0_h | `Cf*(1+1.2*t/c+100*(t/c)^4)*Swet/S; Swet=2*(1+.25*t/c)*S` | 0.00793433405 | DATCOM 4.1.5.1-a / PDF 723 + analytic Cf proxy, fully turbulent, full tail exposed proxy |
| induced_drag_factor | `1/(pi*A*e)` | 0.0996506997 | DATCOM polar 4.1.5.2 / PDF 757; e assumed |
| l_H | `dx*cos(iw)-dz*sin(iw)` | 5.42146473 | distance parallel wing root chord; representative root-plane proxy |
| h_H | `dx*sin(iw)+dz*cos(iw)` | 0.692336966 | height normal to root chord plane; magnitude used in KH |
| KA | `1/A-1/(1+A^1.7)` | 0.116861308 | DATCOM Fig 4.4.1-69a / PDF 1271 |
| Klambda | `(10-3*lambda)/7` | 1.16869301 | DATCOM Fig 4.4.1-69b / PDF 1271 |
| KH | `(1-abs(h/b))/(2*l/b)^(1/3)` | 0.910551766 | DATCOM Fig 4.4.1-70 / PDF 1272 |
| epsilon_alpha | `4.44*(KA*Klambda*KH*sqrt(cos(L25)))^1.19 * aw(M)/aw(0)` | 0.380383705 | DATCOM 4.4.1-h,i / PDF 1209; equivalent straight-tapered wing; geometry uncertainty |
| epsilon_ref_rad | `epsilon_alpha*CLwing/aw` | 0.0284865934 | ENGINEERING zero-lift anchor; Method 2 supplies gradient only |
| epsilon_flap | `epsilon_alpha/aw * dCLwing/dflap at zero` | 0.0888153623 | ENGINEERING local split-flap coupling only near flap Up; not validated at large flap |

## Thông số thực sự đưa vào component

| Trường | Giá trị |
|---|---|
| tail.Area | 3.5724 |
| tail.TailSpan | 3.664 |
| tail.TailMAC | 1.011 |
| tail.TailAR | 3.757948718 |
| tail.IncidenceAngle | -0.03490658504 |
| tail.MomentArm | 5.3448 |
| tail.ElevatorArea | 1.286064 |
| tail.LiftCurveSlope | 3.7349732 |
| tail.ZeroLiftAngle | 0 |
| tail.ZeroLiftDragCoefficient | 0.00793433405 |
| tail.InducedDragFactor | 0.09965069966 |
| tail.ElevatorEffectiveness | 0.6079565671 |
| tail.PitchMomentCoefficient | 0 |
| tail.PitchMomentCurveSlope | 0 |
| tail.ElevatorPitchMomentEffectiveness | -0.5676753712 |
| flow.referenceBodyAlphaRad | 0.02145520909 |
| flow.referenceDownwashRad | 0.02848659342 |
| flow.downwashGradientPerRad | 0.3803837046 |
| flow.referenceFlapRad | 0 |
| flow.flapDownwashGradientPerRad | 0.08881536227 |
| flow.dynamicPressureRatio | 0.95 |

Cm0_AC=Cm_alpha_AC=0 cho profile đối xứng là giả thiết baseline. Elevator có Cm_delta_AC proxy riêng; không nhập đạo hàm toàn máy bay. Pitch-rate response đến từ omega cross r; không cộng Cmq đuôi lần nữa.
Method 2 dùng các công thức được in trong Fig 4.4.1-69/70, không phải đường fit tự chọn. K_H dùng |h_H/b|; l_H,h_H đã quay về hệ dây cung gốc wing. Tuy nhiên áp dụng cánh tương đương, cao độ wing proxy và sweep rất nhỏ vẫn tạo bất định. Epsilon_ref và epsilon_flap là closure kỹ thuật, eta=0.95 là prior, không tuyên bố đã tính theo biểu đồ wake của DATCOM. Hệ số Mach/Re đóng băng ở điểm seed; chỉ dùng cục bộ trước stall và flap gần Up.

## Tải kiểm tra elevator=flap=0, p=q=r=0

```json
{
  "CL": -0.15663719165575718,
  "CD": 0.010379284874146657,
  "epsilon_deg": 1.632161575695666,
  "force_body_N": [
    -134.79276522338813,
    0.0,
    2276.745754100379
  ],
  "moment_CG_Nm": [
    0.0,
    12255.692040084796,
    0.0
  ]
}
```
Đây là riêng tải đuôi; không ép tổng máy bay trim. Âm CL tương ứng lực đuôi xuống. Moment CG gồm M_AC+r cross F, có cả z*Fx. Độ lệch elevator dương tăng lift, thường tạo moment chúi mũi.

## Sử dụng và tái lập

Trong cấu hình máy bay: `aircraft.addLoadComponent(makeT6CHorizontalTailWithDownwash());` sau khi include `trainer_aircraft/config/T6CTailSeed.hpp`. Factory này gắn flow vào tail thực sự; không cần tự truyền pointer. Chưa có mass/inertia/vertical-tail seed nên không đánh dấu toàn máy bay sẵn sàng.

```sh
python tools/generate_t6c_wing_seed.py --check
python tools/generate_t6c_tail_seed.py --check
cmake -S . -B build-t6c
cmake --build build-t6c
ctest --test-dir build-t6c --output-on-failure
```

Thay input trong generator rồi chạy không có --check. Tail generator đọc dữ liệu tính từ wing generator để không giữ epsilon_ref độc lập với CL wing. Khi cập nhật CG hoặc cao độ, phải cập nhật cả cấu hình hình học proxy và seed rồi kiểm tra tải. Các giới hạn elevator [-25,+20] độ là ước lượng, cần thay khi có tài liệu FCS; không được gắn nhãn giá trị đo.
