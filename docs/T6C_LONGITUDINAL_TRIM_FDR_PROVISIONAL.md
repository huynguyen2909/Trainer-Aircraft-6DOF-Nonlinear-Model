# T-6C / PC-9M: thử nghiệm hiệu chỉnh trim dọc bằng FDR 173

Trạng thái: **nghiệm thử có điều kiện, chưa đạt xác nhận CS-FSTD 2.c.5**. Mã
`tools/optimize_t6c_longitudinal_trim.py` gọi SciPy `least_squares` và
`tools/t6c_trim_loads.cpp` để tính **chính các component C++** của mô hình.
Nguồn FDR: `173.txt`, các mẫu Relative 13962–14027 s (66 mẫu, UTC
01:36:20–01:37:25 ngày 25-02-2026). Chỉ lưu thống kê, không đưa FDR gốc lên Git.
SHA-256 của nguồn: `f0e33627b9384cce4e912f46ca1c6bb2d4898181f1a48ef470817463f8584077`.

## Giá trị FDR và giả thiết

| Đại lượng | Giá trị trung bình / trạng thái | Xuất xứ và giới hạn |
|---|---:|---|
| TAS | 171,0227 kt = 87,9817 m/s | `ASPD TRUE` |
| Mach | 0,253833 | `MACH-1` |
| Pitch | +1,63636° | `PITCH-1`, header Degrees |
| Elevator | −0,675758° **giả định** | `ELEVPOS1` header không ghi đơn vị; dấu/góc cánh lái cần xác minh |
| Pitch trim | −2,30758 theo kênh | `ELETRIM`, không rõ đơn vị/tỉ lệ sang góc; chưa so được dung sai 0,5° |
| AOA | khoảng +1,229 | `AOA-2`, header không ghi đơn vị; không được dùng như góc θ của trim |
| IVS | +34,67 ft/min | Chứng tỏ khoảng FDR chưa phải bay bằng tuyệt đối |
| RPM | NP 100,202% | Áp dụng vào tốc độ trục 2.000 RPM proxy, **không** suy ra lực đẩy |
| Khối lượng | **2.800 kg, giả thiết** | Không có khối lượng tức thời trong tệp FDR |
| Mật độ | **1,10681 kg/m³, proxy** | Điểm seed DATCOM trước đây |
| Lực đẩy FDR | **không có** | Header FDR không có thrust; torque %/PCL không phải net thrust |

Khung trim giả định không gió, không góc nghiêng, bay bằng đều, không tốc độ góc:
`alpha = theta`, `beta = p = q = r = 0`, flap Up, NP theo FDR, gear thu. Vì
AOA quan sát khoảng 1,229 còn pitch 1,636° và IVS khác không, giả định này
**không tái tạo đúng trạng thái bay tức thời**. CG giữ seed proxy 30% MAC;
moment quán tính được cấp placeholder dương để tạo `TrainerAircraftModel`,
không ảnh hưởng cân bằng tĩnh.

## Danh sách tham số khí động và khả năng nhận dạng

| Bộ phận | Có ảnh hưởng ở trim dọc đứng yên (giá trị seed liên quan) | Lựa chọn ở **một** điểm FDR |
|---|---|---|
| Cánh chính | `CL0=0,202879`, `CL_alpha=4,53795 rad⁻¹`, `CD0=0,006497`, `oswald_e=0,82`, `Cm0=−0,0622203`, `Cm_alpha=0`; incidence/CG là hình học | Chỉ tối ưu **CL0, Cm0**; slope, drag và hình học giữ nguyên |
| Đuôi ngang + elevator | `LiftCurveSlope=3,73497 rad⁻¹`, `ZeroLiftAngle=0`, `CD0=0,007934`, `InducedDragFactor=0,099651`, `ElevatorEffectiveness=0,607957`, `PitchMomentCoefficient=0`, `PitchMomentCurveSlope=0`, `ElevatorPitchMomentEffectiveness=−0,567675` | Giữ nguyên; một điểm không tách được slope/elevator/downwash/CG |
| Dòng wing–tail | `referenceDownwashRad=0,028487`, `downwashGradientPerRad=0,380384`, `dynamicPressureRatio=0,95`; phần flap khi flap Up bằng 0 | Giữ nguyên |
| Fuselage | `formFactor=1,18576` và diện tích ướt tạo `CD0` phụ thuộc Re/M; `baseDragCoefficient=0`, `normalForceSlopePerRad=0,130105`, `pitchingMomentZero=0`, `pitchingMomentSlopePerRad=0,222931` | Giữ nguyên; chỉ có một α thì intercept/slope bị đồng tuyến |
| Đuôi đứng | Ở β=δr=0, chỉ `ZeroLiftDragCoefficient` đóng góp drag, vị trí fin chuyển lực drag thành moment; các `SideForceCurveSlope`, `RudderEffectiveness`, `YawMoment*` không được kích thích | **Chưa có factory T6C vertical-tail DATCOM seed**. Bridge khóa proxy **S=2 m², CD0=0,009**, x=−5 m, z=−0,9 m; không diễn giải là số đo T-6C |
| Propeller | Blade pitch/chord/polar và NP của BEMT quyết định thrust, torque và tải tại hub | Chỉ tối ưu `effective_thrust_scale`, nhân tải BEMT sau khi tính; đây là hiệu chỉnh lực đẩy **thử nghiệm**, chưa phải bản đồ công suất/governor hay hệ số DATCOM đã nhận dạng |
| Landing gear | Không có khí động gear trong cấu hình trên | Không đăng ký component ground contact; `landingGearExtended=false` |

Các hệ số `CL_q`, `Cm_q`, flap increments, lateral/yaw derivatives và damping
không nhận dạng được khi `q=β=δf=0`; cần thêm các điểm bay ở α/V/flap/điều
khiển khác nhau. Để hiệu chỉnh các bộ phận riêng biệt phải đo thêm mass/CG,
đặc tính động cơ–propeller (thrust hoặc power + blade pitch), chuẩn hóa góc
elevator, xác nhận seed fin, và thu nhiều điểm trim độc lập.

## Phương trình, nghiệm và sai lệch

Tại mỗi lần gọi bridge, mô hình cộng Wing, Horizontal tail với downwash,
Fuselage, Vertical tail proxy và Propeller. `RigidBody6DOF` cộng trọng lực
đúng một lần. SciPy dùng ba residual chuẩn hóa

`r = [m·u_dot/(mg), m·w_dot/(mg), My/(mg·1,65 m)]`.

Lần solve thứ nhất giữ hệ số aero seed để tìm (`pitch`, `elevator`, scale
thrust) sao cho `r=0`; lần thứ hai giữ pitch/elevator FDR để tìm (`CL0`,
`Cm0`, scale thrust) theo `least_squares` có giới hạn; sau cùng giải trim lại
để dự báo đầu ra. Chỉ tiêu thrust được so thêm khi có `--target-net-thrust-n`
độc lập; nếu không có, thrust là ẩn phải suy ra từ cân bằng lực, **không phải
sai số so với FDR**.

| Component / hệ số | Trước | Sau | Thay đổi so với seed |
|---|---:|---:|---:|
| Wing `CL0` | 0,20287914 | 0,25420181 | +25,2972% |
| Wing `Cm0` | −0,06222033 | −0,13885299 | +123,1634% theo tỉ lệ trị số đại số |
| Propeller `effective_thrust_scale` | 1,000000 | 0,28793715 | −71,2063% |

| Điểm trim | FDR | Trước (giữ aero seed, giải trim) | Sau (giải trim) | Dung sai CS-FSTD Issue 1, 2.c.5 |
|---|---:|---:|---:|---:|
| Elevator | −0,67576°* | +1,74656° | −0,67576°* | ±1° (chỉ có nghĩa sau khi xác minh kênh FDR) |
| Pitch | +1,63636° | +1,98201° | +1,63636° | ±1° |
| Pitch trim | chưa xác định góc | chưa mô hình hóa | chưa mô hình hóa | ±0,5°; chưa thể chấm |
| Net thrust | **không đo** | 1.639,42 N suy ra | 1.707,51 N suy ra | Không có ngưỡng net thrust trong **Issue 1 năm 2026** |

*Giả sử số `ELEVPOS1` là độ và quy ước dấu khớp với input C++.*

Propeller cố định blade pitch ở seed có thrust BEMT lớn hơn lực đẩy cần ở
cruise: thay đổi scale −71% cho thấy cần governor/collective/power map,
không chứng minh blade polar thực sai 71%. Thay đổi `Cm0` cũng hấp thụ sai số
CG, downwash, elevator mapping và thrust line; **không ghi các số hậu hiệu
chỉnh vào factory seed** khi chưa có chứng cứ tách từng nguyên nhân.

Theo bản [CS-FSTD Issue 1 của EASA](https://www.easa.europa.eu/en/document-library/certification-specifications/cs-fstd-issue-1), bảng 2.c.5 ở trang 376 của PDF dự án có `elevator ±1°`, `pitch trim ±0,5°`, `pitch ±1°`; yêu cầu bay bằng đều và thrust để duy trì level flight được mô tả ở trang 382. Tài liệu CS-FSTD(A) đời cũ có thể có tiêu chí `net thrust ±5%`, **không gán tiêu chí đó sang Issue 1 này**. Hai góc sau optimize khớp vì được dùng làm mục tiêu tại chính một snapshot, chưa phải đánh giá trên tập kiểm định độc lập.

## Chạy lại

```bash
g++ -std=c++17 -O1 -Iinclude tools/t6c_trim_loads.cpp $(rg --files src -g '*.cpp') -o /tmp/t6c_trim_loads
python tools/optimize_t6c_longitudinal_trim.py \
  --fdr /path/to/173.txt --bridge /tmp/t6c_trim_loads \
  --mass-kg 2800 --assume-elevator-degrees \
  --out calibration_results/t6c_longitudinal_trim_provisional.json
```

Kết quả JSON gồm FDR trung bình, các residual, từng parameter trước/sau và
% thay đổi; CSV cùng tên hậu tố `_parameters.csv`. Đây là phép tính thử
được gắn điều kiện rõ ràng, không phải kết luận đạt chuẩn hay seed triển khai.
