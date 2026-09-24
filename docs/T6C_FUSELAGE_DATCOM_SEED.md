# Seed Fuselage T-6C / PC-9M theo DATCOM

Seed body-alone tại trim gần thẳng, **chưa xác thực hoặc optimize**. Tài liệu gốc: USAF Stability and Control DATCOM §§4.2.1.1, 4.2.2.1, 4.2.3.1, 4.2.3.2, 4.3.1.2, 4.3.2.1, 4.3.3.1. Trang PDF bản AFWAL-TR-83-3048 tương ứng: 815, 885, 939, 1008, 1036, 1075, 1128. Các phương trình DATCOM được dùng với hình học thân tròn tương đương; đường bao PC-9M không đối xứng tròn nên **không phải output Digital DATCOM**.

## Cách đọc hình và quy ước

Bản vẽ PC-9-M-Model-Building-Plan.pdf, trang 5, cho hình chiếu bên/bằng và các SP/FR theo chiều X. Gốc tổng thể X dương về đuôi, SP/FR 0 tại X=3.000 m; Z dương lên. FRP FRONT Z=2.000 m, FRP REAR Z=2.045 m theo người dùng. Các đường Z trên/dưới và bề rộng được **ước lượng trực tiếp từ nét in** ở cùng tỷ lệ X; không phải cao độ hoặc kích thước tiết diện được nhà sản xuất ghi. Độ bất định thực tế khoảng ±0.12 m ở phần nhìn rõ, lớn hơn khi canopy/cánh/đuôi che thân. Chỉ gồm silhouette thân và spinner; loại cánh, càng, tailplane và vây khỏi tiết diện.

Bản vẽ không chỉ ra chính xác điểm đóng thân trong tailplane: đầu 0.55 m và cuối 10.30 m là proxy, do đó chiều dài thân 9.750 m khác chiều dài toàn máy bay 10.175 m và giá trị factory Nicolosi 10.18 m. Z_CG=2.000 m là proxy, X_CG=4.3675 m theo 30% MAC.

## Hình học trạm (m, m²)

| X toàn cục | SP/FR tương đối | Z trên | Z dưới | Bề rộng | Diện tích elip | Chú thích |
|---:|---:|---:|---:|---:|---:|---|
| 0.5500 | -2.4500 | 2.360 | 2.340 | 0.020 | 0.0003 | spinner tip / visual |
| 1.1000 | -1.9000 | 2.580 | 2.220 | 0.380 | 0.1074 | spinner aft / visual |
| 1.6500 | -1.3500 | 2.680 | 1.770 | 0.680 | 0.4860 | cowling / visual |
| 2.3500 | -0.6500 | 2.760 | 1.660 | 0.820 | 0.7084 | cowling / visual |
| 3.0000 | +0.0000 | 2.800 | 1.610 | 0.910 | 0.8505 | SP/FR 0 / visual |
| 3.3750 | +0.3750 | 3.020 | 1.580 | 0.950 | 1.0744 | SP/FR 1a / visual |
| 3.7475 | +0.7475 | 3.240 | 1.570 | 0.970 | 1.2723 | SP/FR 2 / visual |
| 4.1200 | +1.1200 | 3.400 | 1.570 | 0.990 | 1.4229 | SP/FR 2a / visual |
| 4.4855 | +1.4855 | 3.460 | 1.590 | 1.010 | 1.4834 | SP/FR 3 / visual |
| 4.8260 | +1.8260 | 3.480 | 1.610 | 1.010 | 1.4834 | SP/FR 3a / visual |
| 5.2810 | +2.2810 | 3.430 | 1.630 | 0.990 | 1.3996 | SP/FR 3c / visual |
| 5.5450 | +2.5450 | 3.380 | 1.650 | 0.980 | 1.3316 | SP/FR 3d / visual |
| 5.9650 | +2.9650 | 3.210 | 1.700 | 0.950 | 1.1267 | SP/FR 4 / visual |
| 6.6480 | +3.6480 | 3.030 | 1.880 | 0.890 | 0.8039 | SP/FR 5 / visual |
| 7.1480 | +4.1480 | 2.990 | 2.000 | 0.780 | 0.6065 | SP/FR 6 / visual |
| 7.7680 | +4.7680 | 2.940 | 2.120 | 0.630 | 0.4057 | SP/FR 7 / visual |
| 8.2200 | +5.2200 | 2.910 | 2.200 | 0.540 | 0.3011 | SP/FR 8 / visual |
| 8.6800 | +5.6800 | 2.860 | 2.290 | 0.440 | 0.1970 | SP/FR 9 / visual |
| 9.2000 | +6.2000 | 2.790 | 2.370 | 0.340 | 0.1122 | SP/FR 10 / visual |
| 9.4800 | +6.4800 | 2.750 | 2.410 | 0.280 | 0.0748 | SP/FR 11 / visual |
| 9.9500 | +6.9500 | 2.690 | 2.490 | 0.170 | 0.0267 | afterbody / visual |
| 10.3000 | +7.3000 | 2.650 | 2.620 | 0.040 | 0.0009 | aft cone closure / proxy |

Mỗi tiết diện là elip có bán trục a=W/2, b=(Ztrên−Zdưới)/2; S(x)=πab. Chu vi gần đúng Ramanujan; thể tích và diện tích ướt tích phân hình thang theo X. Vùng giao cánh–thân chưa trừ diện tích che phủ, nên S_ướt có thể cao; không cộng cản junction riêng.

## Input → phương pháp → output → nguồn

| Input / giả thiết | Phương pháp | Kết quả | DATCOM / mức chắc chắn |
|---|---|---:|---|
| Trạm đầu/cuối | Xcuối−Xđầu | L=9.75 m | Hình chiếu; hai đầu proxy |
| S(x), P(x) | ∫S dx, ∫P dx | V=6.72152 m³; Sướt=26.8301 m² | Elip tương đương; §4.2.3.1 |
| max S(x) | √(4Smax/π); L/d | d_eq=1.3743 m; f=7.09452 | Hình học tương đương; §4.2.3.1 |
| dS/dx sau Smax | x1 tại khoảng dS/dx âm cực đại; x0=x1 | x0=5.965 m; S0=1.12665 m² | x0=x1 là proxy của Hình 4.2.1.1-20b |
| k₂−k₁=0.94 | 2(k₂−k₁)S0 / Sref | CNα=0.13010497 rad⁻¹ | Eq. 4.2.1.1-a / PDF 815; k ước lượng, body tròn tương đương |
| S(x), XCG | 2k ∫(XCG−X) dS / (Sref cbar), X≤x0 | Cmα,CG=0.22293112 rad⁻¹ | Eq. 4.2.2.1-b / PDF 887; tích phân tuyến tính từng trạm |
| Re=5.15835e+07, Mach=0.25383 | Cf=.455/[log10(Re)^2.58(1+.144M²)^.65] | Cf=0.0023250876 | Cf toàn rối, tương quan giải tích proxy cho §4.2.3.1 |
| f, Sướt, Sref | CD0=Cf(1+60/f³+.0025f)Sướt/Sref | CD0=0.0045436627 | §4.2.3.1 / PDF 939; base=0 proxy |
| Camber thân và cản α | Cm0=0; ΔCDα=0 tại seed | Cm0=0; ΔCDα=0 | Chưa có body-asymmetry chart/§4.2.3.2; giả định, không phải kết quả DATCOM |

DATCOM §4.2.1.1 không xác nhận công thức body-of-revolution cho thân không tròn. Phương pháp elip tương đương và k, x0 ước lượng là nguồn sai số lớn. Chưa tính tương tác wing–body (§§4.3.1–4.3.3): nếu thêm vào Wing hoặc Fuselage phải dùng phần gia số, không cộng cả hệ số wing–body lên các hệ số cô lập. Chưa mô phỏng propwash, cản tăng ở α lớn, vortex/crossflow, pitching dynamics, lực cạnh hoặc yaw mới.

## Tích hợp và quy chiếu

Hệ số CN, CD dùng Sref=16.28 m²; Cm dùng Sref và cbar=1.650 m. Bộ số lúc chạy dùng Cf(Re,M) cập nhật cùng điều kiện bay; CNα, Cmα đóng băng tại seed. Trong hệ body FRD, F_D=−D V_air/|V_air|; F_N=(0,0,−q Sref CNα α). Cmα được tích phân **về CG** nên M_CG,y=q Sref cbar (Cm0+Cmα α), không áp dụng lại r×F_N. Lực cản đặt tại CG là closure ban đầu. Tại α gần 0, mô hình phù hợp bài trim; ngoài vùng này cần hiệu chỉnh bảng theo α.

Cấu hình: `aircraft.addLoadComponent(makeT6CFuselageWithDatcomSeed());` sau khi include `trainer_aircraft/config/T6CFuselageSeed.hpp`. Không gắn đồng thời `makeT6cReferenceFuselageConfig()` vào một aircraft vì sẽ tính hai lần cản và moment thân. Public baseline không đổi; toàn aircraft còn thiếu các hạng mass/inertia đã xác thực.

## Tải mẫu tại điểm FDR tham chiếu (β=0, α nhỏ)

```json
{
  "alpha_rad": 0.021455209087647465,
  "CD0": 0.004543662710926037,
  "CN": 0.0027914293074646,
  "CmCG": 0.004783033789969939,
  "force_body_N": [
    -316.5956277748822,
    0.0,
    -201.34106646132892
  ],
  "moment_CG_Nm": [
    0.0,
    550.0297589501607,
    0.0
  ]
}
```

Chạy `python tools/generate_t6c_fuselage_seed.py` để tái tạo C++/YAML/Markdown; thêm `--check` để phát hiện bản đã lệch. Giữ riêng `reference_data/t6c/fuselage_source_parameters.yaml` làm hồ sơ Nicolosi lịch sử, không đọc vào factory mới.
