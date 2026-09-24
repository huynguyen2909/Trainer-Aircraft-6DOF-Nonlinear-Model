#!/usr/bin/env python3
"""PC-9M silhouette-derived, equivalent-body DATCOM small-alpha seed.

Read-only inputs are the digitized station list below; generated code, ledger,
and documentation are refreshed together. --check compares all three files.
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]

# X is global drawing X (m), NOT fuselage SP/FR (SP/FR 0 = global X 3.000).
# Z is up from the drawing Z=0 line. Width is full symmetric breadth.
# Z and breadth are VISUAL ESTIMATES (roughly +/-0.12 m, worse beneath wing).
# Nose includes the spinner, whose hub drag is not modeled elsewhere. The
# final tail-cone closure is obscured by empennage and is a shape proxy.
# Entries at SP/FR 0,2,3,4,5,6,7,8,9,10,11 are identifiable in the PDF;
# auxiliary entries preserve the canopy and forward cowling curvature.
STATIONS = [
    #   X,     Zupper, Zlower, width,   feature/source
    (0.55,   2.36,   2.34,  0.02, "spinner tip / visual"),
    (1.10,   2.58,   2.22,  0.38, "spinner aft / visual"),
    (1.65,   2.68,   1.77,  0.68, "cowling / visual"),
    (2.35,   2.76,   1.66,  0.82, "cowling / visual"),
    (3.00,   2.80,   1.61,  0.91, "SP/FR 0 / visual"),
    (3.375,  3.02,   1.58,  0.95, "SP/FR 1a / visual"),
    (3.7475, 3.24,   1.57,  0.97, "SP/FR 2 / visual"),
    (4.12,   3.40,   1.57,  0.99, "SP/FR 2a / visual"),
    (4.4855, 3.46,   1.59,  1.01, "SP/FR 3 / visual"),
    (4.826,  3.48,   1.61,  1.01, "SP/FR 3a / visual"),
    (5.281,  3.43,   1.63,  0.99, "SP/FR 3c / visual"),
    (5.545,  3.38,   1.65,  0.98, "SP/FR 3d / visual"),
    (5.965,  3.21,   1.70,  0.95, "SP/FR 4 / visual"),
    (6.648,  3.03,   1.88,  0.89, "SP/FR 5 / visual"),
    (7.148,  2.99,   2.00,  0.78, "SP/FR 6 / visual"),
    (7.768,  2.94,   2.12,  0.63, "SP/FR 7 / visual"),
    (8.220,  2.91,   2.20,  0.54, "SP/FR 8 / visual"),
    (8.680,  2.86,   2.29,  0.44, "SP/FR 9 / visual"),
    (9.200,  2.79,   2.37,  0.34, "SP/FR 10 / visual"),
    (9.480,  2.75,   2.41,  0.28, "SP/FR 11 / visual"),
    (9.950,  2.69,   2.49,  0.17, "afterbody / visual"),
    (10.30,  2.65,   2.62,  0.04, "aft cone closure / proxy"),
]


def build():
    from generate_t6c_wing_seed import build as wing_build

    w = wing_build()["runtime_config"]
    sw = w["wing.area_m2"]
    chord = 1.650
    cg_x = 3.8725 + 0.30 * chord
    s = []
    for x, upper, lower, width, feature in STATIONS:
        if not (0.0 < width and upper > lower):
            raise ValueError(f"Invalid section at X={x}")
        a, b = width / 2.0, (upper - lower) / 2.0
        perimeter = math.pi * (3.0 * (a + b) -
                               math.sqrt((3.0 * a + b) * (a + 3.0 * b)))
        s.append(dict(x_m=x, fuselage_station_m=x-3.0, z_upper_m=upper,
                      z_lower_m=lower, breadth_m=width, area_m2=math.pi*a*b,
                      perimeter_m=perimeter, centroid_z_m=(upper+lower)/2,
                      source=feature))
    if any(b["x_m"] <= a["x_m"] for a, b in zip(s, s[1:])):
        raise ValueError("Body stations must be strictly increasing")
    length = s[-1]["x_m"] - s[0]["x_m"]
    volume = sum((a["area_m2"]+b["area_m2"])*(b["x_m"]-a["x_m"])/2
                 for a,b in zip(s,s[1:]))
    swet = sum((a["perimeter_m"]+b["perimeter_m"])*(b["x_m"]-a["x_m"])/2
               for a,b in zip(s,s[1:]))
    max_area = max(v["area_m2"] for v in s)
    max_diameter = math.sqrt(4*max_area/math.pi)
    fineness = length/max_diameter
    slopes = [(b["area_m2"]-a["area_m2"])/(b["x_m"]-a["x_m"])
              for a,b in zip(s,s[1:])]
    max_idx = max(range(len(s)), key=lambda i:s[i]["area_m2"])
    # First strong afterbody area decrease. x0=x1 is a transparent proxy to
    # Figure 4.2.1.1-20b; not an actual digitization of that correlation.
    x1_idx = min(range(max_idx,len(slopes)),key=lambda i:slopes[i])
    x0_idx = x1_idx+1
    x0 = s[x0_idx]["x_m"]
    s0 = s[x0_idx]["area_m2"]
    mass_factor = 0.94  # rough read of Figure 4.2.1.1-20a, NOT measured
    cn_alpha = 2*mass_factor*s0/sw  # 4.2.1.1-a, change V^(2/3) to Sw
    # 4.2.2.1-b: potential-flow section load 2*q*k*(dS/dx)*alpha.
    # Accumulate about CG: body +Y pitch for an upward forebody force.
    moment_integral = sum((cg_x-(a["x_m"]+b["x_m"])/2)*
                          (b["area_m2"]-a["area_m2"])
                          for a,b in zip(s[:x0_idx],s[1:x0_idx+1]))
    cm_alpha = 2*mass_factor*moment_integral/(sw*chord)
    form_factor = 1+60/fineness**3+0.0025*fineness
    rho = w["flight.rho_kg_m3"]
    speed = w["flight.speed_m_s"]
    mach = wing_build()["inputs"]["Mach"]["value"]
    mu = 1.84e-5
    re = rho*speed*length/mu
    cf = .455/(math.log10(re)**2.58*(1+.144*mach*mach)**.65)
    cd0 = cf*form_factor*swet/sw
    alpha = math.radians(w["flight.alpha_deg"])
    q = .5*rho*speed*speed
    fdrag = -q*sw*cd0
    fnormal = -q*sw*cn_alpha*alpha
    mpitch = q*sw*chord*cm_alpha*alpha
    config = dict(referenceAreaM2=sw,referenceChordM=chord,bodyLengthM=length,
                  wettedAreaM2=swet,formFactor=form_factor,baseDragCoefficient=0.0,
                  normalForceSlopePerRad=cn_alpha,pitchingMomentZero=0.0,
                  pitchingMomentSlopePerRad=cm_alpha)
    return dict(source="PC-9-M-Model-Building-Plan.pdf, side and plan view; visual readings",
                inputs=dict(CG_X_m=cg_x,CG_Z_m=2.0,reference_area_m2=sw,
                            reference_MAC_m=chord,apparent_mass_factor_proxy=mass_factor,
                            x0_selection="end of first strongest falling-area interval",
                            body_cross_section="ellipse fitted to estimated side/top silhouettes",
                            body_Mach_reference=mach,rho_kgm3=rho,TAS_mps=speed,
                            viscosity_Pa_s=mu,alpha_reference_rad=alpha),
                stations=s,geometry=dict(length_m=length,volume_m3=volume,
                     wetted_area_m2=swet,max_equivalent_diameter_m=max_diameter,
                     fineness_ratio=fineness,max_cross_section_m2=max_area,
                     x1_m=s[x1_idx+1]["x_m"],x0_m=x0,S_x0_m2=s0),
                results=dict(CN_alpha_per_rad=cn_alpha,Cm_alpha_CG_per_rad=cm_alpha,
                             Cm0_CG=0.0,CD0_at_reference=cd0,Re_ref=re,Cf_ref=cf,
                             crossflow_drag_alpha_increment=0.0,
                             body_lift_zero=0.0),
                runtime_config=config,
                snapshot=dict(alpha_rad=alpha,CD0=cd0,CN=cn_alpha*alpha,
                              CmCG=cm_alpha*alpha,
                              force_body_N=[fdrag*math.cos(alpha),0.0,
                                            fdrag*math.sin(alpha)+fnormal],
                              moment_CG_Nm=[0.0,mpitch,0.0]))


def outputs(d):
    fmt=lambda v:format(v,'.17g')
    cpp=['// Generated by tools/generate_t6c_fuselage_seed.py',
         '#include "trainer_aircraft/config/T6CFuselageSeed.hpp"','',
         'namespace trainer_aircraft {',
         'fuselage::DatcomFuselageConfig makeT6CFuselageDatcomSeed() {',
         '    fuselage::DatcomFuselageConfig c;']
    for k,v in d['runtime_config'].items(): cpp.append(f'    c.{k} = {fmt(v)};')
    cpp+=['    return c;','}',
          'std::unique_ptr<fuselage::DatcomFuselageComponent> makeT6CFuselageWithDatcomSeed() {',
          '    return std::make_unique<fuselage::DatcomFuselageComponent>(makeT6CFuselageDatcomSeed());',
          '}','} // namespace trainer_aircraft','']
    g=d['geometry']; r=d['results']; i=d['inputs']
    md=['# Seed Fuselage T-6C / PC-9M theo DATCOM','',
        'Seed body-alone tại trim gần thẳng, **chưa xác thực hoặc optimize**. '
        'Tài liệu gốc: USAF Stability and Control DATCOM §§4.2.1.1, 4.2.2.1, 4.2.3.1, '
        '4.2.3.2, 4.3.1.2, 4.3.2.1, 4.3.3.1. Trang PDF bản AFWAL-TR-83-3048 '
        'tương ứng: 815, 885, 939, 1008, 1036, 1075, 1128. '
        'Các phương trình DATCOM được dùng với hình học thân tròn tương đương; '
        'đường bao PC-9M không đối xứng tròn nên **không phải output Digital DATCOM**.',
        '', '## Cách đọc hình và quy ước','',
        'Bản vẽ PC-9-M-Model-Building-Plan.pdf, trang 5, cho hình chiếu bên/bằng '
        'và các SP/FR theo chiều X. Gốc tổng thể X dương về đuôi, '
        'SP/FR 0 tại X=3.000 m; Z dương lên. FRP FRONT Z=2.000 m, '
        'FRP REAR Z=2.045 m theo người dùng. Các đường Z trên/dưới và '
        'bề rộng được **ước lượng trực tiếp từ nét in** ở cùng tỷ lệ X; '
        'không phải cao độ hoặc kích thước tiết diện được nhà sản xuất ghi. '
        'Độ bất định thực tế khoảng ±0.12 m ở phần nhìn rõ, lớn hơn khi '
        'canopy/cánh/đuôi che thân. Chỉ gồm silhouette thân và spinner; '
        'loại cánh, càng, tailplane và vây khỏi tiết diện.',
        '', 'Bản vẽ không chỉ ra chính xác điểm đóng thân trong tailplane: '
        'đầu 0.55 m và cuối 10.30 m là proxy, do đó chiều dài thân '+
        f'{g["length_m"]:.3f} m khác chiều dài toàn máy bay 10.175 m và '
        'giá trị factory Nicolosi 10.18 m. Z_CG=2.000 m là proxy, '
        f'X_CG={i["CG_X_m"]:.4f} m theo 30% MAC.',
        '', '## Hình học trạm (m, m²)','',
        '| X toàn cục | SP/FR tương đối | Z trên | Z dưới | Bề rộng | Diện tích elip | Chú thích |',
        '|---:|---:|---:|---:|---:|---:|---|']
    for s in d['stations']:
        md.append(f'| {s["x_m"]:.4f} | {s["fuselage_station_m"]:+.4f} | '
                  f'{s["z_upper_m"]:.3f} | {s["z_lower_m"]:.3f} | '
                  f'{s["breadth_m"]:.3f} | {s["area_m2"]:.4f} | {s["source"]} |')
    md+=['', 'Mỗi tiết diện là elip có bán trục a=W/2, b=(Ztrên−Zdưới)/2; '
         'S(x)=πab. Chu vi gần đúng Ramanujan; thể tích và diện tích ướt '
         'tích phân hình thang theo X. Vùng giao cánh–thân chưa trừ diện tích '
         'che phủ, nên S_ướt có thể cao; không cộng cản junction riêng.',
         '', '## Input → phương pháp → output → nguồn','',
         '| Input / giả thiết | Phương pháp | Kết quả | DATCOM / mức chắc chắn |',
         '|---|---|---:|---|',
         f'| Trạm đầu/cuối | Xcuối−Xđầu | L={g["length_m"]:.6g} m | Hình chiếu; hai đầu proxy |',
         f'| S(x), P(x) | ∫S dx, ∫P dx | V={g["volume_m3"]:.6g} m³; Sướt={g["wetted_area_m2"]:.6g} m² | Elip tương đương; §4.2.3.1 |',
         f'| max S(x) | √(4Smax/π); L/d | d_eq={g["max_equivalent_diameter_m"]:.6g} m; f={g["fineness_ratio"]:.6g} | Hình học tương đương; §4.2.3.1 |',
         f'| dS/dx sau Smax | x1 tại khoảng dS/dx âm cực đại; x0=x1 | x0={g["x0_m"]:.6g} m; S0={g["S_x0_m2"]:.6g} m² | x0=x1 là proxy của Hình 4.2.1.1-20b |',
         f'| k₂−k₁={i["apparent_mass_factor_proxy"]:.3g} | 2(k₂−k₁)S0 / Sref | CNα={r["CN_alpha_per_rad"]:.8g} rad⁻¹ | Eq. 4.2.1.1-a / PDF 815; k ước lượng, body tròn tương đương |',
         f'| S(x), XCG | 2k ∫(XCG−X) dS / (Sref cbar), X≤x0 | Cmα,CG={r["Cm_alpha_CG_per_rad"]:.8g} rad⁻¹ | Eq. 4.2.2.1-b / PDF 887; tích phân tuyến tính từng trạm |',
         f'| Re={r["Re_ref"]:.6g}, Mach={i["body_Mach_reference"]:.5g} | Cf=.455/[log10(Re)^2.58(1+.144M²)^.65] | Cf={r["Cf_ref"]:.8g} | Cf toàn rối, tương quan giải tích proxy cho §4.2.3.1 |',
         f'| f, Sướt, Sref | CD0=Cf(1+60/f³+.0025f)Sướt/Sref | CD0={r["CD0_at_reference"]:.8g} | §4.2.3.1 / PDF 939; base=0 proxy |',
         '| Camber thân và cản α | Cm0=0; ΔCDα=0 tại seed | Cm0=0; ΔCDα=0 | Chưa có body-asymmetry chart/§4.2.3.2; giả định, không phải kết quả DATCOM |',
         '', 'DATCOM §4.2.1.1 không xác nhận công thức body-of-revolution '
         'cho thân không tròn. Phương pháp elip tương đương và k, x0 ước lượng '
         'là nguồn sai số lớn. Chưa tính tương tác wing–body (§§4.3.1–4.3.3): '
         'nếu thêm vào Wing hoặc Fuselage phải dùng phần gia số, không cộng cả '
         'hệ số wing–body lên các hệ số cô lập. Chưa mô phỏng propwash, '
         'cản tăng ở α lớn, vortex/crossflow, pitching dynamics, lực cạnh hoặc yaw mới.',
         '', '## Tích hợp và quy chiếu','',
         'Hệ số CN, CD dùng Sref=16.28 m²; Cm dùng Sref và cbar=1.650 m. '
         'Bộ số lúc chạy dùng Cf(Re,M) cập nhật cùng điều kiện bay; CNα, Cmα '
         'đóng băng tại seed. Trong hệ body FRD, F_D=−D V_air/|V_air|; '
         'F_N=(0,0,−q Sref CNα α). Cmα được tích phân **về CG** nên '
         'M_CG,y=q Sref cbar (Cm0+Cmα α), không áp dụng lại r×F_N. '
         'Lực cản đặt tại CG là closure ban đầu. Tại α gần 0, mô hình '
         'phù hợp bài trim; ngoài vùng này cần hiệu chỉnh bảng theo α.',
         '', 'Cấu hình: `aircraft.addLoadComponent(makeT6CFuselageWithDatcomSeed());` '
         'sau khi include `trainer_aircraft/config/T6CFuselageSeed.hpp`. '
         'Không gắn đồng thời `makeT6cReferenceFuselageConfig()` vào một aircraft '
         'vì sẽ tính hai lần cản và moment thân. Public baseline không đổi; '
         'toàn aircraft còn thiếu các hạng mass/inertia đã xác thực.',
         '', '## Tải mẫu tại điểm FDR tham chiếu (β=0, α nhỏ)','', '```json',
         json.dumps(d['snapshot'],indent=2), '```','',
         'Chạy `python tools/generate_t6c_fuselage_seed.py` để tái tạo '
         'C++/YAML/Markdown; thêm `--check` để phát hiện bản đã lệch. '
         'Giữ riêng `reference_data/t6c/fuselage_source_parameters.yaml` '
         'làm hồ sơ Nicolosi lịch sử, không đọc vào factory mới.', '']
    return {'src/config/T6CFuselageSeed.cpp':'\n'.join(cpp),
            'reference_data/t6c/fuselage_datcom_seed.yaml':json.dumps(d,indent=2,ensure_ascii=False)+'\n',
            'docs/T6C_FUSELAGE_DATCOM_SEED.md':'\n'.join(md)}


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--check',action='store_true')
    args=p.parse_args()
    d=build()
    for name, content in outputs(d).items():
        path=ROOT/name
        if args.check:
            if not path.exists() or path.read_text()!=content:
                raise SystemExit('Stale generated file: '+name)
        else:
            path.write_text(content)
    print(json.dumps(dict(geometry=d['geometry'],results=d['results'],snapshot=d['snapshot']),indent=2))
