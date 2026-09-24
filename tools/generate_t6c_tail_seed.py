#!/usr/bin/env python3
"""Reproducible tail + wing-tail seed, DATCOM/proxy provenance kept explicit."""
import sys
sys.dont_write_bytecode = True
import argparse, json, math
from pathlib import Path
from generate_t6c_wing_seed import build as wing_build
R=Path(__file__).resolve().parents[1]
rad=math.radians

def build():
    wing=wing_build(); w=wing['runtime_config']; rows=[]; inputs={}
    def inp(k,v,source): inputs[k]={'value':v,'source':source}; return v
    def out(k,v,f,ref): rows.append(dict(parameter=k,value=v,formula=f,reference=ref));return v
    b=inp('span_m',3.664,'USER: full tail/elevator span, not 3.100 m balance span')
    cr=inp('root_chord_m',1.3,'USER/DRAWING');ct=inp('tip_chord_m',.65,'USER/DRAWING')
    mac=inp('reference_MAC_m',1.011,'USER/DRAWING: retain rounded reference length')
    hinge=inp('hinge_fraction',.64,'USER: unswept hinge; full-span elevator')
    inp('airfoil','NACA 0012','USER simplification, including outboard profile')
    ih=inp('incidence_deg',-2.,'USER: relative FRL; zero twist and dihedral')
    inp('balance_span_m',3.100,'USER: balance plate, not elevator span; no hinge-moment model')
    xle=inp('wing_MAC_LE_X_m',3.8725,'DRAWING corrected: global X aft, not fuselage station zero')
    xh=inp('tail_quarter_MAC_X_m',9.7123,'USER/DRAWING global X')
    zf=inp('FRP_FRONT_Z_m',2.,'USER');zr=inp('FRP_REAR_Z_m',zf+.045,'USER +45 mm')
    zh=inp('tail_quarter_MAC_Z_m',zr+.600,'USER +600 mm above FRP REAR')
    zw=inp('wing_representative_Z_m',2.,'ASSUMPTION: representative wing plane at FRP FRONT; not measured MAC height')
    zcg=inp('CG_Z_m',2.,'ASSUMPTION: retain provisional CG height; 30% MAC does not determine Z')
    hcg=inp('CG_MAC_fraction',.30,'USER longitudinal CG assumption')
    cgx=xle+hcg*1.65;xw=xle+.25*1.65
    M=inp('Mach',wing['inputs']['Mach']['value'],'same FDR seed as Main Wing; frozen coefficients')
    rho=inp('rho',w['flight.rho_kg_m3'],'same wing seed');V=inp('TAS',w['flight.speed_m_s'],'same wing seed')
    mu=inp('mu',1.84e-5,'same wing seed');alpha=rad(inp('body_alpha_deg',w['flight.alpha_deg'],'same wing seed; sensor/body bias assumed zero'))
    kappa=inp('section_slope_ratio',.95,'ENGINEERING: viscous reduction of 2*pi')
    e=inp('oswald_e',.85,'ENGINEERING prior')
    eff=inp('elevator_effectiveness_ratio',.85,'ENGINEERING correction to thin-airfoil flap effectiveness')
    eta=inp('wake_dynamic_pressure_ratio',.95,'ENGINEERING prior, not DATCOM wake chart evaluation; no propwash')
    limits=inp('elevator_limits_deg',[-25.,20.],'ENGINEERING mechanical proxy: up negative/down positive, awaiting FCS confirmation')
    arm=inp('elevator_increment_arm_chord',.25,'ENGINEERING lift-increment arm behind AC; not hinge moment')
    S=out('S_h',b*(cr+ct)/2,'b*(cr+ct)/2','Geometry: projected trapezoid includes fuselage patch, neutral elevator')
    A=out('AR_h',b*b/S,'b^2/S','DATCOM 2.2.2 / PDF 235-237');lam=ct/cr
    out('MAC_geometric',2*cr/3*(1+lam+lam*lam)/(1+lam),'2*cr/3*(1+lambda+lambda^2)/(1+lambda)','2.2.2 / PDF 235')
    out('y_MAC',b/6*(1+2*lam)/(1+lam),'b/6*(1+2*lambda)/(1+lambda)','trapezoid geometry')
    s25=math.atan((hinge-.25)*(cr-ct)/(b/2));s50=math.atan((hinge-.5)*(cr-ct)/(b/2))
    out('sweep_LE_deg',math.degrees(math.atan(hinge*(cr-ct)/(b/2))),'atan(hinge*(cr-ct)/(b/2))','unswept hinge geometry')
    out('sweep_quarter_deg',math.degrees(s25),'atan((hinge-.25)*(cr-ct)/(b/2))','geometry')
    def slope(ar,sweep,m):
        B=math.sqrt(1-m*m)
        return 2*math.pi*ar/(2+math.sqrt(4+(ar*B/kappa)**2*(1+math.tan(sweep)**2/B**2)))
    a=out('CLalpha_h',slope(A,s50,M),'2*pi*A/[2+sqrt(4+(A*B/kappa)^2*(1+tan(L50)^2/B^2))]','DATCOM Fig 4.1.3.2-49 / PDF 549; kappa assumed')
    f=1-hinge;theta=math.acos(2*f-1);tau=out('tau_e',eff*(1-(theta-math.sin(theta))/math.pi),'0.85*[1-(theta-sin(theta))/pi], theta=acos(2*ce/c-1)','Thin-airfoil engineering proxy; not digitized DATCOM elevator charts')
    clde=out('CLdelta_e',a*tau,'a_h*tau_e','per rad, tail-area normalization')
    cmde=out('Cmdelta_e_AC',-arm*clde,'-increment_arm*CLdelta_e','ENGINEERING proxy about tail AC; CG transfer separate')
    Re=rho*V*mac/mu;cf=.455/(math.log10(Re)**2.58*(1+.144*M*M)**.65)
    cd0=out('CD0_h',cf*(1+1.2*.12+100*.12**4)*2*(1+.25*.12),'Cf*(1+1.2*t/c+100*(t/c)^4)*Swet/S; Swet=2*(1+.25*t/c)*S','DATCOM 4.1.5.1-a / PDF 723 + analytic Cf proxy, fully turbulent, full tail exposed proxy')
    kd=out('induced_drag_factor',1/(math.pi*A*e),'1/(pi*A*e)','DATCOM polar 4.1.5.2 / PDF 757; e assumed')
    dx=xh-xw;dz=zh-zw;iw=rad(w['wing.incidence_deg'])
    l=out('l_H',dx*math.cos(iw)-dz*math.sin(iw),'dx*cos(iw)-dz*sin(iw)','distance parallel wing root chord; representative root-plane proxy')
    h=out('h_H',dx*math.sin(iw)+dz*math.cos(iw),'dx*sin(iw)+dz*cos(iw)','height normal to root chord plane; magnitude used in KH')
    Aw=w['wing.aspect_ratio'];bw=math.sqrt(Aw*w['wing.area_m2']);lw=w['wing.taper']
    ka=out('KA',1/Aw-1/(1+Aw**1.7),'1/A-1/(1+A^1.7)','DATCOM Fig 4.4.1-69a / PDF 1271')
    kl=out('Klambda',(10-3*lw)/7,'(10-3*lambda)/7','DATCOM Fig 4.4.1-69b / PDF 1271')
    kh=out('KH',(1-abs(h/bw))/(2*l/bw)**(1/3),'(1-abs(h/b))/(2*l/b)^(1/3)','DATCOM Fig 4.4.1-70 / PDF 1272')
    low=4.44*(ka*kl*kh*math.sqrt(math.cos(rad(w['wing.quarter_chord_sweep_deg']))))**1.19
    sw50=rad(next(r['value'] for r in wing['results'] if r['parameter']=='sweep_half_outer'))
    ea=out('epsilon_alpha',low*w['aero.CL_alpha_per_rad']/slope(Aw,sw50,0),'4.44*(KA*Klambda*KH*sqrt(cos(L25)))^1.19 * aw(M)/aw(0)','DATCOM 4.4.1-h,i / PDF 1209; equivalent straight-tapered wing; geometry uncertainty')
    CLw=w['aero.CL0']+w['aero.CL_alpha_per_rad']*(alpha+iw)
    eps=out('epsilon_ref_rad',ea*CLw/w['aero.CL_alpha_per_rad'],'epsilon_alpha*CLwing/aw','ENGINEERING zero-lift anchor; Method 2 supplies gradient only')
    dcldf=next(r['value'] for r in wing['results'] if r['parameter']=='flap_CL_delta_at_zero')
    ef=out('epsilon_flap',ea*dcldf/w['aero.CL_alpha_per_rad'],'epsilon_alpha/aw * dCLwing/dflap at zero','ENGINEERING local split-flap coupling only near flap Up; not validated at large flap')
    tail={'Area':S,'TailSpan':b,'TailMAC':mac,'TailAR':A,'IncidenceAngle':rad(ih),'MomentArm':xh-cgx,'ElevatorArea':f*S,'LiftCurveSlope':a,'ZeroLiftAngle':0.,'ZeroLiftDragCoefficient':cd0,'InducedDragFactor':kd,'ElevatorEffectiveness':tau,'PitchMomentCoefficient':0.,'PitchMomentCurveSlope':0.,'ElevatorPitchMomentEffectiveness':cmde}
    pos=[cgx-xh,0.,zcg-zh]
    flow={'referenceBodyAlphaRad':alpha,'referenceDownwashRad':eps,'downwashGradientPerRad':ea,'referenceFlapRad':0.,'flapDownwashGradientPerRad':ef,'dynamicPressureRatio':eta}
    ah=alpha-eps+rad(ih);cl=a*ah;cd=cd0+kd*cl*cl;q=.5*rho*V*V*eta;L=q*S*cl;D=q*S*cd;af=alpha-eps
    Fx=-D*math.cos(af)+L*math.sin(af);Fz=-D*math.sin(af)-L*math.cos(af);My=pos[2]*Fx-pos[0]*Fz
    return dict(inputs=inputs,results=rows,tail=tail,position_body_m=pos,elevator_limits_rad=list(map(rad,limits)),flow=flow,coordinates=dict(wing_MAC_LE_X=xle,wing_quarter_X=xw,CG_X=cgx,CG_Z=zcg,wing_Z=zw,tail_X=xh,tail_Z=zh),snapshot=dict(CL=cl,CD=cd,epsilon_deg=math.degrees(eps),force_body_N=[Fx,0.,Fz],moment_CG_Nm=[0.,My,0.]))

def outputs(d):
    fmt=lambda v:format(v,'.17g')
    src=['// Generated by tools/generate_t6c_tail_seed.py','#include "trainer_aircraft/config/T6CTailSeed.hpp"','','namespace trainer_aircraft {','HorizontalStabilizerConfig makeT6CHorizontalTailSeed() {','    HorizontalStabilizerConfig c;']
    for k,v in d['tail'].items():src.append(f'    c.{k} = {fmt(v)};')
    for i,v in enumerate(d['position_body_m']):src.append(f'    c.PositionWrtCG[{i}] = {fmt(v)};')
    for i,v in enumerate(d['elevator_limits_rad']):src.append(f'    c.ElevatorMechanicalLimit[{i}] = {fmt(v)};')
    src+=['    return c;','}','WingTailFlowConfig makeT6CWingTailFlowSeed() {','    WingTailFlowConfig c;']
    for k,v in d['flow'].items():src.append(f'    c.{k} = {fmt(v)};')
    src+=['    return c;','}','std::unique_ptr<HorizontalStabilizer> makeT6CHorizontalTailWithDownwash() {','    return std::make_unique<HorizontalStabilizer>(makeT6CHorizontalTailSeed(),','        std::make_shared<WingTailFlowField>(makeT6CWingTailFlowSeed()));','}','} // namespace trainer_aircraft','']
    doc=['# Seed đuôi ngang T-6C/PC-9M và downwash','',
    'Seed sơ bộ tại điểm FDR của Main Wing; chưa optimize và không phải output Digital DATCOM. Góc radian trong code; hệ số đuôi chuẩn hóa bằng S_h và MAC_h, không bằng diện tích cánh chính.',
    '## Tọa độ và giả thiết cao độ','',
    'Gốc bản vẽ tổng thể có X dương về đuôi, Y phải, Z lên. Station thân 0 tại X=3 m. Không dùng station thân 0 làm gốc cho số X0=9.7123. Body FRD lấy CG làm gốc: r_B=(X_CG-X, Y-Y_CG, Z_CG-Z). Giả sử FRL song song trục X bản vẽ.',
    'FRP FRONT Z=2.000 m; FRP REAR Z=2.045 m; quarter-MAC tail Z=2.645 m theo người dùng. CG 30% MAC chốt X=4.3675 m nhưng không xác định Z. Giữ Z_CG=2.000 m và wing representative plane Z=2.000 m như proxy của seed trước. Đây KHÔNG phải số đo cao độ MAC thực; chưa đưa phân bố dihedral/twist vào tọa độ AC đại diện. Cần hiệu chỉnh cao độ nếu xác định được từ bản vẽ/loading.',
    'Wing MAC LE X=3.8725 m, quarter-MAC X=4.2850 m; 266 mm là độ lệch mép trước, không phải station MAC. Vị trí body wing=(+0.0825,0,0) m, tail=(-5.3448,0,-0.645) m. MAC span station ±0.8142 m không phải y của hợp lực toàn đuôi: y=0. MAC được dùng như chiều dài chiếu bằng trong chuyển tọa độ proxy; chưa áp dụng offset Z theo incidence.',
    'Không đổi lift/drag seed Wing: với CG vẫn ở 30% MAC, cùng giả thiết cao độ, cánh tay đòn wing không đổi. Dữ liệu datum cũ 0.266/0.761 được thay bằng 3.8725/4.3675 m.',
    '## Input và nguồn','', '| Input | Giá trị | Nguồn / giả thiết |','|---|---|---|']
    for k,v in d['inputs'].items():doc.append(f"| {k} | {v['value']} | {v['source']} |")
    doc+=['','Elevator phủ toàn sải 3.664 m, hinge thẳng 64% chord. Diện tích elevator danh nghĩa =0.36*S_h; bỏ qua phần balance trước hinge, khe, bo tip và che thân. Chord sau hinge gốc/tip =0.468/0.234 m. 3.100 m là sải tấm cân bằng theo người dùng. NACA 0012 toàn sải là giản lược người dùng chọn; bản vẽ còn ghi NACA 0008 ngoài cánh. Không có mô hình hinge moment hoặc actuator.',
    '','## Input → công thức → kết quả → reference','', '| Output | Công thức / input | Giá trị | Reference / phương pháp |','|---|---|---:|---|']
    for r in d['results']:doc.append(f"| {r['parameter']} | `{r['formula']}` | {r['value']:.9g} | {r['reference']} |")
    doc+=['','## Thông số thực sự đưa vào component','', '| Trường | Giá trị |','|---|---|']
    for section in ['tail','flow']:
        for k,v in d[section].items():doc.append(f'| {section}.{k} | {v:.10g} |')
    doc+=['','Cm0_AC=Cm_alpha_AC=0 cho profile đối xứng là giả thiết baseline. Elevator có Cm_delta_AC proxy riêng; không nhập đạo hàm toàn máy bay. Pitch-rate response đến từ omega cross r; không cộng Cmq đuôi lần nữa.',
    'Method 2 dùng các công thức được in trong Fig 4.4.1-69/70, không phải đường fit tự chọn. K_H dùng |h_H/b|; l_H,h_H đã quay về hệ dây cung gốc wing. Tuy nhiên áp dụng cánh tương đương, cao độ wing proxy và sweep rất nhỏ vẫn tạo bất định. Epsilon_ref và epsilon_flap là closure kỹ thuật, eta=0.95 là prior, không tuyên bố đã tính theo biểu đồ wake của DATCOM. Hệ số Mach/Re đóng băng ở điểm seed; chỉ dùng cục bộ trước stall và flap gần Up.',
    '','## Tải kiểm tra elevator=flap=0, p=q=r=0','', '```json',json.dumps(d['snapshot'],indent=2),'```',
    'Đây là riêng tải đuôi; không ép tổng máy bay trim. Âm CL tương ứng lực đuôi xuống. Moment CG gồm M_AC+r cross F, có cả z*Fx. Độ lệch elevator dương tăng lift, thường tạo moment chúi mũi.',
    '','## Sử dụng và tái lập','',
    'Trong cấu hình máy bay: `aircraft.addLoadComponent(makeT6CHorizontalTailWithDownwash());` sau khi include `trainer_aircraft/config/T6CTailSeed.hpp`. Factory này gắn flow vào tail thực sự; không cần tự truyền pointer. Chưa có mass/inertia/vertical-tail seed nên không đánh dấu toàn máy bay sẵn sàng.',
    '','```sh','python tools/generate_t6c_wing_seed.py --check','python tools/generate_t6c_tail_seed.py --check','cmake -S . -B build-t6c','cmake --build build-t6c','ctest --test-dir build-t6c --output-on-failure','```','',
    'Thay input trong generator rồi chạy không có --check. Tail generator đọc dữ liệu tính từ wing generator để không giữ epsilon_ref độc lập với CL wing. Khi cập nhật CG hoặc cao độ, phải cập nhật cả cấu hình hình học proxy và seed rồi kiểm tra tải. Các giới hạn elevator [-25,+20] độ là ước lượng, cần thay khi có tài liệu FCS; không được gắn nhãn giá trị đo.', '']
    return {'src/config/T6CTailSeed.cpp':'\n'.join(src),'reference_data/t6c/tail_datcom_seed.yaml':json.dumps(d,indent=2,ensure_ascii=False)+'\n','docs/T6C_HORIZONTAL_TAIL_DATCOM_SEED.md':'\n'.join(doc)}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');args=p.parse_args();d=build()
    for name,text in outputs(d).items():
        path=R/name
        if args.check:
            if not path.exists() or path.read_text()!=text:raise SystemExit('Stale generated file: '+name)
        else:path.write_text(text)
    print(json.dumps(d['snapshot'],indent=2))
