#!/usr/bin/env python3
"""Reproducible, provisional wing-only seed. Python standard library only.

DATCOM equations and engineering proxies are explicitly distinguished. This is
not Digital DATCOM output and does not fit or upload the original FDR records.
"""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PI = math.pi
rad = math.radians
deg = math.degrees


def integrate(f, lo, hi, n=2048):
    """Composite midpoint quadrature; split at every geometric discontinuity."""
    h = (hi - lo) / n
    return h * math.fsum(f(lo + (j + 0.5) * h) for j in range(n))


def build():
    rows = []
    inputs = {}

    def inp(key, value, unit, origin):
        inputs[key] = dict(value=value, unit=unit, provenance=origin)
        return value

    def result(key, value, unit, provenance, formula, reference):
        rows.append(dict(parameter=key, value=value, unit=unit, provenance=provenance,
                         formula=formula, reference=reference))
        return value

    S = inp('S_ref', 16.28, 'm2', 'USER: reference wing area')
    b = inp('b', 10.124, 'm', 'DRAWING_PROXY: PC-9M projected span; public T-6C span remains 10.20 m')
    cbar = inp('cbar_ref', 1.650, 'm', 'DRAWING_PROXY: mean aerodynamic chord')
    cr0 = inp('root_chord_raw', 1.880, 'm', 'USER/DRAWING: chord interpreted at R6')
    ct0 = inp('tip_chord_raw', 1.140, 'm', 'USER/DRAWING: chord at R25')
    yi = inp('y_R6', 0.65, 'm', 'PROXY: approximate rib station from drawing; center panel constant chord')
    yd = inp('y_R11', 1.35, 'm', 'PROXY: approximate station; USER: dihedral starts here')
    yf = inp('y_R5', 0.45, 'm', 'PROXY: nominal flap inboard station')
    ya = inp('y_R17', 2.80, 'm', 'PROXY: flap outboard / aileron inboard station')
    half = b / 2
    inc = inp('incidence_root', 0.5, 'deg', 'USER: root incidence')
    twist = inp('twist_tip_minus_root', -2.5, 'deg', 'USER: linear R6-R25; constant inboard')
    gamma = rad(inp('outer_dihedral', 7.0, 'deg', 'USER/DRAWING: zero inboard R11'))
    inp('airfoil', 'NACA 4312', '-', 'USER_ASSUMPTION: all stations; not a verified PIL airfoil polar')
    tc = inp('thickness_ratio', 0.12, '-', 'NACA designation')
    inp('maximum_camber', 0.04, '-', 'NACA designation')
    inp('maximum_camber_x_over_c', 0.30, '-', 'NACA designation')
    inp('maximum_thickness_x_over_c', 0.30, '-', 'PROXY: choose DATCOM thickness factor L=1.2')
    spar = inp('unswept_spar_chord_fraction', 0.35, '-', 'PROXY: straight spar from drawing')
    M = inp('Mach', 0.25383, '-', 'FDR summary: Relative 13962..14027, 66 one-second rows')
    rho = inp('rho', 1.10681, 'kg/m3', 'FDR-derived: Pt-Qc and TAT, recovery factor 1')
    V = inp('TAS', 87.953, 'm/s', 'FDR-derived: Mach + TAT')
    mu = inp('mu', 1.8400e-5, 'Pa s', 'Sutherland at Ts=298.751 K')
    alpha = rad(inp('alpha_body_seed', 1.2292929292929293, 'deg',
                    'FDR AOA-2/3/4 average; ASSUMPTION: equals body alpha; bias=0'))
    inp('alpha_sensor_bias', 0.0, 'deg', 'PROXY: no calibration available; freeze initially')
    inp('beta_seed', 0.0, 'deg', 'ASSUMPTION: symmetric near-level condition')
    inp('rates_seed', [0.0, 0.0, 0.0], 'rad/s', 'ASSUMPTION: p=q=r=0 for static seed evaluation')
    inp('flap_seed', 0.0, 'deg', 'FDR Up mapped to nominal zero')
    inp('aileron_seed', 0.0, 'deg', 'ASSUMPTION: symmetric seed; FDR mean -0.209 deg not imposed')
    hcg = inp('h_CG', 0.30, 'MAC', 'USER_ASSUMPTION: CG; positive aft')
    hac = inp('h_aero', 0.25, 'MAC', 'PROXY: quarter-MAC aerodynamic center')
    inp('z_aero_minus_CG', 0.0, 'm', 'PROXY: same height; avoids inventing a vertical lever arm')
    inp('MAC_LE_drawing_x_aft', 3.8725, 'm', 'DRAWING corrected global X; fuselage station zero is X=3 m; 266 mm is LE offset, not MAC station')
    visc = inp('section_slope_viscous_ratio', 0.95, '-', 'PROXY: 5% reduction from 2*pi; not measured polar')
    e = inp('oswald_e', 0.82, '-', 'PROXY: moderate AR tapered wing; includes viscous drag-due-to-lift')
    rough = inp('surface_roughness_height', 6.35e-6, 'm', 'PROXY: smooth painted surface scenario')
    rls = inp('lifting_surface_drag_factor', 1.05, '-', 'PROXY for R_LS chart, not digitized DATCOM data')
    inp('transition', 'fully_turbulent', '-', 'ASSUMPTION: conservative profile drag seed')
    inp('wing_body_interference_factor', 1.0, '-', 'ASSUMPTION: isolated wing; body loads treated separately')
    inp('slipstream_dynamic_pressure_ratio', 1.0, '-', 'ASSUMPTION: free stream; propwash coupling not identified')
    inp('ground_effect_factor', 1.0, '-', 'ASSUMPTION: airborne seed, no ground effect')
    cf_ratio = inp('split_flap_chord_ratio', 0.25, '-', 'PROXY: constant cf/c; mechanical hinge loads excluded')
    inp('split_flap_panel_boundaries', [yf, (yf + ya) / 2, ya], 'm',
        'PROXY: two equal-span panels each side; four total, contiguous, no gap')
    ca = inp('aileron_absolute_chord', 0.285, 'm', 'USER: quarter of raw tip chord, held fixed after area closure')
    inp('aileron_up_down_limits', [20.0, 11.0], 'deg', 'USER/FCS: physical up/down magnitudes')
    aeff = inp('aileron_effectiveness_ratio', 0.85, '-', 'PROXY: reduction of thin-airfoil plain-flap effectiveness')
    feff = inp('split_flap_effectiveness_ratio', 0.75, '-', 'PROXY: split-flap effectiveness; not a DATCOM chart result')
    fscale = inp('split_flap_nonlinearity_angle', 45.0, 'deg', 'PROXY: smooth saturation scale')
    fangles = inp('split_flap_table_angles', [0.0, 10.0, 20.0, 23.0, 30.0, 40.0, 50.0, 60.0],
                 'deg', 'PROXY: tabulation and max domain 60 deg; only zero-flap seed is current calibration target')
    fdrag = inp('split_flap_pressure_drag_factor', 1.2, '-', 'PROXY: Cd_plate multiplier on flap frontal projection')
    farm = inp('split_flap_increment_arm', 0.25, 'local chord', 'PROXY: lift increment acts 0.25 chord aft of quarter-chord')
    fextra = inp('split_flap_extra_induced_factor', 0.10, '-', 'PROXY: additional nonuniform-loading drag, separate from total-CL polar')
    kmt = inp('Cm_twist_factor', -0.0002, '1/deg', 'PROXY: small-sweep estimate inspired by DATCOM Fig 4.1.4.1-5, not digitized')
    kyaw = inp('aileron_adverse_yaw_factor', -0.15, '-', 'PROXY for DATCOM Fig 6.2.2.1-9, not digitized')
    inp('CL_alpha_dot', 0.0, '-', 'ASSUMPTION: omitted; triangular-wing DATCOM formula does not apply')
    inp('Cm_alpha_dot', 0.0, '-', 'ASSUMPTION: omitted; not identifiable from steady segment')
    inp('alpha_working_range', [-2.0, 8.0], 'deg body', 'PROXY: local pre-stall model, not a stall model')

    def raw_chord(y):
        return cr0 if y <= yi else cr0 + (ct0 - cr0) * (y - yi) / (half - yi)

    area_raw = 2 * (yi * cr0 + (half - yi) * (cr0 + ct0) / 2)
    scale = S / area_raw
    chord = lambda y: scale * raw_chord(y)
    eps = lambda y: 0.0 if y <= yi else twist * (y - yi) / (half - yi)
    G = lambda y: 0.0 if y < yd else gamma

    def integral(f, lo=0.0, hi=half):
        breaks = sorted(set([lo, hi] + [v for v in [yf, yi, yd, ya] if lo < v < hi]))
        return sum(integrate(f, a, z) for a, z in zip(breaks, breaks[1:]))

    A = result('aspect_ratio', b*b/S, '-', 'GEOMETRY', 'b^2/S', '2.2.2-1; PDF 235')
    result('raw_planform_area', area_raw, 'm2', 'GEOMETRY', '2*integral(c_raw dy)', '2.2.2-2; PDF 236')
    result('chord_area_scale', scale, '-', 'ASSUMPTION/CLOSURE', 'S_ref/S_raw', 'Engineering geometry closure')
    result('closed_planform_area', 2*integral(chord), 'm2', 'GEOMETRY', '2*integral(c dy)', '2.2.2-2; PDF 236')
    result('c_root_effective', chord(0), 'm', 'GEOMETRY/CLOSURE', 'scale*1.880', 'Not a measured dimension')
    result('c_tip_effective', chord(half), 'm', 'GEOMETRY/CLOSURE', 'scale*1.140', 'Not a measured dimension')
    cg = result('MAC_geometric', 2*integral(lambda y: chord(y)**2)/S, 'm', 'GEOMETRY',
                '2*integral(c^2 dy)/S', '2.2.2-1; PDF 235; ref MAC retained as 1.650')
    lam = ct0/cr0
    dc = chord(0)-chord(half)
    sweep25 = math.atan((spar-.25)*dc/(half-yi))
    sweep50 = math.atan((spar-.50)*dc/(half-yi))
    sweepLE = math.atan(spar*dc/(half-yi))
    for name, angle in [('sweep_LE_outer', sweepLE), ('sweep_quarter_outer', sweep25), ('sweep_half_outer', sweep50)]:
        result(name, deg(angle), 'deg', 'GEOMETRY_PROXY', 'atan(delta(x_LE+f*c)/delta y)', '2.2.2-3; PDF 237; outer panel representative sweep')
    B = math.sqrt(1-M*M)
    cla0 = 2*PI*visc
    cla = result('cl_alpha_section', cla0/B, '1/rad', 'PROXY+PG', '0.95*2*pi/sqrt(1-M^2)', '4.1.1.2-1; PDF 471; viscous factor assumed')
    a = result('CL_alpha', 2*PI*A/(2+math.sqrt(4+(A*B/visc)**2*(1+(math.tan(sweep50)/B)**2))),
               '1/rad', 'DATCOM+EQUIVALENT_WING', '2*pi*A/[2+sqrt(4+(A*B/kappa)^2*(1+tan(Lambda_half)^2/B^2))]',
               'Fig 4.1.3.2-49; PDF 549; kappa=0.95, representative outer sweep')
    a0section = result('alpha_zero_section', .93*(1.6*4/6-9.12*.8*4/6), 'deg', 'DATCOM',
                       '0.93*(alpha_i-9.12*c_li); alpha_i=1.6*4/6, c_li=0.8*4/6', 'Table 4.1.1-D, Eq 4.1.1.1-a; PDF 467-468')
    mean_twist = result('chord_weighted_twist', 2*integral(lambda y: chord(y)*eps(y))/S,
                        'deg', 'STRIP_PROXY', '2*integral(c*twist dy)/S', 'Replaces a full lifting-line/twist-chart calculation')
    a0wing = result('alpha_zero_wing', a0section-mean_twist, 'deg', 'STRIP_PROXY',
                   'alpha_zero_section - chord_weighted_twist', 'DATCOM 4.1.3.1-c structure; PDF 496-500; weighting is a proxy')
    CL0 = result('CL0_wing', -a*rad(a0wing), '-', 'DERIVED', '-CL_alpha*alpha_zero_wing_rad', 'Linear lift law; incidence not included here')
    cmsec = result('cm0_section', -.124*4/6, '-', 'DATCOM_THEORETICAL', '-0.124*4/6', 'Table 4.1.1-D; 4.1.2.1-1; PDF 467,489')
    cm0 = result('Cm0_at_quarter_MAC', A*math.cos(sweep25)**2/(A+2*math.cos(sweep25))*cmsec+kmt*twist,
                 '-', 'DATCOM+TWIST_PROXY', 'A*cos(Lambda25)^2/(A+2*cos(Lambda25))*cm0_section + K_mtheta*twist',
                 '4.1.4.1-a,c; PDF 654; Fig 4.1.4.1-5, PDF 658-659')
    result('Cm_alpha_at_quarter_MAC', 0.0, '1/rad', 'AC_ASSUMPTION', '(h_ref-h_AC)*CL_alpha = 0', '4.1.4.2-a; PDF 662')
    Re = result('Re_MAC', rho*V*cbar/mu, '-', 'AIR_DATA', 'rho*V*cbar/mu', 'FDR summary + Sutherland; rounded inputs')
    cutoff = result('roughness_Re_cutoff', 38.21*(cbar/rough)**1.053, '-', 'CORRELATION_PROXY',
                    '38.21*(cbar/k)^1.053', 'Engineering substitute for roughness chart 4.1.5.1-27; not its exact equation')
    Cf = result('skin_friction_Cf', .455/(math.log10(min(Re,cutoff))**2.58*(1+.144*M*M)**.65), '-',
                'CORRELATION_PROXY', '0.455/[log10(min(Re,Re_cut))^2.58*(1+0.144*M^2)^0.65]',
                'Analytic approximation replacing Fig 4.1.5.1-26, not a digitized chart')
    Swet = result('S_wet_exposed', 2*(1+.25*tc)*2*integral(chord, yi, half), 'm2', 'GEOMETRY_PROXY',
                  '2*(1+0.25*t/c)*S_exposed', 'Two surfaces; center patch excluded from friction to avoid counting fuselage skin')
    CD0 = result('CD0', Cf*(1+1.2*tc+100*tc**4)*rls*Swet/S, '-', 'DATCOM+DRAG_PROXIES',
                 'Cf*(1+1.2*t/c+100*(t/c)^4)*R_LS*S_wet/S', '4.1.5.1-a; PDF 723')
    CL = CL0+a*(alpha+rad(inc))
    CD = CD0+CL*CL/(PI*A*e)
    result('CL_at_FDR_seed', CL, '-', 'STATIC_SEED', 'CL0+CL_alpha*(alpha_body+incidence)', 'AOA->body interpretation is assumed')
    result('CD_at_FDR_seed', CD, '-', 'DATCOM_POLAR+E_PROXY', 'CD0+CL^2/(pi*A*e)', '4.1.5.2-e; PDF 757')
    qbar = result('dynamic_pressure', .5*rho*V*V, 'Pa', 'AIR_DATA', 'rho*V^2/2', 'Dynamic pressure definition')

    xbar = hac-hcg
    CLq = result('CL_q', (.5+2*xbar)*a, '-', 'DATCOM', '(0.5+2*(h_AC-h_CG))*CL_alpha', '7.1.1.1-a,b; PDF 2470; rotation about CG')
    pitch_factor = .7+.2*max(0,min(1,(A-6)/4))
    result('Cmq_empirical_factor', pitch_factor, '-', 'FAIRING_PROXY', '0.7+0.2*clip((A-6)/4,0,1)', '7.1.1.2-1; PDF 2488 recommends fairing; interpolation chosen here')
    cs = math.cos(sweep25)
    tan2 = math.tan(sweep25)**2
    Bq = math.sqrt(1-M*M*cs*cs)
    cmq_low = -pitch_factor*cla0*cs*(A*(.5+xbar+2*xbar*xbar)/(A+2*cs) + A**3*tan2/(24*(A+6*cs)) + .125)
    cmq = cmq_low*(A**3*tan2/(A*Bq+6*cs)+3/Bq)/(A**3*tan2/(A+6*cs)+3)
    result('Cm_q_about_CG', cmq, '-', 'DATCOM+FAIRING_PROXY',
           '-k*c_lalpha0*cos(L25)*[A*(0.5+x+2*x^2)/(A+2*cos(L25))+A^3*tan(L25)^2/(24*(A+6*cos(L25)))+1/8], then Mach correction',
           '7.1.1.2-a,b; PDF 2488-2489; x=h_AC-h_CG')
    lift_to_CZ = math.cos(alpha)+2*CL/(PI*A*e)*math.sin(alpha)
    result('CL_at_zero_body_alpha', CL0+a*rad(inc), '-', 'REFERENCE_CONVENTION',
           'CL0_wing+CL_alpha*incidence_rad', 'Body alpha=0; wing-axis CL0 excludes incidence')
    result('Cm_alpha_about_CG_at_seed', (hcg-hac)*(a*lift_to_CZ-CL*math.sin(alpha)+CD*math.cos(alpha)),
           '1/rad', 'REFERENCE_TRANSFER',
           '(h_CG-h_AC)*[CL_alpha*cos(alpha)-CL*sin(alpha)+CD_alpha*sin(alpha)+CD*cos(alpha)]',
           'Exact local derivative of r cross F; Cm_alpha at AC=0; wing-only, not aircraft stability')
    cmq_ref = result('Cm_q_at_aero_reference', cmq-(hcg-hac)*lift_to_CZ*CLq, '-', 'REFERENCE_TRANSFER',
                     'Cm_q_CG-(h_CG-h_AC)*(cos(alpha)+dCD/dCL*sin(alpha))*CL_q',
                     'Remove r cross F derivative once; local linearization at alpha_seed')

    J2 = integral(lambda y: chord(y)*y*y)
    Jg = integral(lambda y: chord(y)*y*math.sin(G(y)))
    tau = lambda f: 1-(math.acos(2*f-1)-math.sin(math.acos(2*f-1)))/PI
    lateral = dict(CY_beta=-2*a*integral(lambda y: chord(y)*math.sin(G(y))**2)/S,
                   CY_p=-4*a*Jg/(S*b), CY_r=0.0, CY_da=0.0,
                   Cl_beta=-2*a*Jg/(S*b), Cl_p=-4*a*J2/(S*b*b),
                   Cl_r=8*CL*J2/(S*b*b),
                   Cl_da=2*a*aeff*integral(lambda y: y*chord(y)*tau(ca/chord(y)),ya,half)/(S*b),
                   Cn_beta=0.0, Cn_p=-CL/6, Cn_r=-8*CD*J2/(S*b*b))
    lateral['Cn_da']=kyaw*CL*lateral['Cl_da']
    formulas = dict(CY_beta='-2*a/S*integral(c*sin(Gamma)^2 dy)', CY_p='-4*a/(S*b)*integral(c*y*sin(Gamma) dy)',
                    CY_r='0 (omitted wing-only yaw side force)', CY_da='0 (omitted wing-only control side force)',
                    Cl_beta='-2*a/(S*b)*integral(c*y*sin(Gamma) dy)',Cl_p='-4*a/(S*b^2)*integral(c*y^2 dy)',
                    Cl_r='8*CL/(S*b^2)*integral(c*y^2 dy)',Cl_da='2*a*0.85/(S*b)*integral_aileron(y*c*tau dy)',
                    Cn_beta='0 (low sweep seed, no claimed directional stability)',Cn_p='-CL/6 (low-sweep proxy)',
                    Cn_r='-8*CD/(S*b^2)*integral(c*y^2 dy)',Cn_da='K_yaw*CL*Cl_da')
    refs = dict(CY_beta='5.1.1.1; PDF 1531',Cl_beta='5.1.2.1; PDF 1538',Cn_beta='5.1.3.1; PDF 1576',
                CY_p='7.1.2.1; PDF 2521-2522',Cl_p='7.1.2.2; PDF 2531-2533',Cn_p='7.1.2.3; PDF 2558-2560',
                CY_r='7.1.3.1; PDF 2578 (no general method)',Cl_r='7.1.3.2; PDF 2580-2582',Cn_r='7.1.3.3; PDF 2592-2593',
                CY_da='6.2.3.1; PDF 13 (not provided)',Cl_da='6.2.1.1; PDF 2238-2240',Cn_da='6.2.2.1-a; PDF 2292')
    for key,value in lateral.items():
        result(key,value,'1/rad or per normalized rate','STRIP/ENGINEERING_PROXY',formulas[key],
               refs[key]+'; topical reference only; formula here is a proxy, not evaluated DATCOM graphs')

    Sflap = 2*integral(chord,yf,ya)
    frac = Sflap/S
    cflap = integral(lambda y: chord(y)**2,yf,ya)/integral(chord,yf,ya)
    flap=[]
    for angle in fangles:
        dcl=a*tau(cf_ratio)*feff*rad(angle)/(1+(angle/fscale)**2)*frac
        dp=fdrag*frac*cf_ratio*math.sin(rad(angle))**2
        dm=-farm*cflap/cbar*dcl
        flap.append(dict(angle_deg=angle,delta_CL=dcl,delta_CD_profile=dp,delta_Cm=dm))
    result('flapped_wing_area',Sflap,'m2','GEOMETRY','2*integral_R5^R17(c dy)','Not flap plate area; nominal inboard R5 included')
    result('flap_CL_delta_at_zero',a*tau(cf_ratio)*feff*frac,'1/rad','ENGINEERING_PROXY',
           'a*tau(cf/c)*0.75*S_flapped/S','6.1.4.1-a architecture; PDF 2028-2029; span weighting is proxy')
    result('aileron_effective_limit',15.5,'deg','CONTROL_CONVENTION','(20+11)/2',
           'USER/FCS physical limits; runtime input is half-difference, symmetric equivalent')

    cfg = {'model.derivative_axes':'body','wing.area_m2':S,'wing.aspect_ratio':A,'wing.mean_chord_m':cbar,
           'wing.taper':lam,'wing.sweep_le_deg':deg(sweepLE),'wing.quarter_chord_sweep_deg':deg(sweep25),
           'wing.flapped_area_m2':Sflap,'wing.incidence_deg':inc,
           'flight.rho_kg_m3':rho,'flight.speed_m_s':V,'flight.alpha_deg':deg(alpha),
           'controls.aileron_limit_deg':15.5,'reference.aero_h':hac,'reference.output_h':hcg,
           'reference.aero_z_m':0.0,'reference.output_z_m':0.0,'reference.stability_angle_deg':0.0,
           'aero.cl_alpha_per_rad':cla,'aero.CL_alpha_per_rad':a,'aero.CL0':CL0,'aero.CD0':CD0,
           'aero.oswald_e':e,'aero.Cm0':cm0,'aero.Cm_alpha_per_rad':0.0,'aero.CL_q':CLq,'aero.Cm_q':cmq_ref,
           'flap.eta_in':yf/half,'flap.eta_out':ya/half,'flap.chord_ratio':cf_ratio,'flap.thickness_ratio':tc,
           'flap.section_effectiveness_per_rad':cla*tau(cf_ratio)*feff,
           'flap.effectiveness_ratio_actual':1.0,'flap.effectiveness_ratio_reference':1.0,
           'flap.induced_factor_K':math.sqrt(fextra),'flap.interference_factor':0.0,
           'tabulated_flap.enabled':True,'tabulated_flap.extra_induced_factor':fextra}
    cfg.update({'lateral.'+k:v for k,v in lateral.items()})
    stations=[]
    for name,y in [('center',0.0),('R5',yf),('R6',yi),('R11',yd),('R17',ya),('R25',half)]:
        stations.append(dict(station=name,y_m=y,eta=y/half,raw_chord_m=raw_chord(y),chord_m=chord(y),
                             incidence_deg=inc+eps(y),dihedral_deg=deg(G(y)),
                             x_LE_from_root_aft_m=spar*(chord(0)-chord(y))))
    CX=-CD*math.cos(alpha)+CL*math.sin(alpha)
    CZ=-CL*math.cos(alpha)-CD*math.sin(alpha)
    CmCG=cm0-(hcg-hac)*CZ
    smoke=dict(CL=CL,CD=CD,Cm_aero=cm0,Cm_CG=CmCG,force_body_N=[qbar*S*CX,0,qbar*S*CZ],
               moment_CG_Nm=[0,qbar*S*cbar*CmCG,0],note='Wing-only static snapshot, NOT a whole-aircraft trim or FDR fit.')
    data=dict(schema_version=2,status='provisional_DATCOM_plus_explicit_engineering_proxies_not_optimized',
              component='isolated_main_wing',sources=dict(datcom='AFWAL-TR-83-3048, uploaded 3134-page PDF',
              geometry='PC-9M model building plan, page 5; interpreted and scaled per conversation',
              controls='User-supplied FCS T-6C limits',fdr='Relative 13962..14027 summary only; raw records not committed'),
              inputs=inputs,results=rows,stations=stations,split_flap_table=flap,runtime_config=cfg,smoke_expected=smoke)
    return data


def outputs(data):
    def val(v):
        return json.dumps(v,ensure_ascii=False) if isinstance(v,(str,list,bool)) else f'{v:.10g}'
    doc=['# Main Wing T-6C/PC-9M: bộ seed DATCOM và các giả thiết sơ bộ', '',
         'Bộ số khởi tạo có thể chạy và tái lập; **chưa optimize, chưa xác thực T-6C, không phải output Digital DATCOM**.',
         'Mọi đạo hàm góc trong code dùng radian; p_hat=p*b/(2V), q_hat=q*cbar/(2V), r_hat=r*b/(2V).',
         'DATCOM là AFWAL-TR-83-3048. `PDF` là thứ tự trang trong bản 3134 trang; số mục/trang in đứng trước nó.', '',
         '## Các lựa chọn để khép kín input', '',
         '- Giữ số đọc PC-9M 1,880/1,140 m; tăng đồng đều chord để diện tích hình học đạt 16,28 m². Đây là hình học hiệu dụng cho seed, không phải sửa số đo.',
         '- Cánh trong R6 có chord/incidence không đổi; twist tuyến tính R6–R25. Dihedral đổi từ 0° sang 7° tại R11. Dầm 35% chord được coi không sweep.',
         '- Điểm gãy là chỗ đổi quy luật hình học: R6 là gãy mặt bằng trong mô hình này; R11 là gãy dihedral, không phải một chord mới độc lập. Dihedral trong bảng là góc của panel ngay phía ngoài trạm.',
         '- Sải proxy 10,124 m chỉ áp dụng factory cánh này. Cấu hình public baseline vẫn lưu sải T-6C 10,20 m. MAC tham chiếu 1,650 m khác nhẹ MAC hình học tích phân.',
         '- CL_alpha dùng công thức DATCOM cho cánh tương đương với AR toàn cánh và sweep đoạn ngoài. Hiệu chỉnh twist dùng trọng số chord; chưa giải bài toán lifting-line đầy đủ.',
         '- NACA 4312, giảm slope tiết diện 5%, e=0,82, roughness, airfoil polar và hiệu quả mặt lái là giả thiết; việc optimize sau này không chứng minh chúng là giá trị đo.',
         '- CG=0,30 MAC; AC tạm ở 0,25 MAC; cùng cao độ. Hệ số mô men trong config ở AC, runtime chuyển về CG đúng một lần.',
         '- AOA-2/3/4 trung bình 1,229293° được tạm đồng nhất alpha_body. Cần xem bias/trục cảm biến là nguồn bất định; không thay âm thầm bằng AOA-1.',
         '- Hệ số ngang hướng dùng strip theory/proxy cánh riêng, đóng băng tại điểm FDR. Không dùng đạo hàm toàn máy bay khác để thay cho wing-only.',
         '- Chỉ snapshot flap Up là mục tiêu hiệu chỉnh hiện tại. Bảng split flap 0–60° là ngoại suy kỹ thuật chưa xác thực; không dùng để tuyên bố đạt bài flap dynamics.', '',
         '## Toàn bộ input', '',
         '| Input | Giá trị | Đơn vị | Nguồn / giả thiết |','|---|---:|---|---|']
    for k,d in data['inputs'].items():doc.append(f"| `{k}` | {val(d['value'])} | {d['unit']} | {d['provenance']} |")
    doc += ['', '## Mặt bằng hiệu dụng', '', '| Trạm | y (m) | eta | chord đọc (m) | chord hiệu dụng (m) | incidence (deg) | dihedral (deg) | x_LE từ gốc, dương sau (m) |', '|---|---:|---:|---:|---:|---:|---:|---:|']
    for d in data['stations']:
        doc.append('| '+d['station']+' | '+' | '.join(f"{d[k]:.6f}" for k in ['y_m','eta','raw_chord_m','chord_m','incidence_deg','dihedral_deg','x_LE_from_root_aft_m'])+' |')
    doc += ['', '## Bảng input – công thức – output – reference', '',
            'Tên biến trong công thức chỉ về bảng input phía trên. `PROXY` có reference DATCOM theo chủ đề để đối chiếu, không có nghĩa con số đã được tra từ đồ thị đó.', '',
            '| Input / loại phương pháp | Công thức sử dụng | Output và giá trị | Reference DATCOM / phạm vi |', '|---|---|---|---|']
    for d in data['results']:
        doc.append(f"| {d['provenance']} | `{d['formula']}` | **{d['parameter']} = {d['value']:.9g} {d['unit']}** | {d['reference']} |")
    doc += ['', '## Split flap: bảng increment tại điểm khí động', '',
            'cf/c=0,25. tau=1-(theta_h-sin(theta_h))/pi, theta_h=acos(2*cf/c-1).',
            'delta_CL = CL_alpha*tau*0.75*delta_rad/[1+(delta_deg/45)^2]*S_flapped/S.',
            'delta_CD_profile = 1.2*(S_flapped/S)*(cf/c)*sin(delta)^2.',
            'delta_Cm = -0.25*(chord_flapped_weighted/cbar_ref)*delta_CL.',
            'Đây là closure kỹ thuật, **không phải đồ thị split-flap DATCOM đã số hóa**. §6.1.4.1 (PDF 2028–2029), §6.1.5.1 (2075–2078), §6.1.7 (2208) chỉ ra phương pháp chi tiết để thay thế.',
            'Code dùng CD=CD0+CL_total²/(pi*A*e)+delta_CD_profile+0.10*delta_CL². Phần 0.10 chỉ là giả thiết tổn thất tải không đều, không cộng lại delta(CL²)/(pi*A*e).', '',
            '| Flap (deg) | delta_CL | delta_CD_profile | delta_Cm tại aero_h |', '|---:|---:|---:|---:|']
    for d in data['split_flap_table']:doc.append('| '+' | '.join(f'{v:.9g}' for v in d.values())+' |')
    doc += ['', '## Giá trị thực sự đưa vào C++', '', '| Trường config | Giá trị |', '|---|---:|']
    for k,v in data['runtime_config'].items():doc.append(f'| `{k}` | {val(v)} |')
    doc += ['', 'Các trường flap kiểu Roskam được điền để giữ tương thích cấu trúc, nhưng không được đánh giá khi `tabulated_flap.enabled=true`; chỉ bảng increment ở trên có tác dụng.',
            'Aileron runtime là delta_a=(delta_L-delta_R)/2, dương tạo roll phải. Miền ±15,5° suy từ 11° down và 20° up. Lệnh/potentiometer FDR phải đổi về quy ước này trước khi replay. Mô hình cục bộ chưa biểu diễn lực nâng đối xứng do differential aileron ở biên hành trình.', '',
            '## Kết nối lực – mô men – RigidBody6DOF', '',
            'Factory `makeT6CWingCalibrationSeed()` trả cấu hình; truyền vào `MainWing` rồi `TrainerAircraftModel::addLoadComponent`. Không có factory toàn máy bay T-6C hoàn chỉnh trong nhánh hiện hành.',
            'Mỗi bước: alpha_w=alpha_body+incidence; CL=CL0+CL_alpha*alpha_w+CL_q*q_hat+delta_CL; Cm_ref=Cm0+Cm_alpha*alpha_w+Cm_q_ref*q_hat+delta_Cm.',
            'Các đạo hàm động quay quanh CG. Cm_q_ref được trừ phần chuyển mô men của CL_q tại alpha_seed, sau đó runtime cộng r×F đúng một lần. Hệ số đóng băng chỉ đúng cục bộ quanh alpha_seed.',
            'CX=-CD*cos(alpha_body)+CL*sin(alpha_body); CZ=-CL*cos(alpha_body)-CD*sin(alpha_body). FY=qbar*S*CY. Biến đổi dọc này phù hợp beta nhỏ; chưa thay bằng biến đổi wind-to-body đầy đủ.',
            'F_body=qbar*S*[CX,CY,CZ]. M_ref=qbar*S*[b*Cl,cbar*Cm,b*Cn]. M_CG=M_ref+(r_ref-r_CG)×F_body.',
            'LoadAccumulator cộng wing với thân, đuôi, cánh quạt và càng. RigidBody6DOF dùng tổng lực, mô men, mass và inertia để tính đạo hàm Newton–Euler; RK4 tích phân trạng thái.',
            'Với trục body x trước, y phải, z xuống: v_dot_body=F_total_body/m + g_body - omega×v_body; omega_dot=I^(-1)*(M_total_CG - omega×(I*omega)). Trọng lực chỉ cộng một lần ở cấp rigid body; cánh chỉ trả tải khí động.',
            'Wing-only không cần tự tạo mass/inertia. Các input còn thiếu của đuôi/cánh quạt và khối lượng chuyến bay nằm ngoài cập nhật này; `T6CConfig::isSimulationReady()` không bị chuyển thành true.', '',
            '## Snapshot kiểm tra, không phải nghiệm trim', '', '```json',json.dumps(data['smoke_expected'],indent=2,ensure_ascii=False),'```', '',
            'Tại alpha này, lực nâng cánh có thể chưa bằng trọng lượng và mô men cánh không bằng 0. Tổng cân bằng cần cả thân/đuôi/lực đẩy và khối lượng chuyến bay.', '',
            '## Tái lập và sử dụng', '', '```sh','python tools/generate_t6c_wing_seed.py --check',
            'cmake -S . -B build-t6c -DTRAINER_AIRCRAFT_BUILD_TESTS=ON',
            'cmake --build build-t6c --config Release','ctest --test-dir build-t6c -C Release --output-on-failure',
            '# Windows multi-config: build-t6c/Release/trainer_aircraft_t6c_wing_seed_demo.exe',
            '# Linux single-config: build-t6c/trainer_aircraft_t6c_wing_seed_demo','```','',
            'Để thay input, sửa generator, chạy lại không có `--check`; YAML, C++ và tài liệu được sinh cùng nguồn. YAML dùng cú pháp JSON hợp lệ theo YAML 1.2, C++ được sinh lúc phát triển, không parse YAML khi chạy.', '',
            '## Optimize cân bằng dọc', '',
            'Vòng đầu chỉ mở delta_CL0, delta_CD0, delta_Cm0 với bounds/prior; giữ hình học, slope, e và các đạo hàm động. Kiểm tra lực đẩy, mass, CG, bias AOA và mô hình đuôi trước khi diễn giải nghiệm.',
            'Một điểm gần ổn định không tách duy nhất CL0/CL_alpha, CD0/e hay lực đẩy/cản. Dùng đoạn khác để kiểm tra; không dùng đường elevator replay làm bằng chứng đạt longitudinal trim.',
            'Bounds khởi tạo tham khảo: delta_CL0 ±0,15; delta_CD0 ±0,005 (CD0 cuối phải dương); delta_Cm0 ±0,04. Đây là engineering priors, không phải dung sai CS-FSTD hoặc bảo đảm nghiệm đúng.', '']

    cpp=['// Generated by tools/generate_t6c_wing_seed.py; edit the generator.',
         '#include "trainer_aircraft/config/T6CWingSeed.hpp"','', 'namespace trainer_aircraft','{',
         'MainWingConfig makeT6CWingCalibrationSeed()', '{','    MainWingConfig c;']
    for k,v in data['runtime_config'].items():
        cpp.append(f'    c.{k} = {json.dumps(v) if isinstance(v,(str,bool)) else format(v,".17g")};')
    for field,key in [('delta_CL','delta_CL'),('delta_CD_profile','delta_CD_profile'),('delta_Cm_at_aero_reference','delta_Cm')]:
        xs=', '.join(format(d['angle_deg'],'.17g') for d in data['split_flap_table'])
        ys=', '.join(format(d[key],'.17g') for d in data['split_flap_table'])
        cpp.append(f'    c.tabulated_flap.{field} = {{{{{xs}}}, {{{ys}}}, "T6C split flap seed {key}"}};')
    cpp += ['    return c;','}','} // namespace trainer_aircraft','']
    return {'reference_data/t6c/wing_datcom_seed.yaml':json.dumps(data,indent=2,ensure_ascii=False)+'\n',
            'docs/T6C_MAIN_WING_DATCOM_SEED.md':'\n'.join(doc),
            'src/config/T6CWingSeed.cpp':'\n'.join(cpp)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    data=build()
    for name,content in outputs(data).items():
        path=ROOT/name
        if args.check:
            if not path.exists() or path.read_text(encoding='utf-8')!=content:
                raise SystemExit('Generated file is out of date: '+name)
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(content,encoding='utf-8')
    print(json.dumps(data['smoke_expected'],indent=2))


if __name__=='__main__':
    main()
