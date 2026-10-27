"""Physics estimate of a frameless motor's winding-to-housing thermal resistance R_wh, and what
potting does to it. Used to check the provisional 35/65 split of the datasheet TPR in
params/actuators.yaml without a bench test.

Three pieces:
1. Winding-bundle conductivity across the wires: enamelled round copper wires in a matrix (air,
   impregnating varnish, or potting epoxy). Coated-cylinder formula for wire + enamel, then the
   Maxwell-Garnett (Rayleigh) formula for cylinders in a matrix. Validated against a fine-grid
   finite-difference solution of a wire array (keff_fd).
2. A 2D finite-difference conduction model of one slot pitch of the stator cross-section
   (homogenized winding bundle with uniform heat generation, slot liner, teeth, back iron), with
   the stator outer surface held at the housing temperature. Gives the slot-copper-to-stator-OD
   resistance per unit stack length. Validated against the exact solution for a heated annulus.
3. A three-node network adding the end turns (copper outside the stack): end-turn copper conducts
   axially into the slot copper and directly to the housing (through internal air, or through
   epoxy if potted); the stator-to-housing interface adds a contact resistance.

R_wh is defined as in the two-node joint model: volume-average winding temperature (what the
resistance rise measures) minus housing temperature, divided by copper loss.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

K_CU, K_STEEL = 398.0, 25.0          # W/m K: copper; silicon-steel laminations, in-plane
MATRIX = {"air": 0.026, "varnish": 0.20, "epoxy": 0.70}   # impregnation; epoxy = filled potting grade


# ---------------------------------------------------------------- 1. bundle conductivity
def coated_wire_k(k_cu=K_CU, k_enamel=0.20, enamel_ratio=0.05):
    """Effective conductivity of a copper cylinder with an enamel coat; enamel_ratio = t / r_outer."""
    a2 = (1.0 - enamel_ratio) ** 2
    return k_enamel * ((k_cu + k_enamel) + (k_cu - k_enamel) * a2) / ((k_cu + k_enamel) - (k_cu - k_enamel) * a2)


def bundle_k(phi, k_matrix, k_enamel=0.20, enamel_ratio=0.05):
    """Transverse conductivity of a bundle with wire (incl. enamel) area fraction phi."""
    kw = coated_wire_k(k_enamel=k_enamel, enamel_ratio=enamel_ratio)
    km = k_matrix
    return km * ((1 + phi) * kw + (1 - phi) * km) / ((1 - phi) * kw + (1 + phi) * km)


def keff_fd(phi, k_matrix, k_enamel=0.20, enamel_ratio=0.05, n=240, packing="square"):
    """Fine-grid FD of one periodic cell of round enamelled wires: impose a temperature difference
    across the cell (x direction), adiabatic in y, and compute the effective conductivity."""
    if packing == "square":
        Lx = Ly = 1.0
        centers = [(0.5, 0.5)]
    else:  # hexagonal cell: 1 x sqrt(3), wires at centre and corners
        Lx, Ly = 1.0, np.sqrt(3.0)
        centers = [(0.5, Ly / 2), (0.0, 0.0), (1.0, 0.0), (0.0, Ly), (1.0, Ly)]
    area_per_wire = Lx * Ly / (1 if packing == "square" else 2)
    r_o = np.sqrt(phi * area_per_wire / np.pi)
    r_c = r_o * (1 - enamel_ratio)
    nx, ny = n, int(round(n * Ly / Lx))
    h = Lx / nx
    xs = (np.arange(nx) + 0.5) * h
    ys = (np.arange(ny) + 0.5) * (Ly / ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    k = np.full((nx, ny), k_matrix)
    for cx, cy in centers:
        d = np.hypot(X - cx, Y - cy)
        k[d <= r_o] = k_enamel
        k[d <= r_c] = K_CU
    return _flux_keff(k, h, Ly / ny, Lx, Ly)


def _flux_keff(k, hx, hy, Lx, Ly):
    nx, ny = k.shape
    idx = np.arange(nx * ny).reshape(nx, ny)
    rows, cols, vals = [], [], []
    b = np.zeros(nx * ny)
    diag = np.zeros(nx * ny)
    def add(i, j, g):
        rows.extend([i, j]); cols.extend([j, i]); vals.extend([-g, -g])
        diag[i] += g; diag[j] += g
    gx = 2 * k[:-1] * k[1:] / (k[:-1] + k[1:]) * hy / hx
    for a, bb, g in zip(idx[:-1].ravel(), idx[1:].ravel(), gx.ravel()):
        add(a, bb, g)
    gy = 2 * k[:, :-1] * k[:, 1:] / (k[:, :-1] + k[:, 1:]) * hx / hy
    for a, bb, g in zip(idx[:, :-1].ravel(), idx[:, 1:].ravel(), gy.ravel()):
        add(a, bb, g)
    # Dirichlet faces: T=1 at x=0, T=0 at x=Lx (half-cell conductance)
    g0 = 2 * k[0] * hy / hx
    g1 = 2 * k[-1] * hy / hx
    diag[idx[0]] += g0
    b[idx[0]] += g0 * 1.0
    diag[idx[-1]] += g1
    A = sp.csr_matrix((vals, (rows, cols)), shape=(nx * ny, nx * ny)) + sp.diags(diag)
    T = spla.spsolve(A, b).reshape(nx, ny)
    q = float((g0 * (1.0 - T[0])).sum())        # W per unit depth through the x=0 face
    return q * Lx / (Ly * 1.0)                    # k = q L / (H dT)


# ---------------------------------------------------------------- 2. slot-pitch FD
@dataclass
class Geometry:
    D_out: float = 59.06e-3     # m, stator OD [datasheet]
    L_stack: float = 12.70e-3   # m [datasheet]
    D_in: float = 39.5e-3       # m, stator bore [ASSUMPTION: 28.7 mm thru-bore + ~4 mm rotor + 0.5 mm gap]
    n_slots: int = 18           # [ASSUMPTION: 12-pole/18-slot concentrated winding, common for frameless]
    t_yoke: float = 3.0e-3      # m [ASSUMPTION]
    w_tooth: float = 3.0e-3     # m, parallel-sided tooth [ASSUMPTION]
    t_liner: float = 0.15e-3    # m slot liner [ASSUMPTION]
    k_liner: float = 0.14       # W/m K, aramid paper
    phi_slot: float = 0.40      # copper (incl. enamel) fraction of slot area [ASSUMPTION]
    end_over: float = None      # m end-turn axial overhang per side (None: from coil geometry)
    r_contact: float = 2e-4     # m2 K/W, stator OD to housing [ASSUMPTION, bonded/press fit]
    h_end_air: float = 15.0     # W/m2 K, end-turn surface to housing through still internal air
    t_pot_end: float = 3.0e-3   # m, epoxy between end turns and housing when potted
    k_enamel: float = 0.20
    enamel_ratio: float = 0.08   # t_enamel / r_wire, grade-2 enamel on ~0.6-0.8 mm wire [ASSUMPTION]

    def with_(self, **kw):
        return replace(self, **kw)


def sector_fd(g: Geometry, k_bundle: float, h: float = 0.05e-3):
    """Steady 2D conduction in one slot pitch. Returns R' (K m / W): average slot-copper temperature
    rise above the stator OD per W of copper loss per metre of stack, for ONE slot pitch; plus the
    material map for plotting."""
    Ro, Ri = g.D_out / 2, g.D_in / 2
    th = 2 * np.pi / g.n_slots
    half_w = Ro * np.sin(th / 2) + h
    xs = np.arange(-half_w, half_w, h) + h / 2
    ys = np.arange(Ri * np.cos(th / 2) - h, Ro + h, h) + h / 2
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    r = np.hypot(X, Y)
    ang = np.arctan2(X, Y)
    active = (r >= Ri) & (r < Ro) & (np.abs(ang) <= th / 2)
    # distance to the nearest tooth centreline (teeth sit on the sector edges)
    d_edge = r * np.sin(th / 2 - np.abs(ang))
    steel = active & ((r >= Ro - g.t_yoke) | (d_edge <= g.w_tooth / 2))
    slot = active & ~steel
    # liner: slot cells within t_liner of steel
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~steel, sampling=h)
    liner = slot & (dist <= g.t_liner)
    bundle = slot & ~liner
    # copper fraction in the bundle region so that copper/slot area = phi_slot
    phi_b = min(0.9, g.phi_slot * slot.sum() / max(bundle.sum(), 1))
    k = np.zeros_like(X)
    k[steel] = K_STEEL
    k[liner] = g.k_liner
    k[bundle] = k_bundle(phi_b) if callable(k_bundle) else k_bundle
    q = np.where(bundle, 1.0, 0.0)
    q *= 1.0 / (q.sum() * h * h)                      # 1 W per metre of stack in this sector
    T = _solve_dirichlet_outside(k, active, q, h, r >= Ro, X, Y, Ro)
    Tcu = float(T[bundle].mean())
    return Tcu, dict(steel=steel, liner=liner, bundle=bundle, T=T, X=X, Y=Y, phi_bundle=phi_b,
                     slot_area=float(slot.sum() * h * h))


def _solve_dirichlet_outside(k, active, q, h, outside, X, Y, Ro):
    nx, ny = k.shape
    ids = -np.ones((nx, ny), int)
    ids[active] = np.arange(active.sum())
    N = int(active.sum())
    rows, cols, vals = [], [], []
    diag = np.zeros(N)
    b = q[active] * h * h
    for di, dj in ((1, 0), (0, 1)):
        a = active[:nx - di, :ny - dj]
        bb = active[di:, dj:]
        ka, kb = k[:nx - di, :ny - dj], k[di:, dj:]
        both = a & bb
        g = np.where(both, 2 * ka * kb / np.maximum(ka + kb, 1e-30), 0.0)
        ia, ib = ids[:nx - di, :ny - dj][both], ids[di:, dj:][both]
        gg = g[both]
        rows += [ia, ib]; cols += [ib, ia]; vals += [-gg, -gg]
        np.add.at(diag, ia, gg); np.add.at(diag, ib, gg)
        # active cell next to an outside (r >= Ro) cell: Dirichlet T = 0 at the OD surface
        for src, dst, ksrc, idsrc in ((a, outside[di:, dj:], ka, ids[:nx - di, :ny - dj]),
                                      (bb, outside[:nx - di, :ny - dj], kb, ids[di:, dj:])):
            m = src & dst
            np.add.at(diag, idsrc[m], 2 * ksrc[m])
    A = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(N, N))
    A = A + sp.diags(diag)
    T = np.full((nx, ny), np.nan)
    T[active] = spla.spsolve(A.tocsc(), b)
    return T


def annulus_check(h=0.05e-3, k=25.0, Ri=0.015, Ro=0.025, n_sector=8):
    """FD of a uniformly heated annulus sector (adiabatic bore, Dirichlet OD) vs the exact mean
    temperature rise. Returns (fd, exact) in K per (W/m) of total annulus loss."""
    th = 2 * np.pi / n_sector
    half_w = Ro * np.sin(th / 2) + h
    xs = np.arange(-half_w, half_w, h) + h / 2
    ys = np.arange(Ri * np.cos(th / 2) - h, Ro + h, h) + h / 2
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    r = np.hypot(X, Y)
    ang = np.arctan2(X, Y)
    active = (r >= Ri) & (r < Ro) & (np.abs(ang) <= th / 2)
    kk = np.where(active, k, 0.0)
    q = np.where(active, 1.0, 0.0)
    q *= (1.0 / n_sector) / (q.sum() * h * h)          # sector carries 1/n of 1 W/m
    T = _solve_dirichlet_outside(kk, active, q, h, r >= Ro, X, Y, Ro)
    fd = float(np.mean(T[active]))              # Cartesian cells are equal-area
    # exact: q''' = 1 / (pi (Ro^2 - Ri^2)); T(r) = q/(4k)(Ro^2 - r^2) + q Ri^2/(2k) ln(r/Ro)
    qv = 1.0 / (np.pi * (Ro**2 - Ri**2))
    rr = np.linspace(Ri, Ro, 20001)
    Tex = qv / (4 * k) * (Ro**2 - rr**2) + qv * Ri**2 / (2 * k) * np.log(rr / Ro)
    exact = float(np.trapezoid(Tex * rr, rr) / np.trapezoid(rr, rr))
    return fd, exact


# ---------------------------------------------------------------- 3. end turns + network
def r_wh(g: Geometry, matrix: str, h: float = 0.05e-3, return_parts: bool = False):
    """Winding-to-housing resistance (K/W) for impregnation `matrix` ('air'|'varnish'|'epoxy').
    Potting ('epoxy') also fills the end-turn space: end turns then conduct to the housing through
    t_pot_end of epoxy instead of convecting to internal air."""
    km = MATRIX[matrix]
    kb = lambda phi: bundle_k(phi, km, g.k_enamel, g.enamel_ratio)
    Tsec, info = sector_fd(g, kb, h)
    R2d = Tsec / (g.n_slots * g.L_stack)                 # slot copper -> stator OD, whole motor (K/W)
    # coil geometry (concentrated winding around each tooth): two coil sides share each slot
    A_slot = info["slot_area"]
    A_cu_coil_side = 0.5 * g.phi_slot * A_slot           # copper area of one coil side
    w_coil = 0.5 * (2 * np.pi * (g.D_in / 2 + (g.D_out / 2 - g.t_yoke - g.D_in / 2) / 2)
                    / g.n_slots - g.w_tooth)             # coil-side width at mid-slot
    depth = g.D_out / 2 - g.t_yoke - g.D_in / 2
    end_len = np.pi / 2 * (g.w_tooth + w_coil)           # end-turn path per side (semicircle around tooth)
    over = g.end_over if g.end_over is not None else w_coil + 1e-3
    # volumes (whole motor): n_slots coils, each with 2 coil sides (active) and 2 end turns
    n_coils = g.n_slots
    V_act = n_coils * 2 * A_cu_coil_side * g.L_stack
    V_end = n_coils * 2 * A_cu_coil_side * end_len
    f_end = V_end / (V_act + V_end)
    # axial conduction end-turn centroid -> slot copper: along the wires (k = phi_b * k_cu)
    k_ax = info["phi_bundle"] * K_CU
    A_ax = n_coils * 2 * A_cu_coil_side / info["phi_bundle"]   # bundle cross-section, all coil sides
    L_ax = end_len / 2 + g.L_stack / 4
    G_se = k_ax * A_ax / L_ax
    # end turns -> housing (both ends): exposed surface of the end-turn bundles
    A_end_surf = n_coils * 2 * (end_len * (depth + w_coil))     # top + side faces, rough
    if matrix == "epoxy":
        G_eh = 1.0 / (g.t_pot_end / (MATRIX["epoxy"] * A_end_surf) + 1.0 / (2 * kb(info["phi_bundle"])
                      * A_end_surf / max(w_coil, 1e-4)))
    else:
        G_eh = g.h_end_air * A_end_surf
    G_sh = 1.0 / R2d
    R_c = g.r_contact / (np.pi * g.D_out * g.L_stack)    # stator OD -> housing
    # network: nodes S (slot copper), E (end copper), O (stator OD), H (housing = 0)
    # loss split by copper volume; solve for T_S, T_E, T_O
    P = 1.0
    Ps, Pe = P * (1 - f_end), P * f_end
    G_oh = 1.0 / R_c
    A = np.array([[G_sh + G_se, -G_se, -G_sh],
                  [-G_se, G_se + G_eh, 0.0],
                  [-G_sh, 0.0, G_sh + G_oh]])
    Ts, Te, To = np.linalg.solve(A, [Ps, Pe, 0.0])
    Tw = (1 - f_end) * Ts + f_end * Te
    R = Tw / P
    if return_parts:
        return R, dict(R_2d=R2d, R_contact=R_c, f_end=f_end, end_len=end_len, G_se=G_se, G_eh=G_eh,
                       T_slot=Ts, T_end=Te, T_od=To, k_bundle=kb(info["phi_bundle"]),
                       phi_bundle=info["phi_bundle"], copper_g=(V_act + V_end) * 8960 * 1e3,
                       slot_area_mm2=A_slot * 1e6, info=info)
    return R
