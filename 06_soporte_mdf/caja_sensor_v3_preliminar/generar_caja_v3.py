"""
Caja del sensor IR v3 (GY-906 / MLX90614) - sensor FIJO, sin tubo roscado.
Medidas exteriores iguales a la v2 (entra en la ventana del porta-sensor de MDF: 15.9 x 20.9 mm).

Orientacion en la maquina: la TAPA va ABAJO (mira a la lamina), el CUERPO arriba.
  - La placa del sensor queda prensada entre los rieles de la tapa (abajo) y 4 postes del cuerpo (arriba).
  - La lata del sensor atraviesa la tapa y sobresale por debajo, para que NADA le tape el campo de vision
    (en la v2 la lata quedaba hundida dentro del cilindro roscado y las paredes le recortaban la vision).
  - Los cables salen por la ranura del techo del cuerpo.

Uso:  python generar_caja_v3.py      -> cuerpo_v3.stl, tapa_v3.stl, vista_corte_v3.png
Cambia las MEDIDAS DEL SENSOR abajo (mide tu placa con calibrador) y vuelve a correrlo.
"""
import numpy as np, trimesh
from manifold3d import Manifold, CrossSection

# ---------------- MEDIDAS EXTERIORES (fijas por el soporte de MDF) ----------------
BOX_X, BOX_Y, BOX_H = 15.625, 20.625, 11.0
# ---------------- MEDIDAS DEL SENSOR (VERIFICAR CON CALIBRADOR) -------------------
PCB_W, PCB_L, PCB_T = 11.0, 17.0, 1.6     # placa GY-906 (ancho x largo x espesor)
CAN_FLANGE_D = 9.4                        # diametro de la pestana de la lata (pegada a la placa)
CAN_H        = 4.5                        # altura de la lata medida desde la placa
CAN_DX, CAN_DY = 0.0, 0.0                 # descentrado de la lata respecto al centro de la placa
SMD_H        = 1.0                        # altura de los componentes del lado de la lata
HEADER_END   = +1                         # +1: pines del lado +Y ; -1: lado -Y
# ---------------- PARAMETROS DE IMPRESION ----------------
CLR   = 0.20      # holgura placa-bolsillo
FIT   = 0.15      # holgura labio de la tapa - cuerpo
LID_T = 1.2       # espesor de la placa de la tapa
SEAT  = SMD_H + 0.3   # altura de los rieles (deja aire a los componentes)
RAIL  = 0.9       # ancho de los rieles donde apoya la placa
TOP_T = 1.6       # espesor del techo del cuerpo
SLOT_W, SLOT_L = 3.4, 11.5   # ranura para cables/conector

def box(x, y, z, cx=0, cy=0, z0=0):
    return Manifold.cube([x, y, z]).translate([cx - x/2, cy - y/2, z0])
def cyl(d, h, cx=0, cy=0, z0=0, d2=None, n=96):
    return Manifold.cylinder(h, d/2, (d2 if d2 else d)/2, n).translate([cx, cy, z0])

cav_x, cav_y = PCB_W + 2*CLR, PCB_L + 2*CLR
wall_x, wall_y = (BOX_X - cav_x)/2, (BOX_Y - cav_y)/2
assert wall_x >= 1.2 and wall_y >= 1.2, "paredes muy delgadas: revisa medidas de la placa"

# ======================= TAPA (z=0 cara que mira a la lamina) =======================
body_h = BOX_H - LID_T                       # el cuerpo va encima de la placa de la tapa
lip_h  = SEAT                                # labio que entra en el cuerpo y forma los rieles
tapa = box(BOX_X, BOX_Y, LID_T)
# labio perimetral dentro del cuerpo (marco), con los rieles largos donde apoya la placa
lip_out = box(cav_x - 2*FIT, cav_y - 2*FIT, lip_h, z0=LID_T)
lip_in  = box(cav_x - 2*FIT - 2*RAIL, cav_y - 2*FIT - 2*RAIL, lip_h + 1, z0=LID_T - 0.5)
tapa = tapa + (lip_out - lip_in)
# agujero de la lata (pasante) con avellanado de 50 grados por debajo para no recortar la vision
cx, cy = CAN_DX, CAN_DY
hole_d = CAN_FLANGE_D + 0.4
tapa = tapa - cyl(hole_d, LID_T + lip_h + 2, cx, cy, -1)
tapa = tapa - cyl(hole_d + 2*LID_T*1.2, 0.01, cx, cy, -0.005, d2=hole_d)  # (marcador)
tapa = tapa - Manifold.cylinder(LID_T + 0.01, (hole_d + 2*LID_T*1.19)/2, hole_d/2, 96).translate([cx, cy, -0.005])
# muesca de orientacion: esquina recortada del lado de los pines
key = box(2.0, 2.0, LID_T + 1, cx=BOX_X/2, cy=HEADER_END*BOX_Y/2, z0=-0.5).rotate([0, 0, 0])
tapa = tapa - key

# ======================= CUERPO (marco + techo) =======================
cuerpo = box(BOX_X, BOX_Y, body_h, z0=LID_T) - box(cav_x, cav_y, body_h - TOP_T, z0=LID_T - 0.01)
# postes que prensan la placa por arriba (4 esquinas de la placa)
z_pcb_top = LID_T + SEAT + PCB_T
post_len = (LID_T + body_h - TOP_T) - z_pcb_top
assert post_len > 0.5, "no cabe: revisa CAN_H / SEAT / alturas"
for sx in (-1, 1):
    for sy in (-1, 1):
        px = sx*(PCB_W/2 - 0.9); py = sy*(PCB_L/2 - 0.9)
        cuerpo = cuerpo + box(1.6, 1.6, post_len, px, py, z_pcb_top)
# ranura de cables en el techo, sobre los pines
slot_cy = HEADER_END*(PCB_L/2 - 2.0)
cuerpo = cuerpo - box(SLOT_L, SLOT_W, TOP_T + 2, 0, slot_cy, LID_T + body_h - TOP_T - 1)
# canal para la brida (zip tie) que prensa la caja contra el fondo de MDF
GROOVE_W, GROOVE_D = 3.2, 0.8
gy = -HEADER_END*(BOX_Y/2 - 5.0)
cuerpo = cuerpo - box(BOX_X + 2, GROOVE_W, GROOVE_D + 0.01, 0, gy, LID_T + body_h - GROOVE_D)
for sx in (-1, 1):
    cuerpo = cuerpo - box(GROOVE_D + 0.01, GROOVE_W, body_h + 2, sx*(BOX_X/2 - GROOVE_D/2), gy, LID_T - 1)
# muesca de orientacion igual que la tapa
cuerpo = cuerpo - box(2.0, 2.0, body_h + 2, cx=BOX_X/2, cy=HEADER_END*BOX_Y/2, z0=LID_T - 1)

def to_tm(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.array(me.vert_properties)[:, :3], np.array(me.tri_verts))
t_tapa, t_cuerpo = to_tm(tapa), to_tm(cuerpo)
t_tapa.export("tapa_v3.stl")
# el cuerpo se imprime con el techo apoyado en la cama
t_c = t_cuerpo.copy(); t_c.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])); t_c.apply_translation([0, 0, -t_c.bounds[0][2]])
t_c.export("cuerpo_v3.stl")

# ---- informe ----
can_below = CAN_H - SEAT - LID_T
print(f"Cavidad {cav_x:.2f} x {cav_y:.2f} mm | paredes {wall_x:.2f} / {wall_y:.2f} mm")
print(f"La lata sobresale {can_below:.2f} mm por debajo de la tapa (el fondo de MDF mide 3 mm)")
print(f"Postes de presion: {post_len:.2f} mm | tapa {t_tapa.is_watertight} cuerpo {t_cuerpo.is_watertight}")

# ---- vista en corte ----
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(11, 5.2))
for a, (axis, hidx, lab, w) in zip(ax, [([1, 0, 0], 1, "Corte a lo LARGO (eje Y)", PCB_L), ([0, 1, 0], 0, "Corte a lo ANCHO (eje X)", PCB_W)]):
    for mesh, col in ((t_tapa, "#d9b36c"), (t_cuerpo, "#9e9e9e")):
        s3 = mesh.section(plane_origin=[cx, cy, 0], plane_normal=axis)
        if s3 is None: continue
        for d in s3.discrete:
            a.fill(d[:, hidx], d[:, 2], color=col, ec="k", lw=0.6)
    a.add_patch(plt.Rectangle((-w/2, LID_T + SEAT), w, PCB_T, color="#2e7d32", alpha=.85))
    a.add_patch(plt.Rectangle((-CAN_FLANGE_D/2 + 0.6, LID_T + SEAT - CAN_H), CAN_FLANGE_D - 1.2, CAN_H, color="#455a64"))
    a.add_patch(plt.Rectangle((-BOX_Y/2-6, -3), BOX_Y/2+6-5.3, 3, color="#c8a27a")); a.add_patch(plt.Rectangle((5.3, -3), BOX_Y/2+1, 3, color="#c8a27a"))
    a.text(0, -5.2, "fondo MDF 3 mm (agujero 10.6)", fontsize=7, ha="center")
    zt = LID_T + SEAT - CAN_H
    for sg in (-1, 1): a.plot([sg*2, sg*(2+9)], [zt, zt-9], "r--", lw=.9)
    a.text(0, BOX_H + 0.6, "cables hacia arriba (ranura)", fontsize=7, ha="center")
    a.set_aspect("equal"); a.set_title(lab, fontsize=10); a.set_ylim(-11, 13); a.set_xlim(-14, 14); a.grid(alpha=.3)
ax[0].text(0, -10.3, "rojo = cono de vision de 90°: no lo tapa nada", ha="center", fontsize=8, color="r")
ax[1].text(0, -10.3, "verde = placa GY-906   gris oscuro = lata del sensor", ha="center", fontsize=8)
fig.tight_layout(); fig.savefig("vista_corte_v3.png", dpi=130)
