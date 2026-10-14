"""STL (üçgen ağ) -> STEP (düzgün dört kenarlı bikübik yüzeyler).

Yöntem:
  1. Her üçgen; köşe, kenar ortası ve ağırlık merkezi kullanılarak 3 dörtgene bölünür.
     Yeni noktalar orijinal üçgen düzleminde kaldığı için şekil kaydırılmaz.
  2. Her dörtgen kenarı, uç noktalarındaki normallerden kübik Bézier eğrisi olur
     (PN-üçgen yöntemindeki kenar kuralı). Komşu iki yüz aynı kenar eğrisini paylaştığı
     için sonuç su geçirmezdir.
  3. Dihedral açısı eşikten büyük kenarlar "keskin kenar" sayılır: bu kenarlar düz
     çizgi olur ve normaller iki tarafta ayrı hesaplanır.
  4. Her dörtgenin iç kontrol noktaları ayrık Coons formülüyle bulunur (bikübik Bézier).
  5. Yüzler OpenCASCADE ile dikilir, katıya çevrilir ve STEP olarak yazılır.

Kullanım:
  python stl2step.py girdi.stl cikti.step [--hedef-ucgen N] [--keskin-aci 30]
"""

import argparse
import sys
import time

import numpy as np
import trimesh
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.Geom import Geom_BezierSurface
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.ShapeFix import ShapeFix_Solid
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
try:
    from OCP.TColgp import TColgp_Array2OfPnt
except ImportError:  # OCCT 8
    from OCP.OCP.collections import Array2_gp_Pnt as TColgp_Array2OfPnt
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

_shell = getattr(TopoDS, "Shell_s", None) or TopoDS.Shell
_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face


def ucgenleri_dortgene_bol(V, F):
    """Her üçgeni 3 dörtgene böler. Yeni noktalar üçgen düzleminde kalır."""
    nv = len(V)
    kenarlar = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    benzersiz, ters = np.unique(kenarlar, axis=0, return_inverse=True)
    ters = ters.reshape(3, -1).T  # (nf, 3): ab, bc, ca kenar indeksleri
    kenar_ortasi = V[benzersiz].mean(axis=1)
    merkez = V[F].mean(axis=1)
    ne = len(benzersiz)
    P = np.vstack([V, kenar_ortasi, merkez])
    e = ters + nv
    f = np.arange(len(F)) + nv + ne
    a, b, c = F[:, 0], F[:, 1], F[:, 2]
    eab, ebc, eca = e[:, 0], e[:, 1], e[:, 2]
    Q = np.concatenate([
        np.stack([a, eab, f, eca], 1),
        np.stack([b, ebc, f, eab], 1),
        np.stack([c, eca, f, ebc], 1),
    ])
    return P, Q


def dortgen_normali(P, Q):
    n = np.cross(P[Q[:, 2]] - P[Q[:, 0]], P[Q[:, 3]] - P[Q[:, 1]])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-300)


def kenar_haritasi(Q):
    """Yönsüz kenar -> [(yüz, yerel kenar indeksi), ...]"""
    harita = {}
    for qi, q in enumerate(Q):
        for k in range(4):
            a, b = int(q[k]), int(q[(k + 1) % 4])
            harita.setdefault((min(a, b), max(a, b)), []).append((qi, k))
    return harita


def kose_normalleri(P, Q, QN, keskin, harita):
    """Her (yüz, köşe) için normal. Keskin kenarı geçmeden ulaşılan yüzlerin ortalaması."""
    olay = {}
    for qi, q in enumerate(Q):
        for k in range(4):
            olay.setdefault(int(q[k]), []).append((qi, k))
    N = np.zeros((len(Q), 4, 3))
    for v, liste in olay.items():
        yuzler = [qi for qi, _ in liste]
        ebeveyn = {qi: qi for qi in yuzler}

        def bul(x):
            while ebeveyn[x] != x:
                ebeveyn[x] = ebeveyn[ebeveyn[x]]
                x = ebeveyn[x]
            return x

        for qi, k in liste:
            q = Q[qi]
            for komsu in (int(q[(k + 1) % 4]), int(q[(k - 1) % 4])):
                anahtar = (min(v, komsu), max(v, komsu))
                if anahtar in keskin:
                    continue
                for qj, _ in harita[anahtar]:
                    if qj in ebeveyn:
                        ebeveyn[bul(qi)] = bul(qj)
        gruplar = {}
        for qi in yuzler:
            gruplar.setdefault(bul(qi), []).append(qi)
        for grup in gruplar.values():
            n = QN[grup].sum(axis=0)
            n /= max(np.linalg.norm(n), 1e-300)
            for qi, k in liste:
                if qi in grup:
                    N[qi, k] = n
    return N


def kenar_kontrol(Pa, Na, Pb, Nb, duz):
    t = Pb - Pa
    if duz:
        return [Pa, Pa + t / 3.0, Pb - t / 3.0, Pb]
    c1 = Pa + (t - np.dot(t, Na) * Na) / 3.0
    c2 = Pb - (t - np.dot(t, Nb) * Nb) / 3.0
    return [Pa, c1, c2, Pb]


def bezier_ag(p, n, duz):
    """p: 4 köşe (q0..q3), n: köşe normalleri, duz: 4 kenar için keskin mi.
    b[i][j], i u yönünde (q0->q1), j v yönünde (q0->q3)."""
    b = [[None] * 4 for _ in range(4)]
    alt = kenar_kontrol(p[0], n[0], p[1], n[1], duz[0])  # v=0
    sag = kenar_kontrol(p[1], n[1], p[2], n[2], duz[1])  # u=1
    ust = kenar_kontrol(p[3], n[3], p[2], n[2], duz[2])  # v=1 (q3->q2)
    sol = kenar_kontrol(p[0], n[0], p[3], n[3], duz[3])  # u=0
    for i in range(4):
        b[i][0] = alt[i]
        b[i][3] = ust[i]
    for j in range(4):
        b[0][j] = sol[j]
        b[3][j] = sag[j]

    def ic(i, j):
        # Ayrık Coons: b11 = (-4b00 + 6(b10+b01) - 2(b30+b03) + 3(b31+b13) - b33) / 9
        # En yakın köşeye göre yansıtılarak dört iç noktaya uygulanır.
        x = (lambda k: k) if i == 1 else (lambda k: 3 - k)
        y = (lambda k: k) if j == 1 else (lambda k: 3 - k)
        B = lambda r, s: b[x(r)][y(s)]
        return (-4 * B(0, 0) + 6 * (B(1, 0) + B(0, 1)) - 2 * (B(3, 0) + B(0, 3))
                + 3 * (B(3, 1) + B(1, 3)) - B(3, 3)) / 9.0

    for i in (1, 2):
        for j in (1, 2):
            b[i][j] = ic(i, j)
    return b


def duzlemsel_mi(b, n, tol):
    """16 kontrol noktası tek düzlemde ve 4 kenar düz çizgi mi?"""
    pts = np.array([b[i][j] for i in range(4) for j in range(4)])
    if np.abs((pts - pts[0]) @ n).max() > tol:
        return False
    for kenar in ([b[i][0] for i in range(4)], [b[i][3] for i in range(4)],
                  [b[0][j] for j in range(4)], [b[3][j] for j in range(4)]):
        a, c = kenar[0], kenar[3]
        t = c - a
        L = np.linalg.norm(t)
        if L == 0:
            return False
        t = t / L
        for p in kenar[1:3]:
            r = p - a
            if np.linalg.norm(r - (r @ t) * t) > tol:
                return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("girdi")
    ap.add_argument("cikti")
    ap.add_argument("--hedef-ucgen", type=int, default=0,
                    help="Önce ağı bu üçgen sayısına indir (0 = indirme)")
    ap.add_argument("--keskin-aci", type=float, default=30.0,
                    help="Bu açıdan (derece) keskin kenarlar düz bırakılır")
    ap.add_argument("--tolerans", type=float, default=1e-4, help="Dikiş toleransı (mm)")
    ap.add_argument("--birlestirme", action="store_true",
                    help="Düz bölgeleri tek düzlem yüzde birleştirmeyi KAPAT")
    ap.add_argument("--sapma", action="store_true",
                    help="Yüzeylerden örnek alıp girdi ağına uzaklığı ölç")
    a = ap.parse_args()

    t0 = time.time()
    m = trimesh.load(a.girdi, force="mesh")
    m.merge_vertices()
    print(f"Girdi: {len(m.faces)} üçgen, su geçirmez={m.is_watertight}")
    if a.hedef_ucgen and len(m.faces) > a.hedef_ucgen:
        m = m.simplify_quadric_decimation(face_count=a.hedef_ucgen)
        m.merge_vertices()
        print(f"İndirgendi: {len(m.faces)} üçgen, su geçirmez={m.is_watertight}")
    if not m.is_watertight:
        sys.exit("HATA: ağ su geçirmez değil; kenar başına 2 yüz gerekli.")

    P, Q = ucgenleri_dortgene_bol(np.asarray(m.vertices, float), np.asarray(m.faces))
    QN = dortgen_normali(P, Q)
    harita = kenar_haritasi(Q)
    esik = np.cos(np.radians(a.keskin_aci))
    keskin = {k for k, v in harita.items()
              if len(v) == 2 and np.dot(QN[v[0][0]], QN[v[1][0]]) < esik}
    N = kose_normalleri(P, Q, QN, keskin, harita)
    print(f"Dörtgen: {len(Q)}, keskin kenar: {len(keskin)}")

    ornek = [] if a.sapma else None
    dikis = BRepBuilderAPI_Sewing(a.tolerans)
    for qi, q in enumerate(Q):
        p = P[q]
        duz = [(min(q[k], q[(k + 1) % 4]), max(q[k], q[(k + 1) % 4])) in keskin for k in range(4)]
        duz = [bool(x) for x in duz]
        b = bezier_ag(p, N[qi], duz)
        arr = TColgp_Array2OfPnt(1, 4, 1, 4)
        for i in range(4):
            for j in range(4):
                arr.SetValue(i + 1, j + 1, gp_Pnt(*map(float, b[i][j])))
        if not a.birlestirme and duzlemsel_mi(b, QN[qi], a.tolerans * 0.1):
            poli = BRepBuilderAPI_MakePolygon(*[gp_Pnt(*map(float, x)) for x in p], True)
            dikis.Add(BRepBuilderAPI_MakeFace(poli.Wire(), True).Face())
            if ornek is not None:
                for u in (0.25, 0.5, 0.75):
                    for v in (0.25, 0.5, 0.75):
                        ornek.append((1-u)*(1-v)*p[0] + u*(1-v)*p[1] + u*v*p[2] + (1-u)*v*p[3])
            continue
        yuzey = Geom_BezierSurface(arr)
        if ornek is not None:
            for u in (0.25, 0.5, 0.75):
                for v in (0.25, 0.5, 0.75):
                    pt = yuzey.Value(u, v)
                    ornek.append((pt.X(), pt.Y(), pt.Z()))
        dikis.Add(BRepBuilderAPI_MakeFace(yuzey, 1e-7).Face())
    dikis.Perform()
    sekil = dikis.SewedShape()

    exp = TopExp_Explorer(sekil, TopAbs_SHELL)
    yapici = BRepBuilderAPI_MakeSolid()
    kabuk_sayisi = 0
    while exp.More():
        yapici.Add(_shell(exp.Current()))
        kabuk_sayisi += 1
        exp.Next()
    duzelt = ShapeFix_Solid(yapici.Solid())
    duzelt.Perform()
    kati = duzelt.Solid()
    if not a.birlestirme:
        birlestir = ShapeUpgrade_UnifySameDomain(kati, True, True, True)
        birlestir.Build()
        kati = birlestir.Shape()

    gecerli = BRepCheck_Analyzer(kati).IsValid()
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(kati, props)
    yuz = duzlem = ucgen = 0
    exp = TopExp_Explorer(kati, TopAbs_FACE)
    while exp.More():
        f = _face(exp.Current())
        yuz += 1
        if BRepAdaptor_Surface(f).GetType() == GeomAbs_Plane:
            duzlem += 1
            kenar_say = 0
            e2 = TopExp_Explorer(f, TopAbs_EDGE)
            while e2.More():
                kenar_say += 1
                e2.Next()
            if kenar_say == 3:
                ucgen += 1
        exp.Next()

    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    w = STEPControl_Writer()
    w.Transfer(kati, STEPControl_AsIs)
    if w.Write(a.cikti) != IFSelect_RetDone:
        sys.exit("HATA: STEP yazılamadı")

    print(f"Kabuk: {kabuk_sayisi}, yüz: {yuz} (düzlem: {duzlem}, eğrisel: {yuz - duzlem}), "
          f"3 kenarlı yüz: {ucgen}, geçerli katı: {gecerli}")
    print(f"Hacim: STEP={abs(props.Mass()):.3f}  ağ={m.volume:.3f}  "
          f"fark=%{100 * abs(abs(props.Mass()) - m.volume) / m.volume:.3f}")
    if ornek:
        _, d, _ = trimesh.proximity.closest_point(trimesh.load(a.girdi, force="mesh"), np.array(ornek))
        print(f"Girdi ağına uzaklık ({len(ornek)} örnek): ortalama={d.mean():.4f} mm, "
              f"en fazla={d.max():.4f} mm")
    print(f"Süre: {time.time() - t0:.1f} sn -> {a.cikti}")


if __name__ == "__main__":
    main()
