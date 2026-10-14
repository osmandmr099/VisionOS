"""STL (üçgen ağ) -> STEP (üçgen yüz içermeyen katı model).

Akış:
  1. Ağ onarılır (manifold olmayan küçük kusurlar silinip doldurulur).
  2. Tam çevreli iç silindirler (delikler) bulunur, ağdan çıkarılıp tıkanır.
     Sonda gerçek silindirlerle yeniden kesilir.
  3. Ağ, topolojisi korunarak hedef üçgen sayısına indirgenir (isteğe bağlı).
  4. Düz bölgeler tek düzlem yüze dönüşür.
  5. Organik bölgede komşu üçgenler eşlenip dörtgen olur. Her dörtgen, kenarları
     köşe normallerinden kurulan kübik eğrilerle bikübik Bézier yüzeye çevrilir.
     Eşlenemeyen üçgenler 3 dörtgene bölünür. Kenar eğrileri global olduğu için
     komşu yüzler aynı eğriyi paylaşır ve gövde su geçirmez kalır.
  6. Yüzler dikilir, katı yapılır, delikler kesilir, STEP yazılır.

Kullanım:
  python stl2step.py girdi.stl cikti.step [--olcek 0.1] [--hedef-ucgen 30000] [--sapma]
"""

import argparse
import sys
import time
from collections import defaultdict

import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.Geom import Geom_BezierSurface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.ShapeFix import ShapeFix_Solid
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

try:
    from OCP.TColgp import TColgp_Array2OfPnt
    from OCP.TopTools import TopTools_ListOfShape
except ImportError:  # OCCT 8
    from OCP.OCP.collections import Array2_gp_Pnt as TColgp_Array2OfPnt
    from OCP.OCP.collections import List_TopoDS_Shape as TopTools_ListOfShape

_shell = getattr(TopoDS, "Shell_s", None) or TopoDS.Shell
_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face


def log(*a):
    print(*a, flush=True)


# ----------------------------------------------------------------------------- ağ işlemleri

def kenar_anahtari(F, nv):
    he = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    s = np.sort(he, 1)
    return he, s[:, 0].astype(np.int64) * nv + s[:, 1]


def _yelpaze(V, d, yeni):
    if len(d) < 3:
        return
    if len(d) == 3:
        yeni.append(list(d))
        return
    c = len(V)
    V.append(np.mean([V[i] for i in d], axis=0))
    for j in range(len(d)):
        yeni.append([d[j], d[(j + 1) % len(d)], c])


def donguleri_doldur(V, F):
    """Açık sınır döngülerini, döngü merkezine eklenen bir köşeden yelpazeyle kapatır."""
    he, k = kenar_anahtari(F, len(V))
    _, ters, say = np.unique(k, return_inverse=True, return_counts=True)
    tek = he[say[ters] == 1]
    sonraki = defaultdict(list)
    for a, b in tek:
        sonraki[int(b)].append(int(a))  # kapak yönü: b -> a
    kalan = {(int(b), int(a)) for a, b in tek}
    V = list(V)
    yeni = []
    while kalan:
        a, b = kalan.pop()
        d = [a, b]
        while d[-1] != d[0]:
            u = d[-1]
            aday = [w for w in sonraki[u] if (u, w) in kalan]
            if not aday:
                break
            w = aday[0]
            kalan.discard((u, w))
            if w in d[1:]:  # sekiz şekli: alt döngüyü ayrı kapat
                i = d.index(w)
                _yelpaze(V, d[i:], yeni)
                d = d[:i + 1]
                continue
            d.append(w)
        _yelpaze(V, d[:-1] if d[-1] == d[0] else d, yeni)
    if yeni:
        F = np.vstack([F, np.array(yeni)])
    return np.array(V), F


def onar(m):
    m = trimesh.Trimesh(m.vertices, m.faces, process=True)
    nv = len(m.vertices)
    _, k = kenar_anahtari(m.faces, nv)
    u, say = np.unique(k, return_counts=True)
    bozuk = u[say != 2]
    if len(bozuk):
        kv = np.unique(np.c_[bozuk // nv, bozuk % nv])
        sil = np.isin(m.faces, kv).any(1) & (m.area_faces < m.area_faces.mean() * 0.05)
        m = trimesh.Trimesh(m.vertices, m.faces[~sil], process=True)
        m.remove_unreferenced_vertices()
        V, F = donguleri_doldur(m.vertices, m.faces)
        m = trimesh.Trimesh(V, F, process=True)
    return m


def bolgeler(m, keskin_aci):
    adj = m.face_adjacency
    yumusak = np.degrees(m.face_adjacency_angles) < keskin_aci
    g = coo_matrix((np.ones(yumusak.sum()), (adj[yumusak, 0], adj[yumusak, 1])),
                   shape=(len(m.faces),) * 2)
    return connected_components(g, directed=False)


# ----------------------------------------------------------------------------- delikler

def delik_bul(m, keskin_aci):
    """Keskin kenarlarla çevrili, tam çevreli, içe bakan silindir bölgelerini bulur."""
    nc, lab = bolgeler(m, keskin_aci)
    N = m.face_normals
    C = m.triangles_center
    delikler = []
    for r in range(nc):
        idx = np.where(lab == r)[0]
        if len(idx) < 8:
            continue
        n = N[idx]
        w = m.area_faces[idx]
        ev, evec = np.linalg.eigh((n * w[:, None]).T @ n)
        ax = evec[:, 0]
        if ev[0] > 1e-3 * ev.sum():
            continue
        pts = m.vertices[np.unique(m.faces[idx])]
        u = np.cross(ax, [1, 0, 0])
        if np.linalg.norm(u) < 0.1:
            u = np.cross(ax, [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(ax, u)
        P = np.c_[pts @ u, pts @ v]
        s = np.linalg.lstsq(np.c_[2 * P, np.ones(len(P))], (P ** 2).sum(1), rcond=None)[0]
        c = s[:2]
        R = np.sqrt(s[2] + c @ c)
        hata = np.abs(np.linalg.norm(P - c, axis=1) - R).max()
        th = np.sort(np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0]))
        bosluk = np.degrees(np.max(np.r_[np.diff(th), th[0] + 2 * np.pi - th[-1]]))
        ice = np.mean((C[idx] @ u - c[0]) * (n @ u) + (C[idx] @ v - c[1]) * (n @ v)) < 0
        if hata < 0.03 * R and ice and bosluk < 30:
            delikler.append(dict(yuz=idx, R=R, eksen=ax, nokta=c[0] * u + c[1] * v))
    return delikler


def delikleri_tika(m, delikler):
    """Delik duvarlarını siler, açılan döngüleri kapatır, kesilecek silindirleri döndürür."""
    nv = len(m.vertices)
    _, k = kenar_anahtari(m.faces, nv)
    fid = np.tile(np.arange(len(m.faces)), 3)
    kenar_yuz = defaultdict(list)
    for kk, f in zip(k.tolist(), fid.tolist()):
        kenar_yuz[kk].append(f)
    sil = np.zeros(len(m.faces), bool)
    araclar = []
    for d in delikler:
        sil[d["yuz"]] = True
        ax, p0, R = d["eksen"], d["nokta"], d["R"]
        bolge = set(d["yuz"].tolist())
        t = (m.vertices[np.unique(m.faces[d["yuz"]])] - p0) @ ax
        tmin, tmax = t.min(), t.max()
        acik = {"alt": True, "ust": True}
        _, kd = kenar_anahtari(m.faces[d["yuz"]], nv)
        for kk in set(kd.tolist()):
            dis = [f for f in kenar_yuz[kk] if f not in bolge]
            if len(dis) != 1:
                continue
            r = m.triangles_center[dis[0]] - p0
            if np.linalg.norm(r - (r @ ax) * ax) < R * 0.98:  # komşu yüz delik izdüşümünde: taban
                a, b = divmod(kk, nv)
                tk = ((m.vertices[a] + m.vertices[b]) / 2 - p0) @ ax
                acik["alt" if tk < (tmin + tmax) / 2 else "ust"] = False
        pay = 0.05 * R
        t0 = tmin - (pay if acik["alt"] else 0.0)
        t1 = tmax + (pay if acik["ust"] else 0.0)
        araclar.append((p0 + t0 * ax, ax, R, t1 - t0))
    V, F = donguleri_doldur(np.asarray(m.vertices), np.asarray(m.faces[~sil]))
    m2 = trimesh.Trimesh(V, F, process=True)
    m2.remove_unreferenced_vertices()
    return m2, araclar


# ----------------------------------------------------------------------------- indirgeme

def indirge(m, hedef):
    try:
        import pymeshlab as pml
        ms = pml.MeshSet()
        ms.add_mesh(pml.Mesh(m.vertices, m.faces))
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=hedef, preservetopology=True, preservenormal=True,
            planarquadric=True, qualitythr=0.5, optimalplacement=True, autoclean=True)
        cm = ms.current_mesh()
        d = trimesh.Trimesh(cm.vertex_matrix(), cm.face_matrix(), process=True)
    except ImportError:
        log("UYARI: pymeshlab yok, trimesh ile indirgeniyor (topoloji korunmayabilir)")
        d = m.simplify_quadric_decimation(face_count=hedef)
        d.merge_vertices()
    if not d.is_watertight:
        d = onar(d)
    return d


def temizle(m, esik):
    """Birbirine `esik`ten yakın köşeleri birleştirir, ince (T-köşeli) üçgenleri giderir.
    Dikiş toleransından kısa kenarlar katıyı geçersiz yapar."""
    try:
        import pymeshlab as pml
    except ImportError:
        return m
    deger = getattr(pml, "PureValue", None) or getattr(pml, "AbsoluteValue")
    ms = pml.MeshSet()
    ms.add_mesh(pml.Mesh(m.vertices, m.faces))
    ms.meshing_merge_close_vertices(threshold=deger(esik))
    ms.meshing_remove_duplicate_faces()
    ms.meshing_remove_null_faces()
    ms.meshing_remove_t_vertices(method="Edge Collapse", threshold=40, repeat=True)
    ms.meshing_remove_null_faces()
    ms.meshing_repair_non_manifold_edges(method="Remove Faces")
    ms.meshing_repair_non_manifold_vertices()
    ms.meshing_remove_unreferenced_vertices()
    cm = ms.current_mesh()
    d = trimesh.Trimesh(cm.vertex_matrix(), cm.face_matrix(), process=True)
    if not d.is_watertight:
        d = onar(d)
    return d


# ----------------------------------------------------------------------------- yüzey kurulumu

def kenar_kontrol(Pa, Na, Pb, Nb, duz):
    t = Pb - Pa
    if duz:
        return np.array([Pa, Pa + t / 3.0, Pb - t / 3.0, Pb])
    c1 = Pa + (t - np.dot(t, Na) * Na) / 3.0
    c2 = Pb - (t - np.dot(t, Nb) * Nb) / 3.0
    return np.array([Pa, c1, c2, Pb])


def ikiye_bol(c):
    """Kübik Bézier'i t=0.5'ten de Casteljau ile böler."""
    p01, p12, p23 = (c[0] + c[1]) / 2, (c[1] + c[2]) / 2, (c[2] + c[3]) / 2
    p012, p123 = (p01 + p12) / 2, (p12 + p23) / 2
    o = (p012 + p123) / 2
    return np.array([c[0], p01, p012, o]), np.array([o, p123, p23, c[3]])


def coons_ag(alt, sag, ust, sol):
    """alt: q0->q1, sag: q1->q2, ust: q3->q2, sol: q0->q3 (her biri 4 kontrol noktası).
    İç noktalar ayrık Coons: b11 = (-4b00 + 6(b10+b01) - 2(b30+b03) + 3(b31+b13) - b33) / 9"""
    b = np.zeros((4, 4, 3))
    b[:, 0] = alt
    b[:, 3] = ust
    b[0, :] = sol
    b[3, :] = sag
    for i in (1, 2):
        for j in (1, 2):
            x = (lambda r: r) if i == 1 else (lambda r: 3 - r)
            y = (lambda r: r) if j == 1 else (lambda r: 3 - r)
            B = lambda r, s: b[x(r), y(s)]
            b[i, j] = (-4 * B(0, 0) + 6 * (B(1, 0) + B(0, 1)) - 2 * (B(3, 0) + B(0, 3))
                       + 3 * (B(3, 1) + B(1, 3)) - B(3, 3)) / 9.0
    return b


def bezier_yuz(b):
    arr = TColgp_Array2OfPnt(1, 4, 1, 4)
    for i in range(4):
        for j in range(4):
            arr.SetValue(i + 1, j + 1, gp_Pnt(*map(float, b[i, j])))
    return BRepBuilderAPI_MakeFace(Geom_BezierSurface(arr), 1e-7).Face()


def poligon_yuz(noktalar):
    p = BRepBuilderAPI_MakePolygon()
    for x in noktalar:
        p.Add(gp_Pnt(*map(float, x)))
    p.Close()
    f = BRepBuilderAPI_MakeFace(p.Wire(), True)
    return f.Face() if f.IsDone() else None


def esle(m, serbest, yasak_kenar):
    """Organik üçgenleri, ortak kenarı keskin olmayan komşularıyla dörtgen olacak şekilde eşler.
    Önce açgözlü (en düz kenar önce), sonra artırıcı yollarla eşsiz kalanlar azaltılır."""
    adj = m.face_adjacency
    ae = np.sort(m.face_adjacency_edges, 1)
    ac = np.degrees(m.face_adjacency_angles)
    uygun = serbest[adj[:, 0]] & serbest[adj[:, 1]]
    uygun &= np.array([(int(a), int(b)) not in yasak_kenar for a, b in ae], bool)
    es = -np.ones(len(m.faces), int)
    komsu = defaultdict(list)
    for i in np.where(uygun)[0]:
        a, b = int(adj[i, 0]), int(adj[i, 1])
        komsu[a].append(b)
        komsu[b].append(a)
    sira = np.where(uygun)[0]
    for i in sira[np.argsort(ac[sira])]:
        a, b = adj[i]
        if es[a] < 0 and es[b] < 0:
            es[a], es[b] = b, a
    for s in [int(f) for f in np.where(serbest & (es < 0))[0]]:
        if es[s] >= 0:
            continue
        onceki = {s: None}
        kuyruk = [s]
        bulundu = None
        while kuyruk and bulundu is None and len(onceki) < 2000:
            yeni = []
            for x in kuyruk:
                for y in komsu[x]:
                    if y in onceki:
                        continue
                    if es[y] < 0:
                        onceki[y] = x
                        bulundu = y
                        break
                    z = int(es[y])
                    if z in onceki:
                        continue
                    onceki[y] = x
                    onceki[z] = y
                    yeni.append(z)
                if bulundu is not None:
                    break
            kuyruk = yeni
        if bulundu is None:
            continue
        y = bulundu
        while y is not None:  # yol üzerindeki eşleşmeleri ters çevir
            x = onceki[y]
            sonraki = onceki[x]
            es[y], es[x] = x, y
            y = sonraki
    return es


def yuzleri_kur(m, keskin_aci, duz_tol, duz_aci=0.25):
    V = np.asarray(m.vertices, float)
    F = np.asarray(m.faces)
    FN = m.face_normals
    nf = len(F)
    adj = m.face_adjacency
    ae = np.sort(m.face_adjacency_edges, 1)
    ang = np.degrees(m.face_adjacency_angles)

    # düz bölgeler: neredeyse aynı normalli ve aynı düzlemdeki komşu üçgenler (en az 2 üçgen)
    benzer = ang < duz_aci
    benzer &= np.abs(np.einsum("ij,ij->i", V[F[adj[:, 0]][:, 0]] - V[F[adj[:, 1]][:, 0]],
                               FN[adj[:, 1]])) < duz_tol
    _, lab = connected_components(
        coo_matrix((np.ones(benzer.sum()), (adj[benzer, 0], adj[benzer, 1])), shape=(nf, nf)),
        directed=False)
    duzlem = np.bincount(lab)[lab] >= 2
    serbest = ~duzlem

    keskin = {(int(e[0]), int(e[1])) for e, a in zip(ae, ang) if a >= keskin_aci}
    duz_kenar = set(keskin)
    for (f0, f1), e in zip(adj, ae):
        if duzlem[f0] or duzlem[f1]:
            duz_kenar.add((int(e[0]), int(e[1])))

    # köşe normalleri: keskin kenar geçilmeden ulaşılan yüzlerin alan ağırlıklı ortalaması
    alan = m.area_faces
    kose_yuz = defaultdict(list)
    kenar_yuz = defaultdict(list)
    for f in range(nf):
        for j in range(3):
            a, b = int(F[f, j]), int(F[f, (j + 1) % 3])
            kose_yuz[a].append(f)
            kenar_yuz[(min(a, b), max(a, b))].append(f)
    KN = {}
    for v, fs in kose_yuz.items():
        ebeveyn = {f: f for f in fs}

        def bul(x):
            while ebeveyn[x] != x:
                ebeveyn[x] = ebeveyn[ebeveyn[x]]
                x = ebeveyn[x]
            return x

        for f in fs:
            for w in F[f]:
                w = int(w)
                if w == v:
                    continue
                e = (min(v, w), max(v, w))
                if e in keskin:
                    continue
                for g in kenar_yuz[e]:
                    if g in ebeveyn:
                        ebeveyn[bul(f)] = bul(g)
        grup = defaultdict(list)
        for f in fs:
            grup[bul(f)].append(f)
        for g in grup.values():
            n = (FN[g] * alan[g][:, None]).sum(0)
            n /= max(np.linalg.norm(n), 1e-300)
            for f in g:
                KN[(f, v)] = n

    egri = {}

    def kenar(a, b, f):
        """a->b yönünde kübik kontrol noktaları. Her kenar için bir kez hesaplanır ve paylaşılır."""
        e = (min(a, b), max(a, b))
        if e not in egri:
            x, y = e
            egri[e] = kenar_kontrol(V[x], KN[(f, x)], V[y], KN[(f, y)], e in duz_kenar)
        c = egri[e]
        return c if a == e[0] else c[::-1]

    es = esle(m, serbest, duz_kenar)
    bolunen = np.where(serbest & (es < 0))[0]

    orta = {}
    for f in bolunen:
        for j in range(3):
            a, b = int(F[f, j]), int(F[f, (j + 1) % 3])
            orta[(min(a, b), max(a, b))] = ikiye_bol(kenar(a, b, f))[0][3]

    yuzler = []
    tip = {"dortgen": 0, "bolunmus": 0, "duzlem_ucgen": 0}
    for f in range(nf):
        if duzlem[f]:
            pts = []
            for j in range(3):
                a, b = int(F[f, j]), int(F[f, (j + 1) % 3])
                pts.append(V[a])
                e = (min(a, b), max(a, b))
                if e in orta:
                    pts.append(orta[e])
            yz = poligon_yuz(pts)
            if yz is not None:
                yuzler.append(yz)
                tip["duzlem_ucgen"] += 1
            continue
        g = int(es[f])
        if g >= 0:
            if g < f:
                continue
            a, b, c = (int(x) for x in F[f])
            for _ in range(3):  # f'nin g ile ortak kenarı a->b olsun
                if a in F[g] and b in F[g]:
                    break
                a, b, c = b, c, a
            d = int([x for x in F[g] if x != a and x != b][0])
            q = [a, d, b, c]
            yuzler.append(bezier_yuz(coons_ag(kenar(q[0], q[1], g), kenar(q[1], q[2], g),
                                              kenar(q[3], q[2], f), kenar(q[0], q[3], f))))
            tip["dortgen"] += 1
            continue
        # eşsiz üçgen -> 3 dörtgen (köşe, kenar ortası, PN merkez, kenar ortası)
        kose = [int(x) for x in F[f]]
        Ps = [V[v] for v in kose]
        Ns = [KN[(f, v)] for v in kose]
        yarim = {}
        E = []
        for j in range(3):
            x, y = kose[j], kose[(j + 1) % 3]
            c3 = kenar(x, y, f)
            yarim[(x, y)] = ikiye_bol(c3)
            E.extend(c3[1:3])
        Ec = np.mean(E, axis=0)
        b111 = Ec + (Ec - np.mean(Ps, axis=0)) / 2
        merkez = (sum(Ps) + 3 * sum(E) + 6 * b111) / 27
        Nm = np.sum(Ns, 0)
        Nm /= np.linalg.norm(Nm)
        for j in range(3):
            sonra, once = kose[(j + 1) % 3], kose[(j - 1) % 3]
            h_sonra = yarim[(kose[j], sonra)][0]
            h_once = yarim[(once, kose[j])][1][::-1]
            N1 = Ns[j] + Ns[(j + 1) % 3]
            N2 = Ns[j] + Ns[(j - 1) % 3]
            ic1 = kenar_kontrol(h_sonra[3], N1 / np.linalg.norm(N1), merkez, Nm, False)
            ic2 = kenar_kontrol(h_once[3], N2 / np.linalg.norm(N2), merkez, Nm, False)
            yuzler.append(bezier_yuz(coons_ag(h_sonra, ic1, ic2, h_once)))
        tip["bolunmus"] += 1
    return yuzler, tip


# ----------------------------------------------------------------------------- ölçüm

def kati_say(sekil):
    yuz = duzlem = silindir = ucgen = 0
    exp = TopExp_Explorer(sekil, TopAbs_FACE)
    while exp.More():
        f = _face(exp.Current())
        yuz += 1
        t = BRepAdaptor_Surface(f).GetType()
        if t == GeomAbs_Plane:
            duzlem += 1
        elif t == GeomAbs_Cylinder:
            silindir += 1
        k = 0
        e2 = TopExp_Explorer(f, TopAbs_EDGE)
        while e2.More():
            k += 1
            e2.Next()
        if k == 3:
            ucgen += 1
        exp.Next()
    return yuz, duzlem, silindir, ucgen


def sekli_aga_cevir(sekil, sapma):
    BRepMesh_IncrementalMesh(sekil, sapma, False, 0.3, True)
    Vs, Fs, o = [], [], 0
    exp = TopExp_Explorer(sekil, TopAbs_FACE)
    while exp.More():
        f = _face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            tr = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(tr)
                Vs.append((p.X(), p.Y(), p.Z()))
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                Fs.append((a - 1 + o, b - 1 + o, c - 1 + o))
            o += tri.NbNodes()
        exp.Next()
    return trimesh.Trimesh(np.array(Vs), np.array(Fs), process=False)


# ----------------------------------------------------------------------------- ana akış

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("girdi")
    ap.add_argument("cikti")
    ap.add_argument("--olcek", type=float, default=1.0,
                    help="Koordinatları bu katsayıyla çarp (ör. 0,1 mm birimli STL için 0.1)")
    ap.add_argument("--hedef-ucgen", type=int, default=0,
                    help="Ağı önce bu üçgen sayısına indir (0 = indirme)")
    ap.add_argument("--keskin-aci", type=float, default=30.0,
                    help="Bu açıdan (derece) büyük kenarlar keskin kalır")
    ap.add_argument("--delik-yok", action="store_true", help="Delikleri silindire çevirme")
    ap.add_argument("--sapma", action="store_true", help="Sonucun girdi ağından sapmasını ölç")
    a = ap.parse_args()

    t0 = time.time()
    m = trimesh.load(a.girdi, force="mesh")
    m.merge_vertices()
    if a.olcek != 1.0:
        m.apply_scale(a.olcek)
    log(f"Girdi: {len(m.faces)} üçgen, su geçirmez={m.is_watertight}")
    if not m.is_watertight:
        m = onar(m)
        log(f"Onarıldı: su geçirmez={m.is_watertight}")
        if not m.is_watertight:
            sys.exit("HATA: ağ onarılamadı")
    orijinal = m.copy()
    tol = np.linalg.norm(m.extents) * 1e-6

    araclar = []
    if not a.delik_yok:
        m, araclar = delikleri_tika(m, delik_bul(m, a.keskin_aci))
        log(f"Delik: {len(araclar)} silindir bulundu, ağ tıkandı (su geçirmez={m.is_watertight})")
        if not m.is_watertight:
            m = onar(m)

    if a.hedef_ucgen and len(m.faces) > a.hedef_ucgen:
        m = indirge(m, a.hedef_ucgen)
        log(f"İndirgendi: {len(m.faces)} üçgen, su geçirmez={m.is_watertight} ({time.time() - t0:.0f} sn)")

    m = temizle(m, tol * 30)
    e = m.edges_unique_length
    log(f"Temizlendi: {len(m.faces)} üçgen, en kısa kenar={e.min():.5f}, su geçirmez={m.is_watertight}")
    if not m.is_watertight:
        sys.exit("HATA: temizleme sonrası ağ su geçirmez değil")

    yuzler, tip = yuzleri_kur(m, a.keskin_aci, duz_tol=tol * 10)
    log(f"Yüz: {len(yuzler)} (dörtgen {tip['dortgen']}, bölünen üçgen {tip['bolunmus']}, "
        f"düzlem parçası {tip['duzlem_ucgen']}) ({time.time() - t0:.0f} sn)")

    dikis = BRepBuilderAPI_Sewing(tol * 10)
    for f in yuzler:
        dikis.Add(f)
    dikis.Perform()
    sekil = dikis.SewedShape()
    log(f"Dikildi: serbest kenar={dikis.NbFreeEdges()}, çoklu kenar={dikis.NbMultipleEdges()} "
        f"({time.time() - t0:.0f} sn)")

    yapici = BRepBuilderAPI_MakeSolid()
    exp = TopExp_Explorer(sekil, TopAbs_SHELL)
    while exp.More():
        yapici.Add(_shell(exp.Current()))
        exp.Next()
    duzelt = ShapeFix_Solid(yapici.Solid())
    duzelt.Perform()
    kati = duzelt.Solid()
    birlestir = ShapeUpgrade_UnifySameDomain(kati, True, True, True)
    birlestir.Build()
    kati = birlestir.Shape()
    log(f"Katı: geçerli={BRepCheck_Analyzer(kati).IsValid()} ({time.time() - t0:.0f} sn)")

    if araclar:
        argl, tl = TopTools_ListOfShape(), TopTools_ListOfShape()
        argl.Append(kati)
        for p, ax, R, h in araclar:
            tl.Append(BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(*map(float, p)), gp_Dir(*map(float, ax))), float(R), float(h)).Shape())
        kes = BRepAlgoAPI_Cut()
        kes.SetArguments(argl)
        kes.SetTools(tl)
        kes.SetFuzzyValue(tol * 10)
        kes.SetRunParallel(True)
        kes.Build()
        if not kes.IsDone():
            log("UYARI: delik kesimi başarısız, delikler tıkalı bırakıldı")
        else:
            birlestir = ShapeUpgrade_UnifySameDomain(kes.Shape(), True, True, True)
            birlestir.Build()
            kati = birlestir.Shape()
        log(f"Delikler kesildi ({time.time() - t0:.0f} sn)")

    gecerli = BRepCheck_Analyzer(kati).IsValid()
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(kati, props)
    yuz, duzlem, silindir, ucgen = kati_say(kati)
    nsolid = 0
    exp = TopExp_Explorer(kati, TopAbs_SOLID)
    while exp.More():
        nsolid += 1
        exp.Next()

    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    w = STEPControl_Writer()
    w.Transfer(kati, STEPControl_AsIs)
    if w.Write(a.cikti) != IFSelect_RetDone:
        sys.exit("HATA: STEP yazılamadı")

    log(f"Sonuç: katı={nsolid}, geçerli={gecerli}, yüz={yuz} (düzlem {duzlem}, silindir {silindir}, "
        f"eğrisel {yuz - duzlem - silindir}), 3 kenarlı yüz={ucgen}")
    log(f"Hacim: STEP={abs(props.Mass()):.3f}  girdi={orijinal.volume:.3f}  "
        f"fark=%{100 * abs(abs(props.Mass()) - orijinal.volume) / orijinal.volume:.3f}")
    if a.sapma:
        ag = sekli_aga_cevir(kati, np.linalg.norm(orijinal.extents) * 2e-5)
        p1, _ = trimesh.sample.sample_surface(orijinal, 100000, seed=1)
        p2, _ = trimesh.sample.sample_surface(ag, 100000, seed=2)
        _, d1, _ = trimesh.proximity.closest_point(ag, p1)
        _, d2, _ = trimesh.proximity.closest_point(orijinal, p2)
        d = np.r_[d1, d2]
        log(f"Sapma (iki yönlü, 200 bin örnek): ortalama={d.mean():.4f}  %99={np.percentile(d, 99):.4f}  "
            f"en fazla={d.max():.4f}")
    log(f"Süre: {time.time() - t0:.0f} sn -> {a.cikti}")


if __name__ == "__main__":
    main()
