"""Organik bölgeyi az sayıda büyük B-spline yüze çeviren mod.

Akış:
  1. Organik üçgenler, normalleri benzer ve disk biçimli bölgelere ayrılır (VSA, L2,1 ölçütü).
     Keskin kenarlar bölge içine alınmaz.
  2. Bölgeler arası her sınır parçası tek bir B-spline eğri olur ve iki komşu bölge aynı
     kenarı kullanır.
  3. Her organik bölgeye, orijinal ağın noktalarına en küçük karelerle (düzgünlük terimli)
     kübik bir B-spline yüzey uydurulur. Düz bölgeler düzlem, delik duvarları silindir kalır.
  4. Yüzler sınır telleriyle kırpılır, dikilir ve katıya çevrilir.
"""

import heapq
from collections import defaultdict

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
import trimesh
from scipy.interpolate import BSpline

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
)
from OCP.Geom import Geom_BSplineSurface, Geom_Plane
from OCP.GeomAbs import GeomAbs_C2
from OCP.GeomAPI import GeomAPI_PointsToBSpline
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt
from OCP.ShapeFix import ShapeFix_Face
from OCP.TopoDS import TopoDS

_kenar = getattr(TopoDS, "Edge_s", None) or TopoDS.Edge

try:
    from OCP.TColgp import TColgp_Array1OfPnt, TColgp_Array2OfPnt
    from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal
except ImportError:  # OCCT 8
    from OCP.OCP.collections import Array1_double as TColStd_Array1OfReal
    from OCP.OCP.collections import Array1_gp_Pnt as TColgp_Array1OfPnt
    from OCP.OCP.collections import Array1_int as TColStd_Array1OfInteger
    from OCP.OCP.collections import Array2_gp_Pnt as TColgp_Array2OfPnt


# ----------------------------------------------------------------------------- bölgeleme

def _komsuluk(m, izin):
    """izin[f] olan yüzler arasında, keskin olmayan kenar komşuluğu."""
    komsu = defaultdict(list)
    for (a, b) in m.face_adjacency:
        if izin[a] and izin[b]:
            komsu[a].append(b)
            komsu[b].append(a)
    return komsu


def _buyut(tohumlar, vekil, FN, A, komsu, serbest_idx):
    """Tohumlardan öncelik kuyruğuyla bölge büyütme (L2,1 hatası)."""
    etiket = {}
    kuyruk = []
    for k, s in enumerate(tohumlar):
        etiket[s] = k
        for g in komsu[s]:
            heapq.heappush(kuyruk, (A[g] * np.sum((FN[g] - vekil[k]) ** 2), g, k))
    while kuyruk:
        _, f, k = heapq.heappop(kuyruk)
        if f in etiket:
            continue
        etiket[f] = k
        for g in komsu[f]:
            if g not in etiket:
                heapq.heappush(kuyruk, (A[g] * np.sum((FN[g] - vekil[k]) ** 2), g, k))
    return etiket


def _disk_mi(F, fs):
    """Yüz kümesi tek sınırlı disk mi (Euler karakteristiği 1 ve tek sınır döngüsü)?"""
    alt = F[fs]
    kenar = np.sort(np.concatenate([alt[:, [0, 1]], alt[:, [1, 2]], alt[:, [2, 0]]]), 1)
    u, say = np.unique(kenar, axis=0, return_counts=True)
    nv = len(np.unique(alt))
    if nv - len(u) + len(fs) != 1:
        return False
    sinir = u[say == 1]
    if not len(sinir):
        return False
    derece = np.bincount(sinir.ravel())
    return derece[derece > 0].max() == 2


def bolgele(m, serbest, keskin_kenar, hedef_alan, koni=60.0, tur=6, log=print):
    """Organik yüzleri bölgelere ayırır. Döndürür: yüz -> bölge (organik olmayanlar -1)."""
    F = np.asarray(m.faces)
    FN = m.face_normals
    A = m.area_faces
    C = m.triangles_center
    izin = serbest.copy()
    komsu = defaultdict(list)
    for (a, b), e in zip(m.face_adjacency, np.sort(m.face_adjacency_edges, 1)):
        if izin[a] and izin[b] and (int(e[0]), int(e[1])) not in keskin_kenar:
            komsu[a].append(b)
            komsu[b].append(a)
    serbest_idx = np.where(serbest)[0]
    K = max(1, int(np.ceil(A[serbest_idx].sum() / hedef_alan)))

    # en uzak nokta örneklemesiyle başlangıç tohumları
    rng = np.random.default_rng(0)
    tohumlar = [int(rng.choice(serbest_idx))]
    uzak = np.linalg.norm(C[serbest_idx] - C[tohumlar[0]], axis=1)
    for _ in range(K - 1):
        i = int(np.argmax(uzak))
        tohumlar.append(int(serbest_idx[i]))
        uzak = np.minimum(uzak, np.linalg.norm(C[serbest_idx] - C[serbest_idx[i]], axis=1))
    cos_koni = np.cos(np.radians(koni))

    # her bağlı bileşende en az bir tohum olsun
    from scipy.sparse.csgraph import connected_components
    satir = [a for a, bs in komsu.items() for _ in bs]
    sutun = [b for a, bs in komsu.items() for b in bs]
    _, bil = connected_components(sp.coo_matrix((np.ones(len(satir)), (satir, sutun)),
                                                shape=(len(F), len(F))), directed=False)
    tohumlu = {bil[t] for t in tohumlar}
    for f in serbest_idx:
        if bil[f] not in tohumlu:
            tohumlar.append(int(f))
            tohumlu.add(bil[f])

    for dis in range(40):
        vekil = [FN[s] for s in tohumlar]
        for _ in range(tur):
            etiket = _buyut(tohumlar, vekil, FN, A, komsu, serbest_idx)
            gruplar = defaultdict(list)
            for f, k in etiket.items():
                gruplar[k].append(f)
            yeni_t, yeni_v = [], []
            for k in range(len(tohumlar)):
                fs = np.array(gruplar.get(k, []))
                if not len(fs):
                    continue
                n = (FN[fs] * A[fs][:, None]).sum(0)
                n /= np.linalg.norm(n)
                hata = A[fs] * np.sum((FN[fs] - n) ** 2, 1)
                yeni_t.append(int(fs[np.argmin(hata)]))
                yeni_v.append(n)
            tohumlar, vekil = yeni_t, yeni_v
        etiket = _buyut(tohumlar, vekil, FN, A, komsu, serbest_idx)
        gruplar = defaultdict(list)
        for f, k in etiket.items():
            gruplar[k].append(f)
        # koşulları denetle: koni ve disk
        eklenen = 0
        for k, fs in gruplar.items():
            fs = np.array(fs)
            n = vekil[k]
            en_kotu = fs[np.argmin(FN[fs] @ n)]
            if FN[en_kotu] @ n < cos_koni:  # izdüşümle parametrelenebilsin
                tohumlar.append(int(en_kotu))
                eklenen += 1
        log(f"  bölgeleme turu {dis + 1}: {len(gruplar)} bölge, {eklenen} bölge bölünüyor")
        if not eklenen:
            break
    sonuc = -np.ones(len(F), int)
    for f, k in etiket.items():
        sonuc[f] = k
    return sonuc, vekil


# ----------------------------------------------------------------------------- sınır ağı

def sinir_agi(F, bolge, V):
    """Bölgeler arası sınır kenarlarını, kavşaklar arasında kalan parçalara ayırır.
    Döndürür: parcalar (köşe dizileri), kenar_parca {(a,b): (parça, yön)}."""
    he = defaultdict(list)  # yönsüz kenar -> [(yüz, a, b)]
    for f in range(len(F)):
        for j in range(3):
            a, b = int(F[f, j]), int(F[f, (j + 1) % 3])
            he[(min(a, b), max(a, b))].append((f, a, b))
    sinir = {}
    kose_bolge = defaultdict(set)
    for e, l in he.items():
        if len(l) != 2:
            continue
        r0, r1 = bolge[l[0][0]], bolge[l[1][0]]
        for f, a, b in l:
            kose_bolge[a].add(bolge[f])
            kose_bolge[b].add(bolge[f])
        if r0 != r1:
            sinir[e] = (min(r0, r1), max(r0, r1))
    komsu = defaultdict(list)
    for (a, b) in sinir:
        komsu[a].append(b)
        komsu[b].append(a)
    kavsak = {v for v in komsu if len(kose_bolge[v]) >= 3 or len(komsu[v]) != 2}
    gorulen = set()
    parcalar = []
    kenar_parca = {}

    def yuru(bas, ilk):
        dizi = [bas, ilk]
        cift = sinir[(min(bas, ilk), max(bas, ilk))]
        while dizi[-1] not in kavsak and dizi[-1] != bas:
            u = dizi[-1]
            sonraki = [w for w in komsu[u] if w != dizi[-2]
                       and sinir[(min(u, w), max(u, w))] == cift]
            if not sonraki:
                break
            dizi.append(sonraki[0])
        return dizi

    def kaydet(dizi):
        p = len(parcalar)
        parcalar.append(dizi)
        for i in range(len(dizi) - 1):
            a, b = dizi[i], dizi[i + 1]
            e = (min(a, b), max(a, b))
            gorulen.add(e)
            kenar_parca[e] = p

    for v in kavsak:
        for w in komsu[v]:
            if (min(v, w), max(v, w)) not in gorulen:
                kaydet(yuru(v, w))
    for e in sinir:  # kavşaksız kapalı döngüler: ikiye böl
        if e in gorulen:
            continue
        a, b = e
        d = yuru(a, b)
        if d[-1] == d[0] and len(d) > 3:
            orta = len(d) // 2
            kaydet(d[:orta + 1])
            kaydet(d[orta:])
        else:
            kaydet(d)
    return parcalar, kenar_parca


def egri_uydur(P, tol):
    """Nokta dizisinden uçları sabit B-spline eğri."""
    if len(P) == 2:
        return None  # doğru parçası
    arr = TColgp_Array1OfPnt(1, len(P))
    for i, p in enumerate(P):
        arr.SetValue(i + 1, gp_Pnt(*map(float, p)))
    deg_min = min(3, len(P) - 1)
    return GeomAPI_PointsToBSpline(arr, deg_min, 8, GeomAbs_C2, tol).Curve()


# ----------------------------------------------------------------------------- yüzey uydurma

def _bspline_tabani(t, n, a, b, k=3):
    ic = np.linspace(a, b, n - k + 1)
    dugum = np.r_[[a] * k, ic, [b] * k]
    return BSpline.design_matrix(np.clip(t, a, b), dugum, k).tocsr(), ic


def yuzey_uydur(P, n, e1, e2, merkez, duzgunluk=1e-4):
    """P noktalarına, (e1, e2) düzlemine izdüşümle parametrelenmiş kübik B-spline yüzey."""
    u = (P - merkez) @ e1
    v = (P - merkez) @ e2
    pay_u = 0.05 * (u.max() - u.min()) + 1e-3
    pay_v = 0.05 * (v.max() - v.min()) + 1e-3
    ua, ub = u.min() - pay_u, u.max() + pay_u
    va, vb = v.min() - pay_v, v.max() + pay_v
    Bu, ku = _bspline_tabani(u, n, ua, ub)
    Bv, kv = _bspline_tabani(v, n, va, vb)
    # satır i: kron(Bu_i, Bv_i); kübik tabanda her satırda tam 4 sıfırdan farklı değer var
    Bu = sp.csr_matrix(Bu)
    Bv = sp.csr_matrix(Bv)
    N = len(P)
    iu = Bu.indices.reshape(N, -1)
    xu = Bu.data.reshape(N, -1)
    iv = Bv.indices.reshape(N, -1)
    xv = Bv.data.reshape(N, -1)
    cols = (iu[:, :, None] * n + iv[:, None, :]).reshape(N, -1)
    vals = (xu[:, :, None] * xv[:, None, :]).reshape(N, -1)
    rows = np.repeat(np.arange(N), cols.shape[1])
    cols = cols.ravel()
    vals = vals.ravel()
    M = sp.csr_matrix((vals, (rows, cols)), shape=(len(P), n * n))
    # ikinci fark düzgünlük terimi
    D1 = sp.diags([1.0, -2.0, 1.0], [0, 1, 2], shape=(n - 2, n))
    I = sp.identity(n)
    R = sp.vstack([sp.kron(D1, I), sp.kron(I, D1)])
    olcek = (M.multiply(M)).sum() / max(R.multiply(R).sum(), 1)
    A_ = (M.T @ M + duzgunluk * olcek * (R.T @ R)).tocsc()
    cozucu = spla.factorized(A_)
    Q = np.column_stack([cozucu(M.T @ P[:, k]) for k in range(3)])
    hata = np.linalg.norm(M @ Q - P, axis=1)
    return Q.reshape(n, n, 3), (ua, ub, ku), (va, vb, kv), hata


def bspline_yuzey(Q, ku, kv, k=3):
    n_u, n_v = Q.shape[:2]
    poles = TColgp_Array2OfPnt(1, n_u, 1, n_v)
    for i in range(n_u):
        for j in range(n_v):
            poles.SetValue(i + 1, j + 1, gp_Pnt(*map(float, Q[i, j])))

    def dugumler(ic):
        K = TColStd_Array1OfReal(1, len(ic))
        Mu = TColStd_Array1OfInteger(1, len(ic))
        for i, x in enumerate(ic):
            K.SetValue(i + 1, float(x))
            Mu.SetValue(i + 1, k + 1 if i in (0, len(ic) - 1) else 1)
        return K, Mu

    UK, UM = dugumler(ku)
    VK, VM = dugumler(kv)
    return Geom_BSplineSurface(poles, UK, VK, UM, VM, k, k)


# ----------------------------------------------------------------------------- uçtan uca

def donustur(m, sinif, orijinal, keskin_aci=30.0, hedef_alan=400.0, koni=70.0,
             yuzey_tol=0.05, kirinti=10, log=print):
    """sinif: stl2step.siniflandir çıktısı (köşeler düzlem/silindirlere oturtulmuş).
    orijinal: uydurmada kullanılacak tam çözünürlüklü ağ. Yüz listesi ve özet döndürür."""
    from scipy.sparse.csgraph import connected_components

    V = sinif["V"]
    F = np.asarray(m.faces)
    nf = len(F)
    FN = m.face_normals
    adj = m.face_adjacency
    ae = np.sort(m.face_adjacency_edges, 1)
    ang = np.degrees(m.face_adjacency_angles)
    serbest = sinif["serbest"]

    # keskin kenarlar; ama küçük kırıntı bileşenlerinin kenarları keskin sayılmaz
    keskin_m = ang >= keskin_aci
    ok = (~keskin_m) & serbest[adj[:, 0]] & serbest[adj[:, 1]]
    _, bil = connected_components(sp.coo_matrix((np.ones(ok.sum()), (adj[ok, 0], adj[ok, 1])),
                                                shape=(nf, nf)), directed=False)
    boy = np.bincount(bil)
    kucuk = serbest & (boy[bil] < kirinti)
    keskin = set()
    for (a, b), e, k in zip(adj, ae, keskin_m):
        if k and not (kucuk[a] or kucuk[b]):
            keskin.add((int(e[0]), int(e[1])))
    log(f"  {int(kucuk.sum())} üçgenlik {int((boy[np.unique(bil[kucuk])] if kucuk.any() else []).__len__())}"
        f" kırıntı bileşeni komşularına katılabilir")

    bl, vekil = bolgele(m, serbest, keskin, hedef_alan, koni=koni, log=lambda *a: None)
    # bölge kimlikleri: 0.. organik, sonra düzlemler, sonra silindirler
    bolge = -np.ones(nf, int)
    bolge[serbest] = bl[serbest]
    n_org = int(bl[serbest].max()) + 1
    duz_id = {}
    for f in np.where(sinif["duzlem"])[0]:
        r = int(sinif["lab"][f])
        duz_id.setdefault(r, n_org + len(duz_id))
        bolge[f] = duz_id[r]
    sil_id = {}
    for f in np.where(sinif["duvar"] >= 0)[0]:
        h = int(sinif["duvar"][f])
        sil_id.setdefault(h, n_org + len(duz_id) + len(sil_id))
        bolge[f] = sil_id[h]
    assert (bolge >= 0).all()
    log(f"  bölge: {n_org} organik, {len(duz_id)} düzlem, {len(sil_id)} silindir")

    # sınır ağı ve paylaşılan kenarlar
    parcalar, kenar_parca = sinir_agi(F, bolge, V)
    kose_obj = {}

    def kose(v):
        if v not in kose_obj:
            kose_obj[v] = BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, V[v]))).Vertex()
        return kose_obj[v]

    kenarlar = []
    for p in parcalar:
        P = V[p]
        c = egri_uydur(P, 1e-3)
        if c is None:
            kenarlar.append(BRepBuilderAPI_MakeEdge(kose(p[0]), kose(p[-1])).Edge())
        else:
            mk = BRepBuilderAPI_MakeEdge(c, kose(p[0]), kose(p[-1]))
            kenarlar.append(mk.Edge() if mk.IsDone() else
                            BRepBuilderAPI_MakeEdge(c).Edge())
    log(f"  sınır: {len(parcalar)} eğri parçası")

    # her bölgenin yönlü sınır döngüleri -> parça dizileri
    yon = defaultdict(list)
    for f in range(nf):
        for j in range(3):
            a, b = int(F[f, j]), int(F[f, (j + 1) % 3])
            yon[bolge[f]].append((a, b))
    ic_kenar = {}
    for r, l in yon.items():
        s_ = set(l)
        ic_kenar[r] = [(a, b) for a, b in l if (b, a) not in s_]

    def donguler(r):
        sonraki = defaultdict(list)
        for a, b in ic_kenar[r]:
            sonraki[a].append(b)
        kalan = set(ic_kenar[r])
        sonuc = []
        while kalan:
            a, b = kalan.pop()
            d = [a, b]
            while d[-1] != d[0]:
                adaylar = [w for w in sonraki[d[-1]] if (d[-1], w) in kalan]
                if not adaylar:
                    break
                kalan.discard((d[-1], adaylar[0]))
                d.append(adaylar[0])
            # köşe dizisini parça dizisine çevir
            seq = []
            for i in range(len(d) - 1):
                e = (min(d[i], d[i + 1]), max(d[i], d[i + 1]))
                pidx = kenar_parca[e]
                ileri = parcalar[pidx].index(d[i]) < parcalar[pidx].index(d[i + 1]) \
                    if parcalar[pidx][0] != parcalar[pidx][-1] else True
                if not seq or seq[-1][0] != pidx:
                    seq.append((pidx, ileri))
            if len(seq) > 1 and seq[0][0] == seq[-1][0]:
                seq.pop()
            sonuc.append(seq)
        return sonuc

    # tam çözünürlüklü noktaları bölgelere dağıt
    O = np.r_[np.asarray(orijinal.vertices), orijinal.triangles_center]
    _, _, en_yakin = trimesh.proximity.closest_point(m, O)
    O_bolge = bolge[en_yakin]

    yuzler = []
    ozet = defaultdict(int)
    hatalar = []
    for r in range(bolge.max() + 1):
        fs = np.where(bolge == r)[0]
        if not len(fs):
            continue
        if r < n_org:
            n = vekil[r] if r < len(vekil) else FN[fs].mean(0)
            n = np.asarray(n) / np.linalg.norm(n)
            e1 = np.cross(n, [1, 0, 0])
            if np.linalg.norm(e1) < 0.1:
                e1 = np.cross(n, [0, 1, 0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(n, e1)
            vs = np.unique(F[fs])
            P = np.r_[O[O_bolge == r], V[vs]]
            merkez = P.mean(0)
            uz = max(np.ptp((P - merkez) @ e1), np.ptp((P - merkez) @ e2))
            ncp = int(np.clip(uz / 2.0 + 4, 5, 40))
            while True:
                Q, ud, vd, hata = yuzey_uydur(P, ncp, e1, e2, merkez)
                if hata.max() < yuzey_tol or ncp >= 40:
                    break
                ncp = min(40, int(ncp * 1.4) + 1)
            hatalar.append(hata.max())
            yuzey = bspline_yuzey(Q, ud[2], vd[2])
            ozet["organik"] += 1
        elif r in duz_id.values():
            lr = [k for k, v in duz_id.items() if v == r][0]
            nn, d0, _ = sinif["duzlemler"][lr]
            yuzey = Geom_Plane(gp_Pln(gp_Pnt(*map(float, nn * d0)), gp_Dir(*map(float, nn))))
            ozet["duzlem"] += 1
        else:
            h = [k for k, v in sil_id.items() if v == r][0]
            yuzey = sinif["silindirler"][h]
            ozet["silindir"] += 1
        tel = []
        for seq in donguler(r):
            w = BRepBuilderAPI_MakeWire()
            for pidx, ileri in seq:
                w.Add(kenarlar[pidx] if ileri else _kenar(kenarlar[pidx].Reversed()))
            if w.IsDone():
                tel.append(w.Wire())
        if not tel:
            ozet["tel_yok"] += 1
            continue
        mf = BRepBuilderAPI_MakeFace(yuzey, tel[0], True)
        for w in tel[1:]:
            mf.Add(w)
        if not mf.IsDone():
            ozet["yuz_kurulamadi"] += 1
            continue
        sf = ShapeFix_Face(mf.Face())
        sf.Perform()
        yuzler.append(sf.Face())
    ozet["uydurma_en_buyuk_hata"] = float(np.max(hatalar)) if hatalar else 0.0
    return yuzler, dict(ozet)
