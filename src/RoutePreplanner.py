
#RoutePreplanner — Teğet Graf (Görünürlük Grafiği) Tabanlı Geometrik Rota Planlayıcı (v3)

import heapq
import math
from dataclasses import dataclass

from PySide6.QtPositioning import QGeoCoordinate

from src.ServerConnection import ServerAdsData

# Uçak / Ortam Parametreleri (Ortam parametreleri bulunulan konuma göre değişebilir)

CRUISE_SPEED_MS     = 20.0    # m/s — hava hızı
MAX_WIND_MS         = 8.0     # m/s — en kötü durum karşıt/yanlamasına bileşen
GROUND_SPEED_MAX_MS = CRUISE_SPEED_MS + MAX_WIND_MS  

MAX_BANK_DEG        = 45.0    # derece
GRAVITY             = 9.81    # m/s²

# Minimum dönüş yarıçapı: V_ground² / (g·tan(φ)) @ 28 m/s
MIN_TURN_RADIUS_M = (GROUND_SPEED_MAX_MS ** 2) / (
    GRAVITY * math.tan(math.radians(MAX_BANK_DEG))
)

WP_ACCEPT_RADIUS_M = 30.0   # ArduPlane WP_RADIUS (kabul yarıçapı)
GPS_SIGMA_M        = 5.0    # GPS konum hatası tahmini
LATENCY_BUDGET_S   = 2.0    # Sunucu→GCS→TCP→Jetson→MAVLink→otopilot
# NOT:Seyir hızının araca yazılan değeri MainInterface.CRUISE_AIRSPEED_MS'tir

# Planlama tamponu
AVOIDANCE_BUFFER_M = 45.0

# Fence tamponu — dar,sadece gerçek ihlalde fence tetiklensin
FENCE_BUFFER_M = 20.0


# Dar geçit eşiği: iki daire arası boşluk bu değerden azsa birleştir
GAP_MIN_M = 0.75 * WP_ACCEPT_RADIUS_M

# Bir leg çözülürken görünürlük grafiğine dahil edilecek bölgelerin,leg orta
# noktasına olan azami uzaklığına eklenen pay.
PLANNER_ZONE_MARGIN_M = 500.0


# ---------------------------------------------------------------------------
# Veri Yapıları
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class MissionItem:
    """ArduPilot'tan indirilen tek bir görev öğesinin TÜM MAVLink alanlarını
    birebir tutan evrensel veri yapısı. Komut tipinden bağımsız çalışır —
    WAYPOINT, TAKEOFF, LAND, DO_JUMP, LOITER, ROI vb. fark etmeksizin
    her şeyi olduğu gibi saklar ve geri yükler.

    Alanlar:
        seq:          Görev sıra numarası (0 = Home)
        frame:        MAV_FRAME referans çerçevesi (0=GLOBAL/AMSL, 3=RELATIVE_ALT/AGL, ...)
        command:      MAV_CMD komut kodu (16=WAYPOINT, 21=LAND, 22=TAKEOFF, 177=DO_JUMP, ...)
        current:      Şu an aktif görev öğesi mi (genelde 0)
        autocontinue: Otomatik devam bayrağı (genelde 1)
        param1-4:     Komuta özel parametreler (her MAV_CMD için farklı anlam taşır)
        x:            Enlem (derece cinsinden — INT formatından dönüştürülmüş)
        y:            Boylam (derece cinsinden — INT formatından dönüştürülmüş)
        z:            İrtifa (frame'e bağlı olarak AMSL veya AGL)
    """
    seq:          int
    frame:        int
    command:      int
    current:      int
    autocontinue: int
    param1:       float
    param2:       float
    param3:       float
    param4:       float
    x:            float   # enlem (derece)
    y:            float   # boylam (derece)
    z:            float   # irtifa


@dataclass(frozen=True, slots=True)
class RoutePoint:
    lat: float
    lon: float
    alt: float
    command: int             #  16 (WAYPOINT), 21 (LAND), 22 (TAKEOFF)
    origin_idx: int | None   # None = bypass/arc ara noktası


@dataclass
class RoutePlanResult:
    original_waypoints:    list[QGeoCoordinate]
    corrected_waypoints:   list[RoutePoint]
    has_conflicts:         bool
    conflict_zone_ids:     list[int]
    waypoints_inside_zone: list[int]   # Bölge içinde kalan orijinal WP indeksleri


# ---------------------------------------------------------------------------
# koordinat ↔ metre dönüşümü
# ---------------------------------------------------------------------------

def _lat_lon_to_m(lat: float, lon: float,
                  lat0: float, lon0: float) -> tuple[float, float]:
    """Equirectangular dönüşümü: (lat, lon) → (x_east, y_north) metre"""
    R = 6_371_000.0
    x = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = R * math.radians(lat - lat0)
    return x, y


def _m_to_lat_lon(x: float, y: float,
                  lat0: float, lon0: float) -> tuple[float, float]:
    """Ters dönüşüm: metre → (lat, lon)"""
    R = 6_371_000.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon

# 2D düzlemde iki nokta arası mesafe(pisagor)
def _dist(a: tuple, b: tuple) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


# ---------------------------------------------------------------------------
# Daire Geometrisi — Kesişim Testleri
# ---------------------------------------------------------------------------

def _segment_circle_intersects(p1: tuple, p2: tuple,
                                center: tuple, radius: float) -> bool:
    """
    Doğru parçası p1→p2, daire (center, radius) içinden geçiyor mu?
    Teğetlik kabul eşiği: |disc| < eps * 4ac ile oransal (büyük daireler için güvenli).
    """
    cx, cy = center
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    fx = p1[0] - cx
    fy = p1[1] - cy

    a = dx * dx + dy * dy
    if a < 1e-12:
        return math.hypot(fx, fy) < radius

    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c

    # Teğetlik eşiği: kayan nokta hatası için küçük mutlak eşik yeterli
    # (yay uç noktaları tam daire üzerinde → disc küçük ama pozitif → HAYIR sayılmalı)
    if disc <= 1e-2:
        return False

    sqrtd = math.sqrt(disc)
    t1 = (-b - sqrtd) / (2.0 * a)
    t2 = (-b + sqrtd) / (2.0 * a)
    return (0.0 <= t1 <= 1.0) or (0.0 <= t2 <= 1.0) or (t1 < 0.0 < t2)


def _point_in_circle(p: tuple, center: tuple, radius: float) -> bool:
    return _dist(p, center) < radius


def _segment_clear_all(p1: tuple, p2: tuple,
                        zones: list[tuple]) -> bool:
    """p1→p2 çizgisi TÜM güvenli bölgelerin dışında mı?"""
    for (zx, zy, zr, _) in zones:
        if _segment_circle_intersects(p1, p2, (zx, zy), zr):
            return False
    return True


def _zones_near_segment(p1: tuple, p2: tuple, zones: list[tuple]) -> list[tuple]:
    """Bu leg'i etkileyebilecek bölgeleri süz. Bkz. PLANNER_ZONE_MARGIN_M."""
    leg_len = _dist(p1, p2)
    mid = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
    return [z for z in zones
            if _dist(mid, (z[0], z[1])) <= leg_len + z[2] + PLANNER_ZONE_MARGIN_M]


# ---------------------------------------------------------------------------
# Daire Birleştirme (Union): Dar Geçit
# ---------------------------------------------------------------------------

def _merge_overlapping_zones(zones: list[tuple]) -> list[tuple]:
    """
    İki daire arası boşluk GAP_MIN_M'den dar ise (geçilemez) minimum kapsayan
    daire ile birleştirir. Sabit noktaya ulaşana kadar tekrarlar.
    """
    changed = True
    merged = list(zones)
    while changed:
        changed = False
        result: list[tuple] = []
        used = [False] * len(merged)
        for i in range(len(merged)):
            if used[i]:
                continue
            ax, ay, ar, aid = merged[i]
            for j in range(i + 1, len(merged)):
                if used[j]:
                    continue
                bx, by, br, bid = merged[j]
                gap = _dist((ax, ay), (bx, by)) - ar - br  #iki daire arası uzaklık
                if gap < GAP_MIN_M:
                    # Minimum kapsayan daire
                    d = _dist((ax, ay), (bx, by))
                    if ar >= br + d:
                        pass                       # a zaten b'yi kapsıyor
                    elif br >= ar + d:
                        ax, ay, ar = bx, by, br   # b zaten a'yı kapsıyor
                    else:
                        new_r = (d + ar + br) / 2.0
                        ratio = (new_r - ar) / d if d > 1e-6 else 0.0
                        ax = ax + ratio * (bx - ax)
                        ay = ay + ratio * (by - ay)
                        ar = new_r
                    used[j] = True
                    changed = True
            result.append((ax, ay, ar, aid))
        merged = result
    return merged


# ---------------------------------------------------------------------------
# Radyal Acil Çıkış
# ---------------------------------------------------------------------------

def _radial_exit(p: tuple, cx: float, cy: float, r: float) -> tuple:
    """Bölge içindeki noktadan en kısa radyal çıkış noktası (R + buffer dışı)."""
    d = _dist(p, (cx, cy))
    if d < 1e-6:
        return (cx + r + AVOIDANCE_BUFFER_M, cy)
    nx = (p[0] - cx) / d
    ny = (p[1] - cy) / d
    return (cx + nx * (r + AVOIDANCE_BUFFER_M),
            cy + ny * (r + AVOIDANCE_BUFFER_M))


# ---------------------------------------------------------------------------
# Görünürlük Grafiği + Dijkstra
# ---------------------------------------------------------------------------


class HSSAvoidancePlanner:
    """Aşama 1: Kinematik Tabanlı HSS Kaçınma Matematik Modelleri"""
    @staticmethod
    def effective_radius(R_hss: float, buffer_m: float, V_ground: float, bank_deg: float, m_wind: float, m_gps: float, g=9.81):
        R_turn = V_ground**2 / (g * math.tan(math.radians(bank_deg)))
        R_safe = R_hss + buffer_m
        R_eff = max(R_safe, R_turn + m_wind + m_gps)
        infeasible = R_safe < R_turn
        return R_eff, R_turn, infeasible

    @staticmethod
    def _tangents_from_point(p, c, r):
        px, py = p; cx, cy = c
        dx, dy = px - cx, py - cy
        d2 = dx*dx + dy*dy
        if d2 < r*r + 1e-6:
            return []
        d = math.sqrt(d2)
        ad = math.acos(max(-1.0, min(1.0, r/d)))
        t = math.atan2(dy, dx)
        out = []
        for sgn in (+1, -1):
            a = t + sgn*ad
            out.append((cx + r*math.cos(a), cy + r*math.sin(a)))
        return out

    @staticmethod
    def bitangents(c1, r1, c2, r2):
        x1, y1 = c1; x2, y2 = c2
        dx, dy = x2 - x1, y2 - y1
        d = math.hypot(dx, dy)
        res = []
        if d < 1e-6:
            return res
        base = math.atan2(dy, dx)
        for sign_r2, kind in ((+1, 'ext'), (-1, 'int')):
            dr = r1 - sign_r2*r2
            if abs(dr) > d:
                continue
            phi = math.acos(max(-1.0, min(1.0, dr/d)))
            for s in (+1, -1):
                a = base + s*phi
                p1_pt = (x1 + r1*math.cos(a), y1 + r1*math.sin(a))
                p2_pt = (x2 + sign_r2*r2*math.cos(a), y2 + sign_r2*r2*math.sin(a))
                res.append((p1_pt, p2_pt, kind))
        return res

    @staticmethod
    def _arc_points_by_distance(center, r, a_start, a_end, ccw, d_min):
        if ccw:
            dpsi = (a_end - a_start) % (2*math.pi)
        else:
            dpsi = (a_start - a_end) % (2*math.pi)
        arc_len = r * dpsi
        if arc_len <= d_min:
            return []
        n = max(1, int(math.floor(arc_len / d_min)))
        pts = []
        for i in range(1, n):
            t = i / n
            ang = a_start + (dpsi * t) * (1 if ccw else -1)
            pts.append((center[0] + r*math.cos(ang), center[1] + r*math.sin(ang)))
        return pts

    @staticmethod
    def min_spacing(R_turn, wp_radius, turn_dist, k=0.5):
        return max(k*R_turn, turn_dist, 2.0*wp_radius)


def _visibility_graph_dijkstra(
    start: tuple,
    end:   tuple,
    zones: list[tuple],
    d_min: float = 10.0
) -> list[tuple[float, float]]:
    """
    Başlangıç → bitiş için gerçek teğet (bitanjant) görünürlük grafiği + Dijkstra.
    zones: [(cx, cy, r_eff, id)]
    """
    nodes = [(start[0], start[1], None, None), (end[0], end[1], None, None)]
    
    # Tüm teğet noktalarını bul
    for i, z1 in enumerate(zones):
        cx1, cy1, r1, _ = z1
        # start'tan z1'e teğetler
        t_start = HSSAvoidancePlanner._tangents_from_point(start, (cx1, cy1), r1)
        for t in t_start:
            nodes.append((t[0], t[1], i, None))
        
        # end'ten z1'e teğetler
        t_end = HSSAvoidancePlanner._tangents_from_point(end, (cx1, cy1), r1)
        for t in t_end:
            nodes.append((t[0], t[1], i, None))
            
        # Diğer bölgelerle teğetler
        for j in range(i + 1, len(zones)):
            cx2, cy2, r2, _ = zones[j]
            bts = HSSAvoidancePlanner.bitangents((cx1, cy1), r1, (cx2, cy2), r2)
            for p1_pt, p2_pt, kind in bts:
                nodes.append((p1_pt[0], p1_pt[1], i, None))
                nodes.append((p2_pt[0], p2_pt[1], j, None))
                
    n = len(nodes)
    adj: list[list[tuple[int, float]]] = [[] for _ in range(n)]
    
    def pt(i: int) -> tuple: return (nodes[i][0], nodes[i][1])
    
    # 1. Düz kenarlar (farklı daireler veya boşluk)
    for i in range(n):
        for j in range(i + 1, n):
            if nodes[i][2] is not None and nodes[i][2] == nodes[j][2]:
                continue
            if _segment_clear_all(pt(i), pt(j), zones):
                d = _dist(pt(i), pt(j))
                adj[i].append((j, d))
                adj[j].append((i, d))
                
    # 2. Yay kenarları (Aynı daire üstündeki noktalar arası)
    circ_nodes: dict[int, list[int]] = {}
    for idx, nd in enumerate(nodes):
        cidx = nd[2]
        if cidx is not None:
            circ_nodes.setdefault(cidx, []).append(idx)
            
    for cidx, (cx, cy, cr, _) in enumerate(zones):
        c_n_idx = circ_nodes.get(cidx, [])
        m = len(c_n_idx)
        for i in range(m):
            for j in range(i + 1, m):
                ni = c_n_idx[i]
                nj = c_n_idx[j]
                a1 = math.atan2(pt(ni)[1] - cy, pt(ni)[0] - cx)
                a2 = math.atan2(pt(nj)[1] - cy, pt(nj)[0] - cx)
                diff = abs(((a2 - a1) + math.pi) % (2.0 * math.pi) - math.pi)
                arc_cost = cr * diff
                adj[ni].append((nj, arc_cost))
                adj[nj].append((ni, arc_cost))

    # Dijkstra
    dist_arr = [float('inf')] * n
    prev      = [-1] * n
    dist_arr[0] = 0.0
    pq = [(0.0, 0)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist_arr[u] + 1e-9:
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd < dist_arr[v] - 1e-9:
                dist_arr[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if dist_arr[1] == float('inf'):
        return [start, end]   # Çözüm bulunamadı — düz hat fallback

    # Yolu yeniden oluştur
    path_indices: list[int] = []
    cur = 1
    while cur != -1:
        path_indices.append(cur)
        cur = prev[cur]
    path_indices.reverse()

    # Yay kenarlarındaki ara noktaları genişlet
    result: list[tuple[float, float]] = []
    for k, ni in enumerate(path_indices):
        result.append(pt(ni))
        if k + 1 >= len(path_indices):
            break
        nj = path_indices[k + 1]
        # İkisi de aynı daire üzerindeyse → yay ara noktaları
        ci = nodes[ni][2]
        cj = nodes[nj][2]
        if ci is not None and ci == cj:
            cx, cy, cr, _ = zones[ci]
            a1 = math.atan2(pt(ni)[1] - cy, pt(ni)[0] - cx)
            a2 = math.atan2(pt(nj)[1] - cy, pt(nj)[0] - cx)
            ccw = ((a2 - a1) % (2*math.pi)) <= math.pi
            arc_mid = HSSAvoidancePlanner._arc_points_by_distance((cx, cy), cr, a1, a2, ccw, d_min)
            result.extend(arc_mid)

    return result


# ---------------------------------------------------------------------------
# ANA ROTA HESAPLAMA
# ---------------------------------------------------------------------------

def compute_safe_route(
    waypoints:  list[QGeoCoordinate],
    hss_zones:  list[ServerAdsData],
    commands:   list[int] | None = None,
    buffer_m:   float = AVOIDANCE_BUFFER_M,
    origin:     QGeoCoordinate | None = None,
) -> RoutePlanResult:
    """
    Waypoint listesini HSS bölgelerinden kaçınan versiyona dönüştürür.

    Yöntem: Görünürlük Grafiği + Dijkstra (Lozano-Pérez & Wesley, 1979).
    Her leg için bağımsız çözülür; yay ara noktaları sayesinde sabit kanatlı
    için fiziksel olarak uçulabilir rota üretilir.

    Args:
        waypoints: Orijinal görev waypoint'leri.
        hss_zones: HSS bölgelerinin dondurulmuş tuple'ı.
        buffer_m:  Planlama tamponu (varsayılan AVOIDANCE_BUFFER_M ≈ 151 m).
        origin:    Projeksiyon merkezi; None ise ilk waypoint kullanılır.
                   GCS ve Jetson aynı origin'i kullanmalı (P7 tutarlılığı).
    """
    if not waypoints:
        return RoutePlanResult(
            original_waypoints=[], corrected_waypoints=[],
            has_conflicts=False, conflict_zone_ids=[], waypoints_inside_zone=[]
        )

    # Geriye dönük uyumluluk: commands verilmezse hepsi WAYPOINT (16) kabul edilir
    if commands is None:
        commands = [16] * len(waypoints)

    # Outlier filtreleme: (0,0) Null Island ve Home (ilk WP) noktasından >50km uzak noktaları çıkar
    OUTLIER_THRESHOLD_M = 50_000.0  # 50 km
    filtered_wps = []
    filtered_cmds = []
    
    # Geçerli ilk noktayı (Home) bul
    home_coord = None
    for wp in waypoints:
        if abs(wp.latitude()) > 1e-6 or abs(wp.longitude()) > 1e-6:
            home_coord = wp
            break
            
    if home_coord is not None:
        for wp, cmd in zip(waypoints, commands):
            # Null Island (0,0) filtresi
            if abs(wp.latitude()) < 1e-6 and abs(wp.longitude()) < 1e-6:
                continue
            # Home noktasından 50 km filtresi
            if wp.distanceTo(home_coord) > OUTLIER_THRESHOLD_M:
                continue
                
            filtered_wps.append(wp)
            filtered_cmds.append(cmd)
            
        if filtered_wps:
            waypoints = filtered_wps
            commands = filtered_cmds

    # Projeksiyon merkezi — Jetson ile tutarlılık için dışarıdan gelebilir
    if origin is not None:
        lat0, lon0 = origin.latitude(), origin.longitude()
    else:
        lat0 = waypoints[0].latitude()
        lon0 = waypoints[0].longitude()

    # Erken çıkış: HSS yok
    if not hss_zones:
        return RoutePlanResult(
            original_waypoints=list(waypoints),
            corrected_waypoints=[
                RoutePoint(lat=wp.latitude(), lon=wp.longitude(),
                           alt=wp.altitude(), command=commands[i] if i < len(commands) else 16, origin_idx=i)
                for i, wp in enumerate(waypoints)
            ],
            has_conflicts=False, conflict_zone_ids=[], waypoints_inside_zone=[]
        )

    # HSS bölgelerini metre koordinatına dönüştür + planlama tamponu ekle (Etkin Yarıçap Şişirmesi)
    raw_zones: list[tuple[float, float, float, int]] = []
    for hss in hss_zones:
        cx, cy = _lat_lon_to_m(hss.lat, hss.lon, lat0, lon0)
        R_eff, _, _ = HSSAvoidancePlanner.effective_radius(
            R_hss=hss.radius_m, 
            buffer_m=buffer_m, 
            V_ground=GROUND_SPEED_MAX_MS,
            bank_deg=MAX_BANK_DEG, 
            m_wind=MAX_WIND_MS * LATENCY_BUDGET_S, 
            m_gps=GPS_SIGMA_M
        )
        raw_zones.append((cx, cy, R_eff, hss.id))

    # Min waypoint aralığı hesapla (L1 turn_distance ve R_turn tabanlı)
    turn_dist = max(0.3183 * 0.75 * 14.0 * GROUND_SPEED_MAX_MS, 10.0)
    d_min = HSSAvoidancePlanner.min_spacing(MIN_TURN_RADIUS_M, WP_ACCEPT_RADIUS_M, turn_dist)

    # Dar geçitli daireleri birleştir
    zones_m = _merge_overlapping_zones(raw_zones)

    # Waypoint'leri metre koordinatına dönüştür
    waypoints_m  = [_lat_lon_to_m(wp.latitude(), wp.longitude(), lat0, lon0)
                    for wp in waypoints]
    wp_altitudes = [wp.altitude() for wp in waypoints]

    conflict_ids: set[int] = set()
    wp_inside:    list[int] = []

    # Hangi waypoint'ler bölge içinde?
    for idx, wpm in enumerate(waypoints_m):
        for (zx, zy, zr, hid) in zones_m:
            if _point_in_circle(wpm, (zx, zy), zr):
                wp_inside.append(idx)
                conflict_ids.add(hid)
                break

    # (x, y, is_bypass) listesi oluştur
    corrected_m: list[tuple[float, float, bool]] = []

    def _safe_point(p: tuple, cmd: int) -> tuple:
        """Nokta bölge içindeyse radyal çıkışını döndür, dışındaysa kendisini.
        TAKEOFF (22) ve LAND (21) asla taşınmaz."""
        if cmd in (21, 22):
            return p
        for (zx, zy, zr, hid) in zones_m:
            if _point_in_circle(p, (zx, zy), zr):
                conflict_ids.add(hid)
                return _radial_exit(p, zx, zy, zr)
        return p

    for i in range(len(waypoints_m)):
        # İlk waypoint'i ekle (bölge içindeyse güvenli noktaya taşı)
        if i == 0:
            safe_p0 = _safe_point(waypoints_m[0], commands[0] if commands else 16)
            corrected_m.append((safe_p0[0], safe_p0[1], safe_p0 != waypoints_m[0]))

        if i + 1 >= len(waypoints_m):
            break

        p1 = (corrected_m[-1][0], corrected_m[-1][1])   # Son eklenen noktadan başla
        # Hedef: bölge içindeyse güvenli noktaya taşı
        raw_p2 = waypoints_m[i + 1]
        p2 = _safe_point(raw_p2, commands[i + 1] if i + 1 < len(commands) else 16)

        # Sadece bu leg'i etkileyebilecek bölgelerle çalış — görünürlük grafiği
        # bölge sayısına çok duyarlı, uzaktakileri taşımanın anlamı yok.
        leg_zones = _zones_near_segment(p1, p2, zones_m)

        # Bu leg çakışıyor mu?
        leg_conflicts = [z for z in leg_zones
                         if _segment_circle_intersects(p1, p2, (z[0], z[1]), z[2])]

        if not leg_conflicts:
            corrected_m.append((p2[0], p2[1], False))
            continue

        # HSS ID'lerini kaydet
        for (_, _, _, hid) in leg_conflicts:
            conflict_ids.add(hid)

        # Görünürlük grafiği + Dijkstra
        path = _visibility_graph_dijkstra(p1, p2, leg_zones, d_min)

        # path[0] = p1 (zaten eklendi), path[1:] eklenmeli
        n_path = len(path)
        for k, pt in enumerate(path[1:], 1):
            is_bypass = (k < n_path - 1)   # Son eleman hedef WP → orijinal nokta
            corrected_m.append((pt[0], pt[1], is_bypass))

    # Metre → RoutePoint + irtifa/komut interpolasyonu
    corrected_points: list[RoutePoint] = []
    orig_idx = 0

    for (mx, my, is_bypass) in corrected_m:
        lat, lon = _m_to_lat_lon(mx, my, lat0, lon0)
        if not is_bypass:
            # Orijinal waypoint noktası
            clamped = min(orig_idx, len(wp_altitudes) - 1)
            alt = wp_altitudes[clamped]
            cmd = commands[clamped] if clamped < len(commands) else 16
            corrected_points.append(
                RoutePoint(lat=lat, lon=lon, alt=alt, command=cmd, origin_idx=clamped)
            )
            orig_idx = min(orig_idx + 1, len(wp_altitudes) - 1)
        else:
            # Bypass / yay ara noktası — önceki ve sonraki WP irtifasının yükseği
            # BUG FIX: wp_altitudes[0] (Home) MAV_FRAME_GLOBAL (AMSL) formatındadır. 
            # Diğer tüm noktalar MAV_FRAME_GLOBAL_RELATIVE_ALT (AGL) formatındadır.
            # AMSL ve AGL değerlerini "max()" ile kıyaslamak İHA'yı uzaya (1855m AGL) gönderir!
            # Bu yüzden Home noktasını atlıyoruz, en az WP 1 (index 1) irtifasını kullanıyoruz.
            prev_alt = wp_altitudes[max(1, orig_idx - 1)]
            next_alt = wp_altitudes[max(1, min(len(wp_altitudes) - 1, orig_idx))]
            corrected_points.append(
                RoutePoint(lat=lat, lon=lon, alt=max(prev_alt, next_alt),
                           command=16, origin_idx=None) # MAV_CMD_NAV_WAYPOINT (16)
            )

    return RoutePlanResult(
        original_waypoints    = list(waypoints),
        corrected_waypoints   = corrected_points,
        has_conflicts         = len(conflict_ids) > 0,
        conflict_zone_ids     = list(conflict_ids),
        waypoints_inside_zone = wp_inside,
    )


def fence_radius_for_hss(hss_radius_m: float,
                          buffer_m: float = FENCE_BUFFER_M) -> float:
    """
    ArduPlane exclusion fence yarıçapı.
    Planlama tamponu değil, DAR fence tamponu kullanılır (FENCE_BUFFER_M=20m).
    Böylece fence sadece gerçek ihlalde tetiklenir, normal manevrada araya girmez.
    """
    return hss_radius_m + buffer_m


# ---------------------------------------------------------------------------
# Nokta/Doğru — HSS Testleri (kamikaze giriş kapısı ve GUIDED bacakları için)
# ---------------------------------------------------------------------------

def point_hits_zone(lat: float, lon: float, #buradaki nokta hss içinde mi diye bakar
                    hss_zones: list[ServerAdsData],
                    buffer_m: float = FENCE_BUFFER_M) -> ServerAdsData | None:
    """Nokta herhangi bir HSS bölgesinin (tampon dahil) içinde mi?"""
    for hss in hss_zones:
        radius = hss.radius_m + buffer_m
        p = _lat_lon_to_m(lat, lon, hss.lat, hss.lon)
        if _point_in_circle(p, (0.0, 0.0), radius):
            return hss
    return None

#burada verilen iki nokta arasında gidilen rota hss içine giriyor mu diye bakar.
def segment_hits_zone(lat1: float, lon1: float, lat2: float, lon2: float,
                      hss_zones: list[ServerAdsData],
                      buffer_m: float = FENCE_BUFFER_M) -> ServerAdsData | None:
    """(lat1,lon1)→(lat2,lon2) düz çizgisi bir HSS bölgesini kesiyor mu?"""
    for hss in hss_zones:
        radius = hss.radius_m + buffer_m
        p1 = _lat_lon_to_m(lat1, lon1, hss.lat, hss.lon)
        p2 = _lat_lon_to_m(lat2, lon2, hss.lat, hss.lon)
        if _point_in_circle(p1, (0.0, 0.0), radius) or _point_in_circle(p2, (0.0, 0.0), radius):
            return hss
        if _segment_circle_intersects(p1, p2, (0.0, 0.0), radius):
            return hss
    return None
