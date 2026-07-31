"""QR / barkod algılama — İHA için optimize edilmiş sürüm.

Gereksinimler: python -m pip install opencv-contrib-python pyzbar zxing-cpp
(zxing-cpp önerilir: daha hızlı ve eğik/bulanık/parlamalı kodlarda daha başarılı.)

İHA koşullarına dayanıklılık:
  - Güneş yansıması / parlama  -> aydınlatma normalizasyonu + CLAHE
  - Sis / pus / düşük kontrast -> CLAHE (yerel kontrast germe)
  - Gölge / dengesiz ışık      -> aydınlatma normalizasyonu (yerel ortalamaya bölme)
  - Titreşim / hafif bulanıklık -> keskinleştirme (unsharp mask)
  - Yüksek irtifa / küçük kod   -> 2x büyütme + keskinleştirme
  - Sallanma / hareket bulanıklığı -> Wiener dekonvolüsyonu (bulanıklığı
    matematiksel geri alır). QR'ın son görüldüğü bölge hatırlanır ve sadece
    orası işlenir (hızlı); işe yarayan bulanıklık yönü/şiddeti de hatırlanır,
    böylece titreşim sürerken sonraki kareler tek denemede okunur.
  Ham kare önce denenir (hızlı yol); kod bulunamazsa bu aşamalar KARE
  BÜTÇESİ içinde sırayla denenir. İşe yarayan aşama hatırlanır ve sonraki
  karede önce o denenir (koşullar süreklidir).

Sallanma için EN ETKİLİ çözüm kameradadır: --exposure ile pozlamayı kısaltın
(örn. --exposure -6). Kısa pozlama = bulanıklık hiç oluşmaz.

Kullanım örnekleri:
  python "barkod .py"                            # webcam 0, sadece QR, 30 FPS hedefi
  python "barkod .py" --exposure -6              # kısa pozlama (sallanmaya en iyi çare)
  python "barkod .py" --symbols all              # tüm barkod tipleri (EAN, Code128, ...)
  python "barkod .py" --engine zbar              # pyzbar motorunu zorla
  python "barkod .py" --record                   # görüntüyü dosyaya da kaydet
  python "barkod .py" --width 1280 --height 720
  python "barkod .py" --no-enhance               # sadece ham kare (eski davranış)
  python "barkod .py" --no-display --max-frames 100   # penceresiz hız testi

Çıkış: önizleme penceresinde 'q' veya ESC.
"""

import argparse
import platform
import time
from datetime import datetime

import cv2
import numpy as np

# İki algılama motoru da isteğe bağlı; en az biri kurulu olmalı.
try:
    from pyzbar.pyzbar import ZBarSymbol, decode as zbar_decode
except ImportError:
    zbar_decode = None

try:
    import zxingcpp
except ImportError:
    zxingcpp = None

# zbar'a sembol listesi vermek aramayı o tiplerle sınırlar -> belirgin hız kazancı
ZBAR_SYMBOLS = {
    "qr": [ZBarSymbol.QRCODE] if zbar_decode else None,
    "all": None,  # None -> pyzbar tüm sembolleri dener
}

RELOG_SECONDS = 2.0        # aynı kod bu süre boyunca görünmezse tekrar loglanır
DETECT_BUDGET_MS = 24.0    # 30 FPS -> kare başına ~33 ms; algılamaya ayrılan pay
BOX_TTL = 2.0              # son görülen QR konumu bu kadar saniye geçerli sayılır


def resolve_engine(name="auto"):
    """Kullanılacak motoru seç: 'auto' -> zxing varsa zxing, yoksa zbar."""
    if name == "auto":
        if zxingcpp is not None:
            return "zxing"
        if zbar_decode is not None:
            return "zbar"
        raise SystemExit("Motor yok! Kurulum: python -m pip install zxing-cpp pyzbar")
    if name == "zxing" and zxingcpp is None:
        raise SystemExit("zxing-cpp kurulu değil: python -m pip install zxing-cpp")
    if name == "zbar" and zbar_decode is None:
        raise SystemExit("pyzbar kurulu değil: python -m pip install pyzbar")
    return name


def _decode(gray, engine, symbol_key):
    """Tek geçişlik ham çözücü çağrısı. Dönüş: [(veri, tip, Nx2 float32), ...]"""
    found = []
    if engine == "zxing":
        kwargs = {}
        if symbol_key == "qr":
            kwargs["formats"] = zxingcpp.BarcodeFormat.QRCode
        for r in zxingcpp.read_barcodes(gray, **kwargs):
            p = r.position
            pts = np.array(
                [(p.top_left.x, p.top_left.y), (p.top_right.x, p.top_right.y),
                 (p.bottom_right.x, p.bottom_right.y), (p.bottom_left.x, p.bottom_left.y)],
                dtype=np.float32)
            found.append((r.text, r.format.name, pts))
    else:
        for b in zbar_decode(gray, symbols=ZBAR_SYMBOLS[symbol_key]):
            pts = np.array([(pt.x, pt.y) for pt in b.polygon], dtype=np.float32)
            # utf-8 olmayan veri betiği çökertmesin
            found.append((b.data.decode("utf-8", errors="replace"), b.type, pts))
    return found


def _dedupe(results):
    """Aynı veriyi birden çok kez raporlama (varyantlar arası tekrarı önler)."""
    seen, out = set(), []
    for item in results:
        if item[0] not in seen:
            seen.add(item[0])
            out.append(item)
    return out


# ---------------- Zor koşul geliştirme varyantları ----------------
# Her fonksiyon gri kare alır, geliştirilmiş gri kare döner. Ucuz olmalılar
# (720p'de her biri ~1-3 ms), asıl maliyet çözücü geçişidir.

_CLAHE = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


def enhance_clahe(gray):
    """Yerel kontrast germe: sis/pus, soluk baskı, hafif parlama gradyanı."""
    return _CLAHE.apply(gray)


def enhance_illum(gray):
    """Aydınlatma normalizasyonu: kareyi yerel ortalamasına böler.
    Gölge, dengesiz ışık ve geniş parlama lekelerini düzler."""
    bg = cv2.blur(gray, (65, 65))
    return cv2.divide(gray, bg, scale=128)


def enhance_sharpen(gray):
    """Unsharp mask: hafif bulanıklık / odak kayması (İHA titreşimi)."""
    blur = cv2.GaussianBlur(gray, (0, 0), 2.0)
    return cv2.addWeighted(gray, 2.2, blur, -1.2, 0)


def enhance_upscale(gray):
    """2x büyütme + keskinleştirme: yüksek irtifadaki küçük/bulanık kodlar.
    Pahalı bir varyanttır (720p'de ~14 ms)."""
    up = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    return enhance_sharpen(up)


# (isim, fonksiyon, koordinat düzeltme çarpanı, tahmini maliyet ms)
# Çarpan: varyant kareyi büyütüyorsa bulunan noktalar geri ölçeklenir.
# Maliyet tahmini başlangıç değeridir; çalışırken ölçülüp güncellenir.
ENHANCERS = [
    ("clahe", enhance_clahe, 1.0, 5.0),
    ("aydinlatma", enhance_illum, 1.0, 5.0),
    ("keskin", enhance_sharpen, 1.0, 5.0),
    ("buyutme", enhance_upscale, 0.5, 16.0),
]


# ---------------- Hareket bulanıklığı geri alma (Wiener) ----------------
# Sallanma = yönlü doğrusal bulanıklık; frekans uzayında tersine çevrilebilir.
# Doğru (uzunluk, açı) bilinmediği için küçük bir çekirdek bankası taranır;
# işe yarayan hatırlanır (titreşim yönü/şiddeti kareler arasında süreklidir).

MOTION_BANK = [(ln, ang) for ln in (9, 13, 17, 21, 25)
               for ang in (0, 45, 90, 135)]


def motion_kernel(length, angle):
    k = np.zeros((length, length), np.float32)
    k[length // 2, :] = 1.0
    M = cv2.getRotationMatrix2D((length / 2 - 0.5, length / 2 - 0.5), angle, 1.0)
    k = cv2.warpAffine(k, M, (length, length))
    return k / k.sum()


def wiener_deblur(gray, length, angle, snr=0.01):
    """Verilen hareket çekirdeğini Wiener filtresiyle geri alır."""
    h, w = gray.shape
    kernel = motion_kernel(length, angle)
    kp = np.zeros((h, w), np.float32)
    kh, kw = kernel.shape
    kp[:kh, :kw] = kernel
    kp = np.roll(kp, (-(kh // 2), -(kw // 2)), axis=(0, 1))
    K = np.fft.rfft2(kp)
    G = np.fft.rfft2(gray.astype(np.float32))
    H = np.conj(K) / (np.abs(K) ** 2 + snr)
    out = np.fft.irfft2(G * H, s=(h, w))
    return np.clip(out, 0, 255).astype(np.uint8)


class RobustDetector:
    """Bütçeli çok aşamalı algılayıcı.

    1. Ham kare denenir (temiz koşulda tek geçiş = en hızlı yol).
    2. Bulunamazsa geliştirme aşamaları, kare başına ayrılan süre bütçesi
       dolana dek denenir. Denemeler kareler arasında döndürülür, böylece
       bütçe yetmese bile birkaç karede tüm aşamalar taranmış olur.
    3. İşe yarayan aşama hatırlanır ve sonraki karede ilk o denenir.
    4. QR'ın son görüldüğü bölge hatırlanır; sallanma (hareket bulanıklığı)
       aşaması sadece o bölgede Wiener dekonvolüsyonu dener (hızlı).
    """

    def __init__(self, engine, symbol_key="qr", scale=1.0,
                 budget_ms=DETECT_BUDGET_MS, enhance=True):
        self.engine = engine
        self.symbol_key = symbol_key
        self.scale = scale
        self.budget_ms = budget_ms
        self.enhance = enhance
        self.last_variant = "ham"   # son başarılı (veya denenen) aşama adı

        def make_stage(fn, coord_scale):
            def stage(det, deadline):
                res = _decode(fn(det), self.engine, self.symbol_key)
                if coord_scale != 1.0:
                    res = [(d, t, pts * coord_scale) for d, t, pts in res]
                return res
            return stage

        # (isim, aşama fonksiyonu) + maliyet EMA'sı (ms, çalışırken ölçülür)
        self._stages = [(name, make_stage(fn, cs)) for name, fn, cs, _ in ENHANCERS]
        self._stages.append(("sallanma", self._deblur_stage))
        self._cost = [c for _, _, _, c in ENHANCERS] + [12.0]
        self._start = 0          # aşama denemesine başlanacak indeks
        self._kernel_idx = 0     # çekirdek bankasında kalınan yer
        self._last_box = None    # (x0, y0, x1, y1) det koordinatında
        self._last_box_t = 0.0

    # ---- sallanma aşaması: bölgesel Wiener dekonvolüsyonu ----

    def _deblur_stage(self, det, deadline):
        fresh = (time.perf_counter() - self._last_box_t) < BOX_TTL
        if self._last_box is not None and fresh:
            # QR'ın son bilinen çevresi: küçük bölge = hızlı FFT
            x0, y0, x1, y1 = self._last_box
            img = det[y0:y1, x0:x1]
            off = np.array([x0, y0], np.float32)
            pt_scale = 1.0
        else:
            # konum bilinmiyor: yarı çözünürlükte tüm kare (daha yavaş ama olur)
            img = cv2.resize(det, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            off = np.zeros(2, np.float32)
            pt_scale = 2.0
        if img.shape[0] < 32 or img.shape[1] < 32:
            return []

        n = len(MOTION_BANK)
        tried = 0
        for i in range(n):
            if tried and time.perf_counter() > deadline:
                break
            k = (self._kernel_idx + i) % n
            length, angle = MOTION_BANK[k]
            if pt_scale != 1.0:
                # yarı çözünürlükte bulanıklık uzunluğu da yarıya iner
                length = max(5, int(round(length / 2)) | 1)
            res = _decode(wiener_deblur(img, length, angle),
                          self.engine, self.symbol_key)
            tried += 1
            if res:
                self._kernel_idx = k  # titreşim sürüyor; sonraki karede önce bunu dene
                return [(d, t, pts * pt_scale + off) for d, t, pts in res]
        self._kernel_idx = (self._kernel_idx + tried) % n
        return []

    def _remember_box(self, results, w, h):
        """Bulunan kodların çevresini (pay ekleyerek) hatırla."""
        pts = np.vstack([p for _, _, p in results])
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        m = 0.6 * max(x1 - x0, y1 - y0) + 20  # sallanma payı
        self._last_box = (int(max(0, x0 - m)), int(max(0, y0 - m)),
                          int(min(w, x1 + m)), int(min(h, y1 + m)))
        self._last_box_t = time.perf_counter()

    def detect(self, gray):
        t0 = time.perf_counter()
        deadline = t0 + self.budget_ms / 1000.0
        det = gray
        if self.scale != 1.0:
            det = cv2.resize(gray, None, fx=self.scale, fy=self.scale,
                             interpolation=cv2.INTER_AREA)

        found = _decode(det, self.engine, self.symbol_key)
        variant = "ham"

        if not found and self.enhance:
            n = len(self._stages)
            tried = 0
            for i in range(n):
                idx = (self._start + i) % n
                elapsed = (time.perf_counter() - t0) * 1000.0
                # Bu aşamanın (ölçülen) maliyeti bütçeye sığmıyorsa atla;
                # rotasyon sayesinde sonraki karede ilk sırada denenir.
                if elapsed + self._cost[idx] > self.budget_ms:
                    continue
                name, stage = self._stages[idx]
                tv0 = time.perf_counter()
                found = stage(det, deadline)
                cost = (time.perf_counter() - tv0) * 1000.0
                self._cost[idx] = 0.8 * self._cost[idx] + 0.2 * cost
                tried += 1
                if found:
                    variant = name
                    self._start = idx  # bu koşul süreklidir; sonraki karede önce bunu dene
                    break
            if not found:
                # denenen (veya bütçeye sığmayan) aşamalar rotasyonla ilerlesin
                self._start = (self._start + max(tried, 1)) % n

        if found:
            h, w = det.shape[:2]
            self._remember_box(found, w, h)
        if self.scale != 1.0:
            found = [(d, t, pts / self.scale) for d, t, pts in found]
        self.last_variant = variant
        return _dedupe(found)


def detect_codes(gray, engine="zbar", symbol_key="qr", scale=1.0):
    """Tek geçiş algılama (geliştirme yok). Testler ve basit kullanım için.

    scale < 1 ise algılama küçültülmüş karede yapılır, koordinatlar geri ölçeklenir.
    """
    det = gray
    if scale != 1.0:
        det = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    found = _decode(det, engine, symbol_key)
    if scale != 1.0:
        found = [(d, t, pts / scale) for d, t, pts in found]
    return _dedupe(found)


def draw_results(frame, results):
    """Kodların çevresine (dönük) çerçeve ve üstüne metin çizer."""
    for data, sym_type, pts in results:
        box = pts.astype(np.int32)
        cv2.polylines(frame, [box], True, (0, 255, 0), 2)
        x = int(box[:, 0].min())
        y = int(box[:, 1].min())
        cv2.putText(frame, f"{data} ({sym_type})", (x, max(y - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def open_camera(index, width=0, height=0, exposure=None):
    # Windows: MSMF kameranın gerçek FPS'ini verir (ölçüldü: 30 FPS);
    # DSHOW bazı kameralarda 20 FPS'e takılıyor. MSMF'in açılışı biraz
    # daha yavaştır ama bu tek seferlik bir maliyettir.
    if platform.system() == "Windows":
        backends = (cv2.CAP_MSMF, cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)
    cap = None
    for backend in backends:
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            break
        cap.release()
        cap = None
    if cap is None:
        return None
    # Tamponda bayat kare bekletme -> hep en güncel kare, düşük gecikme
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if width and height:
        # MJPG çoğu webcam'de yüksek çözünürlükte 30 FPS'in kilidini açar
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if exposure is not None:
        # Kısa pozlama = hareket bulanıklığı hiç oluşmaz (sallanmaya en iyi çare).
        # 0.25 = manuel mod (DirectShow sözleşmesi); değerler kamera bağımlıdır,
        # tipik aralık -4 .. -8 (daha negatif = daha kısa pozlama = daha karanlık).
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    return cap


def make_video_writer(cap, first_frame, filename, fallback_fps=30.0, codec="mp4v"):
    h, w = first_frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 1:  # bazı kameralar 0/NaN döndürür
        fps = fallback_fps
    writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*codec), fps, (w, h))
    if not writer.isOpened():
        raise RuntimeError("VideoWriter açılamadı! Kodek/dosya yolu desteklenmiyor olabilir.")
    return writer, fps, (w, h)


def parse_args():
    p = argparse.ArgumentParser(description="İHA için optimize QR/barkod algılama")
    p.add_argument("--camera", type=int, default=0, help="kamera indeksi (varsayılan 0)")
    p.add_argument("--engine", choices=["auto", "zxing", "zbar"], default="auto",
                   help="algılama motoru (auto: zxing varsa zxing)")
    p.add_argument("--symbols", choices=["qr", "all"], default="qr",
                   help="qr: sadece QR (hızlı) | all: tüm barkod tipleri")
    p.add_argument("--decode-interval", type=int, default=1, metavar="N",
                   help="her N karede bir algılama yap (yüksek çözünürlükte 2-3 önerilir)")
    p.add_argument("--scale", type=float, default=1.0,
                   help="algılamayı küçültülmüş karede yap (örn. 0.5); küçük kodları kaçırabilir")
    p.add_argument("--target-fps", type=float, default=30.0,
                   help="hedef FPS; döngü bundan hızlı koşmaz (0: sınırsız)")
    p.add_argument("--budget-ms", type=float, default=DETECT_BUDGET_MS,
                   help="kare başına algılama süre bütçesi (ms)")
    p.add_argument("--exposure", type=float, default=None, metavar="E",
                   help="manuel pozlama, örn. -6 (kısa pozlama sallanma bulanıklığını önler)")
    p.add_argument("--no-enhance", action="store_true",
                   help="zor koşul geliştirme aşamalarını kapat (sadece ham kare)")
    p.add_argument("--width", type=int, default=0, help="kamera genişliği (0: varsayılan)")
    p.add_argument("--height", type=int, default=0, help="kamera yüksekliği (0: varsayılan)")
    p.add_argument("--record", action="store_true", help="görüntüyü mp4 olarak kaydet")
    p.add_argument("--output", default="", help="kayıt dosya adı (varsayılan: kayit_TARIH.mp4)")
    p.add_argument("--no-display", action="store_true", help="önizleme penceresi açma")
    p.add_argument("--max-frames", type=int, default=0,
                   help="bu kadar kareden sonra dur (0: sınırsız; test için)")
    return p.parse_args()


def main():
    args = parse_args()
    engine = resolve_engine(args.engine)

    cap = open_camera(args.camera, args.width, args.height, args.exposure)
    if cap is None:
        print("Webcam açılamadı!")
        return

    ret, frame = cap.read()
    if not ret:
        print("Kare okunamadı!")
        cap.release()
        return

    writer = None
    if args.record:
        out_name = args.output or f"kayit_{datetime.now():%Y%m%d_%H%M%S}.mp4"
        try:
            writer, wfps, size = make_video_writer(cap, frame, out_name)
            print(f"Kayıt açık -> {out_name} | FPS: {wfps:.2f} | Boyut: {size[0]}x{size[1]}")
        except Exception as e:
            print("VideoWriter oluşturulamadı:", e)
            cap.release()
            return

    detector = RobustDetector(engine, args.symbols, scale=args.scale,
                              budget_ms=args.budget_ms, enhance=not args.no_enhance)

    h, w = frame.shape[:2]
    print(f"Başladı | motor: {engine} | semboller: {args.symbols} | kare: {w}x{h} "
          f"| hedef FPS: {args.target_fps or 'sınırsız'} "
          f"| geliştirme: {'kapalı' if args.no_enhance else 'açık'}"
          + (f" | pozlama: {args.exposure}" if args.exposure is not None else ""))

    last_results = []      # ara karelerde çizim için son algılama sonuçları
    last_seen = {}         # veri -> son görülme zamanı (log tekrarını önler)
    unique_codes = set()
    frame_idx = 0
    detect_count = 0
    fps_ema = 0.0
    det_ms_ema = 0.0
    frame_period = 1.0 / args.target_fps if args.target_fps > 0 else 0.0
    t_start = time.perf_counter()
    t_prev = t_start
    t_next = t_start

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Akış bitti / Kare alınamadı.")
                break

            # --- Algılama (her N karede bir) ---
            if frame_idx % args.decode_interval == 0:
                t0 = time.perf_counter()
                # pyzbar'a renkli kare verilirse sadece mavi kanalı kullanır;
                # gerçek gri tonlama hem daha doğru hem daha hızlıdır
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                last_results = detector.detect(gray)
                det_ms = (time.perf_counter() - t0) * 1000.0
                det_ms_ema = det_ms if det_ms_ema == 0 else 0.9 * det_ms_ema + 0.1 * det_ms
                detect_count += 1

            # --- Loglama: sadece yeni görülen (veya uzun süredir görünmeyen) kodlar ---
            now = time.perf_counter()
            for data, sym_type, _ in last_results:
                if now - last_seen.get(data, -1e9) > RELOG_SECONDS:
                    print(f"[{datetime.now():%H:%M:%S}] Kod: {data} | Tip: {sym_type} "
                          f"| kaynak: {detector.last_variant}")
                last_seen[data] = now
                unique_codes.add(data)

            draw_results(frame, last_results)

            if writer is not None:
                writer.write(frame)

            # --- FPS ölçümü ve önizleme ---
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                inst = 1.0 / dt
                fps_ema = inst if fps_ema == 0 else 0.9 * fps_ema + 0.1 * inst

            if not args.no_display:
                info = (f"FPS: {fps_ema:5.1f} | algilama: {det_ms_ema:5.1f} ms "
                        f"| motor: {engine} | kaynak: {detector.last_variant}")
                cv2.putText(frame, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 4)
                cv2.putText(frame, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 1)
                cv2.imshow("QR/Barkod Algilama", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):  # q veya ESC
                    print("Kullanıcı çıkış yaptı.")
                    break

            frame_idx += 1
            if args.max_frames and frame_idx >= args.max_frames:
                break

            # --- 30 FPS hedefi: döngüyü sabit tempoya bağla ---
            if frame_period:
                t_next += frame_period
                delay = t_next - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    t_next = time.perf_counter()  # geride kaldıysak birikme yapma
    except KeyboardInterrupt:
        print("Ctrl+C ile durduruldu.")
    finally:
        elapsed = time.perf_counter() - t_start
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        print(f"--- Özet: {frame_idx} kare, {elapsed:.1f} sn, ort. {frame_idx / max(elapsed, 1e-9):.1f} FPS, "
              f"{detect_count} algılama çağrısı ---")
        if unique_codes:
            print("Okunan kodlar:", ", ".join(sorted(unique_codes)))


if __name__ == "__main__":
    main()
