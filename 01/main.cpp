// =============================================================
// main.cpp  –  FLTK front-end + simulation thread
//
// Window: 800 × 468
// ┌──────────────────────────────────────────────────────────┐
// │ RPM Gauge     │  MAP   │ Torque │  CAM   │  CRANK       │ y=5,  h=190
// │  295×190      │ 120×190│120×190 │120×190 │ 120×190      │
// ├──────────────────────────────────────────────────────────┤
// │  CYL 1    │   CYL 2   │  CYL 3    │  CYL 4             │ y=200,h=110
// ├──────────────────────────────────────────────────────────┤
// │ THROTTLE ─────────────────────────────────────────────── │ y=316,h=27
// │ EXT LOAD ─────────────────────────────────────────────── │ y=349,h=27
// │ [START ENGINE]   firing info       RPM ████████░░░░░░   │ y=381,h=50
// ├──────────────────────────────────────────────────────────┤
// │ status bar                                               │ y=436,h=27
// └──────────────────────────────────────────────────────────┘
//
// Threading:
//   sim thread  – calls g_engine.update() every 1 ms via sleep_until
//   FLTK thread – Fl::repeat_timeout(0.05) reads telemetry under g_sim_mutex
// =============================================================
#include "engine_sim.h"

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Toggle_Button.H>
#include <FL/fl_draw.H>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

// ─── shared simulation state ────────────────────────────────
static EngineSimulator   g_engine;
static std::mutex        g_sim_mutex;
static std::atomic<bool> g_sim_stop{false};
static std::thread       g_sim_thread;

// ═══════════════════════════════════════════════════════════
// RpmGauge  – 180° semicircular analogue gauge
// ═══════════════════════════════════════════════════════════
class RpmGauge : public Fl_Widget {
    float _rpm = 0.0f;
    static constexpr float MAX_RPM = 8000.0f;
public:
    RpmGauge(int x, int y, int w, int h)
        : Fl_Widget(x, y, w, h, nullptr) {}

    void set_rpm(float r) { _rpm = r; redraw(); }

    void draw() override {
        fl_color(fl_rgb_color(28, 28, 35));
        fl_rectf(x(), y(), w(), h());

        // Pivot: bottom-centre, radius uses available height
        const int cx = x() + w() / 2;
        const int cy = y() + h() - 10;
        const int r  = std::min(w() / 2 - 10, h() - 22);

        fl_push_clip(x(), y(), w(), h());

        // ── coloured arc zones (0°=east, CCW positive) ──────
        // 0 rpm → 180° (west),  8k → 0° (east)
        fl_line_style(FL_SOLID, 14);
        fl_color(fl_rgb_color(0,   160,  55));  // green  0–3 k
        fl_arc(cx-r, cy-r, 2*r, 2*r, 112.5, 180.0);
        fl_color(fl_rgb_color(210, 165,   0));  // yellow 3–6 k
        fl_arc(cx-r, cy-r, 2*r, 2*r,  45.0, 112.5);
        fl_color(fl_rgb_color(200,  30,  30));  // red    6–8 k
        fl_arc(cx-r, cy-r, 2*r, 2*r,   0.0,  45.0);
        fl_line_style(0);

        // ── tick marks + kRPM labels ─────────────────────────
        for (int rk = 0; rk <= 8000; rk += 500) {
            float theta = (float)M_PI - ((float)rk / MAX_RPM) * (float)M_PI;
            float ct = cosf(theta), st = sinf(theta);
            bool  major = (rk % 1000 == 0);
            int   ri    = r - (major ? 18 : 11);
            fl_color(FL_WHITE);
            fl_line_style(FL_SOLID, major ? 2 : 1);
            fl_line((int)(cx + ri*ct), (int)(cy - ri*st),
                    (int)(cx +  r*ct), (int)(cy -  r*st));
            if (major) {
                char buf[8]; snprintf(buf, sizeof(buf), "%d", rk/1000);
                float rl = r - 34.0f;
                fl_font(FL_HELVETICA, 10);
                int tw = (int)fl_width(buf);
                fl_draw(buf, (int)(cx + rl*ct) - tw/2,
                             (int)(cy - rl*st) + 4);
            }
        }
        fl_line_style(0);

        // ── title ────────────────────────────────────────────
        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(130, 135, 150));
        int tw = (int)fl_width("ENGINE RPM");
        fl_draw("ENGINE RPM", cx - tw/2, y() + 14);

        // ── needle ───────────────────────────────────────────
        float theta_n = (float)M_PI - (_rpm / MAX_RPM) * (float)M_PI;
        float cn = cosf(theta_n), sn = sinf(theta_n);
        int tx = (int)(cx + (r-8)*cn);
        int ty = (int)(cy - (r-8)*sn);

        // shadow
        fl_color(fl_rgb_color(0,0,0));
        fl_line_style(FL_SOLID, 4);
        fl_line(cx+1, cy+1, tx+1, ty+1);

        // needle
        fl_color(_rpm > 6500 ? fl_rgb_color(255,80,80) : FL_WHITE);
        fl_line_style(FL_SOLID, 3);
        fl_line(cx, cy, tx, ty);
        fl_line_style(0);

        // pivot cap
        fl_color(fl_rgb_color(55, 55, 65));
        fl_pie(cx-8, cy-8, 16, 16, 0, 360);
        fl_color(fl_rgb_color(180, 180, 195));
        fl_pie(cx-5, cy-5, 10, 10, 0, 360);

        // ── digital readout ──────────────────────────────────
        char buf[16]; snprintf(buf, sizeof(buf), "%.0f", _rpm);
        fl_font(FL_HELVETICA_BOLD, 22);
        fl_color(FL_WHITE);
        int dw = (int)fl_width(buf);
        fl_draw(buf, cx - dw/2, cy - 26);
        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(120, 125, 140));
        fl_draw("rpm", cx + dw/2 + 1, cy - 26);

        fl_pop_clip();

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

// ═══════════════════════════════════════════════════════════
// InfoBox  – MAP / Torque / Lambda readout tile
// ═══════════════════════════════════════════════════════════
class InfoBox : public Fl_Widget {
    char      _lbl[32], _unit[24];
    float     _val   = 0.0f;
    int       _prec;
    Fl_Color  _accent;
    bool      _is_lambda;
public:
    InfoBox(int x, int y, int w, int h,
            const char* lbl, const char* unit, int prec,
            Fl_Color accent, bool is_lambda = false)
        : Fl_Widget(x, y, w, h, nullptr),
          _prec(prec), _accent(accent), _is_lambda(is_lambda)
    {
        strncpy(_lbl,  lbl,  31);  _lbl[31]  = '\0';
        strncpy(_unit, unit, 23);  _unit[23] = '\0';
    }
    void set_value(float v) { _val = v; redraw(); }

    void draw() override {
        fl_color(fl_rgb_color(38, 40, 52));
        fl_rectf(x(), y(), w(), h());
        // accent stripe
        fl_color(_accent);
        fl_rectf(x(), y(), w(), 4);
        // label
        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(145, 148, 165));
        int lw = (int)fl_width(_lbl);
        fl_draw(_lbl, x() + w()/2 - lw/2, y() + 22);
        // value
        char buf[32]; snprintf(buf, sizeof(buf), "%.*f", _prec, (double)_val);
        fl_font(FL_HELVETICA_BOLD, 30);
        if (_is_lambda) {
            if      (_val < 0.95f) fl_color(fl_rgb_color(255, 140,  30)); // rich
            else if (_val > 1.05f) fl_color(fl_rgb_color( 80, 180, 255)); // lean
            else                   fl_color(fl_rgb_color( 80, 220, 100)); // stoich
        } else {
            fl_color(FL_WHITE);
        }
        int vw = (int)fl_width(buf);
        fl_draw(buf, x() + w()/2 - vw/2, y() + h()/2 + 18);
        // unit
        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(120, 123, 140));
        int uw = (int)fl_width(_unit);
        fl_draw(_unit, x() + w()/2 - uw/2, y() + h() - 10);

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

// ═══════════════════════════════════════════════════════════
// SignalLED  – CAM / CRANK digital signal indicator
// ═══════════════════════════════════════════════════════════
class SignalLED : public Fl_Widget {
    bool     _state = false;
    Fl_Color _on_col;
    char     _lbl[24];
public:
    SignalLED(int x, int y, int w, int h,
              const char* lbl, Fl_Color on_col)
        : Fl_Widget(x, y, w, h, nullptr), _on_col(on_col)
    {
        strncpy(_lbl, lbl, 23); _lbl[23] = '\0';
    }
    void set_state(bool s) { _state = s; redraw(); }

    void draw() override {
        fl_color(fl_rgb_color(38, 40, 52));
        fl_rectf(x(), y(), w(), h());

        // label at top
        fl_font(FL_HELVETICA_BOLD, 10);
        fl_color(fl_rgb_color(145, 148, 165));
        int lw = (int)fl_width(_lbl);
        fl_draw(_lbl, x() + w()/2 - lw/2, y() + 16);

        // LED circle – largest square that fits below label with margins
        int margin = 14;
        int avail_w = w() - 2*margin;
        int avail_h = h() - 32; // below label, above bottom margin
        int dia = std::min(avail_w, avail_h);
        if (dia < 8) dia = 8;
        int lx = x() + w()/2 - dia/2;
        int ly = y() + 20 + (avail_h - dia)/2;

        // outer ring
        fl_color(fl_rgb_color(60, 62, 75));
        fl_pie(lx-3, ly-3, dia+6, dia+6, 0, 360);
        // LED body
        fl_color(_state ? _on_col : fl_rgb_color(35, 38, 48));
        fl_pie(lx, ly, dia, dia, 0, 360);
        // specular highlight
        if (_state) {
            fl_color(fl_color_average(_on_col, FL_WHITE, 0.50f));
            fl_pie(lx + dia/4, ly + dia/5, dia/3, dia/3, 0, 360);
        }

        // HI / LO label below LED
        fl_font(FL_HELVETICA_BOLD, 11);
        fl_color(_state ? FL_WHITE : fl_rgb_color(90, 93, 108));
        const char* st_lbl = _state ? "HI" : "LO";
        int sw = (int)fl_width(st_lbl);
        fl_draw(st_lbl, x() + w()/2 - sw/2, y() + h() - 6);

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

// ═══════════════════════════════════════════════════════════
// CylBox  – per-cylinder INJ/IGN LEDs + lambda bar
// ═══════════════════════════════════════════════════════════
class CylBox : public Fl_Widget {
    int   _num;
    bool  _inj = false, _ign = false;
    float _lam = 1.0f;
public:
    CylBox(int x, int y, int w, int h, int cyl_num)
        : Fl_Widget(x, y, w, h, nullptr), _num(cyl_num) {}

    void update(bool inj, bool ign, float lam) {
        _inj = inj; _ign = ign; _lam = lam; redraw();
    }

    void draw() override {
        fl_color(fl_rgb_color(40, 42, 54));
        fl_rectf(x(), y(), w(), h());

        // header stripe
        static const Fl_Color hdr[4] = {
            fl_rgb_color(0,   120, 200),
            fl_rgb_color(0,   160,  80),
            fl_rgb_color(180,  90,   0),
            fl_rgb_color(150,   0, 160)
        };
        fl_color(hdr[_num-1]);
        fl_rectf(x(), y(), w(), 4);

        // cylinder title
        char title[12]; snprintf(title, sizeof(title), "CYL  %d", _num);
        fl_font(FL_HELVETICA_BOLD, 12);
        fl_color(FL_WHITE);
        int tw = (int)fl_width(title);
        fl_draw(title, x() + w()/2 - tw/2, y() + 18);

        // ── INJ LED (left) ─────────────────────────────────
        const int dia = 30;
        int lx1 = x() + 14, ly1 = y() + 26;
        fl_color(fl_rgb_color(55, 58, 70));
        fl_pie(lx1-2, ly1-2, dia+4, dia+4, 0, 360);
        fl_color(_inj ? fl_rgb_color(30,230,100) : fl_rgb_color(30,70,40));
        fl_pie(lx1, ly1, dia, dia, 0, 360);
        if (_inj) {
            fl_color(fl_rgb_color(140,255,180));
            fl_pie(lx1+7, ly1+6, 10, 10, 0, 360);
        }
        fl_font(FL_HELVETICA, 9);
        fl_color(_inj ? fl_rgb_color(80,255,140) : fl_rgb_color(80,110,90));
        fl_draw("INJ", lx1+4, ly1+dia+11);

        // ── IGN LED (right of INJ) ──────────────────────────
        int lx2 = x() + 60, ly2 = y() + 26;
        fl_color(fl_rgb_color(55, 58, 70));
        fl_pie(lx2-2, ly2-2, dia+4, dia+4, 0, 360);
        fl_color(_ign ? fl_rgb_color(255,215,0) : fl_rgb_color(70,65,20));
        fl_pie(lx2, ly2, dia, dia, 0, 360);
        if (_ign) {
            fl_color(fl_rgb_color(255,250,180));
            fl_pie(lx2+7, ly2+6, 10, 10, 0, 360);
        }
        fl_font(FL_HELVETICA, 9);
        fl_color(_ign ? fl_rgb_color(255,230,60) : fl_rgb_color(110,100,40));
        fl_draw("IGN", lx2+4, ly2+dia+11);

        // ── lambda value + bar (right side of box) ──────────
        Fl_Color lc;
        if      (_lam < 0.92f) lc = fl_rgb_color(255,120, 30);
        else if (_lam > 1.08f) lc = fl_rgb_color( 80,185,255);
        else                   lc = fl_rgb_color( 60,215, 90);

        fl_font(FL_HELVETICA_BOLD, 11);
        fl_color(lc);
        char lb[20]; snprintf(lb, sizeof(lb), "\u03BB %.3f", (double)_lam);
        int bar_x  = x() + 108;
        int bar_w  = w() - 118;
        fl_draw(lb, bar_x, y() + 52);

        // bar: lambda 0.5 .. 1.5, centre at 1.0
        float clamped = std::max(0.5f, std::min(1.5f, _lam));
        float fill    = (clamped - 0.5f);          // 0..1
        int   bx = bar_x, by = y() + 60, bh = 12;
        fl_color(fl_rgb_color(30,32,42));
        fl_rectf(bx, by, bar_w, bh);
        // centre marker
        fl_color(fl_rgb_color(90,92,105));
        fl_rectf(bx + bar_w/2, by, 1, bh);
        // fill
        fl_color(lc);
        fl_rectf(bx, by, (int)(fill * bar_w), bh);
        fl_color(fl_rgb_color(80,82,95));
        fl_rect(bx, by, bar_w, bh);

        fl_color(fl_rgb_color(65,67,82));
        fl_rect(x(), y(), w(), h());
    }
};

// ═══════════════════════════════════════════════════════════
// RpmBar  – horizontal fill bar with redline marker
// ═══════════════════════════════════════════════════════════
class RpmBar : public Fl_Widget {
    float _rpm = 0.0f;
public:
    RpmBar(int x, int y, int w, int h)
        : Fl_Widget(x, y, w, h, nullptr) {}
    void set_rpm(float r) { _rpm = r; redraw(); }

    void draw() override {
        fl_color(fl_rgb_color(30,32,42));
        fl_rectf(x(), y(), w(), h());

        float frac = std::min(1.0f, _rpm / 8000.0f);
        int   fill = (int)(frac * (w()-2));

        Fl_Color fc = (frac < 0.375f) ? fl_rgb_color(  0,155, 55)
                    : (frac < 0.75f)  ? fl_rgb_color(210,165,  0)
                                      : fl_rgb_color(200, 35, 35);
        fl_color(fc);
        fl_rectf(x()+1, y()+1, fill, h()-2);

        // redline at 6500 RPM
        int rl = (int)((6500.0f/8000.0f) * (w()-2));
        fl_color(fl_rgb_color(220,50,50));
        fl_line_style(FL_DASH, 1);
        fl_line(x()+1+rl, y()+1, x()+1+rl, y()+h()-2);
        fl_line_style(0);

        fl_color(fl_rgb_color(65,67,82));
        fl_rect(x(), y(), w(), h());
    }
};

// ─── widget pointers ────────────────────────────────────────
static RpmGauge*         g_rpm_gauge  = nullptr;
static InfoBox*          g_map_box    = nullptr;
static InfoBox*          g_torque_box = nullptr;
static InfoBox*          g_lambda_box = nullptr;
static SignalLED*        g_cam_led    = nullptr;
static SignalLED*        g_crank_led  = nullptr;
static CylBox*           g_cyl[4]     = {};
static Fl_Value_Slider*  g_throttle   = nullptr;
static Fl_Value_Slider*  g_load       = nullptr;
static Fl_Toggle_Button* g_start_btn  = nullptr;
static RpmBar*           g_rpm_bar    = nullptr;
static Fl_Box*           g_status     = nullptr;

// ─── UI timer: 20 Hz telemetry refresh ──────────────────────
static void ui_timer_cb(void*) {
    EngineSimulator::Telemetry t;
    {
        std::lock_guard<std::mutex> lk(g_sim_mutex);
        g_engine.get_telemetry(t);
    }

    g_rpm_gauge->set_rpm(t.rpm);
    g_map_box->set_value(t.map_kpa);
    g_torque_box->set_value(t.torque_nm);
    g_lambda_box->set_value(t.lambda);
    g_cam_led->set_state(t.cam_pos);
    g_crank_led->set_state(t.crank_pos);
    for (int i = 0; i < 4; ++i)
        g_cyl[i]->update(t.cyl_injecting[i], t.cyl_igniting[i], t.cyl_lambda[i]);
    g_rpm_bar->set_rpm(t.rpm);

    // auto-release starter once engine fires
    if (t.rpm > Engine::STARTER_SPEED_THRESHOLD_RPM && g_start_btn->value()) {
        g_start_btn->value(0);
        g_start_btn->label("START ENGINE");
        g_start_btn->color(fl_rgb_color(0,110,55));
        MockHW::digital_io[Pins::START_BUTTON] = HIGH;
    }

    // status bar
    static char sbuf[200];
    snprintf(sbuf, sizeof(sbuf),
        "  RPM: %5.0f   MAP: %5.1f kPa   Torque: %6.2f Nm"
        "   \u03BB: %.3f   CAM: %s   CRANK: %s   |   %s",
        t.rpm, t.map_kpa, t.torque_nm, t.lambda,
        t.cam_pos   ? "HI" : "LO",
        t.crank_pos ? "HI" : "LO",
        t.rpm > 500.0f ? "\u25CF ENGINE RUNNING" : "\u25CB ENGINE STOPPED");
    g_status->copy_label(sbuf);

    Fl::repeat_timeout(0.05, ui_timer_cb);
}

// ─── slider / button callbacks ───────────────────────────────
static void throttle_cb(Fl_Widget* w, void*) {
    double pct = ((Fl_Value_Slider*)w)->value();           // 0..100
    MockHW::analog_in[0] = (int)(pct * 10.23);             // A0 → 0..1023
}

static void load_cb(Fl_Widget* w, void*) {
    double pct = ((Fl_Value_Slider*)w)->value();           // -100..100
    MockHW::analog_in[1] = (int)((pct/100.0)*511.5 + 511.5); // A1 → 0..1023
}

static void start_cb(Fl_Widget* w, void*) {
    bool pressed = ((Fl_Toggle_Button*)w)->value();
    MockHW::digital_io[Pins::START_BUTTON] = pressed ? LOW : HIGH;
    ((Fl_Toggle_Button*)w)->label(pressed ? "STARTER ON  \u23F5" : "START ENGINE");
    ((Fl_Toggle_Button*)w)->color(
        pressed ? fl_rgb_color(170,75,0) : fl_rgb_color(0,110,55));
    Fl::redraw();
}

// ─── simulation thread: 1 ms ticks ───────────────────────────
static void sim_thread_func() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    while (!g_sim_stop.load(std::memory_order_relaxed)) {
        next += milliseconds(1);
        {
            std::lock_guard<std::mutex> lk(g_sim_mutex);
            g_engine.update();
            g_engine.write_telemetry(); // → stdout every 250 ms
        }
        std::this_thread::sleep_until(next);
    }
}

// ─── convenience: dim label box ─────────────────────────────
static Fl_Box* make_label(int x, int y, int w, int h, const char* txt) {
    Fl_Box* b = new Fl_Box(x, y, w, h, txt);
    b->box(FL_NO_BOX);
    b->labelcolor(fl_rgb_color(145, 148, 165));
    b->labelsize(11);
    b->labelfont(FL_HELVETICA_BOLD);
    b->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    return b;
}

// ═══════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════
int main(int argc, char** argv) {

    // ── mock hardware defaults ───────────────────────────────
    MockHW::digital_io[Pins::START_BUTTON] = HIGH; // not pressed
    MockHW::analog_in[0] = 0;                       // throttle closed
    MockHW::analog_in[1] = 511;                     // no external load
    {
        std::lock_guard<std::mutex> lk(g_sim_mutex);
        g_engine.setup();
    }

    // ── FLTK dark theme ─────────────────────────────────────
    Fl::scheme("gtk+");
    Fl::background (42,  42,  50);
    Fl::background2(52,  52,  62);
    Fl::foreground (215, 215, 220);

    // ── window ───────────────────────────────────────────────
    //
    // Layout constants (all in pixels):
    //
    //  Row 1  y=5,   h=190  — RPM gauge + 4 info tiles
    //  Row 2  y=200, h=110  — 4 cylinder boxes
    //  Row 3  y=316, h=27   — throttle slider
    //  Row 4  y=349, h=27   — load slider
    //  Row 5  y=381, h=50   — start button + RPM bar
    //  Row 6  y=436, h=27   — status bar
    //  Total window height  = 436 + 27 + 5 = 468
    //
    //  Row 1 horizontal:
    //    RPM gauge:  x=5,   w=290
    //    gap:        5 px
    //    4 tiles:    x=300, each tile_w = (800-300-5)/4 - 4 gaps / 4
    //                available = 495 px, 3 gaps of 4 = 12 → tile_w = (495-12)/4 = 120
    //                x coords: 300, 424, 548, 672  (step = 124)
    //
    //  Row 2 horizontal:
    //    4 cyl boxes: x=5, each cyl_w = (800-10-15)/4 = 193, gap=5
    //                 x coords: 5, 203, 401, 599

    static constexpr int WIN_W = 800, WIN_H = 468;

    // Row 1
    static constexpr int R1_Y = 5,   R1_H = 190;
    static constexpr int G_X  = 5,   G_W  = 290;           // RPM gauge
    static constexpr int T0_X = 300, TW   = 120, T_GAP = 4; // tiles
    // tile x[i] = T0_X + i*(TW + T_GAP)
    // tile[0]=300 tile[1]=424 tile[2]=548 tile[3]=672  → 672+120=792 ✓

    // Row 2
    static constexpr int R2_Y = 200, R2_H = 110;
    static constexpr int CYL_GAP = 5;
    static constexpr int CYL_W   = (WIN_W - 10 - 3*CYL_GAP) / 4; // 193

    // Sliders
    static constexpr int LBL_W = 88, SL_Y0 = 316, SL_Y1 = 349, SL_H = 27;
    static constexpr int SL_W  = WIN_W - 5 - LBL_W - 5;  // 702

    // Controls row
    static constexpr int CR_Y = 381, CR_H = 50;

    // Status bar
    static constexpr int ST_Y = 436, ST_H = 27;

    Fl_Double_Window* win =
        new Fl_Double_Window(WIN_W, WIN_H, "4-Cylinder Engine Simulator");
    win->color(fl_rgb_color(35, 35, 43));

    // ── Row 1: RPM gauge ─────────────────────────────────────
    g_rpm_gauge = new RpmGauge(G_X, R1_Y, G_W, R1_H);

    // ── Row 1: MAP, Torque, Lambda, (then CAM, CRANK as SignalLEDs)
    //    We have 4 tile slots.  Use: MAP, Torque, CAM, CRANK.
    //    Lambda gets its own InfoBox in the lambda tile slot.
    //    Arrangement: MAP | Torque | CAM | CRANK   (row 1, tiles 0-3)
    //    Lambda is placed below MAP+Torque in a second row... but we have
    //    only one row here.  Instead we use all 4 slots as tiles and put
    //    lambda INSIDE the status info. Or we make 5 items and reduce width.
    //
    //    Final decision: 5 tiles, each 95px wide, 4 gaps of 4 → 5*95+4*4=491 ✓
    //    x coords: 300, 399, 498, 597, 696  → last tile ends at 796 ✓

    static constexpr int TW5  = 95;   // tile width for 5-tile layout
    static constexpr int TG5  = 4;    // gap
    // x[i] = 300 + i*(TW5+TG5)
    //  0→300  1→399  2→498  3→597  4→696   last right edge: 696+95=791 ✓

    g_map_box = new InfoBox(
        T0_X + 0*(TW5+TG5), R1_Y, TW5, R1_H,
        "MAP", "kPa", 1,
        fl_rgb_color(30, 130, 210));

    g_torque_box = new InfoBox(
        T0_X + 1*(TW5+TG5), R1_Y, TW5, R1_H,
        "TORQUE", "Nm", 2,
        fl_rgb_color(200, 130, 0));

    g_lambda_box = new InfoBox(
        T0_X + 2*(TW5+TG5), R1_Y, TW5, R1_H,
        "LAMBDA", "\u03BB (stoich=1.00)", 3,
        fl_rgb_color(0, 170, 80), /*is_lambda=*/true);

    g_cam_led = new SignalLED(
        T0_X + 3*(TW5+TG5), R1_Y, TW5, R1_H,
        "CAM POS", fl_rgb_color(0, 200, 255));

    g_crank_led = new SignalLED(
        T0_X + 4*(TW5+TG5), R1_Y, TW5, R1_H,
        "CRANK POS", fl_rgb_color(0, 255, 130));

    // ── Row 2: 4 cylinder boxes ──────────────────────────────
    for (int i = 0; i < 4; ++i)
        g_cyl[i] = new CylBox(5 + i*(CYL_W + CYL_GAP), R2_Y, CYL_W, R2_H, i+1);

    // ── Row 3: Throttle slider ───────────────────────────────
    make_label(5, SL_Y0, LBL_W, SL_H, "THROTTLE");
    g_throttle = new Fl_Value_Slider(5 + LBL_W, SL_Y0, SL_W, SL_H);
    g_throttle->type(FL_HOR_FILL_SLIDER);
    g_throttle->range(0.0, 100.0);
    g_throttle->step(1.0);
    g_throttle->value(0.0);
    g_throttle->color(fl_rgb_color(38, 42, 55));
    g_throttle->selection_color(fl_rgb_color(0, 160, 70));
    g_throttle->labelsize(10);
    g_throttle->callback(throttle_cb);
    {
        Fl_Box* b = new Fl_Box(WIN_W - 26, SL_Y0, 22, SL_H, "%");
        b->box(FL_NO_BOX);
        b->labelcolor(fl_rgb_color(140, 143, 158));
        b->labelsize(10);
    }

    // ── Row 4: Load slider ───────────────────────────────────
    make_label(5, SL_Y1, LBL_W, SL_H, "EXT LOAD");
    g_load = new Fl_Value_Slider(5 + LBL_W, SL_Y1, SL_W, SL_H);
    g_load->type(FL_HOR_FILL_SLIDER);
    g_load->range(-100.0, 100.0);
    g_load->step(1.0);
    g_load->value(0.0);
    g_load->color(fl_rgb_color(38, 42, 55));
    g_load->selection_color(fl_rgb_color(180, 100, 0));
    g_load->labelsize(10);
    g_load->callback(load_cb);
    {
        Fl_Box* b = new Fl_Box(WIN_W - 30, SL_Y1, 26, SL_H, "Nm\u00B1");
        b->box(FL_NO_BOX);
        b->labelcolor(fl_rgb_color(140, 143, 158));
        b->labelsize(10);
    }

    // ── Row 5: Start button + info + RPM bar ─────────────────
    g_start_btn = new Fl_Toggle_Button(5, CR_Y, 190, CR_H, "START ENGINE");
    g_start_btn->color(fl_rgb_color(0, 110, 55));
    g_start_btn->selection_color(fl_rgb_color(170, 75, 0));
    g_start_btn->labelcolor(FL_WHITE);
    g_start_btn->labelfont(FL_HELVETICA_BOLD);
    g_start_btn->labelsize(13);
    g_start_btn->callback(start_cb);

    {
        Fl_Box* b = new Fl_Box(200, CR_Y, 200, CR_H,
            "Fire order: 1\u2192 3\u2192 4\u2192 2\n"
            "Hold START to crank");
        b->box(FL_NO_BOX);
        b->labelcolor(fl_rgb_color(105, 108, 125));
        b->labelsize(10);
        b->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    }

    // RPM label + bar
    {
        Fl_Box* b = new Fl_Box(406, CR_Y, 38, 16, "RPM");
        b->box(FL_NO_BOX);
        b->labelcolor(fl_rgb_color(140, 143, 155));
        b->labelsize(10);
        b->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    }
    g_rpm_bar = new RpmBar(406, CR_Y + 17, WIN_W - 406 - 5, CR_H - 21);

    // ── Row 6: Status bar ────────────────────────────────────
    {
        Fl_Box* sep = new Fl_Box(0, ST_Y - 2, WIN_W, 2);
        sep->box(FL_FLAT_BOX);
        sep->color(fl_rgb_color(55, 55, 68));
    }
    g_status = new Fl_Box(0, ST_Y, WIN_W, ST_H, "  Initialising...");
    g_status->box(FL_FLAT_BOX);
    g_status->color(fl_rgb_color(28, 28, 36));
    g_status->labelcolor(fl_rgb_color(130, 200, 130));
    g_status->labelsize(10);
    g_status->labelfont(FL_COURIER);
    g_status->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);

    win->end();
    win->resizable(nullptr);
    win->show(argc, argv);

    // ── start simulation thread ──────────────────────────────
    g_sim_thread = std::thread(sim_thread_func);

    // ── start UI refresh timer ───────────────────────────────
    Fl::add_timeout(0.05, ui_timer_cb);

    int ret = Fl::run();

    // ── clean shutdown ───────────────────────────────────────
    g_sim_stop.store(true);
    g_sim_thread.join();
    return ret;
}
