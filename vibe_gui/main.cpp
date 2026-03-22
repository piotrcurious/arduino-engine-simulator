#include "EngineSimulator.h"

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Toggle_Button.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Tabs.H>
#include <FL/Fl_Group.H>
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

// ─── Virtual ECU ────────────────────────────────────────────
class VirtualECU {
public:
    void update(uint64_t now_us) {
        bool crank = MockHW::digital_io[Pins::CRANK_POS].load() == ARD_HIGH;

        if (crank && !prev_crank) {
            uint64_t tooth_dt = now_us - last_tooth_us;
            last_tooth_us = now_us;

            if (tooth_dt > last_tooth_dt * 1.5 && last_tooth_dt > 0) {
                tooth_count = 0;
                sync = true;
            } else {
                tooth_count++;
            }
            last_tooth_dt = tooth_dt;

            if (sync) {
                process_tooth(now_us);
            }
        }
        prev_crank = crank;

        for(int i=0; i<Engine::NUM_CYLINDERS; i++) {
            if (now_us >= inj_end_us[i]) {
                digitalWrite(Pins::INJ[i], ARD_LOW);
            }
            if (now_us >= ign_end_us[i]) {
                digitalWrite(Pins::IGN[i], ARD_LOW);
            }
        }
    }

    void process_tooth(uint64_t now_us) {
        bool cam = MockHW::digital_io[Pins::CAM_POS].load() == ARD_HIGH;
        if (cam) {
             cycle_720_offset = 0;
        } else {
             cycle_720_offset = 360;
        }

        int angle_360 = tooth_count * 6;
        int angle_720 = (angle_360 + cycle_720_offset) % 720;

        for (int i=0; i<Engine::NUM_CYLINDERS; i++) {
            check_and_fire(i, angle_720, (int)Engine::FIRING_START_DEG[i], now_us);
        }
    }

    void check_and_fire(int cyl_idx, int current_angle, int tdc_angle, uint64_t now_us) {
        int inj_angle = (tdc_angle - 360 + 720) % 720;
        if (current_angle == inj_angle && !inj_fired[cyl_idx]) {
            digitalWrite(Pins::INJ[cyl_idx], ARD_HIGH);
            inj_end_us[cyl_idx] = now_us + 10000;
            inj_fired[cyl_idx] = true;
        } else if (current_angle != inj_angle) {
            inj_fired[cyl_idx] = false;
        }

        int ign_angle = (tdc_angle - 12 + 720) % 720;
        if (current_angle == ign_angle && !ign_fired[cyl_idx]) {
             digitalWrite(Pins::IGN[cyl_idx], ARD_HIGH);
             ign_end_us[cyl_idx] = now_us + 1000;
             ign_fired[cyl_idx] = true;
        } else if (current_angle != ign_angle) {
             ign_fired[cyl_idx] = false;
        }
    }

private:
    bool prev_crank = false;
    uint64_t last_tooth_us = 0;
    uint64_t last_tooth_dt = 0;
    int tooth_count = 0;
    bool sync = false;
    int cycle_720_offset = 0;

    uint64_t inj_end_us[Engine::NUM_CYLINDERS] = {0};
    uint64_t ign_end_us[Engine::NUM_CYLINDERS] = {0};
    bool inj_fired[Engine::NUM_CYLINDERS] = {false};
    bool ign_fired[Engine::NUM_CYLINDERS] = {false};
};

// ═══════════════════════════════════════════════════════════
// UI Widgets (RpmGauge, FlywheelWidget, ScopeWidget, InfoBox)
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

        const int cx = x() + w() / 2;
        const int cy = y() + h() - 10;
        const int r  = std::min(w() / 2 - 10, h() - 22);

        fl_push_clip(x(), y(), w(), h());

        fl_line_style(FL_SOLID, 14);
        fl_color(fl_rgb_color(0,   160,  55));
        fl_arc(cx-r, cy-r, 2*r, 2*r, 112.5, 180.0);
        fl_color(fl_rgb_color(210, 165,   0));
        fl_arc(cx-r, cy-r, 2*r, 2*r,  45.0, 112.5);
        fl_color(fl_rgb_color(200,  30,  30));
        fl_arc(cx-r, cy-r, 2*r, 2*r,   0.0,  45.0);
        fl_line_style(0);

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

        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(130, 135, 150));
        int tw = (int)fl_width("ENGINE RPM");
        fl_draw("ENGINE RPM", cx - tw/2, y() + 14);

        float theta_n = (float)M_PI - (_rpm / MAX_RPM) * (float)M_PI;
        float cn = cosf(theta_n), sn = sinf(theta_n);
        int tx = (int)(cx + (r-8)*cn);
        int ty = (int)(cy - (r-8)*sn);

        fl_color(fl_rgb_color(0,0,0));
        fl_line_style(FL_SOLID, 4);
        fl_line(cx+1, cy+1, tx+1, ty+1);

        fl_color(_rpm > 6500 ? fl_rgb_color(255,80,80) : FL_WHITE);
        fl_line_style(FL_SOLID, 3);
        fl_line(cx, cy, tx, ty);
        fl_line_style(0);

        fl_color(fl_rgb_color(55, 55, 65));
        fl_pie(cx-8, cy-8, 16, 16, 0, 360);
        fl_color(fl_rgb_color(180, 180, 195));
        fl_pie(cx-5, cy-5, 10, 10, 0, 360);

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

class FlywheelWidget : public Fl_Widget {
    float _angle = 0.0f;
public:
    FlywheelWidget(int x, int y, int w, int h)
        : Fl_Widget(x, y, w, h, "FLYWHEEL") {
        align(FL_ALIGN_TOP);
    }
    void set_angle(float a) { _angle = a; redraw(); }

    void draw() override {
        fl_color(fl_rgb_color(30, 30, 35));
        fl_rectf(x(), y(), w(), h());

        int cx = x() + w()/2;
        int cy = y() + h()/2;
        int r  = std::min(w(), h()) / 2 - 15;

        fl_color(fl_rgb_color(70, 70, 80));
        fl_pie(cx-r, cy-r, 2*r, 2*r, 0, 360);

        fl_color(fl_rgb_color(40, 40, 45));
        for (int i=0; i<60; ++i) {
            if (i >= 58) continue;
            float a = i * 6.0f;
            float rad = a * (float)M_PI / 180.0f;
            int tx = cx + (int)((r+2) * sinf(rad));
            int ty = cy - (int)((r+2) * cosf(rad));
            int tx2 = cx + (int)((r-5) * sinf(rad));
            int ty2 = cy - (int)((r-5) * cosf(rad));
            fl_line(tx, ty, tx2, ty2);
        }

        float rad = _angle * (float)M_PI / 180.0f;
        int mx = cx + (int)(r * sinf(rad));
        int my = cy - (int)(r * cosf(rad));
        fl_color(FL_RED);
        fl_line_style(FL_SOLID, 3);
        fl_line(cx, cy, mx, my);
        fl_line_style(0);

        fl_color(fl_rgb_color(100, 100, 110));
        fl_pie(cx-10, cy-10, 20, 20, 0, 360);

        char buf[16]; snprintf(buf, sizeof(buf), "%.1f\u00B0", _angle);
        fl_font(FL_HELVETICA, 10);
        fl_color(FL_WHITE);
        int tw = (int)fl_width(buf);
        fl_draw(buf, cx - tw/2, cy + r + 12);

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

class ScopeWidget : public Fl_Widget {
    uint8_t _data[720];
    float _curr_angle = 0;
public:
    ScopeWidget(int x, int y, int w, int h)
        : Fl_Widget(x, y, w, h, "ENGINE CYCLE (720\u00B0)") {
        memset(_data, 0, 720);
        align(FL_ALIGN_TOP);
    }
    void update(const uint8_t* d, float angle) {
        memcpy(_data, d, 720);
        _curr_angle = angle;
        redraw();
    }

    void draw() override {
        fl_color(fl_rgb_color(20, 25, 20));
        fl_rectf(x(), y(), w(), h());

        fl_color(fl_rgb_color(40, 50, 40));
        for (int i=0; i<=720; i+=90) {
            int gx = x() + (i * w()) / 720;
            fl_line(gx, y(), gx, y() + h());
        }

        for (int cyl=0; cyl<4; ++cyl) {
            int base_y_inj = y() + (cyl * 2 + 1) * h() / 9;
            int base_y_ign = y() + (cyl * 2 + 2) * h() / 9;

            char lbl[8];
            fl_font(FL_HELVETICA, 9);
            fl_color(fl_rgb_color(100, 150, 100));
            snprintf(lbl, sizeof(lbl), "INJ%d", cyl+1);
            fl_draw(lbl, x()+2, base_y_inj - 2);
            fl_color(fl_rgb_color(150, 150, 100));
            snprintf(lbl, sizeof(lbl), "IGN%d", cyl+1);
            fl_draw(lbl, x()+2, base_y_ign - 2);

            fl_color(fl_rgb_color(0, 255, 0));
            for (int i=0; i<719; ++i) {
                int x1 = x() + (i * w()) / 720;
                int x2 = x() + ((i+1) * w()) / 720;
                bool s1 = (_data[i] & (1 << cyl)) != 0;
                bool s2 = (_data[i+1] & (1 << cyl)) != 0;
                fl_line(x1, base_y_inj - (s1?10:0), x2, base_y_inj - (s2?10:0));
                if (s1 != s2) fl_line(x2, base_y_inj, x2, base_y_inj-10);
            }

            fl_color(fl_rgb_color(255, 255, 0));
            for (int i=0; i<719; ++i) {
                int x1 = x() + (i * w()) / 720;
                int x2 = x() + ((i+1) * w()) / 720;
                bool s1 = (_data[i] & (1 << (cyl+4))) != 0;
                bool s2 = (_data[i+1] & (1 << (cyl+4))) != 0;
                fl_line(x1, base_y_ign - (s1?10:0), x2, base_y_ign - (s2?10:0));
                if (s1 != s2) fl_line(x2, base_y_ign, x2, base_y_ign-10);
            }
        }

        int cx = x() + ((int)_curr_angle * w()) / 720;
        fl_color(FL_WHITE);
        fl_line(cx, y(), cx, y()+h());

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

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
        fl_color(_accent);
        fl_rectf(x(), y(), w(), 4);
        fl_font(FL_HELVETICA, 10);
        fl_color(fl_rgb_color(145, 148, 165));
        int lw = (int)fl_width(_lbl);
        fl_draw(_lbl, x() + w()/2 - lw/2, y() + 22);
        char buf[32]; snprintf(buf, sizeof(buf), "%.*f", _prec, (double)_val);
        fl_font(FL_HELVETICA_BOLD, 24);
        if (_is_lambda) {
            if      (_val < 0.95f) fl_color(fl_rgb_color(255, 140,  30));
            else if (_val > 1.05f) fl_color(fl_rgb_color( 80, 180, 255));
            else                   fl_color(fl_rgb_color( 80, 220, 100));
        } else {
            fl_color(FL_WHITE);
        }
        int vw = (int)fl_width(buf);
        fl_draw(buf, x() + w()/2 - vw/2, y() + h()/2 + 10);
        fl_font(FL_HELVETICA, 9);
        fl_color(fl_rgb_color(120, 123, 140));
        int uw = (int)fl_width(_unit);
        fl_draw(_unit, x() + w()/2 - uw/2, y() + h() - 8);

        fl_color(fl_rgb_color(65, 65, 80));
        fl_rect(x(), y(), w(), h());
    }
};

// ─── widget pointers ────────────────────────────────────────
static RpmGauge*         g_rpm_gauge  = nullptr;
static InfoBox*          g_map_box    = nullptr;
static InfoBox*          g_torque_box = nullptr;
static InfoBox*          g_lambda_box = nullptr;
static FlywheelWidget*   g_flywheel   = nullptr;
static ScopeWidget*      g_scope      = nullptr;
static Fl_Value_Slider*  g_throttle   = nullptr;
static Fl_Value_Slider*  g_load       = nullptr;
static Fl_Toggle_Button* g_start_btn  = nullptr;
static Fl_Box*           g_status     = nullptr;

static Fl_Value_Slider* ts_inertia;
static Fl_Value_Slider* ts_visc;
static Fl_Value_Slider* ts_static;
static Fl_Value_Slider* ts_brake;
static Fl_Value_Slider* ts_torque;
static Fl_Value_Slider* ts_flow;

// ─── UI timer ───────────────────────────────────────────────
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
    g_flywheel->set_angle(t.flywheel_angle);
    g_scope->update(t.scope_buffer, t.crank_angle_720);

    if (t.rpm > Engine::STARTER_SPEED_THRESHOLD_RPM && g_start_btn->value()) {
        g_start_btn->value(0);
        g_start_btn->label("START ENGINE");
        g_start_btn->color(fl_rgb_color(0,110,55));
        MockHW::digital_io[Pins::START_BUTTON].store(ARD_HIGH);
    }

    static char sbuf[200];
    snprintf(sbuf, sizeof(sbuf),
        "  RPM: %5.0f   MAP: %5.1f kPa   Torque: %6.2f Nm   \u03BB: %.3f   |   %s",
        t.rpm, t.map_kpa, t.torque_nm, t.lambda,
        t.rpm > 500.0f ? "\u25CF ENGINE RUNNING" : "\u25CB ENGINE STOPPED");
    g_status->copy_label(sbuf);

    Fl::repeat_timeout(0.05, ui_timer_cb);
}

// ─── callbacks ──────────────────────────────────────────────
static void throttle_cb(Fl_Widget* w, void*) {
    double pct = ((Fl_Value_Slider*)w)->value();
    MockHW::analog_in[0].store((int)(pct * 10.23));
}

static void load_cb(Fl_Widget* w, void*) {
    double pct = ((Fl_Value_Slider*)w)->value();
    MockHW::analog_in[1].store((int)((pct/100.0)*511.5 + 511.5));
}

static void start_cb(Fl_Widget* w, void*) {
    bool pressed = ((Fl_Toggle_Button*)w)->value();
    MockHW::digital_io[Pins::START_BUTTON].store(pressed ? ARD_LOW : ARD_HIGH);
    ((Fl_Toggle_Button*)w)->label(pressed ? "STARTER ON" : "START ENGINE");
    ((Fl_Toggle_Button*)w)->color(pressed ? fl_rgb_color(170,75,0) : fl_rgb_color(0,110,55));
}

static void tune_cb(Fl_Widget*, void*) {
    EngineSimulator::TunableConfig c;
    c.flywheel_inertia = (float)ts_inertia->value();
    c.viscous_friction = (float)ts_visc->value();
    c.static_friction  = (float)ts_static->value();
    c.engine_brake     = (float)ts_brake->value();
    c.peak_torque      = (float)ts_torque->value();
    c.inj_flow_rate    = (float)ts_flow->value() / 10000.0f;
    {
        std::lock_guard<std::mutex> lk(g_sim_mutex);
        g_engine.set_config(c);
    }
}

// ─── simulation thread ───────────────────────────────────────
static void sim_thread_func() {
    using namespace std::chrono;
    VirtualECU ecu;
    auto start_time = steady_clock::now();

    while (!g_sim_stop.load(std::memory_order_relaxed)) {
        auto now = steady_clock::now();
        uint64_t virtual_us = duration_cast<microseconds>(now - start_time).count();

        {
            std::lock_guard<std::mutex> lk(g_sim_mutex);
            // Run multiple small steps for stability
            for (int i=0; i<5; i++) {
                g_engine.update(0.0002f);
                ecu.update(virtual_us + i*200);
            }
        }
        std::this_thread::sleep_for(milliseconds(1));
    }
}

int main(int argc, char** argv) {
    MockHW::init();
    MockHW::digital_io[Pins::START_BUTTON].store(ARD_HIGH);
    MockHW::analog_in[0].store(0);
    MockHW::analog_in[1].store(511);
    {
        std::lock_guard<std::mutex> lk(g_sim_mutex);
        g_engine.setup();
        g_engine.set_config(EngineSimulator::TunableConfig());
    }

    Fl::scheme("gtk+");
    Fl_Double_Window* win = new Fl_Double_Window(1000, 700, "Vibe Engine Simulator");
    win->color(fl_rgb_color(30, 30, 35));

    Fl_Tabs* tabs = new Fl_Tabs(5, 5, 990, 660);
    {
        Fl_Group* g_dash = new Fl_Group(5, 30, 990, 635, "Dashboard");
        g_dash->begin();

        g_rpm_gauge = new RpmGauge(10, 40, 300, 200);
        g_map_box    = new InfoBox(320, 40, 100, 100, "MAP", "kPa", 1, FL_CYAN);
        g_torque_box = new InfoBox(430, 40, 100, 100, "TORQUE", "Nm", 2, FL_YELLOW);
        g_lambda_box = new InfoBox(320, 150, 210, 90, "LAMBDA", "Stoich=1.0", 3, FL_GREEN, true);

        g_flywheel = new FlywheelWidget(550, 40, 200, 200);
        g_scope = new ScopeWidget(10, 260, 970, 300);

        Fl_Box* bl1 = new Fl_Box(10, 570, 100, 25, "THROTTLE");
        bl1->labelcolor(FL_WHITE);
        g_throttle = new Fl_Value_Slider(110, 570, 380, 25);
        g_throttle->type(FL_HOR_FILL_SLIDER);
        g_throttle->range(0, 100);
        g_throttle->callback(throttle_cb);

        Fl_Box* bl2 = new Fl_Box(500, 570, 100, 25, "EXT LOAD");
        bl2->labelcolor(FL_WHITE);
        g_load = new Fl_Value_Slider(600, 570, 380, 25);
        g_load->type(FL_HOR_FILL_SLIDER);
        g_load->range(-100, 100);
        g_load->callback(load_cb);

        g_start_btn = new Fl_Toggle_Button(10, 610, 200, 40, "START ENGINE");
        g_start_btn->callback(start_cb);
        g_start_btn->color(fl_rgb_color(0, 110, 55));
        g_start_btn->labelcolor(FL_WHITE);

        Fl_Button* force_btn = new Fl_Button(220, 610, 150, 40, "FORCE RUN");
        force_btn->callback([](Fl_Widget*, void*) {
            std::lock_guard<std::mutex> lk(g_sim_mutex);
            g_engine.setRPM(2000);
        });

        Fl_Button* reset_btn = new Fl_Button(380, 610, 150, 40, "RESET SIM");
        reset_btn->callback([](Fl_Widget*, void*) {
            std::lock_guard<std::mutex> lk(g_sim_mutex);
            g_engine.reset();
        });

        g_dash->end();

        Fl_Group* g_tune = new Fl_Group(5, 30, 990, 635, "Tuning");
        g_tune->begin();

        int ty = 50;
        auto add_tuner = [&](const char* lbl, float min, float max, float def, Fl_Value_Slider*& s) {
            Fl_Box* b = new Fl_Box(50, ty, 200, 30, lbl);
            b->labelcolor(FL_WHITE);
            b->align(FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
            s = new Fl_Value_Slider(250, ty, 500, 30);
            s->type(FL_HOR_SLIDER);
            s->range(min, max);
            s->value(def);
            s->callback(tune_cb);
            s->textcolor(FL_WHITE);
            ty += 40;
        };

        EngineSimulator::TunableConfig def;
        add_tuner("Flywheel Inertia (kg*m^2)", 0.01, 1.0, def.flywheel_inertia, ts_inertia);
        add_tuner("Viscous Friction", 0.0001, 0.01, def.viscous_friction, ts_visc);
        add_tuner("Static Friction (Nm)", 0.0, 1.0, def.static_friction, ts_static);
        add_tuner("Engine Brake Factor", 1.0, 10.0, def.engine_brake, ts_brake);
        add_tuner("Peak Torque Scale", 0.1, 5.0, def.peak_torque, ts_torque);
        add_tuner("Inj Flow Rate (x10^-4 kg/s)", 0.1, 20.0, def.inj_flow_rate * 10000.0f, ts_flow);

        g_tune->end();
    }
    tabs->end();

    g_status = new Fl_Box(0, 675, 1000, 25, "Initializing...");
    g_status->box(FL_FLAT_BOX);
    g_status->color(FL_BLACK);
    g_status->labelcolor(FL_GREEN);
    g_status->align(FL_ALIGN_LEFT|FL_ALIGN_INSIDE);

    win->end();
    win->show(argc, argv);

    g_sim_thread = std::thread(sim_thread_func);
    Fl::add_timeout(0.05, ui_timer_cb);

    int ret = Fl::run();
    g_sim_stop.store(true);
    g_sim_thread.join();
    return ret;
}
