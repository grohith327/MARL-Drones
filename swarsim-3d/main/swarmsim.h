#ifndef _SWARMSIM_H
#define _SWARMSIM_H

#include <memory>
#include <unordered_map>

#include <FL/Fl.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Menu_Bar.H>

#include "core/supervisor.h"
#include "core/robot.h"
#include "display/window.h"

enum {
  APP_VIEW_GRID = 0x1000,
  APP_ACTION_PAUSE,
  APP_ACTION_FAST,
  APP_ACTION_NODISPLAY,
  APP_ACTION_SLOWER,
  APP_ACTION_FASTER,
  APP_NEXT_ID,
  APP_PREV_ID,
  APP_TIME_STEP,
};

typedef enum {
  NORMAL,
  FAST,
  NODISPLAY,
} SIMULATION_MODE;

class SimViewer {
 public:
  SimViewer();

  bool InitGUI(int cols, int rows);
  void Show() { mainwin_->show(); }
  void CloseGUI();

  // Action callback
  static void OnAction(Fl_Widget* w, int option);

  // View callback
  static void OnView(Fl_Widget* w, int option);

  // Exit callback
  static void OnExit(Fl_Widget* w, int option);

  // Handle draw callbacks
  static void OnDraw(VisualizerWindow* win, SimViewer* self);

  // Handle idle callbacks
  static void OnIdle(SimViewer* self);

  bool Init();
  void Close();
  void Step(double dt);

  // Accessors.
  double min_step() { return min_step_; }
  double quit() { return quit_; }
  double current_simulation_time() { return current_simulation_time_; }
  void IncrementCurrentTimeBy(double dt) { current_simulation_time_ += dt; };

 public:

  void DrawGrid();
  void DrawInfo();
  void RenderBitmapString(float x, float y, float z, void* font, char* string);
  void SetOrthographicProjection(void);
  void ResetPerspectiveProjection(void);
  void ResetCompleteStack(void);

 private:
  // Top-level window
  std::unique_ptr<Fl_Window> mainwin_;

  // Top menu bar
  std::unique_ptr<Fl_Menu_Bar> menubar_;

  // 3D window
  std::unique_ptr<VisualizerWindow> worldwin_;

  // Robot colors.
  std::unordered_map<int, std::tuple<float, float, float>> robot_type_colors_;

  // Should we quit?
  bool quit_;

  // Are lists already computed?
  bool lists_created_;

  // Display options
  bool draw_grid_;

  // Simulation specifics
  SIMULATION_MODE simulation_mode_;
  double simulation_speedup_;
  double old_simulation_speedup_;
  double min_step_;
  double init_time_;
  double offset_time_;
  double current_simulation_time_;

  // Action options
  bool pause_;
  bool fast_;
  bool nodisplay_;

  // Supervisor.
  std::unique_ptr<Supervisor> supervisor_;
};

#endif
