#include "swarmsim.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>

#include "gflags/gflags.h"
#include "display/window.h"
#include "util/registerer.h"
#include "util/utils.h"

DEFINE_bool(gui, true, "Whether to display the GUI. Make sure to provide --duration in that case.");
DEFINE_bool(pause, false, "Whether to start the simulation in pause mode.");
DEFINE_bool(showtime, true, "Whether to show the simulation time on the display.");

DEFINE_double(duration, 0.0, "Duration of the run in seconds. The program automatically exits after that amount of time has elapsed.");
DEFINE_double(timestep, 0.064, "The largest admissible timestep in seconds. When using flying robots, make sure to keep this low.");
DEFINE_double(skip, 0.0, "Duration to skip. The simulator simulates this many seconds without any GUI and turns on the GUI after that.");

DEFINE_string(supervisor, "DefaultSupervisor", "The supervisor controller to use.");

// Window properties.
constexpr int kInitWindowWidth = 1280;
constexpr int kInitWindowHeight = 720;

constexpr double kArenaSize = 100.0;
constexpr double kGridSpacing = 0.5;

SimViewer::SimViewer() {
  // Init flags.
  quit_ = false;
  lists_created_ = false;
  draw_grid_ = true;
  pause_ = false;
  nodisplay_ = false;
  fast_ = false;

  // Simulation.
  simulation_mode_ = NORMAL;
  simulation_speedup_ = 1.0;
  old_simulation_speedup_ = 1.0;
  min_step_ = FLAGS_timestep;
  offset_time_ = 0.0;
  current_simulation_time_ = 0.0;
}

bool SimViewer::InitGUI(int cols, int rows) {
  int pause_flag = FL_MENU_TOGGLE;
  if (FLAGS_pause) {
    pause_flag |= FL_MENU_VALUE;
  }

  // The menu.
  Fl_Menu_Item menuitems[] =
    {
      {"&File", 0, 0, 0, FL_SUBMENU},
        {"E&xit", 'q', (Fl_Callback *)SimViewer::OnExit},
        {0},
      {"&Action", 0, 0, 0, FL_SUBMENU},
        {"Pause", ' ', (Fl_Callback*)SimViewer::OnAction, (void*)APP_ACTION_PAUSE, pause_flag},
        {"Fast", FL_CTRL + 'f', (Fl_Callback*)SimViewer::OnAction, (void*)APP_ACTION_FAST, FL_MENU_TOGGLE},
        {"No display", 0, (Fl_Callback*)SimViewer::OnAction, (void*)APP_ACTION_NODISPLAY, FL_MENU_TOGGLE},
        {"Slower", '-', (Fl_Callback*)SimViewer::OnAction, (void*)APP_ACTION_SLOWER},
        {"Faster", '+', (Fl_Callback*)SimViewer::OnAction, (void*)APP_ACTION_FASTER},
        {0},
      {"&Options", 0, 0, 0, FL_SUBMENU},
        {"Show grid", 0, (Fl_Callback*)SimViewer::OnView, (void*)APP_VIEW_GRID, FL_MENU_TOGGLE | FL_MENU_VALUE},
        {0},
      {0},
    };

  // Create top-level window.
  mainwin_.reset(new Fl_Window(cols, rows, "Swarmsim"));
  mainwin_->user_data(this);

  mainwin_->begin();

  // Crate the menu bar.
  menubar_.reset(new Fl_Menu_Bar(0, 0, cols, 30));
  menubar_->user_data(this);
  menubar_->copy(menuitems);

  // Create world window.
  worldwin_.reset(
      new VisualizerWindow(0, 30, cols, rows - 30, {(Fl_Callback*)OnDraw}, {this}));
  worldwin_->SetClickMask(0);

  mainwin_->end();

  // Make world window resizable.
  mainwin_->resizable(worldwin_.get());

  // Exit when clicking the cross.
  mainwin_->callback((Fl_Callback *)SimViewer::OnExit);

  // Set mode.
  if (FLAGS_skip > 0.0) {
    worldwin_->Mute();
    simulation_mode_ = NODISPLAY;
    nodisplay_ = true;
  } else if (FLAGS_pause) {
    old_simulation_speedup_ = simulation_speedup_;
    simulation_speedup_ = 0.0;
    pause_ = true;
  }

  return true;
}

void SimViewer::CloseGUI() {
  return;
}

void SimViewer::OnAction(Fl_Widget *w, int option) {
  SimViewer *self;
  self = (SimViewer *)w->user_data();

  double current_time = GetTime();

  if (option == APP_ACTION_PAUSE) {
    // Modify simulation speedup to match a pause.
    if (!self->pause_) {
      self->old_simulation_speedup_ = self->simulation_speedup_;
      self->simulation_speedup_ = 0.0;
      self->pause_ = true;
    } else {
      self->simulation_speedup_ = self->old_simulation_speedup_;
      self->pause_ = false;
    }
    self->offset_time_ = self->current_simulation_time_;
    self->init_time_ = current_time;
  } else if (option == APP_ACTION_FAST) {
    // Change simulation mode.
    if (!self->fast_) {
      if (!self->nodisplay_)
        self->simulation_mode_ = FAST;
      self->fast_ = true;
      self->worldwin_->EnableFastMode();
    } else {
      if (self->nodisplay_) {
        self->worldwin_->Mute();
        self->simulation_mode_ = NODISPLAY;
      }
      else self->simulation_mode_ = NORMAL;
      self->fast_ = false;
      self->worldwin_->DisableFastMode();
    }
    self->offset_time_ = self->current_simulation_time_;
    self->init_time_ = current_time;
  } else if (option == APP_ACTION_NODISPLAY) {
    if (!self->nodisplay_) {
      self->worldwin_->Mute();
      self->simulation_mode_ = NODISPLAY;
      self->nodisplay_ = true;
    } else {
      self->worldwin_->Unmute();
      if (self->fast_) self->simulation_mode_ = FAST;
      else self->simulation_mode_ = NORMAL;
      self->nodisplay_ = false;
    }
    self->offset_time_ = self->current_simulation_time_;
    self->init_time_ = current_time;
  } else if (option == APP_ACTION_SLOWER) {
    if (self->simulation_mode_ == NORMAL) {
      self->simulation_speedup_ = std::max(0.1, self->simulation_speedup_ - 0.1);
      self->offset_time_ = self->current_simulation_time_;
      self->init_time_ = current_time;
    }
  } else if (option == APP_ACTION_FASTER) {
    if (self->simulation_mode_ == NORMAL) {
      self->simulation_speedup_ = std::min(10.0, self->simulation_speedup_ + 0.1);
      self->offset_time_ = self->current_simulation_time_;
      self->init_time_ = current_time;
    }
  }

  return;
}

void SimViewer::OnView(Fl_Widget *w, int option) {
  SimViewer *self;
  self = (SimViewer *)w->user_data();
  if (option == APP_VIEW_GRID) {
    self->draw_grid_ = !self->draw_grid_;
  }
  return;
}

void SimViewer::OnExit(Fl_Widget *w, int option) {
  SimViewer *self;

  self = (SimViewer *)w->user_data();
  self->quit_ = true;

  return;
}

void SimViewer::OnDraw(VisualizerWindow *win, SimViewer *self) {
  if (!self->lists_created_ || self->worldwin_->HasContextChanged()) {
    self->lists_created_ = true;
    // Lighting.
    glEnable(GL_LIGHTING);
    GLfloat global_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
    glShadeModel(GL_SMOOTH);
    GLfloat specular[] = {1.0f, 1.0f, 1.0f , 1.0f};
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    GLfloat ambient[] = { 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    GLfloat position[] = { -1.5f, 1.0f, -4.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glEnable(GL_LIGHT0);
    glEnable(GL_CULL_FACE);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
  }

  // Information.
  if (FLAGS_showtime) {
    self->DrawInfo();
  }
  self->DrawGrid();

  self->supervisor_->DrawState(self->current_simulation_time_, self->worldwin_.get());
}

void SimViewer::DrawGrid() {
  if (!draw_grid_) return;
  double vx, vy, vz;
  worldwin_->GetCenterOfAttention(&vx, &vy, &vz);
  GLfloat x, z;
  GLfloat xavg = kGridSpacing * (double)((int)(vx / kGridSpacing));
  GLfloat xmin = xavg - kArenaSize;
  GLfloat xmax = xavg + kArenaSize;
  GLfloat zavg = kGridSpacing * (double)((int)(vz / kGridSpacing));
  GLfloat zmin = zavg - kArenaSize;
  GLfloat zmax = zavg + kArenaSize;
  glColor3f(0.1, 0.1, 0.1);
  glBegin(GL_LINES);
  glLineWidth(1.0);
  for (x = xmin; x < xmax + kGridSpacing / 2.0; x += kGridSpacing) {
      glVertex3f(x, 0.0, zmin);
      glVertex3f(x, 0.0, zmax);
  }
  for (z = zmin; z < zmax + kGridSpacing / 2.0; z += kGridSpacing) {
      glVertex3f(xmin, 0.0, z);
      glVertex3f(xmax, 0.0, z);
  }
  glEnd();
}

void SimViewer::DrawInfo() {
  char str[30];

  glDisable(GL_LIGHTING);

  // Draw info text.
  SetOrthographicProjection();
  glPushMatrix();
  glLoadIdentity();

  glColor3f(1.0, 1.0, 1.0);
  snprintf(str, 30, "Time elapsed: %.2f", current_simulation_time_);
  RenderBitmapString(5, 20, 0, GLUT_BITMAP_HELVETICA_18, str);

  glColor3f(1.0, 1.0, 1.0);
  snprintf(str, 30, "%.1f x", (current_simulation_time_-offset_time_)/(GetTime() - init_time_));
  RenderBitmapString(worldwin_->w()-50, worldwin_->h()-20, 0, GLUT_BITMAP_HELVETICA_18, str);

  glPopMatrix();
  ResetPerspectiveProjection();

  glEnable(GL_LIGHTING);
}

void SimViewer::RenderBitmapString(float x, float y, float z, void *font, char *string) {
  char *c;
  glRasterPos3f(x, y, z);
  for (c=string; *c != '\0'; c++) {
    glutBitmapCharacter(font, *c);
  }
}

void SimViewer::SetOrthographicProjection() {
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, worldwin_->w(), 0, worldwin_->h());
  glScalef(1, -1, 1);
  glTranslatef(0, -worldwin_->h(), 0);
  glMatrixMode(GL_MODELVIEW);
}

void SimViewer::ResetPerspectiveProjection() {
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void SimViewer::OnIdle(SimViewer* self) {
  // Make simulation step.
  static bool first = true;
  static double previous_time;

  double current_time = GetTime();

  if (first) {
    first = false;
    previous_time = self->init_time_;
  }

  // Update simulation time.
  double dt;
  if (self->simulation_mode_ != NORMAL && !self->pause_) {
    dt = self->min_step_;
    self->current_simulation_time_ += self->min_step_;
  } else {
    dt = (current_time - previous_time)*self->simulation_speedup_;
    // Fail-safe mechanism.
    if (dt > self->min_step_) {
      // Step time was too big.
      dt = self->min_step_;
      self->current_simulation_time_ += dt;
    } else {
      // Normal update.
      self->current_simulation_time_ = ((current_time - self->init_time_)*self->simulation_speedup_ +
                                       self->offset_time_);
    }
  }

  // Simulate.
  self->Step(dt);

  // Update previous time.
  previous_time = current_time;

  // Redraw.
  self->worldwin_->redraw();

  // Do we need to exit.
  if (FLAGS_duration > 0.0 && self->current_simulation_time_ > FLAGS_duration) {
    self->quit_ = true;
  }

  // Do we stop skipping.
  if (FLAGS_skip > 0.0 && self->current_simulation_time_ > FLAGS_skip) {
    FLAGS_skip = 0.0;  // Avoid re-executing this block.
    self->worldwin_->Unmute();
    if (FLAGS_pause) {
      self->old_simulation_speedup_ = self->simulation_speedup_;
      self->simulation_speedup_ = 0.0;
      self->pause_ = true;
    }
    self->simulation_mode_ = NORMAL;
    self->nodisplay_ = false;
    self->offset_time_ = self->current_simulation_time_;
    self->init_time_ = current_time;
  }
}

bool SimViewer::Init() {
  SeedRand();
  init_time_ = GetTime();

  if (!Registry<Supervisor>::CanNew(FLAGS_supervisor)) {
    std::cerr << "Supervisor type \"" << FLAGS_supervisor << "\" not supported. List of supported types:" << std::endl;
    for (const std::string& k : Registry<Supervisor>::GetKeys()) {
      std::cerr << "  " << k << std::endl;
    }
    return false;
  }
  supervisor_ = Registry<Supervisor>::New(FLAGS_supervisor);
  if (!supervisor_->Init()) {
    return false;
  }
  return true;
}

void SimViewer::Close() {
  supervisor_->Close();
  return;
}

void SimViewer::Step(double dt) {
  supervisor_->Step(current_simulation_time_, dt);
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::unique_ptr<SimViewer> sim(new SimViewer());

  if (!FLAGS_gui) {
    // Run the simulation loop as fast as possible.
    double dt = sim->min_step();
    if (sim->Init() != 0) {
      return -1;
    }

    while (1) {
      sim->IncrementCurrentTimeBy(dt);
      sim->Step(dt);
      if (FLAGS_duration > 0.0 && sim->current_simulation_time() > FLAGS_duration) {
        break;
      }
      usleep(100);
    }

  } else {

    // Init GLUT.
    glutInit(&argc, argv);

    // Initialize gui.
    if (!sim->InitGUI(kInitWindowWidth, kInitWindowHeight + 30)) {
      return -1;
    }

    // Initialize algorithms.
    if (!sim->Init()) {
      sim->CloseGUI();
      return -1;
    }

    // Idle callback.
    Fl::add_idle((void (*) (void*))SimViewer::OnIdle, sim.get());

    // Run.
    sim->Show();
    while (!sim->quit()) {
      Fl::wait();
    }

    std::cout << "Exiting..." << std::endl;

    // Clean up.
    sim->CloseGUI();
  }

  // Clean up.
  sim->Close();

  return 0;
}
