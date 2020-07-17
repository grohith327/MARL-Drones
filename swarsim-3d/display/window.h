#ifndef WINDOW_H
#define WINDOW_H

#include <vector>

#include <FL/Fl.H>
#include <FL/gl.h>
#include <FL/Fl_Gl_Window.H>

#ifdef MAC
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

class VisualizerWindow : public Fl_Gl_Window {
  typedef enum {IDLE, ZOOM, ROTATE, TRANSLATE} MOUSE_MODE;
  typedef enum {FREE, FOLLOW} CAMERA_MODE;

 public:
  VisualizerWindow(int x, int y, int w, int h, const std::vector<Fl_Callback*>& draw, const std::vector<void*>& draw_args);

  void SetHorizontalFOV(float hfov);
  void SetClip(float near, float far);
  void SetFollowCoordinates(double* x, double* y, double* z, double* a);
  void GetCenterOfAttention(double* x, double* y, double* z);
  void SetCameraParameters(double x, double y, double z, double dist, double phi, double alpha);
  void GetCameraParameters(double* x, double* y, double* z, double* dist, double* phi, double* alpha);
  void SetCameraFree();
  void SetCameraFollowing();
  bool IsCameraFree();
  bool IsCameraFollowing();
  void GetClickPosition(double* x, double* y, double* z);
  void SetClickMask(unsigned int mask);
  bool LeftClick();
  bool RightClick();
  bool LeftArrow();
  bool RightArrow();
  bool UpArrow();
  bool DownArrow();
  bool Shift();

  void Mute();
  void Unmute();
  bool HasContextChanged();
  void SetBackgroundColor(double r, double g, double b);

  void EnableFastMode() { fast_mode_ = true; }
  void DisableFastMode() { fast_mode_ = false; }
  bool IsFastModeEnabled() const { return fast_mode_; }

 protected:
  void draw() override;
  int handle(int event) override;

 private:
  void SetClickPosition(double x, double y, int button);
  void UpdateCamera();

  // Callback functions
  std::vector<void*> draw_args_;
  std::vector<Fl_Callback*> draw_callback_;

  // mouse
  MOUSE_MODE mouse_mode_;
  double mouse_speedup_;
  int mouse_x_;
  int mouse_y_;
  int mouse_start_x_;
  int mouse_start_y_;
  bool left_click_set_;
  double mouse_3d_x_;
  double mouse_3d_y_;
  double mouse_3d_z_;
  bool right_click_set_;
  unsigned int mouse_click_mask_;
  bool shift_pressed_;

  // keyboard
  bool up_arrow_;
  bool down_arrow_;
  bool left_arrow_;
  bool right_arrow_;

  // camera
  GLfloat viewer_x_;
  GLfloat viewer_y_;
  GLfloat viewer_z_;
  GLfloat viewer_dist_;
  GLfloat viewer_phi_;       // angle from ground (x-z plane).
  GLfloat viewer_alpha_;     // angle from center's x-y plane.
  GLfloat viewer_center_x_;
  GLfloat viewer_center_y_;
  GLfloat viewer_center_z_;
  GLfloat viewer_center_yaw_;

  CAMERA_MODE camera_mode_;
  double* follow_x_;
  double* follow_y_;
  double* follow_z_;
  double* follow_yaw_;

  // Field of view
  float hfov_;

  // Near and far clipping distances
  float near_clip_;
  float far_clip_;

  // Mute input
  bool muted_;
  bool gl_context_changed_;

  // Clear color
  double clear_r_;
  double clear_g_;
  double clear_b_;

  // Should things that draw on here be drawing fast.
  bool fast_mode_;
};

#endif  // WINDOW_H
