#include <math.h>
#include <stdio.h>
#include <string.h>
#include "window.h"

constexpr double kRotateSpeed = 0.005;
constexpr double kZoomSpeed = 0.01;
constexpr double kTranslateSpeed = 0.01;

// Constructor
VisualizerWindow::VisualizerWindow(int x, int y, int w, int h, const std::vector<Fl_Callback*>& draw, const std::vector<void*>& draw_args) : Fl_Gl_Window(x, y, w, h, nullptr) {
  draw_args_ = draw_args;
  draw_callback_ = draw;

  hfov_ = 30;
  near_clip_ = 0.3;
  far_clip_ = 100.0;

  viewer_x_ = 0.0;
  viewer_y_ = 0.0;
  viewer_z_ = 0.0;
  viewer_dist_ = 10.0;
  viewer_phi_ = M_PI / 5.0;   // angle from ground (x-z plane).
  viewer_alpha_ = -M_PI/3.5;  // angle from center's x-y plane.
  viewer_center_x_ = 0.0;
  viewer_center_y_ = 0.0;
  viewer_center_z_ = 0.0;
  viewer_center_yaw_ = 0.0;

  mouse_mode_ = IDLE;
  mouse_speedup_ = 1.0;
  mouse_x_ = 0;
  mouse_y_ = 0;
  mouse_start_x_ = 0;
  mouse_start_y_ = 0;
  left_click_set_ = false;
  right_click_set_ = false;
  mouse_3d_x_ = 0.0;
  mouse_3d_y_ = 0.0;
  mouse_3d_z_ = 0.0;
  mouse_click_mask_ = -1;
  shift_pressed_ = false;

  left_arrow_ = false;
  right_arrow_ = false;
  down_arrow_ = false;
  up_arrow_ = false;

  camera_mode_ = FREE;
  follow_x_ = nullptr;
  follow_y_ = nullptr;
  follow_z_ = nullptr;
  follow_yaw_ = nullptr;

  muted_ = false;
  gl_context_changed_ = false;

  clear_r_ = 0.0;
  clear_g_ = 0.0;
  clear_b_ = 0.0;

  fast_mode_ = false;
}

// Set the horizontal field-of-view.
void VisualizerWindow::SetHorizontalFOV(float hfov) {
  hfov_ = hfov;
}


// Set the clipping planes.
void VisualizerWindow::SetClip(float near, float far) {
  near_clip_ = near;
  far_clip_ = far;
}

void VisualizerWindow::SetFollowCoordinates(double *x, double *y, double *z, double *a) {
  follow_x_ = x;
  follow_y_ = y;
  follow_z_ = z;
  follow_yaw_ = a;
}

void VisualizerWindow::SetCameraParameters(double x, double y, double z, double dist, double phi, double alpha) {
  viewer_center_x_ = x;
  viewer_center_y_ = y;
  viewer_center_z_ = z;
  viewer_dist_ = dist;
  viewer_phi_ = phi;
  viewer_alpha_ = alpha;
}

void VisualizerWindow::GetCameraParameters(double *x, double *y, double *z, double *dist, double *phi, double *alpha) {
  *x = viewer_center_x_;
  *y = viewer_center_y_;
  *z = viewer_center_z_;
  *dist = viewer_dist_;
  *phi = viewer_phi_;
  *alpha = viewer_alpha_;
}

void VisualizerWindow::SetCameraFree() {
  camera_mode_ = FREE;
}

void VisualizerWindow::SetCameraFollowing() {
  camera_mode_ = FOLLOW;
}

bool VisualizerWindow::IsCameraFree() {
  return camera_mode_ == FREE;
}

bool VisualizerWindow::IsCameraFollowing() {
  return camera_mode_ == FOLLOW;
}

void VisualizerWindow::GetClickPosition(double *x, double *y, double *z) {
  *x = mouse_3d_x_;
  *y = mouse_3d_y_;
  *z = mouse_3d_z_;
}

void VisualizerWindow::SetClickMask(unsigned int mask) {
  mouse_click_mask_ = mask;
}

void VisualizerWindow::GetCenterOfAttention(double *x, double *y, double *z) {
  *x = viewer_center_x_;
  *y = viewer_center_y_;
  *z = viewer_center_z_;
}

void VisualizerWindow::Mute() {
  muted_ = true;
  hide();
}

void VisualizerWindow::Unmute() {
  muted_ = false;
  show();
  gl_context_changed_ = true;
}

bool VisualizerWindow::HasContextChanged() {
  bool v = gl_context_changed_;
  gl_context_changed_ = false;
  return v;
}

void VisualizerWindow::SetBackgroundColor(double r, double g, double b) {
  clear_r_ = r;
  clear_g_ = g;
  clear_b_ = b;
}

// Draw the window.
void VisualizerWindow::draw() {
  // Make sure we are using the correct context.
  make_current();

  if (!valid()) {
    // Use the whole window as the context.
    glViewport(0, 0, w(), h());
  }
  glClearColor(clear_r_, clear_g_, clear_b_, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_COLOR_MATERIAL);

  if (muted_) return;

  if (camera_mode_ == FOLLOW && follow_x_ && follow_y_ && follow_z_ && follow_yaw_) {
    viewer_center_x_ = *follow_x_;
    viewer_center_y_ = *follow_y_;
    viewer_center_z_ = *follow_z_;
    viewer_center_yaw_ = *follow_yaw_;
  }
  UpdateCamera();

  for (int i = 0; i < draw_callback_.size(); ++i) {
    (*(draw_callback_[i]))(this, draw_args_[i]);
  }

  return;
};

// Handle events.
int VisualizerWindow::handle(int event) {
  int button = Fl::event_button();
  double x = Fl::event_x();
  double y = Fl::event_y();
  double dx, dy;
  GLfloat dx_x, dx_z, dy_x, dy_z;

  if (muted_) return 1;

  switch (event) {

    case FL_FOCUS:
    case FL_UNFOCUS:
      return 1;

    case FL_KEYDOWN:
      if (Fl::event_key() == FL_Up) {
        up_arrow_ = true;
        return 1;
      } else if (Fl::event_key() == FL_Down) {
        down_arrow_ = true;
        return 1;
      } else if (Fl::event_key() == FL_Left) {
        left_arrow_ = true;
        return 1;
      } else if (Fl::event_key() == FL_Right) {
        right_arrow_ = true;
        return 1;
      }  else return 0;
    case FL_KEYUP:
      if (Fl::event_key() == FL_Up) {
        up_arrow_ = false;
        return 1;
      } else if (Fl::event_key() == FL_Down) {
        down_arrow_ = false;
        return 1;
      } else if (Fl::event_key() == FL_Left) {
        left_arrow_ = false;
        return 1;
      } else if (Fl::event_key() == FL_Right) {
        right_arrow_ = false;
        return 1;
      }  else return 0;

    case FL_PUSH:
      // Check modifier buttons to emulate 3-button mouse on Mac.
      if (Fl::event_state() & FL_ALT)
        button = FL_MIDDLE_MOUSE;
      else if (Fl::event_state() & FL_META)
        button = FL_RIGHT_MOUSE;

      mouse_start_x_ = x;
      mouse_start_y_ = y;
      mouse_x_ = x;
      mouse_y_ = y;

      if (button == FL_MIDDLE_MOUSE) {
        mouse_mode_ = TRANSLATE;
      } else if (button == FL_LEFT_MOUSE) {
        if (mouse_mode_ == ZOOM) mouse_mode_ = TRANSLATE;
        else mouse_mode_ = ROTATE;
      } else {
        if (mouse_mode_ == ROTATE) mouse_mode_ = TRANSLATE;
        else mouse_mode_ = ZOOM;
      }
      return 1;

    case FL_DRAG:
      if (Fl::event_state() & FL_SHIFT)
        mouse_speedup_ = 10.0;
      else
        mouse_speedup_ = 1.0;

      dx = x - mouse_x_;
      dy = y - mouse_y_;

      switch (mouse_mode_) {
      case ZOOM:
        viewer_dist_ += dy * kZoomSpeed * mouse_speedup_;
        if (viewer_dist_ < near_clip_) viewer_dist_ = near_clip_;
        if (viewer_dist_ > far_clip_) viewer_dist_ = far_clip_;
        break;
      case ROTATE:
        viewer_alpha_ += dx * kRotateSpeed * mouse_speedup_;
        if (viewer_alpha_ < -M_PI) viewer_alpha_ += 2.0*M_PI;
        if (viewer_alpha_ > M_PI) viewer_alpha_ -= 2.0*M_PI;
        viewer_phi_ += dy * kRotateSpeed * mouse_speedup_;
        if (viewer_phi_ >= M_PI / 2.0) viewer_phi_ = M_PI/2.0-0.0001; // MAGIC
        if (viewer_phi_ <= 0.0) viewer_phi_ = 0.0001; // MAGIC
        break;
      case TRANSLATE:
        if (camera_mode_ == FREE) {
          dy_x = cos(viewer_phi_) * cos(viewer_center_yaw_ + viewer_alpha_);
          dy_z = cos(viewer_phi_) * sin(viewer_center_yaw_ + viewer_alpha_);
          dx_x = - dy_z;
          dx_z = dy_x;
          viewer_center_x_ += (- dx*dx_x + dy*dy_x) * kTranslateSpeed * mouse_speedup_;
          viewer_center_z_ += (- dx*dx_z + dy*dy_z) * kTranslateSpeed * mouse_speedup_;
        }
        break;
      default:
        break;
      }

      mouse_x_ = x;
      mouse_y_ = y;
      UpdateCamera();
      redraw();

      return 1;

    case FL_RELEASE:
      {
        int d = (mouse_start_x_ - x) * (mouse_start_x_ - x)+(mouse_start_y_ - y) * (mouse_start_y_ - y);
        if (d < 25) {
          if ((Fl::event_state() & mouse_click_mask_) == mouse_click_mask_) {
            SetClickPosition(x, y, button);
            if (Fl::event_state() & FL_SHIFT) {
              shift_pressed_ = true;
            } else {
              shift_pressed_ = false;
            }
          } else  {
            left_click_set_ = false;
            right_click_set_ = false;
          }
        }
        else mouse_mode_ = IDLE;
        return 1;
      }
  }

  return Fl_Gl_Window::handle(event);
}

bool VisualizerWindow::LeftClick() {
  bool r = left_click_set_;
  left_click_set_ = false;
  return r;
}

bool VisualizerWindow::RightClick() {
  bool r = right_click_set_;
  right_click_set_ = false;
  return r;
}

bool VisualizerWindow::LeftArrow() {
  bool r = left_arrow_;
  return r;
}

bool VisualizerWindow::RightArrow() {
  bool r = right_arrow_;
  return r;
}

bool VisualizerWindow::UpArrow() {
  bool r = up_arrow_;
  return r;
}

bool VisualizerWindow::DownArrow() {
  bool r = down_arrow_;
  return r;
}

bool VisualizerWindow::Shift() {
  bool r = shift_pressed_;
  return r;
}

void VisualizerWindow::SetClickPosition(double x, double y, int button) {
  // Make sure we are using the correct context.
  make_current();

  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  GLdouble p1x, p1y, p1z, p2x, p2y, p2z;

  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);

  y = viewport[3] - y;

  gluUnProject((GLfloat)x, (GLfloat)y, 0.0,
               modelview, projection, viewport,
               &p1x, &p1y, &p1z);

  gluUnProject((GLfloat)x, (GLfloat)y, 1.0,
               modelview, projection, viewport,
               &p2x, &p2y, &p2z);

  // Now compute intersection with y = 0.
  if (fabs(p2y - p1y) > 0.000001) {
    double t = -p2y/(p2y - p1y);
    mouse_3d_x_ = p2x + (p2x - p1x)*t;
    mouse_3d_y_ = p2y + (p2y - p1y)*t;
    mouse_3d_z_ = p2z + (p2z - p1z)*t;
    if (button == FL_LEFT_MOUSE) {
      left_click_set_ = true;
      right_click_set_ = false;
    } else if (button == FL_RIGHT_MOUSE) {
      left_click_set_ = false;
      right_click_set_ = true;
    } else {
      left_click_set_ = false;
      right_click_set_ = false;
    }
  }
}

void VisualizerWindow::UpdateCamera() {
  GLfloat dir_x, dir_y, dir_z;
  GLfloat upx = 0.0;
  GLfloat upy = 1.0;
  GLfloat upz = 0.0;

  dir_x = viewer_dist_ *cos(viewer_phi_) * cos(viewer_center_yaw_ + M_PI + viewer_alpha_);
  dir_y = viewer_dist_ *sin(viewer_phi_);
  dir_z = viewer_dist_ *cos(viewer_phi_) * sin(viewer_center_yaw_ + M_PI + viewer_alpha_);

  viewer_x_ = viewer_center_x_ + dir_x;
  viewer_y_ = viewer_center_y_ + dir_y;
  viewer_z_ = viewer_center_z_ + dir_z;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, w(), h());
  gluPerspective(hfov_, (float)w() / (float)h(), near_clip_, far_clip_);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(viewer_x_, viewer_y_, viewer_z_, viewer_center_x_, viewer_center_y_, viewer_center_z_,
            upx, upy, upz);
}
