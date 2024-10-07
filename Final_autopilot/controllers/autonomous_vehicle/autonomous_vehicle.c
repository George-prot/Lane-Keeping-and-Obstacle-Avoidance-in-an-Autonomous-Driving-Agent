/*
 * Description:   Autonoumous vehicle controller. The specific autopilot implements Lane Keeping, Obstacle Avoidance and Crosswalk Full emergency stop if needed.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/supervisor.h>
#include <webots/camera_recognition_object.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;

// Size of the yellow line angle filter
#define FILTER_SIZE 3


#define MAX_CROSSROAD_DIST 70

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

//static int counter=0;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;

// set target speed
void set_speed(double kmh) {
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  wbu_driver_set_cruising_speed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

//return true if the is a crosswalk detection
//or false if there is not
bool testRecognition() {
  if(!has_camera) return false; //return false if there is no camera enabled
  
  const WbCameraRecognitionObject *rec = wb_camera_recognition_get_objects(camera); 
  int total = wb_camera_recognition_get_number_of_objects(camera); //get camera recognition object
  
  if(rec==NULL) return false;
  
  WbNodeRef cr = wb_supervisor_node_get_from_def("CROSSROAD");	//assign the crossroad recognition for detection
  int crosId = wb_supervisor_node_get_id(cr);
  
  for(int i = 0; i <total; i++){
    if(rec[i].id == crosId) return true;	//return true if a crosswalk is detected
  }
  
  return false; //else return false
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return ((double)sum / FILTER_SIZE)+0.25; //+0.25 to follow the middle of the road lane
  }
}

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double *obstacle_dist) {
  const int HALF_AREA = 25;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 12) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed_ms = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

//calculate the angle needed steer correctly for lane keeping
double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return (KP * (yellow_line_angle) + KI * integral + KD * (diff));
}

int main(int argc, char **argv) {
  wbu_driver_init();

  // check if there is a SICK and a display
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0)
      enable_collision_avoidance = true;
    else if (strcmp(name, "display") == 0)
      enable_display = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
    wb_camera_recognition_enable(camera, TIME_STEP);
  }

  // SICK sensor
  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("Sick LMS 291");
    wb_lidar_enable(sick, TIME_STEP);
    sick_width = wb_lidar_get_horizontal_resolution(sick);
    sick_range = wb_lidar_get_max_range(sick);
    sick_fov = wb_lidar_get_fov(sick);
  }

  // initialize gps
  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  // initialize display (speedometer)
  if (enable_display) {
    display = wb_robot_get_device("display");
    speedometer_image = wb_display_image_load(display, "speedometer.png");
  }

  // start engine
  if (has_camera)
    set_speed(30.0);  // km/h
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

  // main loop
  while (wbu_driver_step() != -1) {

    static int i = 0;

    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      // read sensors
      const unsigned char *camera_image = NULL;
      const float *sick_data = NULL;
      if (has_camera)
        camera_image = wb_camera_get_image(camera);
      if (enable_collision_avoidance)
        sick_data = wb_lidar_get_range_image(sick);

      if (has_camera) {
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));
        double obstacle_dist;
        double obstacle_angle;
        if (enable_collision_avoidance)
          obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
        else {
          obstacle_angle = UNKNOWN;
          obstacle_dist = 0;
        }


        set_speed(30.0);//set driving speed when nothing is detected
        if(testRecognition() && obstacle_angle != UNKNOWN){ //full emergency stop if crosswalk is detected AND an obstacle is moving on it
          set_speed(0.0);
          wbu_driver_set_brake_intensity(1.0);
        }
        else if (enable_collision_avoidance && obstacle_angle != UNKNOWN) {
        // avoid obstacles and follow yellow line
          // an obstacle has been detected
          wbu_driver_set_brake_intensity(0.0);
          // compute the steering angle required to avoid the obstacle
          double obstacle_steering = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 2.5)
            obstacle_steering = steering_angle + (obstacle_angle - 0.45) / obstacle_dist;
          else if (obstacle_angle > -2.5)
            obstacle_steering = steering_angle + (obstacle_angle + 0.45) / obstacle_dist;
          double steer = steering_angle;
          // if we see the line we determine the best steering angle to both avoid obstacle and follow the line
          if (yellow_line_angle != UNKNOWN) {
            const double line_following_steering = applyPID(yellow_line_angle);
            if (obstacle_steering > 0 && line_following_steering > 0){
            
              steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
            
            }else if (obstacle_steering < 0 && line_following_steering < 0){
            
              steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
            
            }else if (line_following_steering > 0 && obstacle_steering < 0){
            
              steer = obstacle_steering + line_following_steering/1.8;
            
            }else{
              steer = line_following_steering/1.8 + obstacle_steering;
            }
          } else {
            PID_need_reset = true;
          }
          // apply the computed required angle
          set_steering_angle(steer);

        } else if (yellow_line_angle != UNKNOWN) {
          // no obstacle has been detected, simply follow the line
            wbu_driver_set_brake_intensity(0.0);
          
             set_steering_angle(applyPID(yellow_line_angle));
        } else {
          // no obstacle has been detected but we lost the line => we brake and hope to find the line again
          wbu_driver_set_brake_intensity(0.4);
          PID_need_reset = true;
        }
      }

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display();
    }

    ++i;
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}
