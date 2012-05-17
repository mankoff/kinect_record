/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/time.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <libfreenect/libfreenect-registration.h>

#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>

#define DEPTH_MAX_METRIC_VALUE 10000
#define DEPTH_MAX_RAW_VALUE 2048
#define DEPTH_NO_RAW_VALUE 2047
#define DEPTH_NO_MM_VALUE 0

#define DEPTH_X_OFFSET 1
#define DEPTH_Y_OFFSET 1
#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint8_t *depth_mid, *depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;

// KDM additions
bool RECORD_DN=0;
char *out_dir=0;
//typedef void dumpworld(uint32_t timestamp, void *wx, void *wy, void* wz, int data_size);

FILE *index_fp = NULL;
uint32_t last_timestamp = 0;
#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480

char *depth_name = 0;
char *rgb_name = 0;

FILE *depth_stream=0;
FILE *rgb_stream=0;



double get_time()
{
	struct timeval cur;
	gettimeofday(&cur, NULL);
	return cur.tv_sec + cur.tv_usec / 1000000.;
}


FILE *open_dump(char type, double cur_time, uint32_t timestamp, int data_size, const char *extension)
{
  char *fn = (char *)malloc(strlen(out_dir) + 50);
  sprintf(fn, "%c-%f-%u.%s", type, cur_time, timestamp, extension);
  fprintf(index_fp, "%s\n", fn);
  sprintf(fn, "%s/%c-%f-%u.%s", out_dir, type, cur_time, timestamp, extension);
  FILE* fp = fopen(fn, "w");
  if (!fp) {
	printf("Error: Cannot open file [%s]\n", fn);
	exit(1);
  }
  printf("%s\n", fn);
  free(fn);
  return fp;
}

void dump_depth(FILE *fp, void *data, int data_size)
{
	fprintf(fp, "P5 %d %d 65535\n", FREENECT_FRAME_W, FREENECT_FRAME_H);
	fwrite(data, data_size, 1, fp);
}

void dump_rgb(FILE *fp, void *data, int data_size)
{
	fprintf(fp, "P6 %d %d 255\n", FREENECT_FRAME_W, FREENECT_FRAME_H);
	fwrite(data, data_size, 1, fp);
}

void dump(char type, uint32_t timestamp, void *data, int data_size)
{
  // timestamp can be at most 10 characters, we have a few extra
  double cur_time = get_time();
  FILE *fp;
  last_timestamp = timestamp;
  switch (type) {
  case 'd':
	fp = open_dump(type, cur_time, timestamp, data_size, "pgm");
	dump_depth(fp, data, data_size);
	fclose(fp);
	break;
  case 'r':
	fp = open_dump(type, cur_time, timestamp, data_size, "ppm");
	dump_rgb(fp, data, data_size);
	fclose(fp);
	break;
  case 'a':
	fp = open_dump(type, cur_time, timestamp, data_size, "dump");
	fwrite(data, data_size, 1, fp);
	fclose(fp);
	break;
  }
}

void snapshot_accel(freenect_device *dev)
{
  if (!last_timestamp)
	return;
  freenect_raw_tilt_state* state;
  freenect_update_tilt_state(dev);
  state = freenect_get_tilt_state(dev);
  dump('a', last_timestamp, state, sizeof *state);
}


FILE *open_index(const char *fn)
{
  FILE *fp;
  fp = fopen(fn, "w");
  if (!fp) {
	printf("Error: Cannot open file [%s]\n", fn);
	return 0;
  }
  return fp;
}

void DrawGLScene()
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// When using YUV_RGB mode, RGB frames only arrive at 15Hz, so we shouldn't force them to draw in lock-step.
	// However, this is CPU/GPU intensive when we are receiving frames in lockstep.
	if (current_format == FREENECT_VIDEO_YUV_RGB) {
		while (!got_depth && !got_rgb) {
			pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
		}
	} else {
		while ((!got_depth || !got_rgb) && requested_format != current_format) {
			pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
		}
	}

	if (requested_format != current_format) {
		pthread_mutex_unlock(&gl_backbuf_mutex);
		return;
	}

	uint8_t *tmp;

	if (got_depth) {
		tmp = depth_front;
		depth_front = depth_mid;
		depth_mid = tmp;
		got_depth = 0;
	}
	if (got_rgb) {
		tmp = rgb_front;
		rgb_front = rgb_mid;
		rgb_mid = tmp;
		got_rgb = 0;
	}

	pthread_mutex_unlock(&gl_backbuf_mutex);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_front);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB)
		glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_front);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, rgb_front+640*4);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 0); glVertex3f(1280,0,0);
	glTexCoord2f(1, 1); glVertex3f(1280,480,0);
	glTexCoord2f(0, 1); glVertex3f(640,480,0);
	glEnd();

	glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		die = 1;
		pthread_join(freenect_thread, NULL);
		glutDestroyWindow(window);
		free(depth_mid);
		free(depth_front);
		free(rgb_back);
		free(rgb_mid);
		free(rgb_front);
		// Not pthread_exit because OSX leaves a thread lying around and doesn't exit
		exit(0);
	}
	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's') {
		freenect_angle = 0;
	}
	if (key == 'f') {
		if (requested_format == FREENECT_VIDEO_IR_8BIT)
			requested_format = FREENECT_VIDEO_RGB;
		else if (requested_format == FREENECT_VIDEO_RGB)
			requested_format = FREENECT_VIDEO_YUV_RGB;
		else
			requested_format = FREENECT_VIDEO_IR_8BIT;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == '1') {
		freenect_set_led(f_dev,LED_GREEN);
	}
	if (key == '2') {
		freenect_set_led(f_dev,LED_RED);
	}
	if (key == '3') {
		freenect_set_led(f_dev,LED_YELLOW);
	}
	if (key == '4') {
		freenect_set_led(f_dev,LED_BLINK_GREEN);
	}
	if (key == '5') {
		// 5 is the same as 4
		freenect_set_led(f_dev,LED_BLINK_GREEN);
	}
	if (key == '6') {
		freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		freenect_set_led(f_dev,LED_OFF);
	}
	freenect_set_tilt_degs(f_dev,freenect_angle);
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glEnable(GL_TEXTURE_2D);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_FLAT);

	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	ReSizeGLScene(Width, Height);
}

void *gl_threadfunc(void *arg)
{
	printf("GL thread\n");

	glutInit(&g_argc, g_argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(1280, 480);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("LibFreenect");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(1280, 480);

	glutMainLoop();

	return NULL;
}

uint16_t t_gamma[2048];
//uint16_t t_gamma[10000];

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
  
  dump('d', timestamp, v_depth, freenect_get_current_depth_mode(dev).bytes);
  
  int i;
  uint16_t *depth = (uint16_t*)v_depth;
  
  
  pthread_mutex_lock(&gl_backbuf_mutex);
  for (i=0; i<640*480; i++) {
	int pval = t_gamma[depth[i]];
	int lb = pval & 0xff;
	switch (pval>>8) {
	case 0:
	  depth_mid[3*i+0] = 255;
	  depth_mid[3*i+1] = 255-lb;
	  depth_mid[3*i+2] = 255-lb;
	  break;
	case 1:
	  depth_mid[3*i+0] = 255;
	  depth_mid[3*i+1] = lb;
	  depth_mid[3*i+2] = 0;
	  break;
	case 2:
	  depth_mid[3*i+0] = 255-lb;
	  depth_mid[3*i+1] = 255;
	  depth_mid[3*i+2] = 0;
	  break;
	case 3:
	  depth_mid[3*i+0] = 0;
	  depth_mid[3*i+1] = 255;
	  depth_mid[3*i+2] = lb;
	  break;
	case 4:
	  depth_mid[3*i+0] = 0;
	  depth_mid[3*i+1] = 255-lb;
	  depth_mid[3*i+2] = 255;
	  break;
	case 5:
	  depth_mid[3*i+0] = 0;
	  depth_mid[3*i+1] = 0;
	  depth_mid[3*i+2] = 255-lb;
	  break;
	default:
	  depth_mid[3*i+0] = 0;
	  depth_mid[3*i+1] = 0;
	  depth_mid[3*i+2] = 0;
	  break;
	}
  }
  got_depth++;
  pthread_cond_signal(&gl_frame_cond);
  pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
  dump('r', timestamp, rgb, freenect_get_current_video_mode(dev).bytes);
  
  pthread_mutex_lock(&gl_backbuf_mutex);
  
  // swap buffers
  assert (rgb_back == rgb);
  rgb_back = rgb_mid;
  freenect_set_video_buffer(dev, rgb_back);
  rgb_mid = (uint8_t*)rgb;
  
  got_rgb++;
  pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
    //int accelCount = 0;

	//freenect_set_tilt_degs(f_dev,freenect_angle); // KDM: don't tilt when recording starts
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));

	/* KDM */
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

	freenect_set_video_buffer(f_dev, rgb_back);

	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

	printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");

	while (!die && freenect_process_events(f_ctx) >= 0) {

	  /*
		//Throttle the text output
		if (accelCount++ >= 2000)
		{
			accelCount = 0;
			freenect_raw_tilt_state* state;
			freenect_update_tilt_state(f_dev);
			state = freenect_get_tilt_state(f_dev);
			double dx,dy,dz;
			freenect_get_mks_accel(state, &dx, &dy, &dz);
			printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
			fflush(stdout);
		}

		if (requested_format != current_format) {
			freenect_stop_video(f_dev);
			freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
			freenect_start_video(f_dev);
			current_format = requested_format;
		}
	  */
	  snapshot_accel(f_dev);
	}

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
	return NULL;
}

void usage()
{
  printf("Records the raw Kinect sensor data to a directory.\n"
		 "Also displays data while recording.\n"
		 "Combination of libfrenect 'record' and 'glview'\n");
  printf("Usage:\n");
	printf("  kinect_record <folder>\n");
	exit(0);
}

int main(int argc, char **argv)
{

  int c=1;
  if (argc != 2) {
	usage();
  }
  out_dir = argv[c];
  mkdir(out_dir, S_IRWXU | S_IRWXG | S_IRWXO);
  
  depth_mid = (uint8_t*)malloc(640*480*3);
  depth_front = (uint8_t*)malloc(640*480*3);
  rgb_back = (uint8_t*)malloc(640*480*3);
  rgb_mid = (uint8_t*)malloc(640*480*3);
  rgb_front = (uint8_t*)malloc(640*480*3);
  
  int maxval=2048;
  int i;
  /* KDM: make it look nice for both registered and DN values */
  for (i=0; i<maxval; i++) {
	float v = i/float(maxval);
	v = powf(v, 3)* 6;
	t_gamma[i] = v*6*(256);
	
	// KDM: make it clear when things are too close to the camera
	if (t_gamma[i] == 0) {
	  t_gamma[i] = maxval;
	}
	
  }
  
  g_argc = argc;
  g_argv = argv;
  
  if (freenect_init(&f_ctx, NULL) < 0) {
	printf("freenect_init() failed\n");
	return 1;
  }
  
  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
  
  int nr_devices = freenect_num_devices (f_ctx);
  printf ("Number of devices found: %d\n", nr_devices);
  
  
  int user_device_number = 0;
  if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
	printf("Could not open device\n");
	return 1;
  }
  
  int res;
  res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
  if (res) {
	printf("pthread_create failed\n");
	return 1;
  }

  char *fn = (char *)malloc(strlen(out_dir) + 50);
  sprintf(fn, "%s/INDEX.txt", out_dir);
  index_fp = open_index(fn);
  if (!index_fp) return 1;


  // OS X requires GLUT to run on the main thread
  gl_threadfunc(NULL);

  
  return 0;
}
