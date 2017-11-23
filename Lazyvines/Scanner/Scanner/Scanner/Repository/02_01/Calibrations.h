// *** PROCESSING CONFIGURATIONS ***

// Image Processing Settings (P3 Settings)
#define ROW_PIXEL_STRD          1

// 2D Filtering Definitions 
#define LASER_THRESHOLD_R       150
#define LASER_THRESHOLD_G       75
#define LASER_THRESHOLD_B       75
#define BASE_SAFE_HEIGHT		1

// 3D Filtering Definitions
#define REF_FILTER_THRESHOLD	12

// *** HARDWARE CONFIGURATIONS ***

// CALIBRATION CONSTANTS CAM1 (P3 Settings)
#define CB_1_VP_X                 224.75
#define CB_1_VP_Y                 465.05
#define CB_1_CENTER_X             303
#define CB_1_CENTER_Y             238
#define CB_1_BASE_M               -2.6
#define CB_1_BASE_B               1049.4
#define CB_1_SCALE_BASE           1.0
#define CB_1_VVP_X				  0
#define CB_1_VVP_Y				  0
#define CB_1_WALL_EDGE			  261
#define CB_1_ORIENTATION		  -1

// CALIBRATION CONSTANTS CAM2 (P3 Settings)
#define CB_2_VP_X                 813
#define CB_2_VP_Y                 897
#define CB_2_CENTER_X             328
#define CB_2_CENTER_Y             233
#define CB_2_BASE_M               1.3684
#define CB_2_BASE_B               -215.8352
#define CB_2_SCALE_BASE           1.0
#define CB_2_VVP_X				  0
#define CB_2_VVP_Y				  0
#define CB_2_WALL_EDGE			  400
#define CB_2_ORIENTATION		  1

// COMPARAMETRIC PARAMETERS 
#define CB_RAO					  1.57

// Arduino Settings 
#define ARDUINO_PORT            "COM3"
#define REV_STEPS				160

// Camera Settings (Default Res: 640x480)
#define WIDTH                   640
#define HEIGHT                  480

// Structures 
typedef struct {
	int VP_X; 
	int VP_Y; 
	int VVP_X;
	int VVP_Y;
	int BX; 
	int BY; 
	int WALL_EDGE; 
	int ORIENT; 
	double BM; 
	double BB; 
	double Scale; 
}CAM_CB;