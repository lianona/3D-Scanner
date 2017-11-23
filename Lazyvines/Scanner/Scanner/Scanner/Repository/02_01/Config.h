// *** APPLICATION LEVEL CONFIGURATIONS ***

// Debug Settings: Recommend File Redirection for DBG_V. 
#define DBG_LOG                 1
#define DBG_VIGOROUS			0

// Illustrator Settings 
#define HOR_ANGLE_DELTA			2.0
#define VERT_ANGLE_DELTA		2.0
#define TIP_DEFAULT				-50
#define VIEW_DEFAULT			225
#define SCALE_BASE				1.5
#define ZOOM_DELTA				0.05
#define MOUSE_DRAG_SENSITIVITY	25

// *** CONSTANTS ***

// Mathematics 
#define PI						3.14159265

// Reporting Macros 
#define ERR						1
#define OKAY					0

// Datasize Definitions
#define MAX_POINTS				524288
#define CMD_MAXLEN              128
#define ANG_STRD_DIGIT          4

// Architecture Datasize Mapping 
#define WORD                unsigned char
#define DWORD               short int
#define LONG                int 

// *** SYNTHETICS ***
#define MIN(x,y)            ((x<y)?(x):(y))
#define MAX(x,y)            ((x<y)?(y):(x))
#define UNINIT              -1
#define checkFloatSanity(x)	((x==INFINITY || x== _FE_DIVBYZERO)?0.0:x)

// Structure Definitions 
typedef struct {
	int x; 
	int y;
}PIXEL;

typedef struct {
	int used;
	int max; 
	PIXEL* pl; 
}PIXELS;

typedef struct {
	float x;
	float y;
	float z;
	int s; 
}PT3D;

typedef struct {
	int used; 
	int max; 
	PT3D* pl; 
}PT3DS;

