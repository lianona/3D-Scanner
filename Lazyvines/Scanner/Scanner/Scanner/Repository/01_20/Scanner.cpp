/************************************************************************************************************************

3D Rotational Scanner Main Executable: SEG Scanner
Version 4.2, Updated: 01/16/2015
S. Yang, E. Miao, G. Guan.  

This program is the main executable for the 3D Scanner Project. Includes:
* Config.h:			Software configurations, toggle output messages, user interfaces. etc.
* Calibration.h:	Hardware configurations and calibration data. Image resolutions, ports. etc. 

*************************************************************************************************************************/


/********************************************** Includes **********************************************/

// Include the standard C++ headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <math.h>

// Include Graphical Support Libraries
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// Include Support Headers and Functions
#include "Config.h"
#include "Calibrations.h"

/********************************************** Global Variables **********************************************/
float tip_angle  =	TIP_DEFAULT;
float view_angle =	VIEW_DEFAULT;
float zoomfactor =  SCALE_BASE;
int DISP_A = 1; 
int DISP_B = 1; 
int LOAD_MODE = 0;
CAM_CB CB_A, CB_B; 
PIXELS PX_A, PX_B; 
PT3DS P3D_A, P3D_B;

/********************************************** Basic Functions **********************************************/

// Gracefully Exit Program in case of Error. 
void errorExit(const char* prompt){
	printf("Error occurred: %s. \nProgram completed with error. Press any key to exit. \n", prompt);
	getchar();
	exit(ERR);
}

/********************************************** FRAMER **********************************************/
// Rotates the Motorized Dish for Constant Angular Slices
// Takes picture(s) and stores it in image directory(s) 

// Initializes the framer function. 
void framer(int step_count, HANDLE hSerial){

	// Capture and Store Images 
	// TODO: Alter Open-source code to make picture capture go faster for our application. 
	char    cmd1[CMD_MAXLEN] = "";
	char    cmd2[CMD_MAXLEN] = "";
	char    buffer[ANG_STRD_DIGIT];
	char*   cmd_prefix_1 = "CommCam /devnum 1 /filename Images_A\\";
	char*   cmd_prefix_2 = "CommCam /devnum 2 /filename Images_B\\";
	char*   cmd_postfix = ".bmp 2> nul";

	// Rotates the object disk. 
	if (DBG_LOG) printf("Rotating Disk %d/%d... \n", step_count, REV_STEPS);
	char MOTOR_MV_CMD = '1';
	if (!WriteFile(hSerial, &MOTOR_MV_CMD, 1, NULL, NULL)){
		errorExit("Error sending motor move command to Arduino.");
	}

	// Takes a picture of the object. 
	sprintf_s(buffer, "%d", step_count);
	cmd1[0] = '\0';
	cmd2[0] = '\0';
	strcat_s(cmd1, cmd_prefix_1);
	strcat_s(cmd2, cmd_prefix_2);
	strcat_s(cmd1, buffer);
	strcat_s(cmd2, buffer);
	strcat_s(cmd1, cmd_postfix);
	strcat_s(cmd2, cmd_postfix);

	// Calling CommandCam Application
	if (DBG_LOG) printf("Calling: %s\n", cmd1);
	system(cmd1);
	if (DBG_LOG) printf("Calling: %s\n", cmd2);
	system(cmd2);
}

/********************************************** EXTRACTOR **********************************************/

// Extraction of 2D Points from BMP Images (Frames)
void ExtractPoints(int step, int CAM_ID){

	// Open image file and access images
	char fname1[CMD_MAXLEN]; 
	char fname2[CMD_MAXLEN]; 
	char fbuffer[CMD_MAXLEN];
	sprintf_s(fbuffer, "%d.bmp", step);
	fname1[0] = '\0';
	fname2[0] = '\0'; 
	strcat_s(fname1, "Images_A\\");
	strcat_s(fname2, "Images_B\\");
	strcat_s(fname1, fbuffer); 
	strcat_s(fname2, fbuffer);

	char* fname; 
	fname = (CAM_ID == 1) ? fname1 : fname2; 

	// Extract Image Corresponding to CAM1
	if (DBG_LOG)printf("Extracting Points from: %s...\n", fname);
	FILE* fptr;
	fopen_s(&fptr, fname, "rb");
	if (!fptr){ errorExit("Error occured while opening image file."); }

	// Access File +BMP Headers
	unsigned char header[54];
	fread(header, sizeof(unsigned char), 54, fptr);

	// Object image object's width, height, and image body offset value. 
	int w = *(int*)&header[18];
	int h = *(int*)&header[22];
	if (w != WIDTH || h != HEIGHT){ errorExit("Image dimensions inconsistent with calibration settings."); }

	int data_offset = *(int*)&header[10];
	int offset_max = 3 * w*h;
	unsigned char* data = (unsigned char*)malloc(offset_max*sizeof(unsigned char));

	// Advance to image body and load image, close file when done. 
	// Create temporary buffer to ignore data padding when loading. 
	fseek(fptr, data_offset, 0);

	int rc, cc;
	double wd = w;
	int row_padding = ceil(wd * 3 / 4) * 4 - w * 3;

	unsigned char* buffer = (unsigned char*)malloc(row_padding*sizeof(unsigned char));
	for (rc = 0; rc<h; rc++){
		fread(data + rc * 3 * w, sizeof(unsigned char), w * 3, fptr);
		fread(buffer, sizeof(unsigned char), row_padding, fptr);
	}
	fclose(fptr);

	// Goes through each ROW_PIXEL_STRD rows and get the average of the EVERY laser segment spotted.
	// The generated result is then written back to results array.  

	int B, G, R;
	PIXEL* pxl_ptr = (CAM_ID == 1) ? PX_A.pl : PX_B.pl; 
	int* used_ptr = (CAM_ID == 1) ? &PX_A.used : &PX_B.used; 
	int* max_ptr = (CAM_ID == 1) ? &PX_A.max : &PX_B.max; 
	pxl_ptr += *used_ptr; 

	for (rc = 0; rc<h; rc += ROW_PIXEL_STRD){
		int begin_track_idx = UNINIT;
		int end_track_idx = UNINIT;
		for (cc = 0; cc<w * 3; cc += 3){
			// Look at each pixel whether they satisfy colour intensity requirements. 
			B = (unsigned char)data[rc*w * 3 + cc + 0] > LASER_THRESHOLD_B;
			G = (unsigned char)data[rc*w * 3 + cc + 1] > LASER_THRESHOLD_G;
			R = (unsigned char)data[rc*w * 3 + cc + 2] > LASER_THRESHOLD_R;

			if ((begin_track_idx == UNINIT) && B&&G&&R)       { begin_track_idx = cc / 3; }
			else if (begin_track_idx != UNINIT && !(B&&G&&R)) { 
				end_track_idx = cc / 3; 
				int avg_pxl = (begin_track_idx + end_track_idx) / 2; 

				// Dynamic Heap Management (Simple Implementation)
				if (*used_ptr == *max_ptr){
					errorExit("Point Quantity Overloaded - CONFIG: \'MAX_POINTS\'\n"); 
				}

				// Add this point into dataset 
				pxl_ptr->x = avg_pxl; 
				pxl_ptr->y = rc; 
				if (DBG_VIGOROUS)printf("Adding 2D Point: %d, %d\n", pxl_ptr->x, pxl_ptr->y);
				pxl_ptr++; 
				(*used_ptr)++; 

				// Ready next segment
				begin_track_idx = UNINIT;
				end_track_idx = UNINIT;
			}
		}
	}

	free(data);
	free(buffer);
}

// Sanity Function: Dumps the scanned coordinates onto the screen 
void dump2D(int CAM_ID){
	int counter = 0; 
	PIXEL* curr = (CAM_ID==1)?PX_A.pl:PX_B.pl; 
	int used = (CAM_ID == 1) ? PX_A.used : PX_B.used;
	printf("\nCam %d has %d coord. \n", CAM_ID, used); 
	for (; counter < used; counter++){
		printf("\tX: %d, Y: %d\n", curr->x, curr->y);
		curr++; 
	}
}

/********************************************** MAPPER **********************************************/

// Translation of 2D Image Pixels to 3D Coordinates using planar anti-projection algorithm 
void TranslatePoints(int step, int CAM_ID){

	// Assigning Parameters
	int* used2d = (CAM_ID == 1) ? &PX_A.used : &PX_B.used; 
	int* parsed3d = (CAM_ID == 1) ? &P3D_A.used : &P3D_B.used; 
	PIXEL* ptr_2d = (CAM_ID == 1) ? PX_A.pl : PX_B.pl; 
	PT3D* ptr_3d = (CAM_ID == 1) ? P3D_A.pl : P3D_B.pl; 
	CAM_CB* calib = (CAM_ID == 1) ? &CB_A : &CB_B; 
	
	ptr_2d += *parsed3d;
	ptr_3d += *parsed3d; 

	// Angular Arithmetics 
	float angle = 2 * PI * step / (REV_STEPS);
	if (CAM_ID != 1) angle += CB_RAO; 

	// Data Conversion
	int counter = 0; 
	for (; counter < *used2d - *parsed3d; counter++){
		float IMG_X = (float)ptr_2d->x; 
		float IMG_Y = (float)ptr_2d->y; 

		// Set Default Values for Error Exceptions
		ptr_3d->x = 0;
		ptr_3d->y = 0;
		ptr_3d->z = 0;

		// Compute Z: Applying VP (Vanishing Point) Assumption 
		float slope = (float)(calib->VP_Y - IMG_Y) / (calib->VP_X - IMG_X);
		float IMG_INT = (float)(calib->VP_Y - calib->VP_X *slope);
		float Z_INT = (float)(calib->BX *slope + IMG_INT);

		// Skip Computation if point is out of bounds 
		if ((Z_INT <= calib->BY + BASE_SAFE_HEIGHT) || ((IMG_X >= calib->WALL_EDGE) && calib->ORIENT > 0) ||
			((IMG_X <= calib->WALL_EDGE) && calib->ORIENT < 0) || calib->VP_X == IMG_X || calib->VVP_X == IMG_X ){
			ptr_2d++;
			ptr_3d++;
			continue; 
		}

		// Compute Drop Point: Do NOT Apply Vertical VP Assumption
		float y_dropped = (float)(calib->BM*IMG_X + calib->BB); 
		float hyp = (float)(sqrt(pow((float)(calib->BX - IMG_X), 2) + pow((calib->BY - y_dropped), 2)));
		float XNA = (IMG_X>calib->BX) ? hyp*sin(angle)*-1 : hyp*sin(angle);
		float YNA = (IMG_X>calib->BX) ? hyp*cos(angle)*-1 : hyp*cos(angle);

		// Scale X & Y: Applying Geometric Scaling Assumption 
		// Note: LOGb(x) = LOGc(x) / LOGc(b) 
		// For now we don't scale, see how it looks like. 

		// Before writing to the memory the list of points, we need to normalize the points from 0..1 for OPGL. 
		ptr_3d->x = XNA / (WIDTH / 2);
		ptr_3d->y = YNA / (WIDTH / 2);
		ptr_3d->z = (Z_INT - calib->BY) / (WIDTH / 2);
		
		ptr_3d->x = checkFloatSanity(ptr_3d->x);
		ptr_3d->y = checkFloatSanity(ptr_3d->y);
		ptr_3d->z = checkFloatSanity(ptr_3d->z);
		ptr_3d->s = step; 

		// Advance element counters
		ptr_2d++;
		ptr_3d++;
	}

	// Epilogue
	*parsed3d = *used2d; 

}

// Sanity Function: Dumps the calculated results onto the screen. 
void dump3D(int CAM_ID){
	int counter = 0; 
	PT3D* curr = (CAM_ID == 1) ? P3D_A.pl : P3D_B.pl; 
	int used = (CAM_ID == 1) ? P3D_A.used : P3D_B.used; 
	printf("\nCam %d has %d points. \n", CAM_ID, used);
	for (; counter < used; counter++){
		printf("X: %f, Y: %f, Z:%f\n", curr->x, curr->y, curr->z); 
		curr++; 
	}
}

// Save Points: Saves all 3D points into a prescribed file 
void save3DPoints(){

	printf("Save 3D Data Points? (y/n): ");
	char response = getchar();
	getchar(); 
	if (response == 'y' || response == 'Y'){
		int file_savable = 0;
		char fsname[CMD_MAXLEN] = "Data\\";
		system("if not exist \"Data\" mkdir Data");

		do {
			printf("*** Specify File Name (no extension) to Save: ");
			fsname[0] = '\0';
			strcat_s(fsname, "Data\\");
			char uinput[CMD_MAXLEN - 5];
			gets_s(uinput);
			strcat_s(fsname, uinput);
			strcat_s(fsname, ".3dps");

			FILE* dfile = NULL;
			if (!fopen_s(&dfile, fsname, "r")){
				fclose(dfile);
				printf("%s already exists, overwrite? (y/n): ", fsname);
				char OWfile = getchar();
				getchar(); 
				if (OWfile != 'y' && OWfile != 'Y'){
					printf("Save Unsucessful, try again. \n");
					fclose(dfile); 
					continue;
				}
			}
			file_savable = 1;
		} while (!file_savable);

		FILE* dfile = NULL; 
		fopen_s(&dfile, fsname, "w");
		fprintf(dfile, "%d\n", P3D_A.used);
		fprintf(dfile, "%d\n", P3D_B.used);
		
		int c; 
		PT3D* p1 = P3D_A.pl; 
		PT3D* p2 = P3D_B.pl; 

		for (c = 0; c < P3D_A.used; c++){
			fprintf(dfile, "%f\n%f\n%f\n%d\n", p1->x, p1->y, p1->z, p1->s);
			p1++; 
		}

		for (c = 0; c < P3D_B.used; c++){
			fprintf(dfile, "%f\n%f\n%f\n%d\n", p2->x, p2->y, p2->z, p2->s);
			p2++;
		}

		fclose(dfile); 
		if (DBG_LOG) printf("Data Save Completed. \n");

	}
}

// Load Points: Loads all previous 3D points into the program. 
// Returns 0 if user do not wish to load (i.e. New Session). 
int load3DPoints(){

		printf("Load Previous 3D Data Points? (y/n): ");
		char response = getchar();
		getchar(); 
		if (response != 'y' && response != 'Y') return 0; 
		char usr_input[CMD_MAXLEN -5]; 
		char fname[CMD_MAXLEN]; 
		FILE* fptr; 

		int file_loadable = 0; 
		do {
			printf("*** Enter a *.3dps (no extension) file from Data directory to load: "); 
			gets_s(usr_input); 
			fname[0] = '\0'; 
			strcat_s(fname, "Data\\"); 
			strcat_s(fname, usr_input); 
			strcat_s(fname, ".3dps"); 

			if (!fopen_s(&fptr, fname, "r")){
				file_loadable = 1; 
			}
			else {
				printf("File specified does not exist. Please try again. \n"); 
			}

		} while (!file_loadable); 

		fscanf_s(fptr, "%d", &(P3D_A.used)); 
		fscanf_s(fptr, "%d", &(P3D_B.used));
		
		int c; 
		PT3D* p1 = P3D_A.pl;
		PT3D* p2 = P3D_B.pl; 

		for (c = 0; c < P3D_A.used; c++){
			fscanf_s(fptr, "%f\n%f\n%f\n%d\n", &(p1->x), &(p1->y), &(p1->z), &(p1->s));
			p1++;
		}

		for (c = 0; c < P3D_B.used; c++){
			fscanf_s(fptr, "%f\n%f\n%f\n%d\n", &(p2->x), &(p2->y), &(p2->z), &(p2->s));
			p2++;
		}

		fclose(fptr);
		if (DBG_LOG) printf("File Load Completed. \n");
		return 1; 
}

/********************************************** ILLUSTRATOR **********************************************/

// Frame Key Mappings and Commands. 
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	// Closing the Window or Pressing ESC will close the Graphical Window
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	// Reset Viewing Point 
	if (key == GLFW_KEY_R && action == GLFW_PRESS){ glLoadIdentity(); }

	// Z Axial (Horizontal) Rotational Controls  
	if (key == GLFW_KEY_LEFT){ view_angle += HOR_ANGLE_DELTA; }
	if (key == GLFW_KEY_RIGHT){ view_angle -= HOR_ANGLE_DELTA; }

	// Y Axial (Vertical) Rotational Controls
	if (key == GLFW_KEY_UP){ tip_angle -= VERT_ANGLE_DELTA; }
	if (key == GLFW_KEY_DOWN){ tip_angle += VERT_ANGLE_DELTA; }

	// Zoom
	if (key == GLFW_KEY_Z){ zoomfactor += ZOOM_DELTA; }
	if (key == GLFW_KEY_X){ zoomfactor -= ZOOM_DELTA; }

	// Toggle Camera
	if (key == GLFW_KEY_1 && action == GLFW_PRESS){ DISP_A = (DISP_A == 1) ? 0 : 1; }
	if (key == GLFW_KEY_2 && action == GLFW_PRESS){ DISP_B = (DISP_B == 1) ? 0 : 1; }

}

// This is the function OPGL calls when it enconters an error. 
static void error_callback(int error, const char* description)
{
	errorExit(description);
}

// Opens a Window and Display Vector Points
// Reference: http://www.glfw.org/docs/latest/quick.html
int Illustrator(){

	// Set the error callback
	glfwSetErrorCallback(error_callback);

	// Declare a window object
	if (!glfwInit()) errorExit("Cannot initialize graphical panel.");
	GLFWwindow* window;

	// Create a window and create its OpenGL context
	window = glfwCreateWindow(WIDTH * 2, HEIGHT * 2, "SEG's 3D Scanner: Object Preview Window", NULL, NULL);
	if (!window) {
		glfwTerminate();
		errorExit("Failed to open GLFW window.");
	}

	// This function makes the context of the specified window current on the calling thread. 
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);

	// Initialize GLEW
	GLenum err = glewInit();
	if (err != GLEW_OK)	errorExit("Problem Initializing GLEW - OPGL Extension Library.");

	// Panel Settings 
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glColor3f(1.0f, 1.0f, 1.0f);
	glPointSize(3.0f);
	glLineWidth(3.0f);
	glfwSwapInterval(10);

	do {
		/// Clear color buffer
		glClear(GL_COLOR_BUFFER_BIT);

		// Draw Axes
		glBegin(GL_LINES);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(10, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 10, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 10);
		glVertex3f(0, 0, 0);
		glColor3f(.5f, .5f, .5f);
		glVertex3f(-10, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, -10, 0);
		glVertex3f(0, 0, 0);
		glEnd();

		PT3D* V1 = P3D_A.pl;
		PT3D* V2 = P3D_B.pl;
		int pc = 0; 

		// Input all points into panel (CAM1) 
		if (DISP_A){
			glColor3f(0.0f, 1.0f, 1.0f);
			glBegin(GL_POINTS);
			pc = 0;
			for (pc = 0; pc < P3D_A.used; pc++){
				glVertex3f(V1->x, V1->y, V1->z);
				V1++;
			}
			glEnd();
		}

		// Input all points into panel (CAM2) 
		if (DISP_B){
			glColor3f(1.0f, 0.0f, 1.0f);
			glBegin(GL_POINTS);
			for (pc = 0; pc < P3D_B.used; pc++){
				glVertex3f(V2->x, V2->y, V2->z);
				V2++;
			}
			glEnd();
		}

		// Do Verical THEN Horizontal Translations 
		glLoadIdentity();
		glRotatef(tip_angle, 1.0, 0.0, 0.0);
		glRotatef(view_angle, 0.0, 0.0, 1.0);
		glScalef(zoomfactor, zoomfactor, zoomfactor); 

		// Swap buffers
		glfwSwapBuffers(window);
		// Listen for user inputs
		glfwPollEvents();

	} while (!glfwWindowShouldClose(window));

	//Finalize and clean up GLFW
	glfwDestroyWindow(window);
	glfwTerminate();
	return OKAY;
}


int main(void)
{

	/********************************************* Initialization *********************************************/

	// Initialize Serial Communication Channels
	HANDLE hSerial;

	// Initialize 3D Scanner Data Structures
	P3D_A.used = 0;
	P3D_A.max = MAX_POINTS;
	P3D_A.pl = (PT3D*)malloc(MAX_POINTS*sizeof(PT3D));

	P3D_B.used = 0;
	P3D_B.max = MAX_POINTS;
	P3D_B.pl = (PT3D*)malloc(MAX_POINTS*sizeof(PT3D));

	// Program Load Options
	LOAD_MODE = load3DPoints(); 

	/********************************************* FORK CHILD: ILLUSTRATOR *********************************************/
	HANDLE ILLUSTRATOR_PRM = 0;
	 
	

	/********************************************* DATA AQUISITION *****************************************************/
	if (!LOAD_MODE){

		// Create File Handle (Mem. Map Device)
		hSerial = CreateFileA(ARDUINO_PORT, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		if (hSerial == INVALID_HANDLE_VALUE){
			if (GetLastError() == ERROR_FILE_NOT_FOUND)
				errorExit("Serial port specified does not exist.");
			errorExit("Valid file handle is not obtained during initialization.");
		}

		// Set Handle Parameters
		DCB HandleParams = { 0 };
		HandleParams.DCBlength = sizeof(HandleParams);
		if (!GetCommState(hSerial, &HandleParams)){ errorExit("Error while obtaining handle states."); }

		// Some of the parameters to set, more: 
		//      https://msdn.microsoft.com/en-us/library/windows/desktop/aa363214(v=vs.85).aspx
		HandleParams.BaudRate = CBR_19200;
		HandleParams.ByteSize = 1;
		HandleParams.StopBits = ONESTOPBIT;
		HandleParams.Parity = NOPARITY;
		if (!GetCommState(hSerial, &HandleParams)){ errorExit("Error while configuring handle states."); }

		// TODO: Replace this timeout with a thread that will read the serial inputs. 
		COMMTIMEOUTS timeouts = { 0 };
		timeouts.ReadIntervalTimeout = 100;
		timeouts.ReadTotalTimeoutConstant = 100;
		timeouts.ReadTotalTimeoutMultiplier = 10;
		timeouts.WriteTotalTimeoutConstant = 50;
		timeouts.WriteTotalTimeoutMultiplier = 10;

		if (!SetCommTimeouts(hSerial, &timeouts)){ errorExit("Error while setting device I/O timeouts."); }

		// Reset Image Data from Previous Run
		system("rmdir Images_A /s /q");
		system("mkdir Images_A");

		system("rmdir Images_B /s /q");
		system("mkdir Images_B");

		// Initialize Hardware Calibration Data
		CB_A.BB = CB_1_BASE_B;
		CB_A.BM = CB_1_BASE_M;
		CB_A.BX = CB_1_CENTER_X;
		CB_A.BY = CB_1_CENTER_Y;
		CB_A.VP_X = CB_1_VP_X;
		CB_A.VP_Y = CB_1_VP_Y;
		CB_A.VVP_X = CB_1_VVP_X;
		CB_A.VVP_Y = CB_1_VVP_Y;
		CB_A.Scale = CB_1_SCALE_BASE;
		CB_A.WALL_EDGE = CB_1_WALL_EDGE;
		CB_A.ORIENT = CB_1_ORIENTATION; 

		CB_B.BB = CB_2_BASE_B;
		CB_B.BM = CB_2_BASE_M;
		CB_B.BX = CB_2_CENTER_X;
		CB_B.BY = CB_2_CENTER_Y;
		CB_B.VP_X = CB_2_VP_X;
		CB_B.VP_Y = CB_2_VP_Y;
		CB_B.VVP_X = CB_2_VVP_X;
		CB_B.VVP_Y = CB_2_VVP_Y;
		CB_B.Scale = CB_2_SCALE_BASE;
		CB_B.WALL_EDGE = CB_2_WALL_EDGE;
		CB_B.ORIENT = CB_2_ORIENTATION; 

		// Initialize 2D Scanner Data Structures  
		PX_A.used = 0;
		PX_A.max = MAX_POINTS;
		PX_A.pl = (PIXEL*)malloc(MAX_POINTS*sizeof(PIXEL));

		PX_B.used = 0;
		PX_B.max = MAX_POINTS;
		PX_B.pl = (PIXEL*)malloc(MAX_POINTS*sizeof(PIXEL));

		/********************************************* IMAGE AQUISITION *********************************************/
	
		int step = 0; //REV_STEPS instead of s. 
		for (; step < REV_STEPS; step++){
			// Hardware Spin and Picture Taking
			framer(step, hSerial);
			// Extract 2D Points from Pictures Taken
			ExtractPoints(step, 1);
			ExtractPoints(step, 2);
			// Convert 2D to 3D points. 
			TranslatePoints(step, 1);
			TranslatePoints(step, 2);
		}

	}

	// Stats: Display number of points processed. 
	printf("Apts: %d, Bpts: %d \n", P3D_A.used, P3D_B.used);

	// Remove this line when Illustrator parallelized. 
	Illustrator();

	/********************************************* Option: Save Session *********************************************/
	if (!LOAD_MODE) save3DPoints(); 

	/********************************************* EPILOGUE *********************************************/
	free(P3D_A.pl); 
	free(P3D_B.pl); 
	free(PX_A.pl);
	free(PX_B.pl); 

	printf("Program Completed Successfully. Enter any key to exit: ");
	getchar();
	exit(EXIT_SUCCESS);

}