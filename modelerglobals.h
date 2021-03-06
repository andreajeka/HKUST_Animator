#ifndef _MODELER_GLOBALS_H
#define _MODELER_GLOBALS_H

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif

// This is a list of the controls for the SampleModel
// We'll use these constants to access the values 
// of the controls from the user interface.
enum SampleModelControls
{
	XPOS, 
	YPOS, 
	ZPOS, 
	RIGHTARMX,
	RIGHTARMY,
	RIGHTARMZ,
	RIGHTELBOWX,
	RIGHTELBOWY,
	RIGHTHANDX,
	LEFTARMX,
	LEFTARMY,
	LEFTARMZ,
	LEFTELBOWX,
	LEFTELBOWY,
	LEFTHANDX,
	LEFTLEGX,
	LEFTLEGZ,
	RIGHTLEGX,
	RIGHTLEGZ,
	LEFTKNEE,
	RIGHTKNEE,
	TAILMOVEMENT, 
	METABALLSKIN,
	TEXTURESKIN,
	NINJATURTLE,
	EYEBANDANA,
	NUMCONTROLS
};

// body size
#define UPPER_TORSO_RADIUS 1.1
#define LOWER_TORSO_HEIGHT 0.8
#define HEAD_RADIUS 0.8

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))
#define SETVAL(x, y) (ModelerApplication::Instance()->SetControlValue(x, y))

#endif