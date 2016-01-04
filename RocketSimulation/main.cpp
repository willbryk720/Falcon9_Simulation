/* Author: William Bryk
 
 Simulation of a Falcon v1.1 launch and landing

This program represents an attempt to model a Falcon v1.1 launch, payload delivery, and landing. It includes realistic air resistance, realistic estimates of the changing mass distribution and changing moment of inertia of the falcon, appropriate graviational force vectors (based on distance to Earth's center), accurate falcon dimensions/thrust, and more. It is not a perfect model however. The simulated rocket lacks grid fins and it does not account for rotational air resistance. What affects the angle of the rocket in this simulation are the nitrogen thrusters, air resistance, and the gimbaled thrust system.
 
Naturally, the next step is to make an option to automate the launch, delivery, and landing, and see how well the computer can do. To avoid the exact same simulation every time, I would include random weather formations. I wonder if the computer would be able to land the falcon in a wind storm!
 
 An even cooler version could include a launch and landing on Mars (you would be able to fast forward the months-long-journey!)
 */

#include <GLUT/glut.h>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdlib>
#import "SOIL.h"

GLdouble TimeSinceLaunch = 0.0;
GLdouble TimeofDetach = 0.0;

//CONSTANTS
const GLdouble TIME_INCREMENT = .03;

GLdouble DeltaT = TIME_INCREMENT;


//const GLdouble gfDeltatheta = .1;

// configure window data
GLdouble width = 400.0;
GLdouble height = 400.0;

// configure window data for Earth perspective
GLdouble width_Earth = 400.0;
GLdouble height_Earth = 400.0;

// cutoff to accurately draw atmosphere color
const GLdouble SPACE_HEIGHT = 100000.0;

const GLdouble Pi = 3.141592653;

// necessary for air resistance calculation
GLdouble T;
GLdouble pressure;
GLdouble air_density;

const GLdouble EARTH_RADIUS = 6371000.0;
const GLdouble PAD_DIAMETER= 200.0;


// data taken from http://spaceflight101.com/spacerockets/falcon-9-v1-1-f9r/

const GLdouble OCTAWEB_MASS = 4200.0;  //9.0 M1D's * 470.0;

const GLdouble BOOSTER_LENGTH = 41.2;
const GLdouble BOOSTER_MASS = 19800.0; //without fuel or OctaWeb (Total weight is actually 24000 kg)
const GLdouble BOOSTER_FUEL_MASS = 395700.0;
const GLdouble SPECIFIC_IMPULSE = 282.0;
const GLdouble THRUST_SEALEVEL = 5885000.0;
const GLdouble THRUST_VACUUM = 6444000;

const GLdouble INTERSTAGE_LENGTH = 1.9; // estimated gap between top of first stage and beginning of merlin engine of second

const GLdouble SECONDSTAGE_LENGTH = 13.8;
const GLdouble SECONDSTAGE_MASS = 3900.0; //without fuel
const GLdouble SECONDSTAGE_FUEL_MASS = 92670;

const GLdouble FAIRING_LENGTH = 13.1;
const GLdouble FAIRING_MASS = 1750;

const GLdouble TOTAL_LENGTH = 70.0;



// height from bottom of falcon to nitrogen thrusters - necessary to calculate torque
const GLdouble NITROGEN_HEIGHT = 38.0;

GLdouble star_locations[80][2]={0.0};

// array of texture ID's
GLuint	texture[5];

// keep track of info about Booster, Payload, and general falcon
class RocketPart
{
public:
    // vectors for center of mass position and velocity
    GLdouble pos_cm[2];
    GLdouble vel_cm[2];
    GLdouble mass;
    GLdouble FuelPercentage = 1.0;
    GLdouble GimbalBeta = 0.0;
    
    // for rotation
    GLdouble MomentofInertia;
    GLdouble omega;
    GLdouble theta;
    GLdouble torque;
    
    // distance between pos_cm and center of earth
    GLdouble dist_to_earth;
    
    // vectors for the top point of the part and bottom point (necessary for orientation)
    GLdouble part_top[2];
    GLdouble part_bottom[2];
    
    GLdouble cm_location; // (number between 0 and 1) (where part_top is 0 and part_bottom is 1)
    
    GLdouble part_width = 3.66;
    GLdouble part_height;
    
    // forces (ones with 3 have magnitude in the 3rd element)
    GLdouble gravity[2];
    GLdouble air_resistance[2];
    GLdouble main_thrust[3];
    // nitrogen thrusters
    GLdouble nit_thrust_left[3];
    GLdouble nit_thrust_right[3];
};


// keep track of user inputs
class switches
{
public:
    GLboolean rocketOn = false;
    GLboolean ZoomOut = false;
    GLboolean RotClock = false;
    GLboolean RotCountClock = false;
    GLboolean Detached = false;
    GLboolean Liftoff = false;
    GLboolean GimbalClock = false;
    GLboolean GimbalCountClock = false;
    GLboolean LegsDeployed = false;
    GLboolean Exploded = false;
    GLboolean SecondExploded = false;
    GLboolean LandedSuccess = false;
    GLboolean WelcomeScreen = true;
    GLboolean Paused = false;
};

// create Objects
RocketPart Falcon, SecondStage;
switches CheckList;



// declare functions, organized by which functions are contained within which
void getStars();

void Initialize();
int LoadGLTextures();
void Timer(int iUnused);
    void Draw();
        void drawClouds(GLdouble color);
        void drawStars();
        void ExplodeOrNot();
            void drawExplosion();
        void SecondExplodeOrNot();
            void drawSecondExplosion();
        void getPosition();
            void updateMassAndMoment();
            void updateTorque();
            void updateTheta();
            void updateVelocity();
            void updateForces();
        void getSecStagePosition();
                void updateMainThrust();
void drawText(GLdouble x, GLdouble y, char *string_text);
void keyUp (unsigned char key, int x, int y);
void keyPressed (unsigned char key, int x, int y);
void keySpecialUp (int key, int x, int y);
void keySpecial(int key, int x, int y);
void refreshVariables();

GLdouble MagOfVector(GLdouble x, GLdouble y);
GLdouble twoDCrossMag(GLdouble a, GLdouble b, GLdouble c, GLdouble d);




int main(int iArgc, char** cppArgv) {
    
    //initiallize some variables
    // calculated with bottom of falcon as baseline
    Falcon.mass = OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS + SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS + FAIRING_MASS;
    Falcon.cm_location =
    (OCTAWEB_MASS * 0 +BOOSTER_MASS * BOOSTER_LENGTH/2.0 + BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0 +(SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS) * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +SECONDSTAGE_LENGTH/2.0) +FAIRING_MASS * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +SECONDSTAGE_LENGTH + FAIRING_LENGTH/2.0))/(Falcon.mass*TOTAL_LENGTH);
    
    Falcon.pos_cm[0] = 0.0, Falcon.pos_cm[1] = Falcon.cm_location*TOTAL_LENGTH;
    Falcon.vel_cm[0] = 0.0, Falcon.vel_cm[1] = 0.0;
    
    

    Falcon.theta = Pi/2.0;
    Falcon.dist_to_earth = EARTH_RADIUS + Falcon.pos_cm[1];
    Falcon.part_height = TOTAL_LENGTH; //using fairing
    Falcon.part_top[0] = 0.0, Falcon.part_top[1] = TOTAL_LENGTH;
    Falcon.part_bottom[0] = 0.0, Falcon.part_bottom[1] = 0.0;
    Falcon.air_resistance[0] = 0.0, Falcon.air_resistance[1] = 0.0;
    Falcon.main_thrust[0] = 0.0, Falcon.main_thrust[1] = THRUST_SEALEVEL, Falcon.main_thrust[2] = THRUST_SEALEVEL;
    
    // couldn't find data on nitrogen thrust magnitude
    Falcon.nit_thrust_left[0] = 0.0, Falcon.nit_thrust_left[1] = 0.0, Falcon.nit_thrust_left[2] = 10000.0;
    Falcon.nit_thrust_right[0] = 0.0, Falcon.nit_thrust_right[1] = 0.0, Falcon.nit_thrust_right[2] = 10000.0;
    
    getStars();
    
    glutInit(&iArgc, cppArgv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(600, 600);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Rocket Simulation");
    
    Initialize();
    glutDisplayFunc(Draw);
    Timer(0);
    
    LoadGLTextures();
    
    // check if user is pressing or releasing a key
    glutKeyboardFunc(keyPressed);
    glutKeyboardUpFunc(keyUp);
    // for arrow keys
    glutSpecialFunc(keySpecial);
    glutSpecialUpFunc(keySpecialUp);
    
    glutMainLoop();
    return 0;
}

void Initialize() {
    glClearColor(0.0, 0.0, 1.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, width, 0.0, height, -1.0, 1.0);
}


void Timer(int iUnused)
{
    glutPostRedisplay();
    glutTimerFunc(30, Timer, 0);
}

void Draw() {
    glClear(GL_COLOR_BUFFER_BIT);
    
    if (CheckList.WelcomeScreen)
    {
        // draw instructions;
        glColor3d(1.0f, 1.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        
        glBindTexture(GL_TEXTURE_2D, texture[4]);
        glBegin(GL_QUADS);
        glTexCoord2d(0.0, 0.0); // Point 1. Drawing Counterclockwise...
        glVertex2d(-width/2.0,-height/2.0);
        
        glTexCoord2d(1.0, 0.0); // point 2.
        glVertex2d(width/2.0,-height/2.0);
        
        glTexCoord2d(1.0, 1.0); // point 3.
        glVertex2d(width/2.0,height/2.0);
        
        glTexCoord2d(0.0, 1.0); // point 4.
        glVertex2d(-width/2.0,height/2.0);
        
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_DEPTH_TEST);
        glutSwapBuffers();
        
        glMatrixMode(GL_PROJECTION);
        
        glLoadIdentity();
        
        glOrtho(-width/2.0, width/2.0, -height/2.0, height/2.0, -1.0, 1.0);
        return;
    }
    
    // draw the sky color according to the height
    GLdouble sky_color = 2.0 - pow(2.0, (Falcon.dist_to_earth - EARTH_RADIUS)/SPACE_HEIGHT);
    if (sky_color < 0.0)
        sky_color = 0.0;
    glClearColor(.55 * sky_color, .8 * sky_color, sky_color, 0.0);
    
    if (!CheckList.ZoomOut) // if user is looking at zoomed in view
    {
        
        glColor3d(1.0f, 1.0f, 1.0f);
        drawClouds(sky_color);
        
        // Input stars into the image
        drawStars();
        
        
        // show user Falcon data
        char s[200];
        char s2[200];
        sprintf(s, " Altitude = %f m | x-location = %f m | Fuel = %f Percent", MagOfVector(Falcon.part_bottom[0],Falcon.part_bottom[1]+EARTH_RADIUS) - EARTH_RADIUS, Falcon.part_bottom[0], 100.0 * Falcon.FuelPercentage);
        sprintf(s2," Time Since Launch = %f s | Velocity y = %f m/s, Velocity x = %f m/s", TimeSinceLaunch, Falcon.vel_cm[1], Falcon.vel_cm[0]);
        glColor3d(1.0f, 1.0f, 1.0f);
        glColor3d(1.0, 1.0, 1.0);
        drawText(Falcon.pos_cm[0] - width/2.0 , Falcon.pos_cm[1] + height/2.4 , s);
        drawText(Falcon.pos_cm[0] - width/2.0 , Falcon.pos_cm[1] + height/2.2 , s2);
        
        // draw ground depending on rocket position on Earth (ground could be on left or right)
        
        if ((Falcon.dist_to_earth - EARTH_RADIUS) < MagOfVector(width, height)) // when ground should be
            //visible from window frame
        {
        
            // getting vector from Earth center to point on surface on line to Falcon center of mass
            
            GLdouble D[2];
            D[0] = EARTH_RADIUS * Falcon.pos_cm[0]/MagOfVector(Falcon.pos_cm[0], EARTH_RADIUS + Falcon.pos_cm[1]);
            D[1] = EARTH_RADIUS * (EARTH_RADIUS + Falcon.pos_cm[1])/MagOfVector(Falcon.pos_cm[0], EARTH_RADIUS + Falcon.pos_cm[1]);
        
            glColor3d(1.0f, 1.0f, 1.0f);
            glColor3d(0.0, .8, 0.0);
            glBegin(GL_QUADS);
        
            glVertex3d(D[0]*((EARTH_RADIUS - 20000.0)/EARTH_RADIUS) - 20000.0 * D[1]/MagOfVector(D[0], D[1]), - EARTH_RADIUS + D[1]*((EARTH_RADIUS - 20000.0)/EARTH_RADIUS) + 20000.0 * D[0]/MagOfVector(D[0], D[1]), 0.0);
            glVertex3d(D[0]*((EARTH_RADIUS - 20000.0)/EARTH_RADIUS) + 20000.0 * D[1]/MagOfVector(D[0], D[1]), - EARTH_RADIUS + D[1]*((EARTH_RADIUS - 20000.0)/EARTH_RADIUS) - 20000.0 * D[0]/MagOfVector(D[0], D[1]), 0.0);
        
            glVertex3d(D[0] + 20000.0 * D[1]/MagOfVector(D[0], D[1]), - EARTH_RADIUS + D[1] - 20000.0 * D[0]/MagOfVector(D[0], D[1]), 0.0);
        
            glVertex3d(D[0] - 20000.0 * D[1]/MagOfVector(D[0], D[1]), - EARTH_RADIUS + D[1] + 20000.0 * D[0]/MagOfVector(D[0], D[1]), 0.0);

            glEnd();
        }
        
        // draw landing pad
        glColor3d(1.0f, 1.0f, 1.0f);
        glColor3d(.3, .3, .3);
        glBegin(GL_QUADS);
        glVertex3d(-PAD_DIAMETER/2.0, -10.0,0.0);
        
        glVertex3d(PAD_DIAMETER/2.0, -10.0,0.0);
        
        glVertex3d(PAD_DIAMETER/2.0, 0.0,0.0);
        
        glVertex3d(-PAD_DIAMETER/2.0, 0.0,0.0);
        glEnd();
        
        if (!CheckList.Detached)
        {
            // draw Rocket;
            glColor3d(1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        
            glBindTexture(GL_TEXTURE_2D, texture[0]);
            glBegin(GL_QUADS);
            glTexCoord2d(0.46, 0.05); // Point 1. Drawing Counterclockwise...
            glVertex2d(Falcon.part_bottom[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
        
            glTexCoord2d(0.54, 0.05); // point 2.
            glVertex2d(Falcon.part_bottom[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
        
            glTexCoord2d(0.54, .93); // point 3.
            glVertex2d(Falcon.part_top[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_top[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
        
            glTexCoord2d(0.46, .93); // point 4.
            glVertex2d(Falcon.part_top[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_top[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
        
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_DEPTH_TEST);
        }
        else if (CheckList.Detached)
        {
            // draw rocket;
            glColor3d(1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            
            glBindTexture(GL_TEXTURE_2D, texture[0]);
            glBegin(GL_QUADS);
            glTexCoord2d(0.46, 0.05); // Point 1. Drawing Counterclockwise...
            glVertex2d(Falcon.part_bottom[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glTexCoord2d(0.54, 0.05); // point 2.
            glVertex2d(Falcon.part_bottom[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glTexCoord2d(0.54, .61); // point 3.
            glVertex2d(Falcon.part_top[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_top[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glTexCoord2d(0.46, .61); // point 4.
            glVertex2d(Falcon.part_top[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_top[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_DEPTH_TEST);
            
            
            // draw second stage;
            glColor3d(1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            
            glBindTexture(GL_TEXTURE_2D, texture[0]);
            glBegin(GL_QUADS);
            glTexCoord2d(0.46, 0.7); // Point 1. Drawing Counterclockwise...
            glVertex2d(SecondStage.part_bottom[0]-(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_bottom[1]+(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glTexCoord2d(0.54, 0.7); // point 2.
            glVertex2d(SecondStage.part_bottom[0]+(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_bottom[1]-(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glTexCoord2d(0.54, .93); // point 3.
            glVertex2d(SecondStage.part_top[0]+(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_top[1]-(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glTexCoord2d(0.46, .93); // point 4.
            glVertex2d(SecondStage.part_top[0]-(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_top[1]+(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_DEPTH_TEST);
            
            
            // draw Second Stage propulsion
            if (TimeSinceLaunch - TimeofDetach > 4.0){
            glColor3d(1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texture[2]);
            glBegin(GL_TRIANGLES);
            glTexCoord2d(0.4, 0.35); // Point 1. Drawing Counterclockwise...
            glVertex2d(SecondStage.part_bottom[0]-(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_bottom[1]+(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glTexCoord2d(0.5, 0.0); // point 2.
            glVertex2d(SecondStage.part_bottom[0] - 20.0 * SecondStage.main_thrust[0]/SecondStage.main_thrust[2], SecondStage.part_bottom[1] - 15.0 * SecondStage.main_thrust[1]/SecondStage.main_thrust[2]);
            
            glTexCoord2d(0.6, 0.35); // point 3.
            glVertex2d(SecondStage.part_bottom[0]+(SecondStage.part_width/2.0)*sin(SecondStage.theta), SecondStage.part_bottom[1]-(SecondStage.part_width/2.0)*cos(SecondStage.theta));
            
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_DEPTH_TEST);
                
            }
            
        }
        
        
        // draw center of mass of rocket
        if (!CheckList.Exploded && !CheckList.LandedSuccess)
        {
            glColor3d(1.0f, 1.0f, 1.0f);
            glColor3d(0.0, 0.0, 1.0);
            glPointSize(3);
            glBegin(GL_POINTS);
            glVertex3d(Falcon.pos_cm[0], Falcon.pos_cm[1], 0.0);
            glEnd();
        }
        
        // draw nitrogen thrust
        if (!CheckList.LandedSuccess){
        glColor3d(1.0f, 1.0f, 1.0f);
        glColor3d(.2, 1.0, 1.0);
        glBegin(GL_LINES);
        glVertex3d(Falcon.part_bottom[0] + NITROGEN_HEIGHT*(Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]),Falcon.part_bottom[1] + NITROGEN_HEIGHT*(Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]), 0.0);
        
        glVertex3d(Falcon.part_bottom[0] + NITROGEN_HEIGHT*(Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]) - 7.0 * Falcon.nit_thrust_left[0]/MagOfVector(Falcon.nit_thrust_left[0], Falcon.nit_thrust_left[1]),Falcon.part_bottom[1] + NITROGEN_HEIGHT*(Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]) - 7.0* Falcon.nit_thrust_left[1]/MagOfVector(Falcon.nit_thrust_left[0], Falcon.nit_thrust_left[1]), 0.0);
        
        glVertex3d(Falcon.part_bottom[0] + NITROGEN_HEIGHT*(Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]),Falcon.part_bottom[1] + NITROGEN_HEIGHT*(Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]), 0.0);
        
        glVertex3d(Falcon.part_bottom[0] + NITROGEN_HEIGHT*(Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]) - 7.0 * Falcon.nit_thrust_right[0]/MagOfVector(Falcon.nit_thrust_right[0], Falcon.nit_thrust_right[1]),Falcon.part_bottom[1] + NITROGEN_HEIGHT*(Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0], Falcon.part_top[1] - Falcon.part_bottom[1]) - 7.0* Falcon.nit_thrust_right[1]/MagOfVector(Falcon.nit_thrust_right[0], Falcon.nit_thrust_right[1]), 0.0);
        glEnd();
        }
        
        if (CheckList.rocketOn && (Falcon.FuelPercentage > 0.0) && !CheckList.LandedSuccess)
        {
            // draw Falcon propulsion
            glColor3d(1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texture[2]);
            glBegin(GL_TRIANGLES);
            glTexCoord2d(0.4, 0.35); // Point 1. Drawing Counterclockwise...
            glVertex2d(Falcon.part_bottom[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glTexCoord2d(0.5, 0.0); // point 2.
            glVertex2d(Falcon.part_bottom[0] - 20.0 * Falcon.main_thrust[0]/Falcon.main_thrust[2], Falcon.part_bottom[1] - 20.0 * Falcon.main_thrust[1]/Falcon.main_thrust[2]);
            
            glTexCoord2d(0.6, 0.35); // point 3.
            glVertex2d(Falcon.part_bottom[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_DEPTH_TEST);
        }
        
        if (CheckList.LegsDeployed)
        {
            // draw legs
            glColor3d(1.0f, 1.0f, 1.0f);
            glColor3d(0.1, 0.1, 0.1);
            glLineWidth(3);
            glBegin(GL_LINES);
            glVertex2d(Falcon.part_bottom[0]-(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]+(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glVertex2d(Falcon.part_bottom[0]-5.0*(Falcon.part_width/2.0)*sin(Falcon.theta + Pi/7.5), Falcon.part_bottom[1]+5.0*(Falcon.part_width/2.0)*cos(Falcon.theta + Pi/7.5));
            
            glVertex2d(Falcon.part_bottom[0]+(Falcon.part_width/2.0)*sin(Falcon.theta), Falcon.part_bottom[1]-(Falcon.part_width/2.0)*cos(Falcon.theta));
            
            glVertex2d(Falcon.part_bottom[0]+5.0*(Falcon.part_width/2.0)*sin(Falcon.theta - Pi/7.5), Falcon.part_bottom[1]-5.0*(Falcon.part_width/2.0)*cos(Falcon.theta - Pi/7.5));
            
            glEnd();
        }
        
        if (CheckList.Liftoff)
            ExplodeOrNot();
        
        if (CheckList.Detached)
            SecondExplodeOrNot();
        
    
        glutSwapBuffers();
        
        if (!CheckList.Exploded && !CheckList.LandedSuccess && !CheckList.Paused && !CheckList.WelcomeScreen)
            getPosition();
        
        if (!CheckList.SecondExploded && !CheckList.Paused && !CheckList.WelcomeScreen && CheckList.Detached)
            getSecStagePosition();
        
        // follow center of rocket
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(Falcon.pos_cm[0] - width/2.0, Falcon.pos_cm[0] + width/2.0, Falcon.pos_cm[1] - height/2.0, Falcon.pos_cm[1] + height/2.0, -1.0, 1.0);
    }
    else if (CheckList.ZoomOut) // if user is looking at zoomed out view (for perspective)
    {
        // SCALING EVERYTHING BY FACTOR OF 15000
        
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glColor3d(1.0f, 1.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        
        glBindTexture(GL_TEXTURE_2D, texture[1]);
        glBegin(GL_QUADS);
        
        glTexCoord2d(0.06, 0.0);
        glVertex2d(-480.0, -910.0);
        
        glTexCoord2d(.98, 0.0); // point 2.
        glVertex2d(520.0, -910.0);
        
        glTexCoord2d(.98, 1.0); // point 3.
        glVertex2d(520.0, 65.0);
        
        glTexCoord2d(0.06, 1.0); // point 4.
        glVertex2d(-480.0, 65.0);
        
        
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_DEPTH_TEST);
        
        // zoomed out rocket
        glBegin(GL_LINES);
        glVertex2d(Falcon.part_bottom[0]/15000.0, Falcon.part_bottom[1]/15000.0);
        glVertex2d(Falcon.part_bottom[0]/15000.0 + .2*(Falcon.part_top[0] - Falcon.part_bottom[0]), Falcon.part_bottom[1]/15000.0 + .2*(Falcon.part_top[1] - Falcon.part_bottom[1]));
        glEnd();
        
        
        // zoomed out Second Stage as a point
        
        glPointSize(3);
        glBegin(GL_POINTS);
        if (!CheckList.Detached)
            glVertex2d(Falcon.part_bottom[0]/15000.0 + .2*(Falcon.part_top[0] - Falcon.part_bottom[0]), Falcon.part_bottom[1]/15000.0 + .2*(Falcon.part_top[1] - Falcon.part_bottom[1]));
        else if (CheckList.Detached)
            glVertex2d(SecondStage.part_top[0]/15000.0, SecondStage.part_top[1]/15000.0);
        glEnd();
        
        
        
        if (CheckList.Liftoff)
            ExplodeOrNot();
        
        if (CheckList.Detached)
            SecondExplodeOrNot();
        
        glutSwapBuffers();
        
        // update the position of the rocket
        if (!CheckList.Exploded && !CheckList.LandedSuccess && !CheckList.Paused && !CheckList.WelcomeScreen)
            getPosition();
        
        // if detached, update the position of the second stage
        if (!CheckList.SecondExploded && !CheckList.Paused && !CheckList.WelcomeScreen && CheckList.Detached)
            getSecStagePosition();
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-width_Earth*2.5, width_Earth*2.5, -height_Earth*3.5, height_Earth*1.5, -1.0, 1.0);
    }
    
}

// draw stagnant clouds so user can see how fast rocket is travelling
void drawClouds(GLdouble color) {
    glColor3d(color*.9, color*.9, color*.9);
    glPointSize(30);
         glBegin(GL_POINTS);
    
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            int x_loc = Falcon.pos_cm[0]/300;
            int y_loc = Falcon.pos_cm[1]/300;
            
            glVertex3d(300*(x_loc - i)+90.0,300*(y_loc - j)+230.0, 0.0);
            glVertex3d(300*(x_loc - i)+90.0,300*(y_loc + j)+230.0, 0.0);
            glVertex3d(300*(x_loc + i)+90.0,300*(y_loc - j)+230.0, 0.0);
            glVertex3d(300*(x_loc + i)+90.0,300*(y_loc + j)+230.0, 0.0);
        }
    }
         glEnd();
}

void drawStars(){
    
    glColor3d(0.8, 0.8, 0.8);
    glPointSize(1);
    glBegin(GL_POINTS);
    
    for (int i = 0; i < 79; i++)
            glVertex3d(Falcon.pos_cm[0] - width/2.0 + star_locations[i][0] * width, Falcon.pos_cm[1] - height/2.0 + star_locations[i][1] * height, 0.0);
    glEnd();
    
    // Easter Egg
    glColor3d(0.8, 0.2, 0.2);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex3d(Falcon.pos_cm[0] - width/2.0 + .65 * width, Falcon.pos_cm[1] - height/2.0 + .85 * height, 0.0);
    glEnd();
}



void getStars() {
    for (int i = 0; i < 80; i++)
    {
        // generate random locations for stars
        star_locations[i][0] = static_cast <GLdouble> (rand()) / static_cast <GLdouble> (RAND_MAX);
        star_locations[i][1] = static_cast <GLdouble> (rand()) / static_cast <GLdouble> (RAND_MAX);
    }
}

void ExplodeOrNot(){
    
    if (MagOfVector(Falcon.part_top[0], Falcon.part_top[1] + EARTH_RADIUS) < EARTH_RADIUS)
    {
        CheckList.Exploded = true;
        Falcon.vel_cm[0] = 0.0; Falcon.vel_cm[1] = 0.0; Falcon.omega = 0.0;
        if (!CheckList.ZoomOut)
            drawExplosion();
            
    }
    else if (MagOfVector(Falcon.part_bottom[0], Falcon.part_bottom[1] + EARTH_RADIUS) < EARTH_RADIUS)
    {
        GLdouble vel_bottom = MagOfVector(Falcon.vel_cm[0] + Falcon.omega*Falcon.cm_location*Falcon.part_height* (Falcon.part_top[1]-Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]), Falcon.vel_cm[1] + Falcon.omega*Falcon.cm_location*Falcon.part_height* (Falcon.part_bottom[0]-Falcon.part_top[0])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]));
        
        if ((vel_bottom > 60.0) || !CheckList.LegsDeployed || (Falcon.part_bottom[0] > PAD_DIAMETER/2.0) ||  (Falcon.part_bottom[0] < -PAD_DIAMETER/2.0) || CheckList.Exploded)
        {
            CheckList.Exploded = true;
            Falcon.vel_cm[0] = 0.0; Falcon.vel_cm[1] = 0.0; Falcon.omega = 0.0;
            if (!CheckList.ZoomOut)
                drawExplosion();
        }
        else if ((Falcon.theta > 2.0*Pi/3.0)||(Falcon.theta < Pi/3.0) )
        {
            if ((Falcon.theta > Pi) || (Falcon.theta < 0.0))
            {
                CheckList.Exploded = true;
                Falcon.pos_cm[0] = (Falcon.part_top[0] + Falcon.part_bottom[0])/2.0; Falcon.pos_cm[1] = 0.0;
                if (!CheckList.ZoomOut)
                    drawExplosion();
            }
            else if (Falcon.theta > 2.0*Pi/3.0 )
                Falcon.theta += .3 * DeltaT;
            else if (Falcon.theta < Pi/3.0)
                Falcon.theta -= .3 * DeltaT;
            
            Falcon.part_top[0] = Falcon.part_bottom[0] + Falcon.part_height * cos(Falcon.theta);
            Falcon.part_top[1] = Falcon.part_bottom[1] + Falcon.part_height * sin(Falcon.theta);
            
            // update pos_cm based on top and bottom and Falcon.cm_location
            Falcon.pos_cm[0] = Falcon.cm_location*(Falcon.part_top[0] - Falcon.part_bottom[0]) + Falcon.part_bottom[0];
            Falcon.pos_cm[1] = Falcon.cm_location*(Falcon.part_top[1] - Falcon.part_bottom[1]) + Falcon.part_bottom[1];
            
            Falcon.vel_cm[0] = 0.0; Falcon.vel_cm[1] = 0.0; Falcon.omega = 0.0;
            
        }
        else
        {
            CheckList.LandedSuccess = true;
            Falcon.vel_cm[0] = 0.0; Falcon.vel_cm[1] = 0.0; Falcon.omega = 0.0;
            
            // fix angle so that rocket is upright
            if (Falcon.theta < Pi/2.0 - .01)
                Falcon.theta += .2 * DeltaT;
            else if (Falcon.theta > Pi/2.0 + .01)
                Falcon.theta -= .2 * DeltaT;
                
            Falcon.part_top[0] = Falcon.part_bottom[0] + Falcon.part_height * cos(Falcon.theta);
            Falcon.part_top[1] = Falcon.part_bottom[1] + Falcon.part_height * sin(Falcon.theta);
            
            // HousePartyProtocol();
        }
    }
    
}

void SecondExplodeOrNot(){
    
    if ((MagOfVector(SecondStage.part_top[0], SecondStage.part_top[1] + EARTH_RADIUS) < EARTH_RADIUS) || (MagOfVector(SecondStage.part_bottom[0], SecondStage.part_bottom[1] + EARTH_RADIUS) < EARTH_RADIUS))
    {
        CheckList.SecondExploded = true;
        SecondStage.vel_cm[0] = 0.0; SecondStage.vel_cm[1] = 0.0; SecondStage.omega = 0.0; // left omega for a version 2 of this program
        
        if (!CheckList.ZoomOut)
            drawSecondExplosion();
    }
}

void drawExplosion(){
    
    glColor3d(1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    
    glBindTexture(GL_TEXTURE_2D, texture[3]);
    glBegin(GL_QUADS);
    
    glTexCoord2d(0.0, 0.0); // point 1
    glVertex2d(Falcon.pos_cm[0] - 50.0, Falcon.pos_cm[1] - 50.0);
    
    glTexCoord2d(1.0, 0.0); // point 2
    glVertex2d(Falcon.pos_cm[0] + 50.0, Falcon.pos_cm[1] - 50.0);
    
    glTexCoord2d(1.0, 1.0); // point 3
    glVertex2d(Falcon.pos_cm[0] + 50.0, Falcon.pos_cm[1] + 50.0);
    
    glTexCoord2d(0.0, 1.0); // point 4
    glVertex2d(Falcon.pos_cm[0] - 50.0, Falcon.pos_cm[1] + 50.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
}

void drawSecondExplosion(){
    glColor3d(1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    
    glBindTexture(GL_TEXTURE_2D, texture[3]);
    glBegin(GL_QUADS);
    
    glTexCoord2d(0.0, 0.0); // point 1
    glVertex2d(SecondStage.pos_cm[0] - 30.0, SecondStage.pos_cm[1] - 30.0);
    
    glTexCoord2d(1.0, 0.0); // point 2
    glVertex2d(SecondStage.pos_cm[0] + 30.0, SecondStage.pos_cm[1] - 30.0);
    
    glTexCoord2d(1.0, 1.0); // point 3
    glVertex2d(SecondStage.pos_cm[0] + 30.0, SecondStage.pos_cm[1] + 30.0);
    
    glTexCoord2d(0.0, 1.0); // point 4
    glVertex2d(SecondStage.pos_cm[0] - 30.0, SecondStage.pos_cm[1] + 30.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
}

void getPosition(){
    
    if (CheckList.Liftoff)
        TimeSinceLaunch += DeltaT;
    
    // translation of top and bottom of Falcon
    Falcon.pos_cm[0] += Falcon.vel_cm[0] * DeltaT;
    Falcon.pos_cm[1] += Falcon.vel_cm[1] * DeltaT;

    GLdouble dist2top;
    GLdouble dist2bottom;
    if (!CheckList.Detached)
    {
        dist2bottom = Falcon.cm_location * TOTAL_LENGTH;
        dist2top = TOTAL_LENGTH - dist2bottom;
    }
    else
    {
        dist2bottom = Falcon.cm_location * BOOSTER_LENGTH;
        dist2top = BOOSTER_LENGTH - dist2bottom;
    }
    Falcon.part_top[0] = dist2top*cos(Falcon.theta) + Falcon.pos_cm[0];
    Falcon.part_top[1] = dist2top*sin(Falcon.theta) + Falcon.pos_cm[1];
    Falcon.part_bottom[0] = dist2bottom*cos(Falcon.theta + Pi) + Falcon.pos_cm[0];
    Falcon.part_bottom[1] = dist2bottom*sin(Falcon.theta + Pi) + Falcon.pos_cm[1];
    
    
    // update Mass and Moment of Inertia
    updateMassAndMoment();
    
    // update top and bottom using torque
    updateTorque();
    
    Falcon.omega += DeltaT * Falcon.torque/Falcon.MomentofInertia;
    
    //ROTATION
    updateTheta();
    updateForces();
    updateVelocity();

    
}

void updateMassAndMoment(){
    if (!CheckList.Detached)
    {
        Falcon.mass = OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS * Falcon.FuelPercentage + SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS +  FAIRING_MASS;
        
        // calculated with bottom of falcon as baseline
        Falcon.cm_location =
        (OCTAWEB_MASS * 0 +
         BOOSTER_MASS * BOOSTER_LENGTH/2.0 +
         BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0 +
         (SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS) * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +
                                                           SECONDSTAGE_LENGTH/2.0) +
         FAIRING_MASS * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +
                           SECONDSTAGE_LENGTH + FAIRING_LENGTH/2.0))/Falcon.mass;
        
        // make between 0 and 1
        Falcon.cm_location = Falcon.cm_location/TOTAL_LENGTH;
        
        // approximating using a small width approximation
        // using lots of parallel axis theorem
        Falcon.MomentofInertia = OCTAWEB_MASS * pow(Falcon.cm_location,2.0) +
        
        (1.0/12.0)* BOOSTER_MASS * pow(BOOSTER_LENGTH,2.0) + BOOSTER_MASS * pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - BOOSTER_LENGTH/2.0),2.0) +
        
        (1.0/12.0)* BOOSTER_FUEL_MASS * Falcon.FuelPercentage * pow(BOOSTER_LENGTH * Falcon.FuelPercentage,2.0) + BOOSTER_FUEL_MASS * Falcon.FuelPercentage * pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - BOOSTER_LENGTH * Falcon.FuelPercentage/2.0),2.0) +
        
        (1.0/12.0) * (SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS) * pow(SECONDSTAGE_LENGTH,2.0) + (SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS) * pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - (BOOSTER_LENGTH + INTERSTAGE_LENGTH + SECONDSTAGE_LENGTH/2.0)),2.0) +
        
        (1.0/12.0) * FAIRING_MASS * pow(FAIRING_LENGTH,2.0) + pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - (BOOSTER_LENGTH + INTERSTAGE_LENGTH + SECONDSTAGE_LENGTH + FAIRING_LENGTH/2.0)),2.0);
        
    }
    else
    {
        // update Falcon mass without second stage
        Falcon.mass = OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS * Falcon.FuelPercentage;
        
        
        Falcon.cm_location = (OCTAWEB_MASS * 0 +
                              BOOSTER_MASS * BOOSTER_LENGTH/2.0 +
                              BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0)/Falcon.mass;
        
        // make between 0 and 1
        Falcon.cm_location = Falcon.cm_location/BOOSTER_LENGTH;
        
        Falcon.MomentofInertia = OCTAWEB_MASS * pow(Falcon.cm_location,2.0) +
        
        (1.0/12.0)* BOOSTER_MASS * pow(BOOSTER_LENGTH,2.0) + BOOSTER_MASS * pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - BOOSTER_LENGTH/2.0),2.0) +
        
        (1.0/12.0)* BOOSTER_FUEL_MASS * Falcon.FuelPercentage * pow(BOOSTER_LENGTH * Falcon.FuelPercentage,2.0) + BOOSTER_FUEL_MASS * Falcon.FuelPercentage * pow(std::abs(Falcon.cm_location * TOTAL_LENGTH - BOOSTER_LENGTH * Falcon.FuelPercentage/2.0),2.0);
    }
}

void updateTorque(){
    
    GLdouble torque_air;
    
    if (MagOfVector(Falcon.air_resistance[0], Falcon.air_resistance[1]) > .00001 ) // prevent dividing by zero
    {
    
        torque_air = ((Falcon.part_height/2.0) - Falcon.cm_location * Falcon.part_height) * twoDCrossMag(Falcon.air_resistance[0], Falcon.air_resistance[1],Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
    
        // need to check whether torque is positive or negative, do this by finding sin(theta - alpha) where alpha
        // is angle of air resistance
        GLdouble sin_theta_alpha = sin(Falcon.theta)*(Falcon.air_resistance[0]/MagOfVector(Falcon.air_resistance[0], Falcon.air_resistance[1])) - (Falcon.air_resistance[1]/MagOfVector(Falcon.air_resistance[0], Falcon.air_resistance[1]))*cos(Falcon.theta);
    
        if ((sin_theta_alpha > 0.00001) && (sin_theta_alpha < Pi))
            torque_air = -torque_air;
    
    }
    else
        torque_air = 0.0;
    
    
    // GLdouble torque_gimbal
    GLdouble torque_gimbal;
    
    if (MagOfVector(Falcon.main_thrust[0], Falcon.main_thrust[1]) > .0001 ) // prevent dividing by zero
        
        torque_gimbal = (Falcon.cm_location * Falcon.part_height) * twoDCrossMag(Falcon.main_thrust[0], Falcon.main_thrust[1], Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
    else
        torque_gimbal = 0.0;
        
    if (Falcon.GimbalBeta > 0.0001)
        torque_gimbal = -torque_gimbal;
    
    // sum of torque of air resistance, gimbaled thrust, and each nitrogen thruster
    Falcon.torque = torque_air + torque_gimbal + (NITROGEN_HEIGHT - Falcon.cm_location * TOTAL_LENGTH) * MagOfVector(Falcon.nit_thrust_right[0],Falcon.nit_thrust_right[1]) - (NITROGEN_HEIGHT - Falcon.cm_location * TOTAL_LENGTH) * MagOfVector(Falcon.nit_thrust_left[0],Falcon.nit_thrust_left[1]);
}

void updateTheta(){
    
    GLboolean smallangle = false; //don't want to divide by zero
    if (std::abs(Falcon.part_top[0]-Falcon.part_bottom[0]) < 0.00000001)
        smallangle = true;
    
    if ((Falcon.part_top[0]-Falcon.part_bottom[0]) >= 0.0)
    {
        if ((Falcon.part_top[1]-Falcon.part_bottom[1]) >= 0.0)
        {
            //angle is in first quadrant
            if (!smallangle)
                Falcon.theta = atan((Falcon.part_top[1]-Falcon.part_bottom[1])/(Falcon.part_top[0]-Falcon.part_bottom[0]));
            else
                Falcon.theta = Pi/2.0;
        }
        else if ((Falcon.part_top[1]-Falcon.part_bottom[1]) < 0.0)
        {
            // angle is in fourth quadrant
            if (!smallangle)
                Falcon.theta = atan((Falcon.part_top[1]-Falcon.part_bottom[1])/(Falcon.part_top[0]-Falcon.part_bottom[0])) + 2*Pi;
            else
                Falcon.theta = -Pi/2.0;
        }
    }
    else if ((Falcon.part_top[0]-Falcon.part_bottom[0]) < 0.0)
    {
        if ((Falcon.part_top[1]-Falcon.part_bottom[1]) >= 0.0)
        {
            //angle is in second quadrant
            if (!smallangle)
                Falcon.theta = atan((Falcon.part_top[1]-Falcon.part_bottom[1])/(Falcon.part_top[0]-Falcon.part_bottom[0])) + Pi;
            else
                Falcon.theta = Pi/2.0;
        }
        else if ((Falcon.part_top[1]-Falcon.part_bottom[1]) < 0.0)
        {
            // angle is in third quadrant
            if (!smallangle)
                Falcon.theta = atan((Falcon.part_top[1]-Falcon.part_bottom[1])/(Falcon.part_top[0]-Falcon.part_bottom[0])) + Pi;
            else
                Falcon.theta = -Pi/2.0;
        }
    }
    
     Falcon.theta += DeltaT * Falcon.omega;
}


void updateVelocity(){
    if (CheckList.Liftoff)
    {
        Falcon.vel_cm[0] = Falcon.vel_cm[0] + DeltaT * (Falcon.gravity[0] + Falcon.air_resistance[0] + Falcon.main_thrust[0] + Falcon.nit_thrust_left[0] + Falcon.nit_thrust_right[0])/Falcon.mass;
        
        Falcon.vel_cm[1] = Falcon.vel_cm[1] + DeltaT * (Falcon.gravity[1] + Falcon.air_resistance[1] + Falcon.main_thrust[1] + Falcon.nit_thrust_left[1] + Falcon.nit_thrust_right[1])/Falcon.mass;
    }
}

void updateForces(){
    
    // since center of Earth is located at [0,-EARTH_RADIUS]
    Falcon.dist_to_earth = MagOfVector(Falcon.pos_cm[0],Falcon.pos_cm[1] + EARTH_RADIUS);
    
    // using F = - GmM/r^2  where  GM = 3.98588 * pow(10,14)
    GLdouble grav_magnitude = 3.98588 * pow(10,14)*(Falcon.mass)/pow(Falcon.dist_to_earth,2.0);
    
    Falcon.gravity[0] = - grav_magnitude * Falcon.pos_cm[0]/Falcon.dist_to_earth;
    Falcon.gravity[1] = - grav_magnitude * (Falcon.pos_cm[1] + EARTH_RADIUS)/Falcon.dist_to_earth;
    
    
    // update air resistance force vector
    
    GLdouble sin_alpha;
    GLdouble cos_alpha;
    
    if (MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]) > .00001) // don't want to divide by zero
    {
        sin_alpha = (Falcon.vel_cm[1])/MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]);
        cos_alpha = (Falcon.vel_cm[0])/MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]);
    }
    else
    {
        sin_alpha = 0;
        cos_alpha = 0;
    }
    
    GLdouble A = std::abs(Falcon.part_width * Falcon.part_height*(sin(Falcon.theta)*cos_alpha - sin_alpha * cos(Falcon.theta))) +
    std::abs(Falcon.part_width * Falcon.part_width*(cos(Falcon.theta)*cos_alpha + sin(Falcon.theta)*sin_alpha));
    
    // using wikipedia for formula for air_density. Not as accurate outside troposphere
    
    if ( ((Falcon.dist_to_earth - EARTH_RADIUS) < 43000.0) && ((Falcon.dist_to_earth - EARTH_RADIUS) > 0.0) )
    {
        T = 288.15 - .0065 * (Falcon.dist_to_earth - EARTH_RADIUS);
        pressure = 101.325*pow((1 - .0065 * (Falcon.dist_to_earth - EARTH_RADIUS)/288.15),(9.80665*.02896/(8.31447*.0065)));
        air_density = 1000.0 * pressure * .0289644/(T * 8.31447);
    }
    else
    {
        air_density = 0.0;
    }
    
    // using formula for air resistance: D = DragCoefficient * .5 * A (in direction of velocity) * airdensity * V^2
    // using Drag Coefficient of .6
    
    
    GLdouble D = .6 * .5 * air_density * pow(MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]),2.0) * A;
    
    if ((D*DeltaT < 2.0*(MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]))) &&(MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]) > 0.0)) // prevent faulty air resistance change in velocity due to high DeltaT, and preventdivision by zero
    {
            Falcon.air_resistance[0] = - D * Falcon.vel_cm[0]/MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]);
            Falcon.air_resistance[1] = - D * Falcon.vel_cm[1]/MagOfVector(Falcon.vel_cm[0], Falcon.vel_cm[1]);
    }
    else
    {
        Falcon.air_resistance[0] = 0;
        Falcon.air_resistance[1] = 0;
    }
    
    // update main thrust force vector
    updateMainThrust();
    
    // update side thrust force vectors
    if (CheckList.RotClock && CheckList.Liftoff)
    {
        Falcon.nit_thrust_left[0] =  Falcon.nit_thrust_left[2] * (Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
        
        Falcon.nit_thrust_left[1] =  - Falcon.nit_thrust_left[2] * (Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
    }
    else
    {
        Falcon.nit_thrust_left[0] = 0;
        Falcon.nit_thrust_left[1] = 0;
    }
    
    if (CheckList.RotCountClock && CheckList.Liftoff)
    {
        Falcon.nit_thrust_right[0] = - Falcon.nit_thrust_right[2] * (Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
        
        Falcon.nit_thrust_right[1] = Falcon.nit_thrust_right[2] * (Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
    }
    else
    {
        Falcon.nit_thrust_right[0] = 0;
        Falcon.nit_thrust_right[1] = 0;
    }
}

void updateMainThrust(){
    
    if ((CheckList.GimbalClock) && (Falcon.GimbalBeta < Pi/4.0))
        Falcon.GimbalBeta += .5* DeltaT;
    if ((CheckList.GimbalCountClock) && (Falcon.GimbalBeta > -Pi/4.0))
        Falcon.GimbalBeta -= .5* DeltaT;
    
    if (CheckList.rocketOn)
    {
        // equation for thrust vector is cos(Falcon.GimbalBeta) * rocketUnitVector * thrustMagnitude + sin(Falcon.GimbalBeta)*UnitVectorPerpToRocket * thrustMagnitude
        
        Falcon.main_thrust[0] = Falcon.main_thrust[2] * cos(Falcon.GimbalBeta)*(Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]) +
            Falcon.main_thrust[2] * sin(Falcon.GimbalBeta)* -(Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
        
        Falcon.main_thrust[1] = Falcon.main_thrust[2] * cos(Falcon.GimbalBeta) * (Falcon.part_top[1] - Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]) +
            Falcon.main_thrust[2] * sin(Falcon.GimbalBeta)* (Falcon.part_top[0] - Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0] - Falcon.part_bottom[0],Falcon.part_top[1] - Falcon.part_bottom[1]);
        
        if (Falcon.FuelPercentage > 0.00001)
            Falcon.FuelPercentage = (Falcon.FuelPercentage * BOOSTER_FUEL_MASS - DeltaT * (THRUST_SEALEVEL)/(SPECIFIC_IMPULSE * 9.8))/BOOSTER_FUEL_MASS; // mass flow rate formula using thrust and specific impulse
        else
        {
            Falcon.main_thrust[2] = 0.0;
            Falcon.FuelPercentage = 0.0;
        }
    }
    else
    {
        Falcon.main_thrust[0] = 0.0;
        Falcon.main_thrust[1] = 0.0;
    }
}

void getSecStagePosition(){

    // update second stage mass
    SecondStage.mass = SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS * SecondStage.FuelPercentage +  FAIRING_MASS;
    
    // Find the new angle of the second stage, in version 2.0
    //updateSecStageAngle();
  
    
    // update gravitational force
    SecondStage.dist_to_earth = MagOfVector(SecondStage.pos_cm[0],SecondStage.pos_cm[1] + EARTH_RADIUS);

    // using F = - GmM/r^2  where  GM = 3.98588 * pow(10,14)
    GLdouble grav_magnitude2 = 3.98588 * pow(10.0,14.0)*(SecondStage.mass)/pow(SecondStage.dist_to_earth,2.0);
    
    SecondStage.gravity[0] = - grav_magnitude2 * SecondStage.pos_cm[0]/SecondStage.dist_to_earth;
    SecondStage.gravity[1] = - grav_magnitude2 * (SecondStage.pos_cm[1] + EARTH_RADIUS)/SecondStage.dist_to_earth;
    

    // update main thrust
    if (TimeSinceLaunch - TimeofDetach > 4.0)
    {
        
        SecondStage.main_thrust[0] = SecondStage.main_thrust[2] *(SecondStage.part_top[0] - SecondStage.part_bottom[0])/MagOfVector(SecondStage.part_top[0] - SecondStage.part_bottom[0],SecondStage.part_top[1] - SecondStage.part_bottom[1]);
    
        SecondStage.main_thrust[1] = SecondStage.main_thrust[2] * (SecondStage.part_top[1] - SecondStage.part_bottom[1])/MagOfVector(SecondStage.part_top[0] - SecondStage.part_bottom[0],SecondStage.part_top[1] - SecondStage.part_bottom[1]);
    }
    else
    {
        SecondStage.main_thrust[0] = 0.0; SecondStage.main_thrust[1] = 0.0;
    }
    
    if (SecondStage.FuelPercentage > 0.00001)
        SecondStage.FuelPercentage = (SecondStage.FuelPercentage * SECONDSTAGE_FUEL_MASS - DeltaT * (THRUST_VACUUM/9.0)/(SPECIFIC_IMPULSE * 9.8))/SECONDSTAGE_FUEL_MASS; // mass flow rate formula using thrust and specific impulse
    else
    {
        SecondStage.main_thrust[2] = 0.0;
        SecondStage.FuelPercentage = 0.0;
    }
    
    
    // update velocity
    SecondStage.vel_cm[0] = SecondStage.vel_cm[0] + DeltaT * (SecondStage.gravity[0] + SecondStage.main_thrust[0])/SecondStage.mass;
    SecondStage.vel_cm[1] = SecondStage.vel_cm[1] + DeltaT * (SecondStage.gravity[1] + SecondStage.main_thrust[1])/SecondStage.mass;
    
    
    
    // translation of top and bottom of Second Stage
    SecondStage.pos_cm[0] += SecondStage.vel_cm[0] * DeltaT;
    SecondStage.pos_cm[1] += SecondStage.vel_cm[1] * DeltaT;
    
    
    
    // Rotate Second Stage accordingly
    SecondStage.part_top[0] = ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0)*cos(SecondStage.theta) + SecondStage.pos_cm[0];
    SecondStage.part_top[1] = ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0)*sin(SecondStage.theta) + SecondStage.pos_cm[1];
    
    SecondStage.part_bottom[0] = ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0)*cos(SecondStage.theta + Pi) + SecondStage.pos_cm[0];
    SecondStage.part_bottom[1] = ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0)*sin(SecondStage.theta + Pi) + SecondStage.pos_cm[1];
}

void drawText(GLdouble x, GLdouble y, char *string_text) {
    //set the position of the text
    glRasterPos2f(x,y);
    //get length of the string
    int len = (int) strlen(string_text);
    
    //loop to display character by character
    for (int i = 0; i < len; i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, string_text[i]);
    }
}

void keyUp (unsigned char key, int x, int y) {
    
    if (key == 'c')
    {
        if (!CheckList.Paused)
            CheckList.RotClock = false;
    }
    else if (key == 'z')
    {
        if (!CheckList.Paused)
            CheckList.RotCountClock = false;
    }
    else if (key == 'i')
    {
        if (CheckList.WelcomeScreen)
            CheckList.WelcomeScreen = false;
        else
            CheckList.WelcomeScreen = true;
    }
    else if (key == 'r')
    {
        refreshVariables();
    }
    else if (key == 'e')
    {
        if (CheckList.ZoomOut)
            CheckList.ZoomOut = false;
        else
            CheckList.ZoomOut = true;
    }
    else if (key == 'p')
    {
        if (CheckList.Paused)
            CheckList.Paused = false;
        else
            CheckList.Paused = true;
    }
    else if (key == 'v')
    {
        if (!CheckList.ZoomOut)
        {
            if (width > 750.0) {width = 400.0; height = 400.0;}
            else if (width > 350.0) {width = 200.0; height = 200.0;}
            else if (width > 150.0) {width = 800.0; height = 800.0;}
        }
        else
        {
            if (width_Earth > 3199.0) {width_Earth = 1600.0; height_Earth = 1600.0;}
            else if (width_Earth > 1599.0) {width_Earth = 800.0; height_Earth = 800.0;}
            else if (width_Earth > 799.0) {width_Earth = 400.0; height_Earth = 400.0;}
            else if (width_Earth > 399.0) {width_Earth = 200.0; height_Earth = 200.0;}
            else if (width_Earth > 199.0) {width_Earth = 3200.0; height_Earth = 3200.0;}
        }
    }
    else if ((key == 'l') && (!CheckList.Paused))
    {
        if (CheckList.LegsDeployed)
            CheckList.LegsDeployed = false;
        else
            CheckList.LegsDeployed = true;
    }
    else if (key == 'w')
    {
        if ((DeltaT < 40.0) && (!CheckList.Paused))
            DeltaT *= 2.0;
    }
    else if (key == 'q')
    {
        if ((DeltaT > 0.00001)&& (!CheckList.Paused))
            DeltaT /= 2.0;
    }
    else if (key == 'd')
    {
        if (!CheckList.Paused && !CheckList.Detached && !CheckList.LandedSuccess && !CheckList.Exploded && CheckList.Liftoff)
        {
            CheckList.Detached = true;
            TimeofDetach = TimeSinceLaunch;
            Falcon.pos_cm[0] = Falcon.part_bottom[0] + (((OCTAWEB_MASS * 0 + BOOSTER_MASS * BOOSTER_LENGTH/2.0 + BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0)/(OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS * Falcon.FuelPercentage)))*cos(Falcon.theta);
            Falcon.pos_cm[1] = Falcon.part_bottom[1] + (((OCTAWEB_MASS * 0 + BOOSTER_MASS * BOOSTER_LENGTH/2.0 + BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0)/(OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS * Falcon.FuelPercentage)))*sin(Falcon.theta);
            Falcon.part_height = BOOSTER_LENGTH;
            Falcon.part_top[0] = Falcon.part_bottom[0] + BOOSTER_LENGTH*cos(Falcon.theta);
            Falcon.part_top[1] = Falcon.part_bottom[1] + BOOSTER_LENGTH*sin(Falcon.theta);
            
            SecondStage.pos_cm[0] = Falcon.part_top[0] + ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0) * (Falcon.part_top[0]-Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]);
            SecondStage.pos_cm[1] = Falcon.part_top[1] + ((SECONDSTAGE_LENGTH + FAIRING_LENGTH)/2.0) * (Falcon.part_top[1]-Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]);
            SecondStage.vel_cm[0] = Falcon.vel_cm[0] + 7.0 * (Falcon.part_top[0]-Falcon.part_bottom[0])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]);
            SecondStage.vel_cm[1] = Falcon.vel_cm[1] + 7.0 * (Falcon.part_top[1]-Falcon.part_bottom[1])/MagOfVector(Falcon.part_top[0]-Falcon.part_bottom[0], Falcon.part_top[1]-Falcon.part_bottom[1]);
            SecondStage.mass = SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS + FAIRING_MASS;
            SecondStage.theta = Falcon.theta;
            SecondStage.part_height = SECONDSTAGE_LENGTH + FAIRING_LENGTH; //using fairing
            SecondStage.part_bottom[0] = Falcon.part_top[0];
            SecondStage.part_bottom[1] = Falcon.part_top[1];
            SecondStage.part_top[0] = Falcon.part_top[0] + (SECONDSTAGE_LENGTH + FAIRING_LENGTH)*cos(SecondStage.theta);
            SecondStage.part_top[1] = Falcon.part_top[1] + (SECONDSTAGE_LENGTH + FAIRING_LENGTH)*sin(SecondStage.theta);
            
            SecondStage.main_thrust[2] = THRUST_VACUUM/9.0;
            SecondStage.main_thrust[0] = SecondStage.main_thrust[2] * cos(SecondStage.theta);
            SecondStage.main_thrust[1] =  SecondStage.main_thrust[2] * sin(SecondStage.theta);
            
        }
    }
}

void keyPressed (unsigned char key, int x, int y) {
    
    if (key == 'c')
    {
        if (!CheckList.Paused)
            CheckList.RotClock = true;
    }
    else if (key == 'z')
    {
        if (!CheckList.Paused)
            CheckList.RotCountClock = true;
    }
}

void keySpecialUp (int key, int x, int y) {
    if (key == GLUT_KEY_UP)
    {
        if (!CheckList.Paused)
            CheckList.rocketOn = false;
    }
    else if (key == GLUT_KEY_RIGHT)
    {
        if (!CheckList.Paused)
            CheckList.GimbalClock = false;
    }
    else if (key == GLUT_KEY_LEFT)
    {
        if (!CheckList.Paused)
            CheckList.GimbalCountClock = false;
    }
}
void keySpecial(int key, int x, int y) {
    
    if ((key == GLUT_KEY_UP) && !CheckList.WelcomeScreen && !CheckList.Paused)
    {
        CheckList.rocketOn = true;
        
        // 3..2..1.. LIFTOFF!!!!!!  Houston, initiate simulation!
        if (!CheckList.Liftoff)
            CheckList.Liftoff = true;
    }
    else if (key == GLUT_KEY_RIGHT)
    {
        if (!CheckList.Paused)
            CheckList.GimbalClock = true;
    }
    else if (key == GLUT_KEY_LEFT)
    {
        if (!CheckList.Paused)
            CheckList.GimbalCountClock = true;
    }
    else if (key == GLUT_KEY_DOWN)
    {
        if (!CheckList.Paused)
            Falcon.GimbalBeta = 0.0;
    }
}

int LoadGLTextures()									// Load Bitmaps And Convert To Textures
{
    /* load an image file directly as a new OpenGL texture */
    texture[0] = SOIL_load_OGL_texture
    (
     "Falcon.png",
     SOIL_LOAD_AUTO,
     SOIL_CREATE_NEW_ID,
     SOIL_FLAG_INVERT_Y
     );
    
    if (texture[0] == 0)
        return false;
    
    
    // Typical Texture Generation Using Data From The Bitmap
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    texture[1] = SOIL_load_OGL_texture
    (
     "Earth.png",
     SOIL_LOAD_AUTO,
     SOIL_CREATE_NEW_ID,
     SOIL_FLAG_INVERT_Y
     );
    
    if (texture[1] == 0)
        return false;
    
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    texture[2] = SOIL_load_OGL_texture
    (
     "RocketFire.png",
     SOIL_LOAD_AUTO,
     SOIL_CREATE_NEW_ID,
     SOIL_FLAG_INVERT_Y
     );
    
    if (texture[2] == 0)
        return false;
    
    glBindTexture(GL_TEXTURE_2D, texture[2]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    
    texture[3] = SOIL_load_OGL_texture
    (
     "Explosion.png",
     SOIL_LOAD_AUTO,
     SOIL_CREATE_NEW_ID,
     SOIL_FLAG_INVERT_Y
     );
    
    if (texture[3] == 0)
        return false;
    
    glBindTexture(GL_TEXTURE_2D, texture[3]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    texture[4] = SOIL_load_OGL_texture
    (
     "Instructions.png",
     SOIL_LOAD_AUTO,
     SOIL_CREATE_NEW_ID,
     SOIL_FLAG_INVERT_Y
     );
    
    if (texture[4] == 0)
        return false;
    
    glBindTexture(GL_TEXTURE_2D, texture[4]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    return true;										// Return Success
}

GLdouble MagOfVector(GLdouble x, GLdouble y){
    
    return sqrt(pow(x,2.0) + pow(y,2.0));
    
}

// calculates magnitude of (a,b) in the direction of (c,d)
GLdouble twoDCrossMag(GLdouble a, GLdouble b, GLdouble c, GLdouble d) {
    
    if ((pow((a*c + b*d)/(MagOfVector(a,b)*MagOfVector(c,d)),2.0) > .99999) && (pow((a*c + b*d)/(MagOfVector(a,b)*MagOfVector(c,d)),2.0) < 1.00001))
        return 0.0;
    else
        return  MagOfVector(a,b) * sqrt(std::abs(1- pow((a*c + b*d)/(MagOfVector(a,b)*MagOfVector(c,d)),2.0)));
}



void refreshVariables(){
    
    TimeSinceLaunch = 0.0;
    TimeofDetach = 0.0;
    Falcon.FuelPercentage = 1.0;
    SecondStage.FuelPercentage = 1.0;
    Falcon.mass = OCTAWEB_MASS + BOOSTER_MASS + BOOSTER_FUEL_MASS + SECONDSTAGE_MASS + SECONDSTAGE_FUEL_MASS + FAIRING_MASS;
    Falcon.cm_location =(OCTAWEB_MASS * 0 +BOOSTER_MASS * BOOSTER_LENGTH/2.0 +BOOSTER_FUEL_MASS * Falcon.FuelPercentage * BOOSTER_LENGTH * Falcon.FuelPercentage/2.0 +(SECONDSTAGE_MASS +SECONDSTAGE_FUEL_MASS) * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +SECONDSTAGE_LENGTH/2.0) + FAIRING_MASS * (BOOSTER_LENGTH + INTERSTAGE_LENGTH +SECONDSTAGE_LENGTH + FAIRING_LENGTH/2.0))/(Falcon.mass*TOTAL_LENGTH);
    
    Falcon.pos_cm[0] = 0.0, Falcon.pos_cm[1] = Falcon.cm_location*TOTAL_LENGTH;
    Falcon.vel_cm[0] = 0.0, Falcon.vel_cm[1] = 0.0; Falcon.omega = 0.0;
    Falcon.GimbalBeta = 0.0;
    Falcon.torque = 0.0;
    
    Falcon.theta = Pi/2.0;
    Falcon.dist_to_earth = EARTH_RADIUS + Falcon.pos_cm[1];
    Falcon.part_height = TOTAL_LENGTH;
    Falcon.part_top[0] = 0.0, Falcon.part_top[1] = TOTAL_LENGTH;
    Falcon.part_bottom[0] = 0.0, Falcon.part_bottom[1] = 0.0;
    Falcon.air_resistance[0] = 0.0, Falcon.air_resistance[1] = 0.0;
    Falcon.main_thrust[0] = 0.0, Falcon.main_thrust[1] = THRUST_SEALEVEL, Falcon.main_thrust[2] = THRUST_SEALEVEL;
    
    // couldn't find data on nitrogen thrust magnitude
    Falcon.nit_thrust_left[0] = 0.0, Falcon.nit_thrust_left[1] = 0.0, Falcon.nit_thrust_left[2] = 10000.0;
    Falcon.nit_thrust_right[0] = 0.0, Falcon.nit_thrust_right[1] = 0.0, Falcon.nit_thrust_right[2] = 10000.0;
    
    
    CheckList.rocketOn = false;CheckList.ZoomOut = false;CheckList.RotClock = false;CheckList.RotCountClock = false;CheckList.Detached = false;CheckList.Liftoff = false;CheckList.GimbalClock = false;CheckList.GimbalCountClock = false;CheckList.LegsDeployed = false;CheckList.Exploded = false;CheckList.SecondExploded = false;CheckList.LandedSuccess = false;CheckList.WelcomeScreen = false;CheckList.Paused = false;
    
    DeltaT = TIME_INCREMENT;
}











