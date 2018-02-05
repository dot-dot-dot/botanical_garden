
#include <ht1632c.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

// which motor is on which pin?
#define M1_PIN          (1)
#define M2_PIN          (2)

// which limit switch is on which pin?
#define L_PIN          (2)
#define R_PIN          (7)

// NEMA17 are 200 steps (1.8 degrees) per turn.  If a spool is 0.8 diameter
// then it is 2.5132741228718345 circumference, and
// 2.5132741228718345 / 200 = 0.0125663706 thread moved each step.
// NEMA17 are rated up to 3000RPM.  Adafruit can handle >1000RPM.
// These numbers directly affect the maximum velocity.
#define STEPS_PER_TURN  (200.0)
// plotter limits
// all distances are relative to the calibration point of the plotter.
// (normally this is the center of the drawing area)
// 300cm x 300cm area
static int limit_top = 60;  // distance to top of drawing area.
static int limit_right = 80;  // Distance to right of drawing area.
static int limit_left = -80;  // Distance to left of drawing area.

// Comment out this line to disable findHome and limit switches
#define USE_LIMIT_SWITCH  (1)

// which way are the spools wound, relative to motor movement?
uint8_t M2_REEL_IN  = BACKWARD;
uint8_t M2_REEL_OUT = FORWARD;
uint8_t M1_REEL_IN  = FORWARD;
uint8_t M1_REEL_OUT = BACKWARD;

// calculate some numbers to help us find feed_rate
float SPOOL_DIAMETER1 = 1.1;
float THREADPERSTEP1;  // thread per step

float SPOOL_DIAMETER2 = 1.1;
float THREADPERSTEP2;  // thread per step

boolean MANUAL = true;
boolean STANDBY = true;
int currentX = 32;

ht1632c ledMatrix = ht1632c(&PORTD, 3,4,5,6, GEOM_32x16, 2);
//A0, A1, A2, A3
//data,wr,clk,cs

Adafruit_MotorShield AFMS0 = Adafruit_MotorShield();
Adafruit_StepperMotor *m1;
Adafruit_StepperMotor *m2;

static uint8_t step_delay = 50;
char line1[180], line2[180];
float oldxx = 0;
float oldyy = 0;
String temp;
char standbyMess[65] = "Pick a plant to discover its main producer > > > > > > > > >";
int plant_index;
// motor position
static long laststep1, laststep2;
float xx, yy;

uint16_t PROGMEM plants_icons [20][2][16] = {
  
  // 0: arachidi
  {{
   0x000a, 0x0160, 0x0280, 0x0200, 0x0044, 0x0c13, 0x0885, 0x0c0f, 0x0efe,
   0x07e9, 0x03f9, 0x000f, 0x000f, 0x0006, 0x0007, 0x0001 },{ 
   0xe000, 0x7800, 0x1800, 0x5800, 0x3800, 0xf000, 0x5000, 0xcc80, 0x27e0,
   0x0038, 0x2018, 0x9838, 0x4098, 0xce10, 0xf490, 0xffe0 }},
 
  // 1: asparagi
  {{
   0x0000, 0x0000, 0x0006, 0x007e, 0x00ef, 0x0373, 0x01fe, 0x000f, 0x0000,
   0x00db, 0x0fcc, 0x1dee, 0x6e77, 0x3fd3, 0x01e0, 0x0000 },{ 
   0x0000, 0x0016, 0xdf1a, 0x62d5, 0x717a, 0xbb40, 0x9c03, 0x002d, 0x02d5,
   0xe35a, 0x5aa4, 0x2f40, 0x6800, 0x8000, 0x0000, 0x0000 }},
   
  // 2: cavolo 
  {{
   0x0390, 0x058a, 0x0c90, 0x05c9, 0x0c84, 0x0e92, 0x09d7, 0x0cce, 0x0e7a,
   0x0577, 0x04c9, 0x0460, 0x0520, 0x0254, 0x0028, 0x0197 },{ 
   0xb185, 0xcad6, 0x849c, 0x0155, 0xc54d, 0xe95c, 0xc47a, 0x6f3c, 0x60aa,
   0xbeb2, 0xfbe8, 0x3de5, 0x2fd4, 0x06c7, 0xb1d2, 0x44fe }},

  // 3: cetriolo
  {{
   0x0000, 0x0000, 0x0000, 0x000f, 0x002a, 0x00e6, 0x0050, 0x0224, 0x0041,
   0x0560, 0x0100, 0x01b0, 0x0044, 0x0077, 0x003f, 0x0000 },{ 
   0x319f, 0x244e, 0x8527, 0xf293, 0x5857, 0x0e07, 0xc313, 0x61ae, 0x018c,
   0x23d8, 0x41c0, 0x1f80, 0x3f00, 0xac00, 0xf000, 0x0000 }},
   
  // 4: ciliegie
  {{
   0x0001, 0x0143, 0x07f6, 0x037e, 0x1dfc, 0x1b56, 0x1edf, 0x2b6b, 0x3dba,
   0x17ef, 0x1d5a, 0x0eee, 0x0f76, 0x06b8, 0x03d0, 0x0000 },{ 
   0x8600, 0x0400, 0x0fc0, 0x0f60, 0x3ff0, 0x7558, 0x6ffc, 0x794c, 0x5ef4,
   0x6b5c, 0x5dec, 0x6d6c, 0x3770, 0x3db0, 0x0fc0, 0x0000 }},

  // 5: cipolle
  {{
   0x70a0, 0x1000, 0x3009, 0x1940, 0x9804, 0x0c82, 0xcc40, 0xe600, 0x3b89,
   0x0ff7, 0x015f, 0x0019, 0x0009, 0x0011, 0x0000, 0x0000 },{ 
   0x0931, 0x1823, 0x3073, 0x1246, 0x30cc, 0x31cc, 0xc130, 0x4f60, 0xdfc4,
   0xfd00, 0xd000, 0x6000, 0x2000, 0x2000, 0x0000, 0x0000 }},

  // 6: fagioli
  {{
   0x0088, 0x00fc, 0x01ee, 0x01fe, 0x007e, 0x0000, 0x0018, 0x002c, 0x006e,
   0x005e, 0x007c, 0x007e, 0x006e, 0x003c, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x1c00, 0x7400, 0xda00, 0xed00, 0x7800, 0x0580,
   0x3fc0, 0x1f40, 0x7dc0, 0x3280, 0x3f00, 0x1e00, 0x0000 }},

  // 7: fragole
  {{
   0x0000, 0x000a, 0x0051, 0x009a, 0x02a1, 0x0180, 0x0bca, 0x090a, 0x0118,
   0x0e56, 0x0638, 0x0115, 0x01c9, 0x007c, 0x000d, 0x0000 },{ 
   0x0000, 0x4000, 0x7380, 0x8700, 0x5300, 0x0e00, 0x4700, 0x8780, 0x9770,
   0x87e0, 0x2380, 0x7b00, 0xd380, 0xb0c0, 0xe040, 0x0000 }},

  // 8: lamponi
  {{
   0x0000, 0x0009, 0x0037, 0x003f, 0x00fc, 0x00fe, 0x01f2, 0x00bb, 0x01fd,
   0x00fb, 0x00ef, 0x005d, 0x000c, 0x0007, 0x0000, 0x0000 },{ 
   0x0000, 0x8000, 0xa200, 0xf200, 0xf400, 0xc6c0, 0xd780, 0xf600, 0x5700,
   0x7780, 0xf480, 0xe400, 0x9400, 0x8000, 0x0000, 0x0000 }},

  // 9: lattuga
  {{
   0x0000, 0x0000, 0x0000, 0x0680, 0x1000, 0x0086, 0x1238, 0x2007, 0x301d,
   0x0202, 0x2022, 0x1400, 0x0775, 0x02b9, 0x000c, 0x0003 },{ 
   0x0000, 0x0140, 0x0360, 0x3c18, 0xe1cc, 0x017c, 0x5f18, 0xf024, 0xae04,
   0xe510, 0x9806, 0x952a, 0x160a, 0x80d6, 0x8d2c, 0xa930  }},
   
  // 10: limoni
  {{
   0x0000, 0x0000, 0x0000, 0x000e, 0x0050, 0x00ab, 0x0198, 0x0322, 0x013c,
   0x036d, 0x0184, 0x00e4, 0x005f, 0x001b, 0x0000, 0x0000 },{ 
   0x0000, 0x0040, 0x00e0, 0x41e0, 0x61e0, 0x0dc0, 0xc580, 0x0f80, 0x1480,
   0x4400, 0x2c00, 0x8c00, 0xe800, 0x8000, 0x0000, 0x0000 }},

  // 11: mandorle
  {{
   0x0000, 0x0000, 0x0000, 0x0060, 0x0010, 0x0050, 0x00c8, 0x014d, 0x0165,
   0x0249, 0x014d, 0x0070, 0x01cc, 0x00f8, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x0020, 0x01a0, 0x64a0, 0x8620, 0x9c20, 0x30c0,
   0xe440, 0xe1c0, 0x7f00, 0x1800, 0x0000, 0x0000, 0x0000 }},

  // 12: melanzane
  {{
   0x0000, 0x0000, 0x0010, 0x02a8, 0x0403, 0x0001, 0x1480, 0x0dd0, 0x0d62,
   0x1752, 0x16fc, 0x1e62, 0x05b9, 0x0fb7, 0x01dc, 0x0071 },{ 
   0x0000, 0x0000, 0x0000, 0x0000, 0x0010, 0x41a0, 0x4ca0, 0x00e0, 0x4370,
   0x11fc, 0xc8f8, 0x49f8, 0xde7c, 0x9b3c, 0xbfda, 0xd710 }},
   
  // 13: olive
  {{
   0x0050, 0x012c, 0x02b6, 0x02f7, 0x03dc, 0x03fc, 0x0071, 0x0006, 0x0005,
   0x000f, 0x0007, 0x000f, 0x000f, 0x0007, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x003e, 0xe0fe, 0xdf8e, 0x0c00, 0xd800, 0xe000, 0xf000,
   0xf000, 0xb000, 0xe000, 0xc000, 0x0000, 0x0000, 0x0000 }},
   
  // 14: patate
  {{
   0x0000, 0x0000, 0x0005, 0x00d0, 0x0184, 0x0275, 0x040a, 0x0545, 0x0494,
   0x0000, 0x014a, 0x0621, 0x0349, 0x00ec, 0x004b, 0x0016 },{ 
   0x0000, 0x0000, 0x9800, 0xcc00, 0x38c0, 0x0260, 0x8f30, 0x3010, 0xc290,
   0x2470, 0xa150, 0x4420, 0xf3a0, 0x8d40, 0x7500, 0x4200 }},

  // 15: piselli
  {{
   0x0000, 0x0000, 0xe400, 0x2780, 0x1e28, 0x9974, 0xdae3, 0xfc83, 0x3fd6,
   0xabef, 0x04f6, 0x02ff, 0x000b, 0x0002, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8000, 0x53c0, 0xdd60, 0x3c10,
   0x3698, 0x7930, 0xfc84, 0xfffa, 0x5d28, 0x0000, 0x0000 }},
      
  // 16: pomodoro
  {{
   0x0000, 0x0000, 0x0001, 0x0043, 0x01ff, 0x0077, 0x001f, 0x005f, 0x02bc,
   0x00b8, 0x0200, 0x0180, 0x0568, 0x07a6, 0x03e8, 0x06dd },{ 
   0x0000, 0x0000, 0x0000, 0x8100, 0xff00, 0xfc00, 0xa800, 0xfa40, 0x7700,
   0x3040, 0x0110, 0x0240, 0x0170, 0x2010, 0x4940, 0x0320 }},
   
  // 17: sesamo
  {{
   0x0000, 0x0000, 0x0000, 0x001f, 0x0075, 0x030b, 0x0044, 0x061d, 0x0fae,
   0x10e3, 0x0820, 0x0818, 0x074c, 0x01d8, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x8000, 0xc000, 0xf400, 0x94a0, 0x4ee0, 0x5c20,
   0xf0e0, 0xe040, 0x2b80, 0x1e00, 0x4000, 0x0000, 0x0000 }},

  // 18: susine
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x0008, 0x0017, 0x0008, 0x007f,
   0x0027, 0x00cc, 0x0027, 0x007b, 0x001f, 0x003b, 0x001f},{ 
   0x0400, 0x0a00, 0x1300, 0x7a00, 0x4000, 0xa000, 0xf000, 0xe800, 0xac00,
   0xf800, 0xec00, 0xfc00, 0xdc00, 0xf800, 0x7000, 0xe000 }},
   
  // 19: zucchine
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002, 0x0001,
   0x0008, 0x0006, 0x0057, 0x0073, 0x0355, 0x1deb, 0x77be },{ 
   0x0000, 0x0000, 0x0000, 0x0068, 0x0ad0, 0x3b0c, 0xe584, 0x36ce, 0x5f58,
   0xe5f8, 0x3d20, 0xeda0, 0x77e0, 0xbae0, 0x9f80, 0x7700 }},

};

uint16_t PROGMEM settings_icon [2][16] = {
  // settings 
  {
   0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0007,
   0x000f, 0x001f, 0x003e, 0x005c, 0x008c, 0x00d8, 0x0070 },{ 
   0x7000, 0xe000, 0xc000, 0xc000, 0xc100, 0xe300, 0xff00, 0xfe00, 0x7c00,
   0x8000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
};

float coords[20][2] = {
  {10,4}, // arachidi
  {19,2}, // asparagi
  {52,7}, // cavolo
  {-2,15}, // cetriolo
  {2,9}, // ciliegie
  {0,-5}, // cipolle
  {-1,14}, // fagioli
  {-41,0}, // fragole
  {-2,15}, // lamponi
  {5,-32}, // lattuga
  {11,1}, // limoni
  {20,-5}, // mandorle
  {-2,16}, // melanzane
  {40,-11}, // olive
  {73,-44}, // patate
  {-4,12}, // piselli
  {-2,15}, // pomodoro
  {2,-9}, // sesamo
  {19,2}, // prugne
  {19,-5}, // zucchine 
};

char* PROGMEM plants_names[20][2] = {
  {"Arachidi","Peanuts"},
  {"Asparagi","Asparagus"},
  {"Cavolo Cappuccio","White Cabbage"},
  {"Cetriolo","Cucumber"},
  {"Ciliegie","Cherries"},
  {"Cipolle","Onions"},
  {"Fagioli","Beans"},
  {"Fragole","Strawberries"},
  {"Lamponi","Raspberries"},
  {"Lattuga","Lettuce"},
  {"Limoni","Lemons"},
  {"Mandorle","Almonds"},
  {"Melanzane","Eggplants"},
  {"Olive","Olives"},
  {"Patate","Potatoes"},
  {"Piselli","Peas"},
  {"Pomodoro","Tomatoes"},
  {"Sesamo","Sesame"},
  {"Prugne","Plums"},
  {"Zucchine","Courgettes"},
};

void setup() {
  Serial.begin(9600);
  /*Serial.println("==================");
  Serial.println("Welcome to DotBot!");
  Serial.println("==================\n");
  Serial.println("H) to set Home (center of map)");
  Serial.println("Mline1) to write a single line string");
  Serial.println("Mindex:line1-line2) to write a multiline string and draw icon[index]");
  Serial.println("A) to start Auto mode");
  Serial.println("X:Y) to move in position (X,Y)\n");*/
  // start the motor shield
  AFMS0.begin();
  m1 = AFMS0.getStepper(STEPS_PER_TURN, M1_PIN);
  m2 = AFMS0.getStepper(STEPS_PER_TURN, M2_PIN);
 // cm done with 1 step = spool perimeter (2*r*pi) / steps per turn
  THREADPERSTEP1 = (SPOOL_DIAMETER1*PI)/STEPS_PER_TURN;  // thread per step
  THREADPERSTEP2 = (SPOOL_DIAMETER2*PI)/STEPS_PER_TURN;  // thread per step

  // initialize the plotter position.
  //teleport(0,0);
  // start the led matrix
  ledMatrix.clear();
  ledMatrix.pwm(15);
  ledMatrix.setfont(FONT_5x7W);
  
  SetHome();
  //FindHome();

}

void loop() {
    if (Serial.available() > 0) {
      STANDBY = false;
      char c = Serial.read();
      if (c == '\n'){
        Serial.println("temp " + temp);
        if (temp[0] == 'M'){
            char index[3];
            temp.substring(temp.indexOf('M')+1).toCharArray(index, 3);
            plant_index = atoi(index);
            temp = "";
            if (plant_index < 20) {
              ledMatrix.clear();
              ledMatrix.hscrolltexts(0, plants_names[plant_index][0], 1, 20, 8, plants_names[plant_index][1], 1);
              ledMatrix.putbitmap(0, 0, plants_icons[plant_index][0], 16, 16, GREEN);
              ledMatrix.putbitmap(16, 0, plants_icons[plant_index][1], 16, 16, GREEN);
              ledMatrix.sendframe();
            } else {
              STANDBY = true;
            }
        } else if (temp[0] == 'G'){

          char index[3];
          temp.substring(temp.indexOf('G')+1).toCharArray(index, 3);
          plant_index = atoi(index);
          temp = "";
          
          if (plant_index < 20) {
            xx = coords[plant_index][0];
            yy = coords[plant_index][1];
            Serial.print(xx);
            Serial.print("-");
            Serial.println(yy);
            if (oldxx != xx || oldyy != yy){
              line(xx,yy);
            }
            oldxx = xx;
            oldyy = yy;
          }
        } else if (temp[0] == 'H'){
          temp = "";
          teleport(0,0);
          oldxx=0;
          oldyy=0;
          Serial.println("This is my home!");
        } else if (temp[0] == 'S'){
          temp = "";
          STANDBY = true;
          currentX = 32;
          ledMatrix.clear();
          if (oldxx != 0 || oldyy != 0){
            line(0,0);
          }
          oldxx = 0;
          oldyy = 0;
        } else if (temp[0] == 'F'){
          temp = "";
          FindHome();
        } else if (temp[0] == 'R'){
          char index[4];
          temp.substring(temp.indexOf('R')+1).toCharArray(index, 4);
          reel(atoi(index));
          temp = "";
        } else {
          char x[4];
          char y[4];
          temp.substring(0, temp.indexOf(':')).toCharArray(x,4);
          temp.substring(temp.indexOf(':')+1).toCharArray(y,4);
          temp = "";
          xx = atof(x);
          yy = atof(y);
          Serial.print(F("Console X: "));
          Serial.println(xx); //get latest parsed data
          Serial.print(F("Console Y: "));
          Serial.println(yy); //get latest parsed data
          if (oldxx != xx || oldyy != yy){
            line(xx,yy);
          }
          oldxx = xx;
          oldyy = yy;
        }
      } else {
        temp += c;
      }
    }
  
  if (STANDBY){
    if (currentX < -300){
      currentX = 32;
    }
    ledMatrix.puttext(currentX,4,standbyMess,GREEN);
    currentX--;
    delay(10);
  }
  /*
  int x = random(1, ledMatrix.x_max);
  int y = random(1, ledMatrix.y_max);
  byte color = random(1,4);
  for (byte rad = 1; rad < ledMatrix.y_max; rad ++) {
    if ((x+rad > ledMatrix.x_max) || (y+rad > ledMatrix.y_max)) break;
    if ((x-rad <= 0) || (y-rad < 0)) break;
    ledMatrix.circle(x,y,rad,color);
    ledMatrix.sendframe();
    delay(100);
    ledMatrix.circle(x,y,rad,0);
    ledMatrix.sendframe();
  }*/
}

void SetHome(){
  teleport(0,0);
  oldxx=0;
  oldyy=0;
}

//------------------------------------------------------------------------------
static char readSwitches() {
#ifdef USE_LIMIT_SWITCH
  // get the current switch state
  return ( (digitalRead(L_PIN)) | (digitalRead(R_PIN)) );
#else
  return 0;
#endif  // USE_LIMIT_SWITCH
}

//------------------------------------------------------------------------------
// find the current robot position and 
static void FindHome() {
  ledMatrix.clear();
  ledMatrix.hscrolltext(4, "Starting auto calibration", RED, 20,1 );
  ledMatrix.putbitmap(0, 0, settings_icon[0], 16, 16, GREEN);
  ledMatrix.putbitmap(16, 0, settings_icon[1], 16, 16, GREEN);
  ledMatrix.sendframe();
            
#ifdef USE_LIMIT_SWITCH
  Serial.println(F("Homing..."));
  
  if(readSwitches()) {
    Serial.println(F("** ERROR **"));
    Serial.println(F("Problem: Plotter is already touching switches."));
    Serial.println(F("Solution: Please unwind the strings a bit and try again."));
    return;
  }
  
  int step_delay=1;
  int safe_out=50;
  
  // reel in the left motor until contact is made.
  Serial.println(F("Find left..."));
  do {
    m1->onestep(M1_REEL_IN, DOUBLE );
    m2->onestep(M2_REEL_OUT, DOUBLE);
    delay(0.5);
  } while(!readSwitches());
  laststep1=0;
  
  Serial.println(F("Back off"));

  // back off so we don't get a false positive on the next motor
  int i;
  for(i=0;i<safe_out;++i) {
    m1->onestep(M1_REEL_OUT, DOUBLE);
    delay(0.5);
  }
  laststep1=safe_out+1000; // cilynder offsets
  
  // reel in the right motor until contact is made
  Serial.println(F("Find right..."));
  do {
    m1->onestep(M1_REEL_OUT, DOUBLE);
    m2->onestep(M2_REEL_IN, DOUBLE);
    delay(0.5);
    laststep1++;
  } while(!readSwitches());
  laststep2=0;
  
  // back off so we don't get a false positive that kills line()
  for(i=0;i<safe_out;++i) {
    m2->onestep(M2_REEL_OUT, DOUBLE);
    delay(0.5);
  }
  laststep2=safe_out+1000; // cilynder offsets
  
  Serial.println(F("Centering..."));
  line(0,0);
#endif // USE_LIMIT_SWITCH

  ledMatrix.clear();
  ledMatrix.hscrolltext(4, "I'm in the center!", RED, 20,1 );
  ledMatrix.clear();

}

//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x,float y) {
  long L1,L2;
  IK(x,y,L1,L2);
  laststep1=L1;
  laststep2=L2;
}

///------------------------------------------------------------------------------
static void line(float x,float y) {
  long l1,l2;
  IK(x,y,l1,l2);
  long d1 = l1 - laststep1;
  long d2 = l2 - laststep2;
   
  long ad1=abs(d1);
  long ad2=abs(d2);
  int dir1=d1<0?M1_REEL_IN:M1_REEL_OUT;
  int dir2=d2<0?M2_REEL_IN:M2_REEL_OUT;
  long over=0;
  long i;
  
  // bresenham's line algorithm.
  if(ad1>ad2) {
    for(i=0;i<ad1;++i) {
      m1->onestep(dir1,DOUBLE);
      over+=ad2;
      if(over>=ad1) {
        over-=ad1;
        m2->onestep(dir2,DOUBLE);
      }
      delay(step_delay/1000);
    }
  } else {
    for(i=0;i<ad2;++i) {
      m2->onestep(dir2,DOUBLE);
      over+=ad1;
      if(over>=ad2) {
        over-=ad2;
        m1->onestep(dir1,DOUBLE);
      }
      delay(step_delay/1000);
    }
  }
  laststep1=l1;
  laststep2=l2;
  Serial.print("laststep1 ");
  Serial.print(laststep1);
  Serial.print(" - laststep2 ");
  Serial.println(laststep2);
}

//------------------------------------------------------------------------------
// Inverse Kinematics - turns XY coordinates into lengths L1,L2
static void IK(float x, float y, long &l1, long &l2) {
  // find length to M1
  float dy = y - limit_top;
  float dx = x - limit_left;
  l1 = floor( sqrt(dx*dx+dy*dy) / THREADPERSTEP1 );
  // find length to M2
  dx = limit_right - x;
  l2 = floor( sqrt(dx*dx+dy*dy) / THREADPERSTEP2 );
}

void reel(int index){
  Serial.println(index);
  if (index == 10)
      m1->step(100, M1_REEL_IN, DOUBLE );
  else if ( index == 11)
      m1->step(100, M1_REEL_OUT, DOUBLE );
  else if (index == 20)
      m2->step(100, M2_REEL_IN, DOUBLE );
  else
      m2->step(100, M2_REEL_OUT, DOUBLE );
}
