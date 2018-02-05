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
char standbyMess[55] = "Pick a plant to discover their route > > > > > > > > >";
int plant_index;
// motor position
static long laststep1, laststep2;
float xx, yy;

uint16_t PROGMEM plants_icons [20][2][16] = {
  
  // 0: agave
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0100, 0x0102, 0x0142, 0x00a1,
   0x00d2, 0x006a, 0x105f, 0x2a6a, 0x0fad, 0x03fe, 0x01ef },{ 
   0x0000, 0x8000, 0x8000, 0x8000, 0x4000, 0x8000, 0xe008, 0x20a8, 0xc240,
   0xb4a0, 0x5280, 0x5900, 0xa418, 0xcaa0, 0xa490, 0xa080 }},
 
  // 1: albero del pane
  {{
   0x0000, 0x0000, 0x0001, 0x000f, 0x0010, 0x0041, 0x0080, 0x0014, 0x0242,
   0x0028, 0x010a, 0x0025, 0x0492, 0x1045, 0x1032, 0x0a9a },{ 
   0x0000, 0x003c, 0xf87e, 0xfe3c, 0x1fac, 0x6b58, 0x9570, 0xbfa0, 0x5760,
   0xadf0, 0x57f0, 0x5bb0, 0xeef0, 0x1bf0, 0xef70, 0xabb0 }},
   
  // 2: ananas 
  {{
   0x0000, 0x0198, 0x0042, 0x0ce7, 0x0c42, 0x1390, 0x1218, 0x0c22, 0x0c67,
   0x0942, 0x139c, 0x0118, 0x0c63, 0x0c63, 0x010a, 0x001c },{ 
   0x0000, 0xc000, 0x1080, 0x3890, 0x10f0, 0xc4e6, 0x85dc, 0x1178, 0x39f6,
   0x11f8, 0xc5fc, 0xc1a0, 0x10b0, 0x3080, 0x4000, 0x0000 }},

  // 3: barbabietola
  {{
   0x7400, 0xb400, 0xdc1c, 0x767f, 0x3dff, 0x1fff, 0x05df, 0x0377, 0x01fd,
   0x03ef, 0x01d5, 0x01fe, 0x00ff, 0x007f, 0x001e, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x0000, 0x8000, 0x8000, 0xc000, 0xe000, 0xa000,
   0xe000, 0xe000, 0xf000, 0xf800, 0xfe0c, 0x0ff0, 0x0000 }},
   
  // 4: cacao
  {{
   0x001f, 0x00f2, 0x0187, 0x06df, 0x097f, 0x13f7, 0x0605, 0x50ff, 0x39ff,
   0x387f, 0x0f0f, 0x07a1, 0x03ff, 0x01ff, 0x00ff, 0x003f },{ 
   0xc000, 0x7800, 0x8f00, 0xf580, 0xffcc, 0xff6e, 0x47ae, 0xfcff, 0xff9e,
   0xfd76, 0xe7eb, 0x4fc7, 0xff81, 0xff06, 0xfc03, 0x8002 }},

  // 5: caffe
  {{
   0x0000, 0x0001, 0x0006, 0x001a, 0x007f, 0x0040, 0x009e, 0x00f3, 0x006f,
   0x007d, 0x0036, 0x001f, 0x0017, 0x000f, 0x0001, 0x0000 },{ 
   0x3780, 0x7ee0, 0xbbf8, 0xff7c, 0xfff2, 0xfff4, 0x1fc6, 0x871e, 0xe03e,
   0xfffc, 0xeffc, 0x7fe8, 0xfff0, 0xffc0, 0xff00, 0x0000 }},

  // 6: camomilla
  {{
   0x0003, 0x001f, 0x02df, 0x018f, 0x06ef, 0x01e7, 0x06ff, 0x03fd, 0x00f8,
   0x0110, 0x05f0, 0x0378, 0x05a0, 0x04ec, 0x059e, 0x007f },{ 
   0x1000, 0x3e00, 0xbf00, 0xdf00, 0xbe40, 0xb8a0, 0xfbf0, 0x76f8, 0x27f8,
   0xdff0, 0x4c80, 0x2bf0, 0x8ff8, 0x0ff0, 0x19f0, 0xfc40 }},

  // 7: canapa indiana
  {{
   0x000a, 0x000d, 0x0002, 0x000b, 0x0000, 0x0007, 0x0882, 0x06a1, 0x0151,
   0x015a, 0x00af, 0x0015, 0x0015, 0x0001, 0x0001, 0x000a },{ 
   0x0300, 0x8100, 0x4280, 0x8300, 0xc540, 0x6780, 0xa201, 0x678e, 0xa937,
   0xdd1a, 0x6eec, 0xb7b0, 0xb6c0, 0xfb00, 0x3f40, 0xdde0 }},

  // 8: cannella
  {{
   0x9c59, 0x410c, 0x6005, 0x3840, 0x9b21, 0x0e37, 0xa71f, 0x91cf, 0x4965,
   0x3420, 0x0a73, 0x03ff, 0x03e7, 0x0005, 0x0000, 0x0000 },{ 
   0x6000, 0x9000, 0x3c00, 0x7e00, 0xbe00, 0xcc00, 0xf000, 0xc800, 0x3700,
   0x1f00, 0xef00, 0x6e00, 0xe000, 0xc000, 0x0000, 0x0000 }},

  // 9: carota
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000, 0x0024,
   0x0081, 0x035c, 0x07ff, 0x0ffe, 0x0000, 0x0000, 0x0000},{ 
   0x0000, 0x0000, 0x0480, 0x0740, 0x0160, 0x03b0, 0x7240, 0x9d60, 0x38a0,
   0x10c0, 0xf000, 0xc000, 0x0000, 0x0000, 0x0000, 0x0000 }},
   
  // 10: dattero
  {{
   0x0380, 0x0f10, 0x1740, 0x1ef0, 0x16c4, 0x1f9c, 0x0fec, 0x0778, 0x03f0,
   0x0001, 0x0001, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x00f0, 0x02d8, 0x0578, 0x25b8, 0x1338, 0xdef0,
   0x91e0, 0x7fe0, 0x9ec0, 0xef80, 0x3c00, 0x0000, 0x0000 }},

  // 11: luppolo
  {{
   0x0000, 0x0000, 0x0007, 0x0027, 0x00f3, 0x0041, 0x0008, 0x031e, 0x031e,
   0x0008, 0x0061, 0x00f3, 0x0007, 0x0007, 0x0000, 0x0000 },{ 
   0x0000, 0x0000, 0x3800, 0xbc00, 0x9e00, 0x1e00, 0x3f00, 0x7f00, 0x7fc0,
   0x3f00, 0x1e00, 0x9e00, 0xbc00, 0x3800, 0x0000, 0x0000 }},

  // 12: melo
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0018, 0x007e, 0x00af, 0x01ee, 0x01f4,
   0x01ef, 0x01ed, 0x00f5, 0x00fa, 0x007f, 0x007f, 0x003f },{ 
   0x3e00, 0x7c00, 0x7800, 0x4000, 0x0800, 0x7800, 0xe700, 0x8280, 0x9800,
   0x2a00, 0xd480, 0x5280, 0xec80, 0xb880, 0xbf00, 0xfe00 }},
   
  // 13: peperoncino
  {{
   0x0400, 0x0e00, 0x0600, 0x0350, 0x01f4, 0x032c, 0x03f6, 0x03ff, 0x01e5,
   0x00fe, 0x007b, 0x003a, 0x003f, 0x000f, 0x0003, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8000, 0xa000,
   0xf500, 0xdf40, 0x73f0, 0xbd78, 0xdfe4, 0xfc00, 0x0000 }},
   
  // 14: pisello
  {{
   0x00b8, 0x00ed, 0x0052, 0x00e5, 0x002c, 0x003e, 0x001f, 0x018f, 0x03b0,
   0x03b2, 0x1d7b, 0x3c7f, 0x19c3, 0x01e1, 0x01c0, 0x0000 },{ 
   0x0000, 0x0000, 0xd000, 0x2a00, 0x2940, 0x4530, 0xef54, 0xf91d, 0xfd97,
   0x5ffe, 0x07f0, 0x8044, 0x0002, 0x0000, 0x0000, 0x0000 }},
   
  // 15: segale
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0021, 0x0043, 0x0063, 0x0067,
   0x00f8, 0x039c, 0x0f86, 0x1dc0, 0x1800, 0x1000, 0x0000 },{ 
   0x0000, 0x0000, 0x0000, 0x0104, 0x135a, 0x9a5c, 0x11fc, 0x3f80, 0xe1c0,
   0xf020, 0x2800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }},
   
  // 16: soia
  {{
   0xaed4, 0x3357, 0xd664, 0xad99, 0xd6d5, 0x6a15, 0x27f4, 0x1512, 0x07ab,
   0x0145, 0x00b8, 0x0053, 0x000a, 0x0222, 0x0000, 0x0000 },{ 
   0x1926, 0xcac0, 0x991b, 0x54a5, 0x524a, 0xab53, 0x542c, 0x5092, 0x4b59,
   0x690c, 0x95a2, 0x6548, 0x1542, 0xc000, 0x0040, 0x0000 }},

  // 17: zafferano
  {{
   0x0020, 0x0029, 0x0043, 0x0042, 0x01ae, 0x02f4, 0x055b, 0x03ee, 0x07fb,
   0x03b0, 0x01e3, 0x0230, 0x032c, 0x0265, 0x0122, 0x00c1 },{ 
   0xd600, 0x8400, 0x2400, 0x0800, 0xb800, 0x2c00, 0xc700, 0x0100, 0xe480,
   0x3080, 0x9b00, 0x6e00, 0x3400, 0x0400, 0x4600, 0xa900 }},
   
  // 18: zenzero
  {{
   0x0000, 0x0007, 0x0005, 0x000c, 0x00ba, 0x076f, 0x0db5, 0x0ef3, 0x07ff,
   0x0ef7, 0x03fe, 0x01fe, 0x00f8, 0x006c, 0x003e, 0x003c },{ 
   0x0200, 0x0480, 0x0100, 0x2f80, 0x67d0, 0x7aac, 0xcdf0, 0xbeec, 0xda60,
   0xe908, 0xfd00, 0x7580, 0x3e00, 0x2180, 0x0000, 0x0000 }},
   
  // 19: zucca
  {{
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0011, 0x0075, 0x015a,
   0x01a6, 0x00af, 0x034b, 0x01ed, 0x058a, 0x03df, 0x038d },{ 
   0x0000, 0x3000, 0x2000, 0xc000, 0xc000, 0x8000, 0x9800, 0x7a00, 0xdd00,
   0x6500, 0x32c0, 0xbac0, 0x4160, 0xba60, 0x9330, 0xd960 }},
};

uint16_t PROGMEM settings_icon [2][16] = {
  // settings 
  {
   0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0007,
   0x000f, 0x001f, 0x003e, 0x005c, 0x008c, 0x00d8, 0x0070 },{ 
   0x7000, 0xe000, 0xc000, 0xc000, 0xc100, 0xe300, 0xff00, 0xfe00, 0x7c00,
   0x8000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
};

char* PROGMEM plants_names[20][2] = {
  {"Agave","Agave"},
  {"Albero del Pane","Breadfruit Tree"},
  {"Ananas","Ananas"},
  {"Barbabietola","Beetroot"},
  {"Cacao","Cocoa"},
  {"Caffe'","Coffee"},
  {"Camomilla","Chamomile"},
  {"Canapa indiana","Indian hemp"},
  {"Cannella","Cinnamon"},
  {"Carota","Carrot"},
  {"Dattero","Date"},
  {"Luppolo","Hops"},
  {"Melo","Apple Tree"},
  {"Peperoncino","Red pepper"},
  {"Pisello","Pea"},
  {"Segale","Rye"},
  {"Soia","Soy"},
  {"Zafferano","Saffron"},
  {"Zenzero","Ginger"},
  {"Zucca","Pumpkin"},
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
  
  //FindHome();

}

void loop() {
    if (Serial.available() > 0) {
      STANDBY = false;
      char c = Serial.read();
      if (c == '\n'){
        Serial.println("temp " + temp);
        if (temp[0] == 'M'){
          /*if (temp.indexOf('-') > 0) { //two lines message
              temp.substring(1,temp.indexOf('-')).toCharArray(line1, 180);
              temp.substring(temp.indexOf('-')+1).toCharArray(line2, 180);
              temp = "";
              Serial.print("Line 1: ");
              Serial.println(line1); //get latest parsed data
              Serial.print("Line 2: ");
              Serial.println(line2); //get latest parsed data
              ledMatrix.clear();
              ledMatrix.hscrolltexts(0, line1, 1, 20, 8, line2, 1); 
          } else {*/ // plant index
            char index[3];
            temp.substring(temp.indexOf('M')+1).toCharArray(index, 3);
            plant_index = atoi(index);
            temp = "";
            if (plant_index < 19) {
              ledMatrix.clear();
              ledMatrix.hscrolltexts(0, plants_names[plant_index][0], 1, 20, 8, plants_names[plant_index][1], 1);
              ledMatrix.putbitmap(0, 0, plants_icons[plant_index][0], 16, 16, GREEN);
              ledMatrix.putbitmap(16, 0, plants_icons[plant_index][1], 16, 16, GREEN);
              ledMatrix.sendframe();
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
  laststep1=safe_out+1400; // cilynder offsets
  
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
  laststep2=safe_out+1400; // cilynder offsets
  
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
