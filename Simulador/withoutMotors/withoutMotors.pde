/**
 * Arm.
 *
 * The angle of each segment is controlled with the mouseX and
 * mouseY position. The transformations applied to the first segment
 * are also applied to the second segment because they are inside
 * the same pushMatrix() and popMatrix() group.
*/
//import processing.opengl.*;
float factor = 2.5;
float x, y;
float angle1 = 0.0;
float angle2 = 0.0;
float segLength = 76*factor;

int i = 0, j = 0;

double l1 = 76*factor, l2 = 76*factor;

double x_1 = 0 ,y_1 = 0;
double[] coordinates = new double[3];
float x_draw;
float y_draw;
float xDrawOld = 0, yDrawOld = 0;
int shapeSize = 1;
// variables of hypotrochoid
float a = 120;
float b_hypo = 20;
float h = 20;
//variables of hypotrochoid
float t = 0;
float resolution_hypo = 1.0/50.0;
float t_limit = 25;
//variables of draw form txt file
int index_coma;
//variables of star equations
float[] m, b;
float theta = 0;
int no_picos = 6;
float radius_1 = 100*factor;
float radius_2 = 150*factor;
//objects of the layers
PGraphics boundary;
PGraphics sandmark;
PGraphics dataScreen;
//mechanics
float microstepping = 16;
float degrees_per_step = (1.8*PI*2.0)/(4.0*180*microstepping);
float q1_real = 0, q2_real = PI;
int motor_working = 0;
long[] steps_of_q;
float step_factor;
int dir_q1,dir_q2, q1_mayor;
long steps_per_second = 3840;
long steps_per_frame = 0;
int tiempo;
//varibales of comunication with arduino
long[] data_arduino, data_arduino2;
double x_file, y_file;
float steps_q1_f, steps_q2_f;
float q1_real_og, q2_real_og;
int numero_linea = 0 ;
//
int numberPoint;

void setup() {
  size(1024, 1024);
  data_arduino = new long[3];
  data_arduino2 = new long[3];
  x = width * 0.5;
  y = height * 0.5;
  boundary = createGraphics(width, height);
  sandmark = createGraphics(width, height);
  dataScreen = createGraphics(width, height);
  boundary.beginDraw();
  boundary.endDraw();
  strokeWeight(40);
  stroke(255, 160);
  //draw_and_calculate();
  steps_of_q = new long[3];
  steps_of_q[1] = 0;
  steps_of_q[2] = 3200;
  frameRate(60);
  printArray(Serial.list());
  mySerial = new Serial(this, "COM10", 115200);
  values = new long[3];
  
}
boolean stop_condition = true;

void draw() {
  for (int drawIterates=0; drawIterates < 50 ; drawIterates++){
    pushMatrix();
    if (stop_condition){
      String value = mySerial.readStringUntil(10);
      if (value != null){
        print(value);
        if (value.length()>=8){
          if (value.indexOf("inicia") != -1){
            println("salio");
            numberPoint = 0;
            stop_condition = false;
          }
        }
      }
    }
    else if (motor_working == 0){
      if(mySerial.available() > 0){
        data(data_arduino);
        steps_of_q[1] += data_arduino[0];
        steps_of_q[2] += data_arduino[1] - data_arduino[0];
        //print(steps_of_q[1]," ");
        //println(steps_of_q[2]);
        steps_per_second =  data_arduino[2] * 1000;
        
        motor_working = 1;
      }
    }
    else{
      //for (int k = 0; k<2; k++){
      background(35);
      //find the greater value
      
      q1_real = steps_of_q[1] * degrees_per_step;
      q2_real = steps_of_q[2] * degrees_per_step;
      motor_working = 0;
      if (q1_real < 0)
        q1_real += 2*PI;
      if (q2_real < 0)
        q2_real += 2*PI;
      if (q1_real > 2*PI)
        q1_real -= 2*PI;
      if (q2_real > 2*PI)
        q2_real -= 2*PI;
      //put q's
      coordinates[1] = q1_real;
      coordinates[2] = q2_real;
      pushMatrix();
      segment(x, y, coordinates[1]);
      segment(segLength, 0, coordinates[2]);
      popMatrix();
      image(dataScreen, 0, 0);
      translate(x , y);
      //draw cicrcle an path
      x_draw = (float)x_dk(coordinates[1],coordinates[2]);
      y_draw = (float)-y_dk(coordinates[1],coordinates[2]);
      //draw_and_calculate();
      sandmark.beginDraw();
      sandmark.stroke(134,201,18);
      sandmark.strokeWeight(1);
      //sandmark.fill(134,201,18);
      sandmark.translate(x , y);
      sandmark.line(xDrawOld,yDrawOld,x_draw,y_draw);
      if (numberPoint == 1){
        sandmark.noStroke();
        sandmark.fill(255,0,0);
        sandmark.circle(x_draw, y_draw, 14);
      }
      sandmark.endDraw();
      xDrawOld = x_draw;
      yDrawOld = y_draw;
      image(sandmark, -x, -y);
      //draw the circle
      noFill();
      strokeWeight(3);
      stroke(255);
      circle(0, 0, 300*factor);
      
    }
    popMatrix();
  }
}

void segment(double x, double y, double a) {
  strokeWeight(40);
  stroke(255, 160);
  translate((float)x, (float)y);
  rotate((float)-a);
  line(0, 0, segLength, 0);
}

double x_dk(double q1, double q2){
  return l1*cos((float)q1) + l2*cos((float)(q1+q2));
}

double y_dk(double q1, double q2){
  return l1*sin((float)q1) + l2*sin((float)(q1+q2));
}
