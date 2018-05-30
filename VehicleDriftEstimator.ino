#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Filter.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
int e = 0, i = 0, c = 0;
double Vx = 0, Vy = 0;
int restx, resty;
double quaternion[4];
int l;
double Xk = 0, Pk = 1, Yk = 0, Pky = 1;
double m00, m01, m02;
double m10, m11, m12;
double m20, m21, m22;
double n0, n1, n2;
double minv[3][3]; // inverse of matrix m
ExponentialFilter<float> FilteredX(20, 0);
ExponentialFilter<float> FilteredY(20, 0);
ExponentialFilter<float> FilteredZ(20, 0);
ExponentialFilter<float> Angle(20, 0);
float Xcoord;
float Ycoord;
void setup() {
  Serial.begin(9600);

  
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  filter.begin(25);

 
  CurieIMU.setAccelerometerRange(2);
 
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  restx = 0;
  resty = 0;
  l = 0;
}

void loop() {
  float drift, driftaLenkis, driftaLenkisRad;
  int aix, aiy, aiz;
  int gix, giy, giz;
  float gx, gy, gz;
  float ax, ay, az;
  double a1, b1, c1, d1;


  // double R, Axr, Ayr, Azr, Rv,  Starpiba;

  float roll, pitch, heading, arez;

  unsigned long microsNow;
  
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
    arez = 0.0f;





    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw() - 180;

    rotationMatrix(pitch * 3.14 / 180, roll * 3.14 / 180, heading * 3.14 / 180);
    //    rotationTest();
    inverseMatrix();
    matrixMultiplication(ay, ax, az);


    //    FilteredZ.Filter(n2);
    //   float SmoothZ = FilteredZ.Current();
    //    float SmoothX = n0;
    //    float SmoothY = n1;
    //    float SmoothZ = n2;
    //    Vx = Vx + SmoothX * microsPerReading / 2000;
    //    Vy = Vy + SmoothY * microsPerReading / 2000;
    if (l < 2000000)
    {
      l++;
      heading -= 0.0055 * l; // žiroskopa noslieces kompensēšana
    }
    // 
    //    if(Vx > 0){
    //      Vx=Vx-0.1;
    //      }else if(Vx<0){
    //      Vx=Vx+0.1;
    //      }
    //      if(Vy > 0){
    //      Vy=Vy-0.1;
    //      }else if(Vy<0){
    //      Vy=Vy+0.1;
    //      }

    //


    
    if (n0 > 0.01 || n0 < -0.01) {
          FilteredX.Filter(n0);
    float SmoothX = FilteredX.Current();
         Vx = Vx + SmoothX * microsPerReading / 1000;
      restx = 0;
    } else {      //      if (Vx > 0.5) {
      Vx = Vx * 0.5;
      //      } else if (Vx < -0.5) {
      //        Vx = Vx * 0.5;
      //
      //      }
      restx++;
    }
    if (n1 > 0.01 || n1 < -0.01) {
          FilteredY.Filter(n1);
    float SmoothY = FilteredY.Current();
          Vy = Vy + SmoothY * microsPerReading / 1000;
      resty = 0;
    }
    else {
      //      if (Vy > 0.5) {
      Vy = Vy * 0.5;
      //      } else if (Vy < -0.5) {
      //        Vy = Vy * 0.5;
      //      }
      resty++;
    }

    if (restx >= 15) {          // pēc laika p
      restx = 0;
      Vx = 0;
    }
    if (resty >= 15) {
      resty = 0;
      Vy = 0;
    }




    
    //    if (Reset >= 10) {
    //      Reset = 0;
    //      // CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    //      // CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    //      // CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    //    }
    //    Xcoord = Xcoord  + Vx * microsPerReading / 1000;
    //    Ycoord  = Ycoord  + Vy * microsPerReading / 1000;


    //for(int j; j<100; j++){
    //  //g
    //}

//     Vx = -10;
//      Vy = 10;

      

    // heading = 360 - heading;
//
    if (Vx == 0 && Vy == 0) {
      arez = heading;
    } else if (Vx > 0 && Vy > 0) { //0-90
          arez = atan(Vy/Vx);
      arez = arez * 57.2958;
        } else if ( Vx > 0 && Vy < 0) { //270-360
         
         arez = atan(Vy/Vx);
          //  Serial.println(abs(arez));
          arez = 3 * 3.14 / 2 + abs(arez);
          arez = arez * 57.2958;
          
    
        } else if (Vx < 0 && Vy > 0) { //90-180
    
          arez = (-1) * atan(Vy/Vx) + (1 / 2) * 3.14;
          arez = arez * 57.2958;
    
        } else if (Vx < 0 && Vy < 0) { //180-270
          arez = atan(Vy/Vx) + 3.14;
          arez = arez * 57.2958;
        }


      

      

    
//    Vx = Vx + n0 * microsPerReading / 2000;
//    Vy = Vy + n1 * microsPerReading / 2000;


   if(arez >= 180){  //lai panāktu intervālu 0-180
    arez-=180;
   }



    Angle.Filter(arez);
    float Lenkis = Angle.Current();
    //arez = arez * 57.2958;


    //  toQuaternion(pitch, roll, heading);
    //   a1 = quaternion[0];
    //  b1 = quaternion[1];
    //  c1 = quaternion[2];
    //  d1 = quaternion[3];
    //  HamiltonProduct1();
    //  HamiltonProduct2(a1, b1, c1, d1);
//    Serial.print(n0*100);
//    Serial.print(" ");
//    Serial.print(n1*100);
//    Serial.print(" ");
//    Serial.println(n2*100);
    //    Serial.print(heading);
    //        Serial.print(" ");
    //         Serial.print(pitch);
    //        Serial.print(" ");
    //        Serial.print(roll);
    //        Serial.print(" ");
//                 Serial.print(ax);
//            Serial.print(" ");
//            Serial.print(ay);
//            Serial.print(" ");
//            Serial.println(az);
    //
    //  heading = heading - 0.6;
    //
    //    Serial.print(" ");
    //  Serial.print(" ");
    //Serial.println((az - 1)*10);
heading= abs(heading);
arez=abs(arez);
    
    if(heading>arez){
      driftaLenkis=heading-arez;
    }else{
      driftaLenkis=arez-heading;
    }
      drift=0;

      if(driftaLenkis<90){
        drift = sqrt(Vx*Vx+Vy*Vy)*driftaLenkis/90;
      }else{
        drift = sqrt(Vx*Vx+Vy*Vy)*(2-driftaLenkis/90);
      }
    // drift = sqrt(Vx*Vx+Vy*Vy)*sin(driftaLenkis*0.01745);
//
        Serial.print(drift);
        Serial.print("; ");
        Serial.print(driftaLenkis);
            Serial.print("; ");
            Serial.print(Vx);
            Serial.print("; ");
            Serial.println(Vy);
       //     Serial.print(" ");
       ///     Serial.println(1*sin(abs(driftaLenkis)));


    // Serial.println(" ");

    //    Serial.print(ax*10);
    //    Serial.print(" ");
    //    Serial.print(ay*10);
    //    Serial.print(" ");
    //    Serial.println((az - 1)*10);
    //
    //  Serial.print(" ");
    //   Serial.println(quaternion[3]);
    //    Serial.print(Xcoord);
    //    Serial.print(" ");
    //    Serial.println(Ycoord);

    microsPrevious = microsPrevious + microsPerReading;
  }



}


float convertRawAcceleration(int aRaw) {
 
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}


float convertRawGyro(int gRaw) {
  

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void rotationMatrix(double pitch, double roll, double yaw) {
  double c1 = cos(pitch);
  double s1 = sin(pitch);         //Tiek izveidoti jauni mainīgie, lai
  double c2 = cos(roll);          //izvairītos no vienādu matemātisku darbību atkārtošanas
  double s2 = sin(roll);
  double c3 = cos(yaw);
  double s3 = sin(yaw);
  m00 = c2 * c3;
  m01 = -c2 * s3;                 //Rotācijas matricas vērtības tiek saglabātas
  m02 = s2;                       //globālos mainīgajos
  m10 = c1 * s3 + c3 * s1 * s2;
  m11 = c1 * c3 - s1 * s2 * s3;
  m12 = -c2 * s1;
  m20 = s1 * s3 - c1 * c3 * s2;
  m21 = c3 * s1 + c1 * s2 * s3;
  m22 = c1 * c2;
}
void rotationTest() {
  n0 = m00 * 1 + m01 * 0 + m02 * 0;
  n1 = m10 * 1 + m11 * 0 + m12 * 0;
  n2 = m20 * 1 + m21 * 0 + m22 * 0;
}
void inverseMatrix() {
 
  double det = m00 * (m11 * m22 - m21 * m12) -
               m01 * (m10 * m22 - m12 * m20) +
               m02 * (m10 * m21 - m11 * m20);

  double invdet = 1 / det;

  
  minv[0][0] = (m11 * m22 - m21 * m12) * invdet;
  minv[0][1] = (m02 * m21 - m01 * m22) * invdet;
  minv[0][2] = (m01 * m12 - m02 * m11) * invdet;
  minv[1][0] = (m12 * m20 - m10 * m22) * invdet;
  minv[1][1] = (m00 * m22 - m02 * m20) * invdet;
  minv[1][2] = (m10 * m02 - m00 * m12) * invdet;
  minv[2][0] = (m10 * m21 - m20 * m11) * invdet;
  minv[2][1] = (m20 * m01 - m00 * m21) * invdet;
  minv[2][2] = (m00 * m11 - m10 * m01) * invdet;
}

void matrixMultiplication(double ax, double ay, double az) {
  n0 = minv[0][0] * ax + minv[0][1] * ay + minv[0][2] * az;
  n1 = minv[1][0] * ax + minv[1][1] * ay + minv[1][2] * az;
  n2 = minv[2][0] * ax + minv[2][1] * ay + minv[2][2] * az;
}




