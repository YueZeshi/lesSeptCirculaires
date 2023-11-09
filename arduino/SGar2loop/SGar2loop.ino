#define freq 20
const float Dt = 1000000/freq;//set loop frequency
#define PWMB 12
#define NE2 18
#define NE1 31
#define BL1 34
#define BL2 35
#define EM 10
unsigned long looptime=0;
unsigned long lastlooptime=0;//measuring loop time for further delay
long count=0;//position measured by encoder, by pulses
long lastcount=0;
unsigned long du_reponse,t_command;
int tc=200,tc_pp=40;
int V;
double c_set,c_cur;
int tape_command,configuration_mode;
int note;
float Vcount=0;//rotation snstantaneous speed, pulse/s
//long counttime[3]={0,0,0};//time between counts, to calculate rotation speed

void setup() {
  c_set=0;
  du_reponse=10;//à déterminer grâce au temps de réponse maximal de moteur
  tape_command=0;
  configuration_mode=1;
  Serial2.begin(9600);
  Serial.begin(9600);
  pinMode(PWMB,OUTPUT);
  pinMode(NE2,INPUT_PULLUP);
  pinMode(NE1,INPUT);
  pinMode(BL1,OUTPUT);
  pinMode(BL2,OUTPUT);
  pinMode(EM,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(NE2),inter,RISING);
  digitalWrite(EM,LOW);
}

void loop() {
  // read command
  lastlooptime=looptime;
  if(Serial2.available()){
    note=Serial2.read(); 
    switch(note){
      case 0:
      case 20:
      case 30:
      case 21:
      case 31:
        configuration_mode=1;
        break;
      default:
        configuration_mode=0;
        c_set = (note-1)*50;
        tape_command=1;
        t_command=millis();
        break;
    }
  }
  if(configuration_mode){
    switch(note){
      case 20:
        rotation(5);
        delay(tc);
        break;
      case 21:
        rotation(2);
        delay(tc_pp);
        break;
      case 30:
        rotation(-5);
        delay(tc);
        break;
      case 31:
        rotation(-2);
        delay(tc_pp);
        break;
      default:
        stop();    
    }
    note=0;
    count=0;
  }
  else{
    //c_set=sweep(601);
    int V_set=-pida(c_set,count,5,3,0);//angle loop controller
    //int V_set=sweep(500);
    //Vcount=(int(counttime[0]>0)+int(counttime[1]>0)+int(counttime[2]>0))*1000000/(abs(counttime[0])-abs(counttime[2]));//measure average speed
    Vcount=(count-lastcount)*1000000/Dt;
    lastcount=count;
    V=pidv(V_set,Vcount,0.008,0.2,0.000001);//speed loop controller //double loop, set your pid parameters here
    V = constrain(V,-9.0,9.0);
    rotation(V);
    Serial.println(count);
    if(tape_command&&(millis()-t_command)>du_reponse){//abs(der_err)<2 &&abs(err)<5 && tape_fin==0){
      digitalWrite(EM,HIGH);
      delay(50);//30 mieux
      digitalWrite(EM,LOW);
      tape_command=0;
    }
  }
  while(1){
    looptime=micros();
    if(looptime>=lastlooptime+Dt){
      break;
    }
  }
}



void rotation(int V){
  if(V>=0){
    int PWM = V*255/12;
    analogWrite(PWMB,PWM);
    digitalWrite(BL1,LOW);
    digitalWrite(BL2,HIGH);
  }
  else{
    int PWM = -V*255/12;
    analogWrite(PWMB,PWM);
    digitalWrite(BL2,LOW);
    digitalWrite(BL1,HIGH);
  }
}

void inter(){
//  counttime[2]=counttime[1];
//  counttime[1]=counttime[0];
  if(digitalRead(NE1)>0){
    count++;
//    counttime[0]=micros();
  }
  else{
    count--;
//    counttime[0]=-micros();
  }
}

float iv=0;  //pid controller of speed and its variables
float dv=0;
float sumerrorv=0;
float lasterrorv=0;
#define i_limv 9
float pidv(float speedcommand,float speedmeasured,float kp,float ki,float kd){
  float error=speedmeasured-speedcommand;
  sumerrorv+=error*Dt/1000000;
  iv=ki*sumerrorv;
  if(iv>i_limv) sumerrorv=i_limv;
  if(iv<-i_limv) sumerrorv=-i_limv;
  dv=kd*(error-lasterrorv)*1000000/Dt;
  lasterrorv=error;
  return kp*error+iv+dv;
}

float ia=0;  //pid controller of angle and its variables
float da=0;
float sumerrora=0;
float lasterrora=0;
#define i_lima 500
float pida(float speedcommand,float speedmeasured,float kp,float ki,float kd){
  float error=speedmeasured-speedcommand;
  sumerrora+=error*Dt/1000000;
  ia=ki*sumerrora;
  if(ia>i_lima) sumerrora=i_lima;
  if(ia<-i_lima) sumerrora=-i_lima;
  da=kd*(error-lasterrora)*1000000/Dt;
  lasterrora=error;
  return kp*error+ia+da;
}

void stop(){
analogWrite(12,0);
}

int sweep_init=0;//test sweep function
int direction_flag=0;
int sweep(int motorspeed){ //motor idling
  if(sweep_init==0){
    sweep_init=1;
    direction_flag=0;
  }
  else{
    if(count>600){
      direction_flag=1;
    }
    if(count<-600){
      direction_flag=0;
    }
  }
  if(direction_flag==0){
    return motorspeed;
  }
  if(direction_flag==1){
    return -motorspeed;
  }
}
