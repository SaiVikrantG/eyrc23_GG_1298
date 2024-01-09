# define attach_offset 22
# define read_offset 1
# define weighted_mean 15
float sensor_average,sensor_sum;
float kp = 4.0;
float ki = 0.0;
float kd = 0.3;
float p;
float i;
float d;
float lp;
// float j ;
float error;
float correction;
float sp =0.0;
int pos,sensor[5];
void setup(){
  Serial.begin(115200);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(34, INPUT);
  pinMode(19, INPUT);
}
void loop(){
    int j =0;
    sensor_average = 0;
    sensor_sum = 0;

    // for (int i = -1; i <= 1; i++)
    // {
    // sensor[i+read_offset] = !digitalRead(i+attach_offset);
    // sensor_average += sensor[i+read_offset] * i * weighted_mean;   //weighted mean   
    // sensor_sum += int(sensor[i+read_offset]);
    // }
    sensor_average = (!digitalRead(19)*(-2)+!digitalRead(21)*(-1)+!digitalRead(22)*(0)+!digitalRead(23)*(1)+!digitalRead(34)*(2))*weighted_mean;
    sensor_sum = (!digitalRead(19)+!digitalRead(21)+!digitalRead(22)+!digitalRead(23)+!digitalRead(34));
    if(sensor_sum){
        pos = int(sensor_average / sensor_sum);
        Serial.print("Sensor Average: ");
        Serial.println(sensor_average);
        Serial.print("Sensor Sum: ");
        Serial.println(sensor_sum);
        Serial.print("Position: ");
        Serial.println(pos);
        error = pos - sp;
        p = error;
        j += p;
        d = p - lp;
        lp = p;
        correction = int(kp * p + ki * j + kd * d);
        Serial.print("error: ");
        Serial.println(error);
        Serial.print("correction: ");
        Serial.println(correction);
    }
      
    else
        Serial.println("divide by zero");

    delay(500);

} 