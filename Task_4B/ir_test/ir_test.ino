
float sensor_average,sensor_sum;
int pos,sensor[];
void setup(){

}
void loop(){
    
    sensor_average = 0;
    sensor_sum = 0;

    for (int i = -2; i <= 2; i++)
    {
    sensor[i+read_offset] = digitalRead(i+attach_offset);
    sensor_average += sensor[i+read_offset] * i * 1000;   //weighted mean   
    sensor_sum += int(sensor[i+read_offset]);
    }

    pos = int(sensor_average / sensor_sum);

    Serial.print(sensor_average);
    Serial.print(' ');
    Serial.print(sensor_sum);
    Serial.print(' ');
    Serial.print(pos);
    Serial.println();
    delay(2000);

} 