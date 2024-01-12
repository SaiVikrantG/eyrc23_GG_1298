#define read_offset 1
#define attach_offset 21
float sp=0;
long sensor[] = {0, 1, 2}; // leftmost - 0, rightmost - 4

int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26;
// speeds
int rspeed;
int lspeed;
const int base_speed = 255;
/// for pid
int j=0;
int pos;
long sensor_average;
int sensor_sum;

int button = 3; // to be pressed to find set point

float p;
float i;
float d;
float lp;
float error;
float correction;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
int startSignal = 0;


void get_PID_consts();
void pid_calc();
void calc_turn();
void motor_drive(int, int);
void stop();
void updateWebValues();

void setup()
{
    // sensors
    pinMode(21, INPUT);
    pinMode(22, INPUT);
    pinMode(23, INPUT);
    pinMode(34, INPUT);
    pinMode(19, INPUT);

    // motors
    pinMode(rmf, OUTPUT);
    pinMode(rmb, OUTPUT);
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);

    Serial.begin(115200);
}

void loop()
{
    if (startSignal)
        {
            pid_calc();
            calc_turn();
        }
    else
        {
            stop();
        }
    }

void stop()
    {
        analogWrite(enr, 0);
        analogWrite(enl, 0);
        digitalWrite(rmf, LOW);
        digitalWrite(rmb, LOW);
        digitalWrite(lmf, LOW);
        digitalWrite(lmb, LOW);
    }

void pid_calc()
{
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
    else 
        stop();
    delay(500); 
    }
}
void calc_turn()
{
    rspeed = base_speed - correction;
    lspeed = base_speed + correction;

    if (rspeed > 255)
        rspeed = 255;

    if (lspeed > 255)
        lspeed = 255;

    if (rspeed < -255)
        rspeed = -255;

    if (lspeed < -255)
        lspeed = -255;

    motor_drive(rspeed, lspeed);
}

void motor_drive(int right, int left)
    {
        if (right > 0)
            {
                analogWrite(enr, right);
                digitalWrite(rmf, HIGH);
                digitalWrite(rmb, 0);
            }
        else
            {
                analogWrite(enr, abs(right));
                digitalWrite(rmf, LOW);
                digitalWrite(rmb, HIGH);
            }

        if (left > 0)
            {
                analogWrite(enl, left);
                digitalWrite(lmf, HIGH);
                digitalWrite(lmb, LOW);
            }
        else
            {
                analogWrite(enl, abs(left));
                digitalWrite(lmf, LOW);
                digitalWrite(lmb, HIGH);
            }
    }