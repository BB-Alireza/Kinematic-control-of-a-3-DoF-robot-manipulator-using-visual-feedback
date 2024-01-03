#include <Servo.h>

Servo R[4];
double R_V[4] {90 , 120 , 70 , 0} , Rate = 0.99;
double R_VA[4] {90 , 120 , 70 , 0};

struct MY_COMMAND {
  int Command;
  int Value;
  struct MY_COMMAND *Next;
};

struct MY_COMMAND *start , *end, *tmp1 , *tmp2;
char data[16] = {0};
String Val;

void setup() {
  // put your setup code here, to run once:

  R[0].attach(3);
  R[1].attach(5);
  R[2].attach(6);
  R[3].attach(9);

  R[0].write(R_VA[0]);
  R[1].write(R_VA[1]);
  R[2].write(R_VA[2]);
  R[3].write(R_VA[3]);

  start = new struct MY_COMMAND;
  start  -> Command = 0;
  start -> Value = 0;
  start -> Next = NULL;

  end = start;

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (start -> Next)
  {
    Serial.print(start->Next->Command);
    Serial.print(",");
    Serial.println(start->Next->Value);

    if (start->Next->Command == 0)
      Rate = ((double)(start->Next->Value) / 10000.0);
    else
      R_V[start->Next->Command-1] = start->Next->Value;

    tmp1 = start;
    start = start->Next;

    delete(tmp1);

  }

  for (int i = 0; i < 4; i++)
  {
    R_VA[i] *= Rate;
    R_VA[i] += (1 - Rate)*R_V[i];
    R[i].write(R_VA[i]);
  }
}

void serialEvent()
{
  while (Serial.available())
  {
      Serial.readBytesUntil(';' , data , 16);

      tmp2 = new struct MY_COMMAND;

      Val = String(data);
      
      tmp2 -> Command = data[0] - '0';
      tmp2 -> Value = Val.substring(1).toInt();
      tmp2 -> Next = NULL;

      end -> Next = tmp2;
      end = tmp2;

      for (int i = 0; i < 16; i++)
        data[i] = 0;
  }
}