void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(Serial.available())
{
  int x = Serial.read();
  if(x >= 210)
  {
    Serial.println(x);
  }
}
}
