#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display


#define INJ_PIN 2 //Форсунка
#define SPEED_PIN 3 //Датчик скорости
#define BTN_PIN 8 //Кнопка
#define RPM_PIN 6 //Обороты двигателя
#define VOLT_PIN A3 //Вольтметр
#define TEMP_ENG_PIN A1 //Температура двигателя
#define TANK_PIN A2 //Датчик уровня топлива
#define RADIO_PIN 9
#define ONE_WIRE_BUS 13 // сигнальный провод датчика
#define VOLUP 0x04
#define VOLDOWN 0x05
#define PULSEWIDTH 555
#define ADDRESS 0x47//47
#define value_radio  3 //на сколько изменить громкость радио на скорости
#define speed_radio_up  85 //на какой скорости менять громкость радио вверх
#define speed_radio_down  70 //на какой скорости менять громкость вниз
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

const int fuel_inj_flow = 152; //роизводительность форсунки с учетом корректировки по давлению топлива.
float temp_inside;
float volt = 12;
int temp, temp_eng;
int tank, tank_lvl,  refuel;
float delta;
const int idx_array  = 200; //количество значений в массиве для топлива
int tank_avg_array[idx_array];
byte idx_avg;
unsigned long tank_avg;

boolean btnState, btnFlag, reset, Shutdown;
unsigned long debounceTimer;

unsigned long rpm;
float liters, l_h, liters_count;
unsigned long fuel_count;
float l_100km,  l_100km_avg, l_100km_count;
byte spd;
unsigned long spd_avg;
unsigned long odo_trip, prev_odo, spd_avg_count, odo_count, count;

unsigned long timer0, timer1, timer2, timer3;
byte screen, value_radio_count;

volatile unsigned long inj_time, inj_time_count;
volatile unsigned long spd_count;


byte size_temp_matrix;
const int temp_matrix [] [2] {
  {705, -10},
  {695, -5},
  {685, 0},
  {675, 5},
  {665, 10},
  {660, 15},
  {655, 20},
  {645, 25},
  {635, 30},
  {625, 35},
  {615, 40},
  {595, 45},
  {580, 50},
  {565, 55},
  {545, 60},
  {515, 65},
  {470, 70},
  {450, 75},
  {435, 80},
  {430, 81},
  {425, 82},
  {420, 83},
  {415, 84},
  {410, 85},
  {405, 86},
  {400, 87},
  {395, 88},
  {390, 89},
  {385, 90},
  {380, 91},
  {375, 92},
  {370, 93},
  {365, 94},
  {360, 95},
  {355, 96},
  {350, 97},
  {345, 98},
  {340, 99},
  {335, 100},
  {330, 101},
  {325, 102},
  {320, 103},
  {315, 104},
  {310, 105},
};


void setup() {

  TCCR1A = 0; //таймеры
  TCCR1B =  (1 << CS11) | (1 << CS10);
  lcd.begin();
  lcd.backlight();
  pinMode(INJ_PIN, INPUT_PULLUP);
  pinMode(RPM_PIN  , INPUT_PULLUP);
  pinMode(SPEED_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(0, inj_func, CHANGE);
  attachInterrupt(1, spd_func, FALLING);
  pinMode(RADIO_PIN, OUTPUT);
  digitalWrite(RADIO_PIN, HIGH);
  pinMode(BTN_PIN, INPUT_PULLUP);
  //Serial.begin(9600);
  EEPROM.get (0, odo_count);
  EEPROM.get (4, liters_count);
  EEPROM.get (8, tank_lvl);
  EEPROM.get (12, screen);
  l_100km_count = liters_count * 1e5 / odo_count;
  size_temp_matrix = sizeof (temp_matrix) / (sizeof (int) * 2) - 1;
  sensor.begin(); // начинаем работу с датчиком
  sensor.setResolution(9);// устанавливаем разрешение датчика от 9 до 12 бит - 0,5, 0,25 , 0,125 и 0,0625 °C.
  sensor.requestTemperatures(); // отправляем запрос на измерение температуры
  temp_inside = sensor.getTempCByIndex(0);// считываем данные из регистра датчика
}


void loop() {
  button();

  if (volt <= 10 && digitalRead(INJ_PIN) == LOW && !Shutdown)  {
    EEPROM.put (0, odo_count += odo_trip);
    EEPROM.put (4, liters_count += liters);
    EEPROM.put (8, tank_lvl);
    EEPROM.put (12, screen);
    Shutdown = true;
  }
  if (Shutdown && volt > 11) { //иногда при старте выполняются все условия записи eeprom
    EEPROM.put (0, odo_count -= odo_trip);
    EEPROM.put (4, liters_count -= liters);
    idx_avg = 0;
    Shutdown = false;
  }

  if (millis() - timer0 >= 666) {
    timer0 = millis();
    rpm = 15e6 / pulseIn(RPM_PIN , LOW, 100000); //15e6 / rpm Перевод в об/мин
    display();
  }

  delta = millis() - timer1;
  if (delta >= 1000) {
    timer1 = millis();
    measure();
    lvl();
    radio();
  }

  if (millis() - timer2 >= 10101) {
    timer2 = millis();
    temp_measure();
  }
}

void measure() {
  odo_trip = spd_count * 2; //10 импульсов за 1м (5 периодов) -> 0.2м за период. *0.96 зимние шины
  spd = (odo_trip - prev_odo) * 36 * (1000.0 / delta) / 100; 
  prev_odo = odo_trip;
  odo_trip /= 10; //в метры
  refuel = tank_lvl / l_100km_count * 100;
  if (rpm > 0) {
    l_h =  inj_time * rpm * 2e-5 * fuel_inj_flow; //моментальный расход
    l_h *= 1e-4;
    fuel_count = inj_time_count * 1e-3 * fuel_inj_flow * 4 / 60; // мм3, max 43 L за поездку. Попарный впрыск, но 1оборот - 1 впрыск * 2 форсунки
    liters = fuel_count * 1e-6;
    spd_avg_count += spd;
    count++;
    spd_avg = spd_avg_count / count;
  }
  else l_h = 0;
  if (odo_trip > 1000) l_100km_avg = liters * 1e5 / odo_trip;
  if (spd > 0 ) l_100km = l_h * 100 / spd;
  else l_100km = 0;
  volt = analogRead(VOLT_PIN) * 15.7 / 1023.0 + 2.9; //5*(4.7+10)/4.7 + 2.8 рез плечо + потери на диоде и транзисторе
  inj_time = 0;
}


void temp_measure() {
  sensor.requestTemperatures(); // отправляем запрос на измерение температуры
  temp_inside = sensor.getTempCByIndex(0);// считываем данные из регистра датчика
  if (rpm > 0) {
    temp = analogRead(TEMP_ENG_PIN);
    temp *= 14.3 / volt;
    int i = size_temp_matrix;
    do {
      temp_eng = temp_matrix [i][1] ;
      i--;
    } while (temp > temp_matrix [i][0] && i > 0 ); 
  }
}


void lvl () {
  if (rpm > 0) {
    tank = analogRead(TANK_PIN);
    tank_avg_array[idx_avg] = tank;
    if (++idx_avg >= idx_array) {
      idx_avg = 0;        // перезаписывая самое старое значение
      tank_avg = 0;          //обнуляем среднее
      for (int i = 0; i < idx_array; i++) {
        tank_avg += tank_avg_array[i];
      }
      tank_avg /= idx_array;
      tank_lvl = 60 - ((tank_avg - 165) / 9.5);
      if (tank_lvl > 60) tank_lvl = 60;
      if (tank_lvl < 0) tank_lvl = 0;
    }
  }
}


void display() {
  if (millis() < 3000) {
    lcd.setCursor(0, 0);
    lcd.print("     PEUGEOT     ");
  } else if (Shutdown) {
    lcd.setCursor(0, 0);
    lcd.print("     PEUGEOT     ");
    lcd.setCursor(0, 1);
    lcd.print("                       ");
  } else if (volt < 11.6 && !Shutdown) {
    timer3++;
    if (timer3 > 20) {
      lcd.setCursor(0, 0);
      lcd.print(" LOW BATTERY!!!     ");
      lcd.setCursor(0, 1);
      lcd.print("                       ");
    }
  } else if (reset) {
    lcd.setCursor(0, 0);
    lcd.println("     reset?       ");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    if (screen == 2) {                     //после отпускания screen++, следущее нажатие 2
      EEPROM.put (0, odo_count = 0);
      EEPROM.put (4, liters_count = 0);    //обнуляем
      reset = false;
      screen = 0;
      lcd.setCursor(0, 0);
      lcd.println("       YES        ");
    }
    if (millis() - debounceTimer > 5000) {
      lcd.setCursor(0, 0);
      lcd.println("       NO        ");
      reset = false;//fasle прерывает
      screen = 0;
    }
  } else {
    switch (screen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print(String(String(liters_count, 1) + "L " + String (refuel) + "km all    " ));
        lcd.setCursor(0, 1);
        lcd.print(String(String(odo_count / 1000) + " ODO " + String (l_100km_count, 1) + " AVG    " ));
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print(String(String(liters) + "L   ") +  String(millis() / 60000) + " min    ");
        lcd.setCursor(0, 1);
        lcd.print(String(String(odo_trip / 1000.0, 1) + " ODO " + String(l_100km_avg, 1) + " AVG    "));
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print(String(String(spd_avg) + " spd   " + String(l_h, 2) + "L.h      "));
        lcd.setCursor(0, 1);
        lcd.print(String(String(l_100km_avg, 1) + " AVG " + String(l_100km, 1) + "L.100     "));
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print(String(String(rpm) + "rpm " + String(l_100km, 1) + "L.100     "));
        lcd.setCursor(0, 1);
        lcd.print(String(String (inj_time) + "us " + String(spd) + " km.h         "));
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print(String(String(volt, 1) + " V  " +  String(temp_inside, 0) + " C ins    "));
        lcd.setCursor(0, 1);
        lcd.print(String(String(tank_lvl) + " L    " + String(temp_eng) + " C eng      " ));
        break;
    }
  }
}

void inj_func() {
  if (bitRead(PIND, INJ_PIN) == LOW) {
    TCNT1H = 0; //обнуляем таймер
    TCNT1L = 0;
  }
  else {
    inj_time = ((unsigned long)TCNT1L | ((unsigned long)TCNT1H << 8)) * 4; //обрабатываем значение таймера
    if (inj_time < 20000) inj_time_count += inj_time; //при старте двигателя ардуино происходит скачок напряжения, макс время впрыска
  }
}

void spd_func() {
  spd_count++;
}

void button() {
  btnState = !digitalRead(BTN_PIN);  // читаем состояние кнопки с инверсией. 1 - нажата, 0 - нет
  if (btnState && !btnFlag && (millis() - debounceTimer > 100)) {
    btnFlag = true;              // запомнили что нажата
    debounceTimer = millis();    // запомнили время нажатия
  }
  if (btnState && btnFlag && millis() - debounceTimer > 2000)  { //долгое нажатие
    reset = true;
    screen = 0;
  }
  if (!btnState && btnFlag && (millis() - debounceTimer > 100)) {    // если отпущена и была нажата (btnFlag 1)
    btnFlag = false;             // запомнили что отпущена
    debounceTimer = millis();    // запомнили время отпускания
    screen++;
    if (screen > 4) screen = 0;
  }
}

void radio() {
  if (spd > speed_radio_up) {
    if (value_radio_count < value_radio) {
      SendCommand(VOLUP);
      delay(2);
      SendCommand(VOLUP);
      delay(20);
      value_radio_count++;
    }
  } 
  else if (spd < speed_radio_down){
    if (value_radio_count > 0) {
      SendCommand(VOLDOWN);
      delay(2);
      SendCommand(VOLDOWN);
      delay(20);
      value_radio_count--;
    }
  }
}


// Send a value (7 bits, LSB is sent first, value can be an address or command)
void SendValue(unsigned char value) {
  unsigned char i, tmp = 1;
  for (i = 0; i < sizeof(value) * 8 - 1; i++) {
    if (value & tmp)  // Do a bitwise AND on the value and tmp
      SendOne();
    else
      SendZero();
    tmp = tmp << 1; // Bitshift left by 1
  }
}

// Send a command to the radio, including the header, start bit, address and stop bits
void SendCommand(unsigned char value) {
  unsigned char i;
  Preamble();
  for (i = 0; i < 1; i++) {           // Repeat address, command and stop bits three times so radio will pick them up properly
    SendValue(ADDRESS);               // Send the address
    SendValue((unsigned char)value);  // Send the command
    Postamble();                      // Send signals to follow a command to the radio
  }
}

// Signals to transmit a '0' bit
void SendZero() {
  digitalWrite(RADIO_PIN, LOW);      // Output LOW for 1 pulse width
  delayMicroseconds(PULSEWIDTH);
  digitalWrite(RADIO_PIN, HIGH);       // Output HIGH for 1 pulse width
  delayMicroseconds(PULSEWIDTH);
}

// Signals to transmit a '1' bit
void SendOne() {
  digitalWrite(RADIO_PIN, LOW);      // Output LOW for 1 pulse width
  delayMicroseconds(PULSEWIDTH);
  digitalWrite(RADIO_PIN, HIGH);       // Output HIGH for 3 pulse widths
  delayMicroseconds(PULSEWIDTH * 3);
}

// Signals to precede a command to the radio
void Preamble() {
  // HEADER: always HIGH (1 pulse width), LOW (16 pulse widths), HIGH (8 pulse widths)
  digitalWrite(RADIO_PIN, HIGH);       // Make sure output is HIGH for 1 pulse width, so the header starts with a rising edge
  delayMicroseconds(PULSEWIDTH * 1);
  digitalWrite(RADIO_PIN, LOW);      // Start of header, output LOW for 16 pulse widths
  delayMicroseconds(PULSEWIDTH * 16);
  digitalWrite(RADIO_PIN, HIGH);       // Second part of header, output HIGH 8 pulse widths
  delayMicroseconds(PULSEWIDTH * 8);

  // START BIT: always 1
  SendOne();
}

// Signals to follow a command to the radio
void Postamble() {
  // STOP BITS: always 1
  SendOne();
  SendOne();
}
