/*
Bibliotecas utilizadas
lib_deps = 
	d03n3rfr1tz3/HC-SR04@^1.1.3
	dlloydev/ESP32 ESP32S2 AnalogWrite@^5.0.2
	waspinator/AccelStepper@^1.64
	adafruit/Adafruit GFX Library@^1.12.3
	adafruit/Adafruit SSD1306@^2.5.15
	br3ttb/PID@^1.2.1
*/
#include <AccelStepper.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

// ==== TIMING ====
unsigned long lastTempMillis = 0;

// Definição dos pinos do driver
#define dirPin 4
#define stepPin 2
#define motorInterfaceType 1
#define defaultSpeed 2000

// Ajuste conforme seu display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1    // -1 se o módulo não tem pino RST conectado
#define I2C_ADDRESS 0x3C // endereço I2C comum (0x3C ou 0x3D)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ajuste dos pinos I2C do ESP32 (padrão: SDA=21, SCL=22)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Pino do potenciômetro da velocidade
const int potPin = 32;
long speed = 0;

// Conexão do termistor
const int pinTermistorTemp = 34;
const int pinPotTemp = 33;
const int pwmPin = 13;
const int ponteHAct1Pin = 5;
const int ponteHAct2Pin = 18;

// Cooler PonteH
const int coolerPwmPin = 12;
const int coolerAct1Pin = 19;
const int coolerAct2Pin = 3;
const int coolerPWMDefault = 100;

// Parâmetros do termistor
const double beta = 3950.0;
const double r0 = 115000.0;
const double t0 = 273.0 + 25.0;
const double rx = r0 * exp(-beta / t0);

// Parâmetros do circuito de temperatura
const double vcc = 3.3;
const double R = 10000.0;

// Parâmetros do PID
double setpoint, input, output;
const double Kp = 0.5, Ki = 0.0, Kd = 2;

// Criação dos objetos
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Controle de temperatura
double speedAnt = 0.0;

// Cria instância do motor
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// ----- Tarefa do Core 0 - Controle de Temperatura -----
void Core0Monitor(void *pvParameters)
{
  for (;;)
  {
    // Le o sensor algumas vezes
    int soma = 0;
    int amostragem = 40;
    // coleta vários dados para melhor performance
    for (int i = 0; i < amostragem; i++)
    {
      soma = soma + ((long)analogRead(pinTermistorTemp));
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // calcula a media
    soma = soma / amostragem;

    // Determina a resistência do termistor
    double v = (vcc * soma) / 4095.0;
    double rt = ((vcc * R) / v) - R;

    // Calcula a temperatura
    double input = (long)((beta / log(rt / rx)) - 270);

    setpoint = (long)((120.0 * ((long)analogRead(pinPotTemp))) / 700.0);

    if ((setpoint) <= input)
    {
      output = 0.0;
    }
    else
    {
      if (input > 150 && input > (setpoint - 15))
      {
        output = 255;
      }
      else
      {
        output = 255;
      }
    }
    // pid.Compute(); // Quando implementado o PID é aqui que executa ele
    analogWrite(pwmPin, output); // Aplica no PWM

    Serial.print("PWM:");
    Serial.println(output);

    // Limpar e mostrar contagem
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Speed:");
    display.setCursor(70, 0);
    display.println(speed);
    display.setCursor(0, 20);
    display.println("Set:");
    display.setCursor(50, 20);
    display.println(setpoint);
    display.setCursor(0, 40);
    display.println("Tmp:");
    display.setCursor(50, 40);
    display.println(input);
    display.display();

    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ----- Tarefa do Core 1 - Controle de Movimento -----
void Core1Movimento(void *pvParameters)
{
  for (;;)
  {
    // Lê o valor do potenciômetro (0 a 4095)
    int potValue = analogRead(potPin);

    // Mapeia o valor para a faixa de velocidade (ex: -1000 a 1000 passos/s)
    speed = map(potValue, 0, 4095, defaultSpeed * -1, defaultSpeed);

    if (speed < 0)
    {
      // Define a velocidade ajustada
      stepper.setSpeed(-speed);
      speedAnt = -speed;

      // Executa o movimento contínuo na velocidade definida
      stepper.run();
    }
    else
    {
      speedAnt = 0;
    }
  }
}

void setup()
{
  delay(2000);
  Serial.begin(115200);

  // Inicializa I2C explicitamente com os pinos desejados
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicializa o display
  display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS);

  display.clearDisplay();
  display.setTextSize(1);

  // desenhando alguns elementos
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.drawLine(0, 20, SCREEN_WIDTH, 20, SSD1306_WHITE);
  display.fillCircle(110, 52, 6, SSD1306_WHITE);

  // Cria a task no Core 0
  xTaskCreatePinnedToCore(Core0Monitor, "Core0Monitor", 2048, NULL, 1, &TaskCore0, 0);

  // Cria a task no Core 1
  xTaskCreatePinnedToCore(Core1Movimento, "Core1Movimento", 2048, NULL, 1, &TaskCore1, 1);

  // Motor
  stepper.setMaxSpeed(defaultSpeed);
  stepper.setSpeed(0);

  pinMode(pwmPin, OUTPUT);

  // Inicializa PID
  input = 0; // inicializa variavel temperatura do sensor
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255); // PWM padrão de 8 bits

  // Controle Ponte H - PWM
  pinMode(ponteHAct1Pin, OUTPUT);
  pinMode(ponteHAct2Pin, OUTPUT);
  digitalWrite(ponteHAct1Pin, 0);
  digitalWrite(ponteHAct2Pin, 1);

  // Controle Ponte H - Cooler
  pinMode(coolerPwmPin, OUTPUT);
  pinMode(coolerAct1Pin, OUTPUT);
  pinMode(coolerAct2Pin, OUTPUT);
  digitalWrite(coolerAct1Pin, 1);
  digitalWrite(coolerAct2Pin, 0);
  analogWrite(coolerPwmPin, coolerPWMDefault);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}