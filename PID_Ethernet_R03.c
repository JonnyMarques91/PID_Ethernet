#include <TimerOne.h>
#include <Ethernet.h>
#include <UbidotsEthernet.h>
#include <SPI.h>

/***** POST *****/
char const * TOKEN = "A1E-YEyfTZX6P5Fmnk0mGypTzQo8OhA5de";    // Assign your Ubidots TOKEN
char const * VARIABLE_LABEL_1 = "temperature";                // Assign the unique variable label to send the data
char const * VARIABLE_LABEL_2 = "power";                      // Assign the unique variable label to send the data
char const * VARIABLE_LABEL_3 = "presence";                   // Assign the unique variable label to send the data

/***** ID's (Não utilizados...) *****/
char const * TEMP_ID = "5bc52e96c03f974c1670ad02";            // Assign the unique variable label to send the data
char const * POWER_ID = "5bc52ea1c03f974c1670ad03";           // Assign the unique variable label to send the data

/***** GET *****/
char const * DEVICE_LABEL = "PID";            // Assign the unique device label
char const * VARIABLE_LABEL_4 = "Set_Point";  // Assign the unique variable label to get the last value
char const * VARIABLE_LABEL_5 = "kp";         // Assign the unique variable label to get the last value
char const * VARIABLE_LABEL_6 = "ki";         // Assign the unique variable label to get the last value
char const * VARIABLE_LABEL_7 = "kd";         // Assign the unique variable label to get the last value

/* Enter a MAC address for your controller below */
/* Newer Ethernet shields have a MAC address printed on a sticker on the shield */
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

Ubidots client(TOKEN);  /* initialize the instance */

/***** Definições - Temperatura *****/
#define PINO_TERMISTOR A15       // Pino do Termistor  
#define TERMISTOR_NOMINAL 10000  // Valor do termistor na temperatura nominal
#define TEMPERATURA_NOMINAL 25   // Temp. nominal descrita no Manual
#define N_AMOSTRAS 5             // Número de amostragens para Média
#define COEFICIENTE 3977         // Beta do Termistor
#define V_RESISTOR 10000         // Valor do resistor em série (Mesmo do Termistor)
/***** Definições - PID *****/
#define pSENSOR   A1
#define pCONTROLE 3
#define pPRESENCE 33
/***** Definições - Potência *****/
#define triacApin 13             //Define que o Dimmer será comandado pelo pino 13

/***** Variáveis do Controle de Temperatura *****/ 
volatile float temperatura;
volatile int bPresence;

int amostras;
int i; 

/***** Variáveis do Controle de Potência *****/
int frequencia = 60;
int stateTriacA = 0;
int power = 0;                    //Inicializa variável que controla potência na carga com 0 (Desligada)
int teste = 0;
int CNT = 0;

/***** Variáveis de Controle PID *****/
double error = 0;
double temperature;
double lastTemperature;

double kP = 5.00;
double kI = 0.00;
double kD = 0.00;

double 
  P = 0,
  I = 0,
  D = 0;

double PID = 0;

/***** Inicializações *****/
double SetPoint = 45;
int ControlePwm = 50;

long lastProcess = 0;
float deltaTime = 0;

void setup() 
{
  /* INIT Comunicação */
  Serial.begin(9600);
  /* INIT Temperatura */
  analogReference(EXTERNAL);
  /* INIT Potência */
  pinMode(triacApin, OUTPUT);
  digitalWrite(triacApin, HIGH);
  Timer1.initialize();                              //Inicializa biblioteca TimerOne para a frequência
  attachInterrupt(0, zero_cross_detect, FALLING);   //Interrupção para detecção do Zero-Cross

  /* INIT PID */
  pinMode(pSENSOR, INPUT);
  pinMode(pCONTROLE, OUTPUT);
  pinMode(pPRESENCE, INPUT);

  /* INIT Conexão Ethernet */
  Serial.print(F("Starting ethernet..."));
  if (!Ethernet.begin(mac))   Serial.println(F("failed"));
  else                        Serial.println(Ethernet.localIP());

  /* Atraso na inicialização do Ethernet Shield */
  delay(2000);
  Serial.println(F("Ready"));
}

void loop() 
{

  Controle_Temperatura();
  bPresence = digitalRead(pPRESENCE);

  /***** Realiza controle quando há presença *****/
  if (bPresence != 0)
  {
    Controle_PID();
    /***** Limita potência no intervalo 0% ~ 100% *****/ 
    if (ControlePwm > 100)    power = 100;
    else if (ControlePwm < 0) power = 0; 
    else                      power = ControlePwm;
  }
  else  power = 0;

  /***** Monitoração dos valores no console *****/
  Serial.print("SP= "); 
  Serial.print(SetPoint);
  Serial.print(" ; T= "); 
  Serial.print(temperatura);
  Serial.print(" ; P= "); 
  Serial.print(power);
  Serial.print(" ; Presence= "); 
  Serial.print(bPresence);
  Serial.println(" ;"); 

  /* Get e Post à cada 5 segundos */
  if (CNT > 100)
  {
    CNT = 0;
    Ethernet_GetControl();
    Ethernet_PostControl();
  }

  /***** Periódica de 50ms *****/
  CNT++;
  delay(50);
}

/***** Ethernet - GET *****/
void Ethernet_GetControl()
{
  Ethernet.maintain();
  
  /***** GET *****/
  int   getSP = client.getValue(DEVICE_LABEL, VARIABLE_LABEL_4);
  float getkP = client.getValue(DEVICE_LABEL, VARIABLE_LABEL_5);
  float getkI = client.getValue(DEVICE_LABEL, VARIABLE_LABEL_6);
  float getkD = client.getValue(DEVICE_LABEL, VARIABLE_LABEL_7);

  SetPoint = getSP;
  kP = getkP;
  kI = getkI;
  kD = getkD;
}

/***** Ethernet - POST *****/
void Ethernet_PostControl()
{
  Ethernet.maintain();

  /* Variáveis locais para envio de dados */
  float postTemperatura = temperatura;
  int postPower = power;

  /* Sending values to Ubidots */
  /***** POST *****/
  client.add(VARIABLE_LABEL_1, postTemperatura);  //Temp_Real
  client.add(VARIABLE_LABEL_2, postPower);        //Potência
  client.add(VARIABLE_LABEL_3, bPresence);        //Acrescentar sensor de presença
  
  client.sendAll();
}

void Controle_Temperatura()
{
    /* Inicializa média = 0 */
    float media = 0;
    int i = 0;
    amostras = 0;
 
    /* Acumula N amostras do sinal analógico à cada 10ms */
    while (i< N_AMOSTRAS)
    {
        amostras += analogRead(PINO_TERMISTOR);
        i++;
        delay(10);
    }

    /* Calcula valor médio das amostras */
    media = amostras / N_AMOSTRAS;

    /* Converte o valor da tensão média em resistência */
    media = 1023 / media - 1;
    media = V_RESISTOR / media;
  
    /* Faz o Cálculo pelo método do Fator Beta */
    temperatura  = media / TERMISTOR_NOMINAL;               // (R/Ro)
    temperatura  = log(temperatura);                        // ln(R/Ro)
    temperatura /= COEFICIENTE;                             // 1/B * ln(R/Ro)
    temperatura += 1.0 / (TEMPERATURA_NOMINAL + 273.15);    // + (1/To)
    temperatura  = 1.0 / temperatura;                       // Inverte o valor
    temperatura -= 273.15;                                  // Converte para Celsius
}

/* Rotina para cálculo dos fatores PID */
void Controle_PID()
{
    error = (SetPoint - temperatura);                     //Erro é a diferença entre o Set-Point e a Temperatura Real
    deltaTime = (millis() - lastProcess)/1000.0;          //Tempo gasto para execução do último processo
    lastProcess = millis();

    /* Cálculo do fator Proporcional */
    P = error * kP;
    /* Cálculo o fator Integral */
    I += (error * kI) * deltaTime;
    /* Cálculo do fator Derivativo */
    D = ((lastTemperature - temperatura) * kD) / deltaTime;

    /* Última temperatura - Referência para próximo cálculo*/
    lastTemperature = temperatura;

    /* Soma das parcelas */
    PID = P + I + D;
    
    /* Converção para Controle */
    ControlePwm = (PID + 50);
}

/* Rotina de detecção do Zero-Cross e interrupção */
void zero_cross_detect()
{
  if(power > 0)
  {
    long dimtime = int(map(power,0,100,8000,150));  //Calcula o atraso para o disparo do TRIAC
    Timer1.attachInterrupt(gateTRIAC, dimtime);     //Associa a função gateTRIAC com Interrupção do TIMER1
    Timer1.start();                                 //Inicia contagem TIMER1
  }
  else
  {
    digitalWrite(triacApin, HIGH);   //Mantém gate do TRIAC desativado.
    Timer1.stop();
  }
}
 
/* Trata interrupção do Timer 1 */
void gateTRIAC ()
{                                 //Trata interrupção do TIMER1 gerando pulso no gate do TRIAC
  digitalWrite(triacApin, LOW);   //Dispara o TRIAC
  delayMicroseconds(100);         //Aguarda 100 microsegundos para garantir disparo do TRIAC
  digitalWrite(triacApin, HIGH);  //Desabibilta gate do TRIAC
  Timer1.stop();
}

