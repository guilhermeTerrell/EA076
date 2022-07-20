/*Projeto 3 (parte 1) -> Gerar sinal PWM*/
/*Welter Mompeam Sozim              RA: 188625*/
/*Guilherme Augusto Amorim Terrell  RA: 168899*/

/*Importação de bibliotecas*/
#include <LiquidCrystal.h>

/*Definição dos pinos*/
#define pinPWMCanalA 11
#define pinPWMCanalB 3
#define pinEncoder 2
#define valorMaximoPWM 254
#define pinRS 8
#define pinEnable 4
#define pinD4 9
#define pinD5 10
#define pinD6 12
#define pinD7 13

/*Para fins de simução a velocidade começa em 100 e o estadoMotor começa em 1*/
unsigned int velocidadeMotorPWM = 100;
unsigned int velocidadeMotorPWMCanalA = 0;
unsigned int velocidadeMotorPWMCanalB = 0;
int estadoMotor = 1; /*0 -> motor parado; 1 -> ventilador; 2 -> exaustor; 3 -> freio*/
String comandoExecut = "";

/*Variáveis para estimar velocidade RPM*/
unsigned int pulsosPorVolta = 2; // Numero de pas/furos no encoder
unsigned int velocidadeEstimadaRPM = 0;
int counter = 0;
volatile unsigned long passagemDaPa = 0;
unsigned long verificaVelocidade = 25;
/*-------------------------------*/

/*variáveis da parte 1*/
/*-----------------------*/
char carac = '*';
String comando = "";
int velocidade = 0;
/*-----------------------*/

/*variáveis da parte 3*/
/*LCD -> 2 linhas de 16 caracteres*/
LiquidCrystal lcd(pinRS, pinEnable, pinD4, pinD5, pinD6, pinD7);

/*--------------------*/
  
/*Função que converte o valor contido na string comando*/
/*em um valor dentro da escala PWM (0, 254)*/
void convertPercentPWM(){
  if(comandoExecut.substring(0,4) == "VEL "){
    velocidadeMotorPWM = (velocidade*valorMaximoPWM)/100;
    Serial.println(velocidadeMotorPWM);
  }
}

/*Função que define o funcionamento do motor CC*/
/*comando = VENT* -> motor opera como ventilador (girar sentido horário)*/
/*comando = EXAUST* -> motor opera como exaustor (girar sentido anti-horário)*/
/*comando = PARA* -> motor parado*/

void maqEstadosMotor(){
  if(comandoExecut == "PARA*"){
    /*Motor parado*/
    estadoMotor = 0;
    Serial.println(estadoMotor);
  }
  if((estadoMotor == 0) && (comandoExecut == "VENT*")){
    /*motor que estava parado deve operar como ventilador*/
    estadoMotor = 1;
    comandoExecut = "";
    Serial.println(estadoMotor);
  }
  if((estadoMotor == 0) && (comandoExecut == "EXAUST*")){
    /*motor que estava parado deve operar como exaustor*/
    estadoMotor = 2;
    comandoExecut = "";
    Serial.println(estadoMotor);
  }
  if((estadoMotor == 1) && (comandoExecut == "VENT*")){
    /*continuar operando como ventilador*/
    estadoMotor = 1;
    comandoExecut = "";
    Serial.println(estadoMotor);
  }
  if((estadoMotor == 2) && (comandoExecut == "EXAUST*")){
    /*contnuar operando como exaustor*/
    estadoMotor = 2;
    comandoExecut = "";
    Serial.println(estadoMotor);
  }
  if((estadoMotor == 1 || estadoMotor == 3) && (comandoExecut == "EXAUST*")){
    /*motor que estava operando como ventilador deve operar como exaustor*/
    /*Freando o motor*/
    estadoMotor = 3;
    Serial.println(estadoMotor);
    /*Mudar sentido de rotação apenas apos parar de girar*/
    if(velocidadeEstimadaRPM == 0){
      estadoMotor = 2;
      Serial.println(estadoMotor);
    }
    /*Caso não tenha parado deve continuar freando*/
    if(velocidadeEstimadaRPM > 0){
      estadoMotor = 3;
      Serial.println(estadoMotor);
    }
  }
  if((estadoMotor == 2 || estadoMotor == 3) && (comandoExecut == "VENT*")){
    /*motor que estava operando como exaustor deve operar como ventilador*/
    /*Freando o motor*/
    estadoMotor = 3;
    Serial.println(estadoMotor);
    /*Mudar sentido de rotação apenas apos parar de girar*/
    if(velocidadeEstimadaRPM == 0){
      estadoMotor = 1;
      Serial.println(estadoMotor);
    }
    /*Caso não tenha parado deve continuar freando*/
    if(velocidadeEstimadaRPM > 0){
      estadoMotor = 3;
      Serial.println(estadoMotor);
    }
  }
}

/*Função que habilita o sentido de rotação de acordo com o estado do motor*/
void rotacaoMotor(){
  if(estadoMotor == 0){
    /*motor desligado*/
    velocidadeMotorPWMCanalA = 0;/*zerar velocidade no pino 11*/
    velocidadeMotorPWMCanalB = 0;/*zerar velocidade no pino 3*/
    analogWrite(pinPWMCanalA, velocidadeMotorPWMCanalA);
    analogWrite(pinPWMCanalB, velocidadeMotorPWMCanalB);
  }
  if(estadoMotor == 1){
    /*Motor como ventilador*/
    /*Deve receber parâmetro de velocidade do comando VEL XXX**/
    velocidadeMotorPWMCanalA = velocidadeMotorPWM;
    analogWrite(pinPWMCanalA, velocidadeMotorPWMCanalA);
    analogWrite(pinPWMCanalB, 0);
  }
  if(estadoMotor == 2){
    /*Motor como exaustor*/
    /*Deve receber parâmetro de velocidade do comando VEL XXX**/
    velocidadeMotorPWMCanalB = velocidadeMotorPWM;
    analogWrite(pinPWMCanalA, 0);
    analogWrite(pinPWMCanalB, velocidadeMotorPWMCanalB);
  }
  if(estadoMotor == 3){
    velocidadeMotorPWMCanalA = velocidadeMotorPWM;
    velocidadeMotorPWMCanalB = velocidadeMotorPWM;
    /*Frear motor*/
    analogWrite(pinPWMCanalA, velocidadeMotorPWMCanalA);
    analogWrite(pinPWMCanalB, velocidadeMotorPWMCanalB);
  }
}

/*Funções da parte 1*/
/*------------------------------------------------------------*/
// Função: Lê caractere e concatena no comando
void LerComando()
{
  carac = Serial.read();      // Lê caractere
  if (carac!=-1 && carac!='\n'){         // Se for digitado algo, concatena no comando
    comando.concat(carac);
    comandoExecut = comando;
  }
}

// Função: Decodifica o comando
void DecodComando()
{
  if (comando.indexOf('*')!=-1){  // Verifica se o caractere de fim de comando está presente
    // Interpretação dos comandos
    if (comando=="RETVEL*"){
      Serial.println("VEL: X RPM");
    }
    else if (comando=="PARA*"){
      Serial.println("OK PARA");
    }
    else if (comando=="EXAUST*"){
      Serial.println("OK EXAUST");
    }
    else if (comando=="VENT*"){
      Serial.println("OK VENT");
    }
    else if (comando.substring(0,4)=="VEL "){
      // Comando = "VEL *"
      if (comando.substring(4,5)=="*"){
        Serial.println("ERRO: PARAMETRO AUSENTE");
        velocidade = -1;
      }
      // Comando = "VEL xx*" ou "VEL x*"
      else if (comando.substring(4).length()!=4){
        Serial.println("ERRO: COMANDO INEXISTENTE");
      }
      // Comando = "VEL xxx*"
      else{
        velocidade = comando.substring(4,7).toInt();
        if ((velocidade>=0) && (velocidade<=100)){    
          Serial.println("OK VEL " + comando.substring(4,7) + "%");
        }
        else{
          Serial.println("ERRO: PARAMETRO INCORRETO");
        }
      }
    }
    else{
      Serial.println("ERRO: COMANDO INEXISTENTE");    // Comando inexistente
      Serial.println(comando);
    }
    comando="";                     // Reinicializa para receber um novo comando
  }
}

/*-------------------------------------------------------------*/

/*Funções para estimar velocidade*/

// Rotina de servico de interrupcao do temporizador 0
ISR(TIMER0_COMPA_vect){
  counter++;
}

void configuracao_Timer0(){
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Configuracao Temporizador 0 (8 bits) para gerar interrupcoes periodicas a cada 8ms no modo Clear Timer on Compare Match (CTC)
  // Relogio = 16e6 Hz
  // Prescaler = 1024
  // Faixa = 125 (contagem de 0 a OCR0A = 124)
  // Intervalo entre interrupcoes: (Prescaler/Relogio)*Faixa = (64/16e6)*(124+1) = 0.008s
  
  // TCCR0A – Timer/Counter Control Register A
  // COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00
  // 0      0      0      0          1     0
  TCCR0A = 0x02;

  // OCR0A – Output Compare Register A
  OCR0A = 124;

  // TIMSK0 – Timer/Counter Interrupt Mask Register
  // – – – – – OCIE0B OCIE0A TOIE0
  // – – – – – 0      1      0
  TIMSK0 = 0x02;
  
  // TCCR0B – Timer/Counter Control Register B
  // FOC0A FOC0B – – WGM02 CS02 CS01 CS0
  // 0     0         0     1    0    1
  TCCR0B = 0x05;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

// Rotina de servico de interrupcao do pinEncoder
void estimaVelocidade(){
  passagemDaPa++;
}

/*Configuração do sinal PWM no Timer2*/
void configuracao_Timer2(){
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Configuracao Temporizador 2 (8 bits) para gerar sinal PWM no pino 11 e pino 3
  
  // TCCR2A – Timer/Counter Control Register A
  // COM2A1 COM2A0 COM2B1 COM2B0 – – WGM21 WGM20
  // 1      0      1      0          1     1
  DDRB = DDRB & (1<<3);/*Configura pino 3 do PORTB como saída*/
  DDRD = DDRD & (1<<3);/*COnfigura pino 3 do PORTD como saída*/
  TCCR2A = 0xA3;/*fast PWM*/

  /*OCR2A – Output Compare Register A*/
  OCR2A = velocidadeMotorPWMCanalA;
  /*OCR2B – Output Compare Register B*/
  OCR2B = velocidadeMotorPWMCanalB;
  
  // TCCR2B – Timer/Counter Control Register B
  // FOC0A FOC0B – – WGM02 CS02 CS01 CS0
  // 0     0         0     1    0    0
  TCCR2B = 0x01;/*prescaler*/
  TCNT2 = 0x00; /*Inicializar o Timer2*/
  Serial.println(OCR2A);
  Serial.println(OCR2B);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


/*--------------------------------------------------------------*/

/*Funções da parte 3 (LCD)*/
void escreveLCD(unsigned int velocidade){
  if(String(velocidade).length() <= 4){
    lcd.setCursor(0,0);
    lcd.print(String(velocidade));
    lcd.setCursor(13,0);
    lcd.print("RPM");
    lcd.setCursor(2,1);
    lcd.print("(ESTIMATIVA)");
    Serial.println(String(velocidadeEstimadaRPM));
  }
  else{
    lcd.setCursor(0,0);
    lcd.print("ERROR");
  }
}
/*------------------------*/

void setup()
{
  pinMode(pinPWMCanalA, OUTPUT);
  pinMode(pinPWMCanalB, OUTPUT);
  pinMode(pinEncoder, INPUT);
  //digitalWrite(pinEntrada1PonteH, LOW);
  //digitalWrite(pinEntrada2PonteH, LOW);
  Serial.begin(9600);
  cli();
  configuracao_Timer0();
  sei();
  /*Habilita interrupcao do encoder*/
  attachInterrupt(digitalPinToInterrupt(pinEncoder), estimaVelocidade, RISING);
  /*Iniciar e limpar o LCD*/
  lcd.begin(16,2);
  lcd.clear();
}

void loop()
{
  _delay_ms(1);
  LerComando();
  DecodComando();
  maqEstadosMotor();
  rotacaoMotor();
  convertPercentPWM();
  configuracao_Timer2();
  escreveLCD(velocidadeEstimadaRPM);
  
  // Habilita a interrupcao do encoder em um tempo menor de 200ms
  if(counter < verificaVelocidade){
    attachInterrupt(digitalPinToInterrupt(pinEncoder), estimaVelocidade, RISING);
  }
  /* counter = 125 -> 1s */
  /* counter = 25 -> 0.2s */
  /* Durante 0.2s segundo incremento passagemDaPa */
  /* passageDaPa indica o número de voltas foram dadas em 0.2s */
  /* Multiplicar por 5 para estimar a rotação em 1s */
  if(counter >= verificaVelocidade){
    detachInterrupt(digitalPinToInterrupt(pinEncoder)); // Desabilita interrupcao para calcular a velocidade
    velocidadeEstimadaRPM = ((unsigned int)60*1000/pulsosPorVolta)/(verificaVelocidade*8) * passagemDaPa;
    passagemDaPa = 0;
    attachInterrupt(digitalPinToInterrupt(pinEncoder), estimaVelocidade, RISING); // Habilita interrupcoes apos calculo da velocidade
    counter = 0;
    /*Para evitar que o último caractere de uma velocidade n+1 digitos*/
    /*Apareça em uma velocidade de n dígitos*/
    /*Devemos limpar o LCD*/
    lcd.clear();
  }
  //Serial.println(estadoMotor);
}
