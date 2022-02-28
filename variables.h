
// Declara a constante de baudrate do Serial
const int baudRate = 9600;

// Declara as constantes pinos de saída do PWM
const int pwmPinL = 13; // PWM do Pino Esquerdo (Left)
const int pwmPinR = 12; // PWM do Pino Direito (Right)
const int enPinL = 11;  // Habilita do Pino Esquerdo (Left)
const int enPinR = 10;  // Habilita do Pino Direito (Right)


// Declara as varáiveis de tempo de amostragem
long t[2] = {0, 0};

// Informa qual é a potência que não move a bolinha do lugar, nem para cima, nem para baixo. Funciona como um offset.
int compensacao = 0;

// Define alguns limites do algoritmo
long limite_i =  2147483647;    // Tamanho máximo de área que a integral do erro pode atingir/adquirir
int limite_inf_saida = -255;    // Tamanho mínimo que a saida/potência u[n] pode atingir. A limitação trata-se da limitação do PWM que varia de 0 a 100% representado entre valores de 0 a 255
int limite_sup_saida =  255;    // Tamanho máximo que a saida/potência u[n] pode atingir. Repare que no caso a saída pode também assumir valores negativos e isso inverte o sentido de rotação do motor

// Declara a Word (16 bits) que armazenará os bits de controle
uint16_t bits_de_controle = 0b0000000000000000; // O primeiro bit liga o controle

// Armazena a variável de saída anterior para fins de cálculo da variável
float saida_anterior_1 = 0; //u[n-1]

// Lista que define a posição de cada bit de controle
#define BIT_LIGA_DESLIGA 0

// ####################################################################################
// ####### Structures to divide variables in WORDs and exchange data via MODBUS #######
// ####################################################################################
// Essas estruturas de dados foram criadas para dividir variáveis FLOAT que ocupam 32bits
// em duas WORDs de 16bits. Isso foi necessário para armazena-las em registradores MODBUS

union referenciaFloatPointWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} referencia; // Variável responsável por armazenar a altura desejada pelo usuário/operador

union alturaFloatPointWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} altura; // Variável responsável por armazenar a altura obtida pelo sensor

float ultima_altura;

union erroFloatWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} erro; // Armazena o quão distante a altura está da referência

float erro_anterior;

union integralFloatWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} integral; // Armazena a área total abaixo da curva do erro até o instante atual. O tamanho é limitado pela variável limite_i.

union derivadaFloatWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} derivada; // Armazena a atual taxa de variação da altura da bolinha

union saidaFloatWordsIEEE754 { // Armazena qual deve ser a potência aplicada ao motor do secador. Varia de 0 a 100%.
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} saida;

union dtFloatWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} dt; // Taxa de amostragem do algoritmo em segundos. É medida automaticamente pelo algoritmo.

union kpFloatPointWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} kp; // Ganho proporcional

union kiFloatPointWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} ki; // Ganho integral

union kdFloatPointWordsIEEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  float f;
} kd; // Ganho diferencial

union microsLongWordsIEE754 {
  struct {
    uint16_t a : 16;
    uint16_t b : 16;
  } raw;
  long l;
} timestamp; // Recebe o timestamp atual do microcontrolador (us)
