#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ----------
// CONEXIONES

const byte ENA = 5;
const byte IN1 = 8;
const byte IN2 = 9;
const byte ENC_A = 2;
const byte ENC_B = 3;

// Cambiar a 1 para transmitir diagnosticos adicionales del encoder.
// En 0 solo se transmiten las variables principales del motor.
const byte DIAGNOSTICO_ENCODER = 0;

// ----------
// CALIBRACION BASE

const float BASE_CPR = 192.40f;
const float BASE_FACTOR = 1.1556f;
const int8_t BASE_SIGNO = 1;

struct Calibracion {
  uint32_t firma;
  float cpr;
  float factor;
  int8_t signo;
  uint8_t version;
};

const uint32_t FIRMA = 0x4D435231UL;
const byte VERSION = 1;

Calibracion cal;

void usarBase() {
  cal.firma = FIRMA;
  cal.cpr = BASE_CPR;
  cal.factor = BASE_FACTOR;
  cal.signo = BASE_SIGNO;
  cal.version = VERSION;
}

void cargarCalibracion() {
  Calibracion temporal;
  EEPROM.get(0, temporal);

  bool valida =
    temporal.firma == FIRMA &&
    temporal.version == VERSION &&
    temporal.cpr >= 30.0f &&
    temporal.cpr <= 5000.0f &&
    temporal.factor >= 0.5f &&
    temporal.factor <= 2.0f &&
    (temporal.signo == 1 ||
     temporal.signo == -1);

  if (valida) {
    cal = temporal;
  } else {
    usarBase();
  }
}

float pulsosEfectivos() {
  return cal.cpr * cal.factor;
}

// ----------
// ENCODER

volatile long pulsos = 0;
// Flancos ascendentes de A para estimar velocidad y controlar el motor.
volatile unsigned long flancosEncoder = 0;

// Contadores de cuadratura para diagnostico; no alteran directamente el PI.
// Adelante y atras se definen respecto a la secuencia A/B de referencia.
volatile unsigned long cuadAdelante = 0;
volatile unsigned long cuadAtras = 0;
volatile unsigned long cuadInvalidas = 0;
volatile unsigned long cuadRepetidas = 0;
volatile uint8_t estadoAnteriorAB = 0;

// Arduino Uno: A=D2 (PD2), B=D3 (PD3).
// Una sola lectura del puerto reduce diferencias entre señales.
inline uint8_t leerEstadoAB() {
  uint8_t pines = PIND;
  return ((pines & _BV(PD2)) ? 2 : 0) |
         ((pines & _BV(PD3)) ? 1 : 0);
}

// ISR compartida por ambos canales. Solo los flancos ascendentes de A
// actualizan el conteo neto utilizado para RPM y posicion.
void contarEncoder() {
  uint8_t actual = leerEstadoAB();
  uint8_t anterior = estadoAnteriorAB;
  estadoAnteriorAB = actual;

  uint8_t transicion = (anterior << 2) | actual;

  // Secuencia que corresponde al sentido positivo anterior:
  // 00 -> 01 -> 11 -> 10 -> 00 (A = bit 1, B = bit 0).
  if (transicion == 1 || transicion == 7 ||
      transicion == 14 || transicion == 8) {
    cuadAdelante++;
  } else if (transicion == 2 || transicion == 11 ||
             transicion == 13 || transicion == 4) {
    cuadAtras++;
  } else if (actual == anterior) {
    // Se disparo una interrupcion pero no cambio el estado leido.
    cuadRepetidas++;
  } else {
    // Ambos bits distintos: transicion no valida / salto de estado.
    cuadInvalidas++;
  }

  // El control de RPM utiliza solo flancos ascendentes de A y el estado de B.
  if (!(anterior & 2) && (actual & 2)) {
    flancosEncoder++;
    if (actual & 1) {
      pulsos++;
    } else {
      pulsos--;
    }
  }
}

long leerPulsos() {
  noInterrupts();
  long lectura = pulsos;
  interrupts();

  return lectura;
}

// Lectura atomica para evitar inconsistencias durante las interrupciones.
void leerEncoderDiagnostico(
  long &neto, unsigned long &flancos,
  unsigned long &adelante, unsigned long &atras,
  unsigned long &invalidas, unsigned long &repetidas
) {
  noInterrupts();
  neto = pulsos;
  flancos = flancosEncoder;
  adelante = cuadAdelante;
  atras = cuadAtras;
  invalidas = cuadInvalidas;
  repetidas = cuadRepetidas;
  interrupts();
}

// ----------
// CONTROL DEL MOTOR

void mover(int pwm) {
  pwm = constrain(pwm, -255, 255);

  analogWrite(ENA, 0);

  if (pwm == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    analogWrite(ENA, 255);
    return;
  }

  digitalWrite(
    IN1,
    pwm > 0 ? HIGH : LOW
  );

  digitalWrite(
    IN2,
    pwm > 0 ? LOW : HIGH
  );

  analogWrite(ENA, abs(pwm));
}

// ----------
// MODOS

enum Modo {
  PARADO,
  VELOCIDAD_FIJA,
  PERFIL,
  POSICION,
  PWM_FIJO,
  CALIBRANDO,
  CALIBRACION_PAUSADA
};

Modo modo = PARADO;

void frenarMotor() {
  mover(0);
  modo = PARADO;
}

// ----------
// CONTROL PI

const unsigned long CONTROL_MS = 200UL;

const float ALFA_FILTRO = 0.25f;
const float KP = 0.50f;
const float KI = 0.18f;

// Compensacion de PWM estimada experimentalmente.
// Los parametros pueden requerir recalibracion con otras cargas.
const float PWM_BASE = -30.0f;
const float PWM_POR_RPM = 0.83f;

// Margen de correccion del termino integral.
const float LIMITE_INTEGRAL = 110.0f;

// PWM de arranque dependiente de la referencia.
const int PWM_ARRANQUE_MIN = 90;
const int PWM_ARRANQUE_MAX = 160;

// Compensacion anticipada limitada al rango util.
float estimarPWM(float referenciaRPM) {
  return constrain(
    PWM_BASE + PWM_POR_RPM * referenciaRPM,
    75.0f,
    230.0f
  );
}

const int PASO_PWM_SUBIDA = 28;
const int PASO_PWM_BAJADA = 20;

float rpmObjetivo = 0.0f;
float rpmReal = 0.0f;
float rpmFiltrada = 0.0f;

float rpmFiltradaAnterior = 0.0f;
float aceleracionEstimada = 0.0f;

float integralPWM = 0.0f;

bool filtroInicializado = false;

int pwmAplicado = 0;
long conteoAnteriorRPM = 0;
// Referencia de flancos de la ventana anterior.
unsigned long flancosAnterioresRPM = 0;
// Referencias de transiciones de la ventana anterior.
unsigned long adelanteAnteriorRPM = 0;
unsigned long atrasAnteriorRPM = 0;
unsigned long invalidasAnterioresRPM = 0;
unsigned long repetidasAnterioresRPM = 0;

unsigned long ultimoControlRPM = 0;
unsigned long ultimoPulso = 0;

// ----------
// NUEVE PERFILES MATEMATICOS

byte perfilActivo = 0;

unsigned long inicioPerfil = 0;
unsigned long ultimaMuestraRuido = 0;

float referenciaRuido = 150.0f;

// Duracion de cada grafica

unsigned long duracionPerfil(byte numero) {
  if (numero == 2) {
    return 9000UL;
  }

  if (numero == 5) {
    return 15000UL;
  }

  return 12000UL;
}

// ----------
// ECUACIONES DE LAS NUEVE GRAFICAS

float calcularPerfil(byte numero, float t) {
  float resultado = 150.0f;

  switch (numero) {
    // ----------
    // VELOCIDAD CONSTANTE

    case 1:

      resultado = 150.0f;

      break;

    // ----------
    // RAMPA LINEAL

    case 2:

      // 2 segundos a 100 RPM
      // 5 segundos acelerando
      // 2 segundos a 200 RPM

      if (t < 2.0f) {
        resultado = 100.0f;
      } else if (t < 7.0f) {
        resultado =
          100.0f +
          20.0f * (t - 2.0f);
      } else {
        resultado = 200.0f;
      }

      break;

    // ----------
    // TRAPEZOIDAL

    case 3:

      if (t < 3.0f) {
        // Aceleracion
        resultado =
          100.0f +
          100.0f * t / 3.0f;
      } else if (t < 7.0f) {
        // Velocidad constante
        resultado = 200.0f;
      } else if (t < 10.0f) {
        // Desaceleracion
        resultado =
          200.0f -
          100.0f * (t - 7.0f) / 3.0f;
      } else {
        resultado = 100.0f;
      }

      break;

    // ----------
    // SENOIDAL

    case 4:

      resultado =
        150.0f -
        50.0f *
        cos(2.0f * PI * t / 12.0f);

      break;

    // ----------
    // ESCALONADA (DOS NIVELES)

    case 5:

      // p1 = 100 RPM en la primera mitad de T = 15 s.
      // p2 = 200 RPM en la segunda mitad.
      // Se evita pedir 280 RPM en este perfil.
      resultado = (t < 7.5f) ? 100.0f : 200.0f;

      break;

    // ----------
    // CRECIMIENTO POR POTENCIA

    case 6: {
      // Ecuacion del profesor: p(t) = p0 * (1 + alfa*t)^k.
      // p0 = 100, k = 2, T = 12 s, p(T) = 200 RPM.
      // alfa = (sqrt(2) - 1) / 12 = 0.0345178 [1/s].
      // Al elevar al cuadrado con multiplicaciones evitamos pow()
      // en cada vuelta del loop del Arduino Uno.
      const float alfa = 0.0345178f;
      float tiempo = constrain(t, 0.0f, 12.0f);
      float factor = 1.0f + alfa * tiempo;
      resultado = 100.0f * factor * factor;

      break;
    }

    // ----------
    // GAUSSIANA CON BASE DE 100 RPM

    case 7: {
      // Adaptada: p(t) = 100 + 100 * exp(-((t-6)/sigma)^2).
      // sigma = 1.5*sqrt(2), controla el ancho de la campana.
      // El maximo es 200 RPM, respetando el rango habitual.
      float z = (t - 6.0f) / 2.12132034f;

      resultado =
        100.0f +
        100.0f *
        exp(-z * z);

      break;
    }

    // ----------
    // DIENTE DE SIERRA

    case 8: {
      float fase =
        t - 3.0f * floor(t / 3.0f);

      resultado =
        100.0f +
        100.0f * fase / 3.0f;

      break;
    }

    // ----------
    // RUIDO ALEATORIO UNIFORME POR MUESTRAS

    case 9:

      // p(t) = 150 + epsilon(t), epsilon uniforme [-50, 50].
      // Se actualiza cada 500 ms; no es ruido blanco continuo.
      resultado = referenciaRuido;

      break;
  }

  // Limitar todas las referencias
  // entre 20 y 280 RPM

  return constrain(
    resultado,
    20.0f,
    280.0f
  );
}

// ----------
// DIAGNOSTICO PWM

unsigned long inicioPWMFijo = 0;

const unsigned long TIEMPO_DIAGNOSTICO =
  10000UL;

// ----------
// CONTROL DE POSICION

long objetivoPulsos = 0;
long pulsosInicialesPos = 0;
long ultimaCuentaPos = 0;

float gradosPedidos = 0.0f;

unsigned long inicioPos = 0;
unsigned long ultimaActividadPos = 0;

unsigned long ultimaGraficaPos = 0;
unsigned long inicioFasePos = 0;

unsigned long limiteTiempoPos = 0;
unsigned long tiempoImpulsoPos = 0;

byte fasePos = 0;

// ----------
// CALIBRACION LENTA

const int PWM_CAL = 225;

const unsigned long IMPULSO_CAL_MS =
  45UL;

unsigned long pausaCalMs = 900UL;

const unsigned long MAX_CAL_MS =
  600000UL;

long inicioPulsosCal = 0;

unsigned long inicioTramoCal = 0;
unsigned long ultimaFaseCal = 0;

bool impulsoCalActivo = false;

// ----------
// COMUNICACION SERIAL

char entrada[48];
byte indiceEntrada = 0;

// ----------
// MENU PRINCIPAL

void menu() {
  Serial.println(F(""));

  Serial.println(F("----------"));

  Serial.println(F("    MOTOR DC - NUEVE GRAFICAS"));

  Serial.println(F("1 -> Velocidad constante"));

  Serial.println(F("2 -> Rampa lineal"));

  Serial.println(F("3 -> Trapezoidal"));

  Serial.println(F("4 -> Senoidal"));

  Serial.println(F("5 -> Escalonada"));

  Serial.println(F("6 -> Exponencial creciente"));

  Serial.println(F("7 -> Gaussiana"));

  Serial.println(F("8 -> Diente de sierra"));

  Serial.println(F("9 -> Ruido blanco"));

  Serial.println(F("---------------------------------"));

  Serial.println(F("R150 -> Velocidad personalizada"));

  Serial.println(F("G90  -> Movimiento en grados"));

  Serial.println(F("E -> Grafica escalonada"));

  Serial.println(F("L -> Grafica lineal"));

  Serial.println(F("T150 -> Diagnostico PWM fijo"));

  Serial.println(F("P -> Detener y mostrar menu"));

  Serial.println(F("AYUDA -> Mas opciones"));
}

// ----------
// DETENER Y MOSTRAR MENU

void pararYMenu(
  const __FlashStringHelper *mensaje
) {
  frenarMotor();

  Serial.println(mensaje);

  menu();
}

// ----------
// AYUDA

void ayuda() {
  Serial.println(F(""));

  Serial.println(F("OPCIONES ADICIONALES"));

  Serial.println(F("R20 a R280 = Control de RPM"));

  Serial.println(F("G-720 a G720 = Control de grados"));

  Serial.println(F("1 a 9 = Graficas automaticas"));

  Serial.println(F("E = Escalonada"));

  Serial.println(F("L = Rampa lineal"));

  Serial.println(F("T80 a T220 = PWM fijo 10 segundos"));

  Serial.println(F("P = Detener motor"));

  Serial.println(F("BASE = Restaurar calibracion base"));

  Serial.println(F("GUARDAR = Guardar en EEPROM"));

  Serial.println(F("CARGAR = Recuperar EEPROM"));

  Serial.println(F("DATOS = Consultar calibracion"));

  Serial.println(F("CALIBRAR = Calibracion lenta"));

  Serial.println(F("CONTINUAR = Reanudar calibracion"));

  Serial.println(F("A / B = Ajustar posicion del eje"));

  Serial.println(F("LENTO / NORMAL = Ritmo de calibracion"));

  Serial.println(F("MEDIR10 = Medicion de 10 vueltas"));

  Serial.println(F("MEDIR(5 VUELTAS) = Medicion de 5 vueltas"));

  Serial.println(F("CANCELAR = Cancelar operacion"));
}

// ----------
// CONSULTAR CALIBRACION

void mostrarDatos() {
  Serial.print(F("CPR almacenados: "));
  Serial.println(cal.cpr, 2);

  Serial.print(F("Factor de posicion: "));
  Serial.println(cal.factor, 4);

  Serial.print(F("CPR efectivos estimados: "));
  Serial.println(pulsosEfectivos(), 2);
}

// ----------
// PREPARAR CONTROL DE VELOCIDAD

void prepararVelocidad(
  float objetivo,
  int nuevoModo
) {
  frenarMotor();

  rpmObjetivo = objetivo;

  rpmReal = 0.0f;
  rpmFiltrada = 0.0f;

  rpmFiltradaAnterior = 0.0f;
  aceleracionEstimada = 0.0f;

  integralPWM = 0.0f;

  filtroInicializado = false;

  // En PWM fijo, el valor exacto lo define el comando T.
  // En velocidad, el arranque utiliza la referencia solicitada.
  if (nuevoModo == PWM_FIJO) {
    pwmAplicado = 0;
  } else {
    pwmAplicado = constrain(
      (int)(estimarPWM(objetivo) + 0.5f),
      PWM_ARRANQUE_MIN,
      PWM_ARRANQUE_MAX
    );
  }

  // Sincronizar las referencias de todos los contadores.
  leerEncoderDiagnostico(
    conteoAnteriorRPM, flancosAnterioresRPM,
    adelanteAnteriorRPM, atrasAnteriorRPM,
    invalidasAnterioresRPM, repetidasAnterioresRPM
  );

  ultimoControlRPM = millis();
  ultimoPulso = ultimoControlRPM;

  // Activar el modo seleccionado.
  modo = (Modo)nuevoModo;

  mover(pwmAplicado);
}

// ----------
// INICIAR UNA DE LAS 9 GRAFICAS

void comenzarPerfil(byte numero) {
  if (numero < 1 || numero > 9) {
    return;
  }

  perfilActivo = numero;

  referenciaRuido = 150.0f;

  prepararVelocidad(
    calcularPerfil(numero, 0.0f),
    PERFIL
  );

  inicioPerfil = millis();

  ultimaMuestraRuido = inicioPerfil;

  Serial.println(F(""));

  Serial.print(F("PERFIL "));
  Serial.print(numero);

  Serial.println(F(" INICIADO"));

  Serial.print(F("DURACION: "));

  Serial.print(
    duracionPerfil(numero) / 1000UL
  );

  Serial.println(F(" SEGUNDOS"));

  Serial.println(F("ESCRIBE P PARA DETENER"));
}

// ----------
// DIAGNOSTICO PWM FIJO

void comenzarPWMFijo(int pwm) {
  prepararVelocidad(
    0.0f,
    PWM_FIJO
  );

  pwmAplicado = pwm;

  inicioPWMFijo = millis();

  mover(pwmAplicado);

  Serial.print(F("PWM FIJO: "));

  Serial.print(pwmAplicado);

  Serial.println(F("/255 DURANTE 10 SEGUNDOS"));

  Serial.println(F("PWM NO ES LO MISMO QUE RPM"));
}

// ----------
// CONTROL DE GRADOS

void comenzarGrados(float grados) {
  frenarMotor();

  delay(150);

  gradosPedidos = grados;

  pulsosInicialesPos =
    cal.signo * leerPulsos();

  objetivoPulsos =
    pulsosInicialesPos +
    lround(
      grados *
      pulsosEfectivos() / 360.0f
    );

  ultimaCuentaPos = leerPulsos();

  inicioPos = millis();

  ultimaActividadPos = inicioPos;
  ultimaGraficaPos = inicioPos;

  inicioFasePos = inicioPos;

  fasePos = 0;

  limiteTiempoPos =
    15000UL +
    (unsigned long)(
      fabs(grados) * 100.0f
    );

  if (limiteTiempoPos > 90000UL) {
    limiteTiempoPos = 90000UL;
  }

  modo = POSICION;

  Serial.print(F("GRADOS SOLICITADOS: "));

  Serial.println(grados, 1);

  Serial.println(F("PASO RECOMENDADO: 5 GRADOS"));
}

// ----------
// INICIAR CALIBRACION

void comenzarCalibracion() {
  frenarMotor();

  delay(300);

  inicioPulsosCal = leerPulsos();

  modo = CALIBRANDO;

  impulsoCalActivo = false;

  inicioTramoCal = millis();

  ultimaFaseCal = inicioTramoCal;

  Serial.println(F("CALIBRACION LENTA INICIADA"));

  Serial.println(F("CUENTA 10 VUELTAS COMPLETAS"));

  Serial.println(F("P = PAUSAR"));

  Serial.println(F("A/B = ALINEAR EL EJE"));

  Serial.println(F("MEDIR10 = CALCULAR"));
}

// ----------
// PAUSAR CALIBRACION

void pausarCalibracion() {
  mover(0);

  impulsoCalActivo = false;

  modo = CALIBRACION_PAUSADA;

  Serial.println(F("CALIBRACION PAUSADA"));

  menu();
}

// ----------
// CONTINUAR CALIBRACION

void continuarCalibracion() {
  if (modo != CALIBRACION_PAUSADA) {
    Serial.println(F("PRIMERO ESCRIBE CALIBRAR"));

    return;
  }

  modo = CALIBRANDO;

  impulsoCalActivo = false;

  inicioTramoCal = millis();

  ultimaFaseCal = inicioTramoCal;

  Serial.println(F("CALIBRACION REANUDADA"));
}

// ----------
// AJUSTE FINO DE CALIBRACION

void ajusteFino(
  bool adelante,
  unsigned long tiempoMs
) {
  if (modo != CALIBRACION_PAUSADA) {
    Serial.println(F("PRIMERO PAUSA CON P"));

    return;
  }

  mover(
    adelante ? PWM_CAL : -PWM_CAL
  );

  delay(tiempoMs);

  mover(0);

  delay(250);

  Serial.print(F("PULSOS NETOS: "));

  Serial.println(
    labs(leerPulsos() - inicioPulsosCal)
  );
}

// ----------
// MEDIR VUELTAS

void medirVueltas(byte vueltas) {
  if (modo != CALIBRACION_PAUSADA) {
    Serial.println(F("CALIBRA Y PAUSA ANTES DE MEDIR"));

    return;
  }

  mover(0);

  delay(350);

  long diferencia =
    leerPulsos() - inicioPulsosCal;

  float nuevaCPR =
    (float)labs(diferencia) /
    (float)vueltas;

  if (
    nuevaCPR < 30.0f ||
    nuevaCPR > 5000.0f
  ) {
    Serial.println(F("ERROR: RESULTADO FUERA DE RANGO"));

    return;
  }

  cal.firma = FIRMA;
  cal.version = VERSION;

  cal.cpr = nuevaCPR;

  cal.factor = 1.0f;

  cal.signo =
    diferencia >= 0 ? 1 : -1;

  modo = PARADO;

  Serial.println(F(""));

  Serial.println(F("CALIBRACION FINALIZADA"));

  Serial.print(F("PULSOS NETOS: "));

  Serial.println(diferencia);

  Serial.print(F("PULSOS POR VUELTA: "));

  Serial.println(nuevaCPR, 2);

  Serial.println(F("NO GUARDADO EN EEPROM"));

  Serial.println(F("PRUEBA G90 ANTES DE GUARDAR"));

  menu();
}

// ----------
// RECIBIR COMANDOS

void procesar(char *original) {
  char cmd[48];

  byte n = 0;

  // Quitar espacios y convertir
  // el comando a mayusculas

  for (
    byte i = 0;
    original[i] &&
    n < sizeof(cmd) - 1;
    i++
  ) {
    char c = original[i];

    if (
      c == ' ' ||
      c == '\t'
    ) {
      continue;
    }

    if (
      c >= 'a' &&
      c <= 'z'
    ) {
      c -= 32;
    }

    cmd[n++] = c;
  }

  cmd[n] = '\0';

  if (!n) return;

  // ----------
  // P = DETENER

  if (!strcmp(cmd, "P")) {
    if (modo == CALIBRANDO) {
      pausarCalibracion();
    } else if (
      modo == CALIBRACION_PAUSADA
    ) {
      menu();
    } else {
      pararYMenu(
        F("MOTOR DETENIDO")
      );
    }
  }

  // ----------
  // AYUDA

  else if (!strcmp(cmd, "AYUDA")) {
    ayuda();
  }

  // ----------
  // MENU

  else if (!strcmp(cmd, "MENU")) {
    menu();
  }

  // ----------
  // CANCELAR

  else if (!strcmp(cmd, "CANCELAR")) {
    pararYMenu(
      F("OPERACION CANCELADA")
    );
  }

  // ----------
  // BASE

  else if (!strcmp(cmd, "BASE")) {
    frenarMotor();

    usarBase();

    Serial.println(F("CALIBRACION BASE RESTAURADA"));

    menu();
  }

  // ----------
  // CARGAR

  else if (!strcmp(cmd, "CARGAR")) {
    frenarMotor();

    cargarCalibracion();

    Serial.println(F("CALIBRACION EEPROM CARGADA"));

    menu();
  }

  // ----------
  // GUARDAR

  else if (!strcmp(cmd, "GUARDAR")) {
    frenarMotor();

    EEPROM.put(0, cal);

    Serial.println(F("CALIBRACION GUARDADA"));

    menu();
  }

  // ----------
  // DATOS

  else if (!strcmp(cmd, "DATOS")) {
    mostrarDatos();
  }

  // ----------
  // CALIBRAR

  else if (!strcmp(cmd, "CALIBRAR")) {
    comenzarCalibracion();
  }

  // ----------
  // CONTINUAR

  else if (!strcmp(cmd, "CONTINUAR")) {
    continuarCalibracion();
  }

  // ----------
  // LENTO

  else if (!strcmp(cmd, "LENTO")) {
    pausaCalMs = 1400UL;

    Serial.println(F("CALIBRACION EXTRA LENTA"));
  }

  // ----------
  // NORMAL

  else if (!strcmp(cmd, "NORMAL")) {
    pausaCalMs = 900UL;

    Serial.println(F("VELOCIDAD NORMAL DE CALIBRACION"));
  }

  // ----------
  // MEDIR10

  else if (
    !strcmp(cmd, "MEDIR10") ||
    !strcmp(cmd, "MEDIR(10VUELTAS)")
  ) {
    medirVueltas(10);
  }

  // ----------
  // MEDIR(5 VUELTAS)

  else if (
    !strcmp(cmd, "MEDIR(5VUELTAS)")
  ) {
    medirVueltas(5);
  }

  // ----------
  // AJUSTES A Y B

  else if (
    cmd[0] == 'A' ||
    cmd[0] == 'B'
  ) {
    unsigned long ms = 25;

    if (cmd[1]) {
      char *fin;

      long valor =
        strtol(cmd + 1, &fin, 10);

      if (
        fin == cmd + 1 ||
        *fin ||
        valor < 10 ||
        valor > 100
      ) {
        Serial.println(F("USA A10..A100 O B10..B100"));

        return;
      }

      ms = (unsigned long)valor;
    }

    ajusteFino(
      cmd[0] == 'A',
      ms
    );
  }

  // ----------
  // E = ESCALONADA

  else if (!strcmp(cmd, "E")) {
    comenzarPerfil(5);
  }

  // ----------
  // L = RAMPA LINEAL

  else if (!strcmp(cmd, "L")) {
    comenzarPerfil(2);
  }

  // ----------
  // OPCIONES 1 A 9

  else if (
    n == 1 &&
    cmd[0] >= '1' &&
    cmd[0] <= '9'
  ) {
    comenzarPerfil(
      (byte)(cmd[0] - '0')
    );
  }

  // ----------
  // T150 = PWM FIJO

  else if (cmd[0] == 'T') {
    char *fin;

    long valor =
      strtol(cmd + 1, &fin, 10);

    if (
      fin == cmd + 1 ||
      *fin ||
      valor < 80 ||
      valor > 220
    ) {
      Serial.println(F("PWM VALIDO: T80 A T220"));

      return;
    }

    comenzarPWMFijo(
      (int)valor
    );
  }

  // ----------
  // R150 = CONTROL DE VELOCIDAD

  else if (cmd[0] == 'R') {
    char *fin;

    float valor =
      strtod(cmd + 1, &fin);

    if (
      fin == cmd + 1 ||
      *fin ||
      !(valor >= 20.0f &&
        valor <= 280.0f)
    ) {
      Serial.println(F("RPM VALIDAS: 20 A 280"));

      return;
    }

    prepararVelocidad(
      valor,
      VELOCIDAD_FIJA
    );

    Serial.print(F("RPM SOLICITADAS: "));

    Serial.println(valor, 1);

    Serial.println(F("ESCRIBE P PARA DETENER"));
  }

  // ----------
  // G90 = CONTROL DE GRADOS

  else if (cmd[0] == 'G') {
    char *fin;

    float valor =
      strtod(cmd + 1, &fin);

    if (
      fin == cmd + 1 ||
      *fin ||
      !(valor >= -720.0f &&
        valor <= 720.0f) ||
      (valor != 0.0f &&
       fabs(valor) < 5.0f)
    ) {
      Serial.println(F("GRADOS: -720 A 720"));

      Serial.println(F("PASO MINIMO RECOMENDADO: 5"));

      return;
    }

    if (valor == 0.0f) {
      pararYMenu(
        F("MOTOR DETENIDO")
      );
    } else {
      comenzarGrados(valor);
    }
  }

  // ----------
  // COMANDO DESCONOCIDO

  else {
    Serial.println(F("COMANDO NO RECONOCIDO"));

    Serial.println(F("ESCRIBE AYUDA"));
  }
}

// ----------
// TELEMETRIA OPCIONAL DEL ENCODER

void enviarDiagnosticoEncoder(
  unsigned long flancosPeriodo,
  long pulsosNetosPeriodo,
  unsigned long intervaloMs,
  unsigned long adelantePeriodo,
  unsigned long atrasPeriodo,
  unsigned long invalidasPeriodo,
  unsigned long repetidasPeriodo
) {
  if (DIAGNOSTICO_ENCODER != 1) return;

  // Los flancos totales no sustituyen la medicion utilizada por el PI.
  float rpmFlancos =
    flancosPeriodo * 60000.0f /
    (pulsosEfectivos() * (float)intervaloMs);

  // D: flancos, pulsos netos, duracion de la ventana y RPM por flancos.
  Serial.print(F(">D:"));
  Serial.print(flancosPeriodo);
  Serial.print(',');
  Serial.print(pulsosNetosPeriodo);
  Serial.print(',');
  Serial.print(intervaloMs);
  Serial.print(',');
  Serial.println(rpmFlancos, 1);

  // Q: cuadratura hacia adelante, atras, invalida y repetida.
  Serial.print(F(">Q:"));
  Serial.print(adelantePeriodo);
  Serial.print(',');
  Serial.print(atrasPeriodo);
  Serial.print(',');
  Serial.print(invalidasPeriodo);
  Serial.print(',');
  Serial.println(repetidasPeriodo);
}

// ----------
// CONFIGURACION INICIAL

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Inicializar estado A/B antes de habilitar interrupciones.
  // Requiere Arduino Uno (D2/INT0 y D3/INT1).
  noInterrupts();
  estadoAnteriorAB = leerEstadoAB();
  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    contarEncoder,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(ENC_B),
    contarEncoder,
    CHANGE
  );
  interrupts();

  frenarMotor();

  // Recuperar calibracion guardada
  // sin mostrar los datos

  cargarCalibracion();

  randomSeed(
    analogRead(A0)
  );

  menu();
}

// ----------
// PROGRAMA PRINCIPAL

void loop() {
  // ----------
  // LEER COMANDOS

  while (Serial.available()) {
    char c = Serial.read();

    if (
      c == '\r' ||
      c == '\n'
    ) {
      if (indiceEntrada) {
        entrada[indiceEntrada] = '\0';

        procesar(entrada);

        indiceEntrada = 0;
      }
    } else if (
      indiceEntrada <
      sizeof(entrada) - 1
    ) {
      entrada[indiceEntrada++] = c;
    } else {
      indiceEntrada = 0;
    }
  }

  unsigned long ahora = millis();

  // ----------
  // CALIBRACION LENTA

  if (modo == CALIBRANDO) {
    if (
      ahora - inicioTramoCal >=
      MAX_CAL_MS
    ) {
      pausarCalibracion();

      Serial.println(F("PAUSA AUTOMATICA POR TIEMPO"));
    } else if (
      ahora - ultimaFaseCal >=
      (
        impulsoCalActivo ?
        IMPULSO_CAL_MS :
        pausaCalMs
      )
    ) {
      ultimaFaseCal = ahora;

      impulsoCalActivo =
        !impulsoCalActivo;

      mover(
        impulsoCalActivo ?
        PWM_CAL : 0
      );
    }
  }

  // ----------
  // PERFILES MATEMATICOS

  if (modo == PERFIL) {
    unsigned long tMs =
      ahora - inicioPerfil;

    if (
      tMs >= duracionPerfil(perfilActivo)
    ) {
      pararYMenu(
        F("PERFIL FINALIZADO")
      );
    } else {
      // Actualizar el ruido blanco
      // cada medio segundo

      if (
        perfilActivo == 9 &&
        tMs >= 500UL &&
        ahora - ultimaMuestraRuido >=
        500UL
      ) {
        ultimaMuestraRuido = ahora;

        referenciaRuido =
          150.0f + (float)random(-50L, 51L);
      }

      // Calcular RPM de la ecuacion

      rpmObjetivo =
        calcularPerfil(
          perfilActivo,
          tMs / 1000.0f
        );
    }
  }

  // ----------
  // DIAGNOSTICO PWM FIJO

  if (
    modo == PWM_FIJO &&
    ahora - inicioPWMFijo >=
    TIEMPO_DIAGNOSTICO
  ) {
    pararYMenu(
      F("DIAGNOSTICO PWM TERMINADO")
    );
  }

  // ----------
  // CONTROL DE POSICION

  if (modo == POSICION) {
    long lectura = leerPulsos();

    if (lectura != ultimaCuentaPos) {
      ultimaCuentaPos = lectura;

      ultimaActividadPos = ahora;
    }

    // Seguridad por tiempo

    if (
      ahora - inicioPos >
      limiteTiempoPos ||
      ahora - ultimaActividadPos >
      2500UL
    ) {
      pararYMenu(
        F("POSICION CANCELADA")
      );
    }

    // Finalizar impulso

    else if (fasePos == 1) {
      if (
        ahora - inicioFasePos >=
        tiempoImpulsoPos
      ) {
        mover(0);

        fasePos = 2;

        inicioFasePos = ahora;
      }
    }

    // Comprobar posicion

    else if (
      fasePos == 0 ||
      ahora - inicioFasePos >= 140UL
    ) {
      long actual =
        cal.signo * leerPulsos();

      long error =
        objetivoPulsos - actual;

      long tolerancia = max(
        2L,
        (long)(
          pulsosEfectivos() *
          3.0f / 360.0f
        )
      );

      if (
        labs(error) <= tolerancia
      ) {
        pararYMenu(
          F("POSICION ALCANZADA")
        );
      } else {
        float restante =
          labs(error) *
          360.0f /
          pulsosEfectivos();

        if (restante > 45.0f) {
          tiempoImpulsoPos = 18;
        } else if (restante > 12.0f) {
          tiempoImpulsoPos = 9;
        } else {
          tiempoImpulsoPos = 5;
        }

        mover(
          error > 0 ? 200 : -200
        );

        fasePos = 1;

        inicioFasePos = ahora;
      }
    }

    // Grafica de grados en Teleplot

    if (
      modo == POSICION &&
      ahora - ultimaGraficaPos >=
      200UL
    ) {
      ultimaGraficaPos = ahora;

      float gradosMedidos =
        (
          cal.signo * leerPulsos() -
          pulsosInicialesPos
        ) *
        360.0f /
        pulsosEfectivos();

      Serial.print(F(">Grados_Objetivo:"));

      Serial.println(gradosPedidos, 1);

      Serial.print(F(">Grados_Estimados:"));

      Serial.println(gradosMedidos, 1);
    }
  }

  // ----------
  // MEDICION Y CONTROL DE RPM

  ahora = millis();

  if (
    (
      modo == VELOCIDAD_FIJA ||
      modo == PERFIL ||
      modo == PWM_FIJO
    ) &&
    ahora - ultimoControlRPM >=
    CONTROL_MS
  ) {
    unsigned long dt =
      ahora - ultimoControlRPM;

    // Capturar los contadores atomicos en la misma ventana de medicion.
    long lectura;
    unsigned long flancosLectura;
    unsigned long adelanteLectura;
    unsigned long atrasLectura;
    unsigned long invalidasLectura;
    unsigned long repetidasLectura;
    leerEncoderDiagnostico(
      lectura, flancosLectura,
      adelanteLectura, atrasLectura,
      invalidasLectura, repetidasLectura
    );

    long diferencia =
      lectura - conteoAnteriorRPM;

    unsigned long flancosPeriodo =
      flancosLectura - flancosAnterioresRPM;
    unsigned long adelantePeriodo =
      adelanteLectura - adelanteAnteriorRPM;
    unsigned long atrasPeriodo =
      atrasLectura - atrasAnteriorRPM;
    unsigned long invalidasPeriodo =
      invalidasLectura - invalidasAnterioresRPM;
    unsigned long repetidasPeriodo =
      repetidasLectura - repetidasAnterioresRPM;

    conteoAnteriorRPM = lectura;
    flancosAnterioresRPM = flancosLectura;
    adelanteAnteriorRPM = adelanteLectura;
    atrasAnteriorRPM = atrasLectura;
    invalidasAnterioresRPM = invalidasLectura;
    repetidasAnterioresRPM = repetidasLectura;

    ultimoControlRPM = ahora;

    // ----------
    // RPM REALES DEL ENCODER

    rpmReal =
      labs(diferencia) *
      60000.0f /
      (
        pulsosEfectivos() *
        (float)dt
      );

    // ----------
    // FILTRO DE RPM

    if (!filtroInicializado) {
      rpmFiltrada = rpmReal;

      rpmFiltradaAnterior =
        rpmFiltrada;

      filtroInicializado = true;

      aceleracionEstimada = 0.0f;
    } else {
      rpmFiltrada +=
        ALFA_FILTRO *
        (rpmReal - rpmFiltrada);

      aceleracionEstimada =
        (
          rpmFiltrada -
          rpmFiltradaAnterior
        ) *
        1000.0f / dt;

      rpmFiltradaAnterior =
        rpmFiltrada;
    }

    if (diferencia != 0) {
      ultimoPulso = ahora;
    }

    // ----------
    // SEGURIDAD DEL ENCODER

    if (
      ahora - ultimoPulso >
      1800UL
    ) {
      pararYMenu(
        F("ERROR: ENCODER SIN PULSOS")
      );
    } else {
      // ----------
      // CONTROL PI

      // El diagnostico T150 NO usa PI

      if (modo != PWM_FIJO) {
        float error =
          rpmObjetivo - rpmFiltrada;

        // Integral candidata

        float nuevaIntegral =
          constrain(
            integralPWM +
            KI * error *
            dt / 1000.0f,
            -LIMITE_INTEGRAL,
            LIMITE_INTEGRAL
          );

        // PWM estimado segun RPM

        // Misma estimacion utilizada al preparar el arranque.
        float adelantamiento = estimarPWM(rpmObjetivo);

        float salidaSinLimite =
          adelantamiento +
          KP * error +
          nuevaIntegral;

        // Evitar saturacion integral

        if (!(
          (
            salidaSinLimite > 255.0f &&
            error > 0.0f
          ) ||
          (
            salidaSinLimite < 0.0f &&
            error < 0.0f
          )
        )) {
          integralPWM =
            nuevaIntegral;
        }

        // Calcular PWM deseado

        int deseado =
          constrain(
            (int)(
              adelantamiento +
              KP * error +
              integralPWM
            ),
            0,
            255
          );

        // Limitar cambios bruscos

        pwmAplicado =
          constrain(
            deseado,
            max(
              0,
              pwmAplicado -
              PASO_PWM_BAJADA
            ),
            min(
              255,
              pwmAplicado +
              PASO_PWM_SUBIDA
            )
          );

        // Aplicar potencia

        mover(pwmAplicado);
      }

      // ----------
      // DATOS PARA TELEPLOT

      // Solo se envian datos
      // mientras existe una prueba

      if (modo != PWM_FIJO) {
        Serial.print(F(">RPM_Objetivo:"));

        Serial.println(rpmObjetivo, 1);
      }

      Serial.print(F(">RPM_Real:"));

      Serial.println(rpmReal, 1);

      Serial.print(F(">RPM_Filtrada:"));

      Serial.println(rpmFiltrada, 1);

      Serial.print(F(">Aceleracion_RPM_s:"));

      Serial.println(aceleracionEstimada, 1);

      Serial.print(F(">PWM_Aplicado:"));

      Serial.println(pwmAplicado);

      // Enviar las mediciones ampliadas solo si se habilita el diagnostico.
      if (DIAGNOSTICO_ENCODER == 1) {
        enviarDiagnosticoEncoder(
          flancosPeriodo, diferencia, dt,
          adelantePeriodo, atrasPeriodo,
          invalidasPeriodo, repetidasPeriodo
        );
      }
    }
  }
}