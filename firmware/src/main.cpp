#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t PIN_LEITURA = 0;
constexpr uint8_t PIN_CHUTE = 1;
constexpr uint8_t PIN_FAULT = 2;
constexpr uint8_t PIN_DONE = 3;
constexpr uint8_t PIN_CARREGAMENTO = 4;

constexpr float ADC_EM_200V = 3700.0f;
constexpr float TENSAO_CALIBRACAO = 200.0f;
constexpr float FATOR_CALIBRACAO = 1.0f;

constexpr uint8_t AMOSTRAS_ADC = 16;
constexpr unsigned long INTERVALO_LOG_MS = 100;
constexpr unsigned long TIMEOUT_CARGA_MS = 15000;
constexpr unsigned long TEMPO_IGNORAR_SINAIS_MS = 5;
constexpr unsigned long FILTRO_DONE_MS = 2;
constexpr unsigned long FILTRO_FAULT_MS = 2;
constexpr float SETPOINT_MINIMO = 20.0f;
constexpr float SETPOINT_MAXIMO = 200.0f;
constexpr float TENSAO_MAXIMA_SEGURANCA = 205.0f;

bool carregando = false;
unsigned long inicioCarga = 0;
unsigned long ultimoLog = 0;
unsigned long inicioDoneLow = 0;
unsigned long inicioFaultLow = 0;
float valorSetado = 200.0f;
float valorAtual = 0.0f;
uint16_t leituraADC = 0;
char bufferSerial[32];
uint8_t indiceSerial = 0;

void atualizarLeitura()
{
    uint32_t soma = 0;
    for (uint8_t i = 0; i < AMOSTRAS_ADC; i++)
    {
        soma += analogRead(PIN_LEITURA);
        delayMicroseconds(50);
    }

    leituraADC = static_cast<uint16_t>(soma / AMOSTRAS_ADC);
    valorAtual = leituraADC * TENSAO_CALIBRACAO / ADC_EM_200V * FATOR_CALIBRACAO;

    if (valorAtual < 0.5f)
    {
        valorAtual = 0.0f;
    }
}

bool doneAtivo()
{
    return digitalRead(PIN_DONE) == LOW;
}

bool faultAtivo()
{
    return digitalRead(PIN_FAULT) == LOW;
}

void mostrarStatus()
{
    atualizarLeitura();

    Serial.println();
    Serial.println("========== STATUS ==========");
    Serial.print("Carregamento: ");
    Serial.println(carregando ? "ON" : "OFF");
    Serial.print("Setpoint: ");
    Serial.print(valorSetado, 1);
    Serial.println(" V");
    Serial.print("ADC: ");
    Serial.println(leituraADC);
    Serial.print("Tensão medida: ");
    Serial.print(valorAtual, 1);
    Serial.println(" V");
    Serial.print("DONE#: ");
    Serial.print(digitalRead(PIN_DONE));
    Serial.println(doneAtivo() ? " - ATIVO" : " - INATIVO");
    Serial.print("FAULT#: ");
    Serial.print(digitalRead(PIN_FAULT));
    Serial.println(faultAtivo() ? " - ATIVO" : " - INATIVO");
    Serial.print("CHARGE: ");
    Serial.println(digitalRead(PIN_CARREGAMENTO));
    Serial.print("KICK: ");
    Serial.println(digitalRead(PIN_CHUTE));
    Serial.println("============================");
}

void pararCarregamento(const char *motivo)
{
    const float tensaoNoDisparo = valorAtual;
    const uint16_t adcNoDisparo = leituraADC;
    const bool estavaCarregando = carregando;
    const unsigned long tempoCarga = estavaCarregando ? millis() - inicioCarga : 0;

    digitalWrite(PIN_CARREGAMENTO, LOW);
    carregando = false;
    inicioDoneLow = 0;
    inicioFaultLow = 0;

    Serial.println();
    Serial.println("============================");
    Serial.print("Carga encerrada: ");
    Serial.println(motivo);

    if (estavaCarregando)
    {
        Serial.print("Tempo de carga: ");
        Serial.print(tempoCarga);
        Serial.println(" ms");
    }

    Serial.print("ADC no desligamento: ");
    Serial.println(adcNoDisparo);
    Serial.print("Tensão no desligamento: ");
    Serial.print(tensaoNoDisparo, 1);
    Serial.println(" V");
    Serial.print("DONE#: ");
    Serial.println(digitalRead(PIN_DONE));
    Serial.print("FAULT#: ");
    Serial.println(digitalRead(PIN_FAULT));

    delay(5);
    atualizarLeitura();

    Serial.print("ADC após desligar: ");
    Serial.println(leituraADC);
    Serial.print("Tensão após desligar: ");
    Serial.print(valorAtual, 1);
    Serial.println(" V");
    Serial.println("============================");
}

void iniciarCarregamento()
{
    atualizarLeitura();

    if (carregando)
    {
        Serial.println("O carregamento já está ligado.");
        return;
    }

    if (faultAtivo())
    {
        Serial.println();
        Serial.println("Carga não iniciada: FAULT# está ativo.");
        Serial.print("Estado de FAULT#: ");
        Serial.println(digitalRead(PIN_FAULT));
        return;
    }

    if (valorAtual >= valorSetado)
    {
        Serial.println();
        Serial.println("Carga não iniciada.");
        Serial.print("Tensão atual: ");
        Serial.print(valorAtual, 1);
        Serial.println(" V");
        Serial.print("Setpoint: ");
        Serial.print(valorSetado, 1);
        Serial.println(" V");
        return;
    }

    digitalWrite(PIN_CARREGAMENTO, LOW);
    digitalWrite(PIN_CHUTE, LOW);
    delay(5);

    inicioCarga = millis();
    ultimoLog = millis();
    inicioDoneLow = 0;
    inicioFaultLow = 0;
    carregando = true;
    digitalWrite(PIN_CARREGAMENTO, HIGH);

    Serial.println();
    Serial.println("============================");
    Serial.println("Carregamento ON");
    Serial.print("Setpoint: ");
    Serial.print(valorSetado, 1);
    Serial.println(" V");
    Serial.println("============================");
}

void solicitarChute()
{
    if (carregando)
    {
        pararCarregamento("COMANDO KICK");
    }

    digitalWrite(PIN_CARREGAMENTO, LOW);
    digitalWrite(PIN_CHUTE, HIGH);
}

void controlarCarregamento()
{
    if (!carregando)
    {
        inicioDoneLow = 0;
        inicioFaultLow = 0;
        return;
    }

    const unsigned long tempoCarga = millis() - inicioCarga;

    if (valorAtual >= TENSAO_MAXIMA_SEGURANCA)
    {
        pararCarregamento("LIMITE DE SEGURANÇA");
        return;
    }

    if (valorAtual >= valorSetado)
    {
        pararCarregamento("SETPOINT DO ADC");
        return;
    }

    if (tempoCarga >= TIMEOUT_CARGA_MS)
    {
        pararCarregamento("TIMEOUT");
        return;
    }

    if (tempoCarga < TEMPO_IGNORAR_SINAIS_MS)
    {
        inicioDoneLow = 0;
        inicioFaultLow = 0;
        return;
    }

    if (faultAtivo())
    {
        if (inicioFaultLow == 0)
        {
            inicioFaultLow = millis();
        }

        if (millis() - inicioFaultLow >= FILTRO_FAULT_MS)
        {
            pararCarregamento("FAULT DO LT3751");
            return;
        }
    }
    else
    {
        inicioFaultLow = 0;
    }

    if (doneAtivo())
    {
        if (inicioDoneLow == 0)
        {
            inicioDoneLow = millis();
        }

        if (millis() - inicioDoneLow >= FILTRO_DONE_MS)
        {
            pararCarregamento("DONE DO LT3751");
            return;
        }
    }
    else
    {
        inicioDoneLow = 0;
    }
}

void imprimirLog()
{
    if (!carregando || millis() - ultimoLog < INTERVALO_LOG_MS)
    {
        return;
    }

    ultimoLog = millis();
    Serial.print("Set: ");
    Serial.print(valorSetado, 1);
    Serial.print(" V | ADC: ");
    Serial.print(leituraADC);
    Serial.print(" | Tensão: ");
    Serial.print(valorAtual, 1);
    Serial.print(" V | DONE#: ");
    Serial.print(digitalRead(PIN_DONE));
    Serial.print(" | FAULT#: ");
    Serial.print(digitalRead(PIN_FAULT));
    Serial.print(" | Tempo: ");
    Serial.print(millis() - inicioCarga);
    Serial.println(" ms");
}

void processarComando(char *comando)
{
    if (comando == nullptr || comando[0] == '\0')
    {
        return;
    }

    for (uint8_t i = 0; comando[i] != '\0'; i++)
    {
        if (comando[i] == ',')
        {
            comando[i] = '.';
        }
    }

    if (strcmp(comando, "1") == 0)
    {
        iniciarCarregamento();
        return;
    }

    if (strcmp(comando, "2") == 0)
    {
        pararCarregamento("COMANDO MANUAL");
        return;
    }

    if (strcmp(comando, "3") == 0)
    {
        solicitarChute();
        return;
    }

    if (strcmp(comando, "status") == 0 || strcmp(comando, "STATUS") == 0 ||
        strcmp(comando, "s") == 0 || strcmp(comando, "S") == 0)
    {
        mostrarStatus();
        return;
    }

    char *fim = nullptr;
    const float novoSetpoint = strtof(comando, &fim);

    if (fim != comando && *fim == '\0')
    {
        if (carregando)
        {
            Serial.println("Pare o carregamento antes de alterar o setpoint.");
            return;
        }

        if (novoSetpoint >= SETPOINT_MINIMO && novoSetpoint <= SETPOINT_MAXIMO)
        {
            valorSetado = novoSetpoint;
            Serial.print("Novo setpoint: ");
            Serial.print(valorSetado, 1);
            Serial.println(" V");
        }
        else
        {
            Serial.print("Setpoint inválido. Use de ");
            Serial.print(SETPOINT_MINIMO, 0);
            Serial.print(" a ");
            Serial.print(SETPOINT_MAXIMO, 0);
            Serial.println(" V.");
        }
        return;
    }

    Serial.print("Comando inválido: ");
    Serial.println(comando);
}

void lerSerial()
{
    while (Serial.available() > 0)
    {
        const char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (indiceSerial > 0)
            {
                bufferSerial[indiceSerial] = '\0';
                processarComando(bufferSerial);
                indiceSerial = 0;
                bufferSerial[0] = '\0';
            }
        }
        else if (c == '\b' || c == 127)
        {
            if (indiceSerial > 0)
            {
                indiceSerial--;
            }
        }
        else if (indiceSerial < sizeof(bufferSerial) - 1)
        {
            bufferSerial[indiceSerial++] = c;
        }
        else
        {
            indiceSerial = 0;
            bufferSerial[0] = '\0';
            Serial.println("Comando muito longo.");
        }
    }
}

void setup()
{
    pinMode(PIN_CARREGAMENTO, OUTPUT);
    pinMode(PIN_CHUTE, OUTPUT);
    digitalWrite(PIN_CARREGAMENTO, LOW);
    digitalWrite(PIN_CHUTE, LOW);

    pinMode(PIN_LEITURA, INPUT);
    pinMode(PIN_DONE, INPUT_PULLUP);
    pinMode(PIN_FAULT, INPUT_PULLUP);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_LEITURA, ADC_11db);
    bufferSerial[0] = '\0';
    Serial.begin(115200);

    const unsigned long inicioEsperaSerial = millis();
    while (!Serial && millis() - inicioEsperaSerial < 3000)
    {
        delay(10);
    }

    delay(500);
    atualizarLeitura();

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-C3 iniciado corretamente");
    Serial.println("Controle do LT3751 pronto");
    Serial.println("================================");
    Serial.println();
    Serial.println("Mapeamento:");
    Serial.println("GPIO0 = READ");
    Serial.println("GPIO1 = KICK");
    Serial.println("GPIO2 = FAULT#");
    Serial.println("GPIO3 = DONE#");
    Serial.println("GPIO4 = CHARGE");
    Serial.println();
    Serial.print("Setpoint atual: ");
    Serial.print(valorSetado, 1);
    Serial.println(" V");
    Serial.print("Tensão medida: ");
    Serial.print(valorAtual, 1);
    Serial.println(" V");
    Serial.println();
    Serial.println("Comandos:");
    Serial.println("1      - Iniciar carregamento");
    Serial.println("2      - Parar carregamento");
    Serial.println("3      - Chute");
    Serial.println("status - Mostrar estado completo");
    Serial.println("20-200 - Alterar setpoint");
    Serial.println();
    Serial.println("Digite o comando e pressione Enter.");
}

void loop()
{
    atualizarLeitura();
    controlarCarregamento();
    lerSerial();
    imprimirLog();
    delay(1);
}
