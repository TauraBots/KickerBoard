#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include <WiFi.h>
#include <WebServer.h>

// ==================================================
// Pinos conforme o esquema da placa
// ==================================================

constexpr uint8_t PIN_LEITURA      = 0;  // READ
constexpr uint8_t PIN_CHUTE        = 1;  // KICK
constexpr uint8_t PIN_FAULT        = 2;  // FAULT#
constexpr uint8_t PIN_DONE         = 3;  // DONE#
constexpr uint8_t PIN_CARREGAMENTO = 4; // CHARGE

// ==================================================
// Calibração do ADC
// ==================================================

constexpr float ADC_EM_200V = 3700.0f;
constexpr float TENSAO_CALIBRACAO = 200.0f;
constexpr float FATOR_CALIBRACAO = 1.0f;

// ==================================================
// Configurações
// ==================================================

constexpr uint8_t AMOSTRAS_ADC = 16;
constexpr unsigned long INTERVALO_LOG_MS = 100;
constexpr unsigned long TIMEOUT_CARGA_MS = 15000;
constexpr unsigned long TEMPO_IGNORAR_SINAIS_MS = 5;
constexpr unsigned long FILTRO_DONE_MS = 2;
constexpr unsigned long FILTRO_FAULT_MS = 2;

constexpr float SETPOINT_MINIMO = 20.0f;
constexpr float SETPOINT_MAXIMO = 200.0f;
constexpr float TENSAO_MAXIMA_SEGURANCA = 205.0f;

// ==================================================
// Variáveis
// ==================================================

bool carregando = false;

// Variáveis do Novo Gatilho Automático (Porcentagem)
bool autoChuteAtivo = false;
float tensaoAutoChute = 0.0f;

unsigned long inicioCarga = 0;
unsigned long ultimoLog = 0;
unsigned long inicioDoneLow = 0;
unsigned long inicioFaultLow = 0;

float valorSetado = 200.0f;
float valorAtual = 0.0f;
uint16_t leituraADC = 0;

char bufferSerial[32];
uint8_t indiceSerial = 0;

// ==================================================
// Wi-Fi - ESP32-C3 Super Mini
// ==================================================

const char *WIFI_SSID = "TauraBots-Kicker";
const char *WIFI_SENHA = "taurabots2026";

WebServer servidor(80);

// ==================================================
// Prototipação
// ==================================================
void iniciarCarregamento();
void pararCarregamento(const char *motivo);
void solicitarChute();
void iniciarWiFi();
void iniciarServidorWeb();

// ==================================================
// ADC
// ==================================================

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

// ==================================================
// Entradas do LT3751
// ==================================================

bool doneAtivo()
{
    return digitalRead(PIN_DONE) == LOW;
}

bool faultAtivo()
{
    return digitalRead(PIN_FAULT) == LOW;
}

// ==================================================
// Status
// ==================================================

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
    
    // Status do Gatilho Automático
    Serial.print("Auto-Kick: ");
    if (autoChuteAtivo) {
        Serial.print("ARMADO para ");
        Serial.print(tensaoAutoChute, 1);
        Serial.println(" V");
    } else {
        Serial.println("DESARMADO");
    }

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

// ==================================================
// Parar carregamento
// ==================================================

void pararCarregamento(const char *motivo)
{
    // Desarma qualquer disparo automático pendente por segurança
    autoChuteAtivo = false; 

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

    Serial.print("Tensão no desligamento: ");
    Serial.print(tensaoNoDisparo, 1);
    Serial.println(" V");
    Serial.println("============================");
}

// ==================================================
// Iniciar carregamento
// ==================================================

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
        Serial.println("Carga não iniciada: FAULT# está ativo.");
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
    Serial.println("============================");
}

// ==================================================
// KICK
// ==================================================

void solicitarChute()
{
    if (carregando)
    {
        pararCarregamento("COMANDO KICK / AUTO-KICK");
    }

    digitalWrite(PIN_CARREGAMENTO, LOW);
    digitalWrite(PIN_CHUTE, HIGH);
    delay(50); 
    digitalWrite(PIN_CHUTE, LOW);
    Serial.println("KICK disparado!");
}

// ==================================================
// Controle do carregamento
// ==================================================

void controlarCarregamento()
{
    if (!carregando)
    {
        inicioDoneLow = 0;
        inicioFaultLow = 0;
        return;
    }

    const unsigned long tempoCarga = millis() - inicioCarga;

    // Limite absoluto por software.
    if (valorAtual >= TENSAO_MAXIMA_SEGURANCA)
    {
        pararCarregamento("LIMITE DE SEGURANÇA");
        return;
    }

    // ==================================================
    // NOVO: Gatilho Automático por Porcentagem
    // ==================================================
    if (autoChuteAtivo && valorAtual >= tensaoAutoChute)
    {
        Serial.println();
        Serial.println(">> ALVO DE PORCENTAGEM ATINGIDO! Disparando... <<");
        solicitarChute(); // O solicitarChute já desarma o autoChuteAtivo indiretamente via pararCarregamento
        return;
    }

    // Setpoint tradicional solicitado.
    if (valorAtual >= valorSetado)
    {
        pararCarregamento("SETPOINT DO ADC");
        return;
    }

    // Evita carregamento contínuo.
    if (tempoCarga >= TIMEOUT_CARGA_MS)
    {
        pararCarregamento("TIMEOUT");
        return;
    }

    // Ignora possíveis transientes no início.
    if (tempoCarga < TEMPO_IGNORAR_SINAIS_MS)
    {
        return;
    }

    // FAULT# e DONE# omitidos para brevidade (mantive a sua lógica igual)
    if (faultAtivo())
    {
        if (inicioFaultLow == 0) inicioFaultLow = millis();
        if (millis() - inicioFaultLow >= FILTRO_FAULT_MS)
        {
            pararCarregamento("FAULT DO LT3751");
            return;
        }
    }
    else { inicioFaultLow = 0; }

    if (doneAtivo())
    {
        if (inicioDoneLow == 0) inicioDoneLow = millis();
        if (millis() - inicioDoneLow >= FILTRO_DONE_MS)
        {
            pararCarregamento("DONE DO LT3751");
            return;
        }
    }
    else { inicioDoneLow = 0; }
}

// ==================================================
// Log durante a carga
// ==================================================

void imprimirLog()
{
    if (!carregando) return;
    if (millis() - ultimoLog < INTERVALO_LOG_MS) return;
    ultimoLog = millis();

    Serial.print("Tensão: ");
    Serial.print(valorAtual, 1);
    Serial.print(" V | Tempo: ");
    Serial.print(millis() - inicioCarga);
    Serial.println(" ms");
}

// ==================================================
// Processar comandos
// ==================================================

void processarComando(char *comando)
{
    if (comando == nullptr || comando[0] == '\0') return;

    for (uint8_t i = 0; comando[i] != '\0'; i++)
    {
        if (comando[i] == ',') comando[i] = '.';
    }

    // ==================================================
    // NOVO: Lê comando de porcentagem (ex: 80%)
    // ==================================================
    char *sinalPorcentagem = strchr(comando, '%');
    if (sinalPorcentagem != nullptr)
    {
        *sinalPorcentagem = '\0'; // Remove o '%' para converter em número
        float porcentagem = strtof(comando, nullptr);
        
        if (porcentagem > 0.0f && porcentagem <= 100.0f)
        {
            // Calcula o alvo com base no Setpoint Máximo (200V)
            tensaoAutoChute = (porcentagem / 100.0f) * SETPOINT_MAXIMO; 
            autoChuteAtivo = true;
            
            Serial.print(">>> Disparo automatico engatilhado para ");
            Serial.print(porcentagem, 1);
            Serial.print("% (");
            Serial.print(tensaoAutoChute, 1);
            Serial.println(" V) <<<");
            
            // Inicia o carregamento logo em seguida, conforme você pediu
            iniciarCarregamento(); 
        }
        else
        {
            Serial.println("Porcentagem invalida. Envie de 1% a 100%.");
        }
        return;
    }

    // Comandos padrão
    if (strcmp(comando, "1") == 0) { iniciarCarregamento(); return; }
    if (strcmp(comando, "2") == 0) { pararCarregamento("COMANDO MANUAL"); return; }
    if (strcmp(comando, "3") == 0) { solicitarChute(); return; }
    
    if (strcmp(comando, "status") == 0 || strcmp(comando, "s") == 0 || strcmp(comando, "S") == 0)
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
        return;
    }

    Serial.print("Comando inválido: ");
    Serial.println(comando);
}

// ==================================================
// Leitura da Serial
// ==================================================

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
            if (indiceSerial > 0) indiceSerial--;
        }
        else
        {
            if (indiceSerial < sizeof(bufferSerial) - 1)
            {
                bufferSerial[indiceSerial] = c;
                indiceSerial++;
            }
            else
            {
                indiceSerial = 0;
                bufferSerial[0] = '\0';
            }
        }
    }
}

// ==================================================
// Wi-Fi / Servidor Web
// ==================================================

void iniciarWiFi()
{
    Serial.println();
    Serial.println("Iniciando WiFi AP...");

    WiFi.mode(WIFI_AP);
    delay(500);

    bool ok = WiFi.softAP(
        WIFI_SSID,
        WIFI_SENHA,
        1,      // Canal 1
        false,  // SSID visível
        1       // Máximo de 1 cliente
    );

    delay(200);

    // Na ESP32-C3 Super Mini esta potência fez o AP aparecer corretamente.
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    if (ok)
    {
        Serial.println("WiFi criado!");
        Serial.print("Rede: ");
        Serial.println(WIFI_SSID);
        Serial.print("IP: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("Potencia configurada: ");
        Serial.println((int)WiFi.getTxPower());
    }
    else
    {
        Serial.println("ERRO ao criar WiFi!");
    }
}

void iniciarServidorWeb()
{
    servidor.on("/", HTTP_GET, []()
    {
        atualizarLeitura();

        String pagina;
        pagina.reserve(1700);

        pagina += "<!DOCTYPE html><html lang='pt-BR'><head>";
        pagina += "<meta charset='UTF-8'>";
        pagina += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
        pagina += "<title>TauraBots Kicker</title>";
        pagina += "<style>";
        pagina += "body{font-family:Arial,sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:35px 15px;}";
        pagina += ".box{max-width:420px;margin:auto;background:#1d1d1d;padding:25px;border-radius:15px;}";
        pagina += ".v{font-size:32px;font-weight:bold;margin:20px 0;}";
        pagina += "button{width:100%;padding:22px;font-size:25px;font-weight:bold;border:0;border-radius:12px;background:#d62828;color:white;cursor:pointer;}";
        pagina += "button:active{transform:scale(.98);}";
        pagina += ".estado{margin-bottom:20px;color:#bbb;}";
        pagina += "</style></head><body><div class='box'>";
        pagina += "<h1>TauraBots Kicker</h1>";
        pagina += "<div style='color:#f59e0b;font-weight:bold;margin-bottom:15px;'>INTERFACE V2 - PORCENTAGEM</div>";
        pagina += "<div class='v'>";
        pagina += String(valorAtual, 1);
        pagina += " V</div>";
        pagina += "<div class='estado'>Carregamento: ";
        pagina += carregando ? "ON" : "OFF";
        pagina += "</div>";

        pagina += "<div class='estado'>Auto-Kick: ";
        if (autoChuteAtivo)
        {
            pagina += "ARMADO para ";
            pagina += String(tensaoAutoChute, 1);
            pagina += " V";
        }
        else
        {
            pagina += "DESARMADO";
        }
        pagina += "</div>";

        pagina += "<form method='POST' action='/percent'>";
        pagina += "<label for='p'>Porcentagem do disparo:</label><br><br>";
        pagina += "<input id='p' name='p' type='number' min='1' max='100' step='1' value='80' required ";
        pagina += "style='width:100%;box-sizing:border-box;padding:15px;font-size:22px;text-align:center;border-radius:10px;border:0;margin-bottom:12px;'>";
        pagina += "<button type='submit' style='background:#f59e0b;margin-bottom:18px;'>CARREGAR E DISPARAR (%)</button>";
        pagina += "</form>";

        pagina += "<form method='POST' action='/kick' onsubmit=\"return confirm('Confirmar disparo imediato?');\">";
        pagina += "<button type='submit'>DISPARAR AGORA</button>";
        pagina += "</form>";

        pagina += "<p><a href='/' style='color:#aaa'>Atualizar status</a></p>";
        pagina += "</div></body></html>";

        servidor.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        servidor.sendHeader("Pragma", "no-cache");
        servidor.sendHeader("Expires", "0");
        servidor.send(200, "text/html; charset=utf-8", pagina);
    });

    servidor.on("/percent", HTTP_POST, []()
    {
        if (!servidor.hasArg("p"))
        {
            servidor.send(400, "text/plain", "Porcentagem nao informada.");
            return;
        }

        String porcentagem = servidor.arg("p");
        float valor = porcentagem.toFloat();

        if (valor <= 0.0f || valor > 100.0f)
        {
            servidor.send(400, "text/plain", "Porcentagem invalida. Use de 1 a 100.");
            return;
        }

        // Reutiliza a mesma lógica de porcentagem já existente na Serial.
        String comandoString = porcentagem + "%";

        char comandoPercentual[16];
        comandoString.toCharArray(comandoPercentual, sizeof(comandoPercentual));

        Serial.print(">>> COMANDO RECEBIDO PELO WIFI: ");
        Serial.println(comandoString);

        processarComando(comandoPercentual);

        servidor.sendHeader("Location", "/");
        servidor.send(303, "text/plain", "");
    });

    servidor.on("/kick", HTTP_POST, []()
    {
        Serial.println();
        Serial.println(">>> COMANDO KICK RECEBIDO PELO WIFI <<<");

        // Reutiliza exatamente a função de disparo já existente.
        solicitarChute();

        servidor.sendHeader("Location", "/");
        servidor.send(303, "text/plain", "");
    });

    servidor.onNotFound([]()
    {
        servidor.send(404, "text/plain", "Nao encontrado");
    });

    servidor.begin();

    Serial.println("Servidor Web iniciado.");
    Serial.println("INTERFACE WIFI V2 - COM PORCENTAGEM");
    Serial.print("Abra no navegador: http://");
    Serial.println(WiFi.softAPIP());
}

// ==================================================
// Setup
// ==================================================

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
    while (!Serial && millis() - inicioEsperaSerial < 3000) { delay(10); }

    delay(500);
    atualizarLeitura();
    Serial.println("================================");
    Serial.println("ESP32-C3 iniciado");
    Serial.println("================================");

    iniciarWiFi();
    iniciarServidorWeb();
}

// ==================================================
// Loop
// ==================================================

void loop()
{
    atualizarLeitura();
    controlarCarregamento();
    lerSerial();
    servidor.handleClient();
    imprimirLog();
    delay(1);
}