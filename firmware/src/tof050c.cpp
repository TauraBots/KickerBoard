#include "tof050c.h"

#include <cstdio>

#include "bola_detector.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_soft.h"
#include "vl6180x.h"

namespace tof050c {
namespace {

const char *TAG = "tof";

// Prioridade 1 = mesma do loop principal (app_main). A tarefa nunca tem
// prioridade maior que o controle de carga e passa quase todo o tempo em
// vTaskDelay. Cada transação I2C por software leva ~0,4 ms.
constexpr UBaseType_t PRIORIDADE_TAREFA = 1;
constexpr uint32_t PILHA_TAREFA = 4096;

constexpr uint32_t INTERVALO_CONSULTA_MS = 1;         // entre consultas, após ESPERA_MINIMA_MEDICAO_MS
// Meio período de 2 us => SCL de ~200 kHz (VL6180X aceita até 400 kHz).
constexpr uint32_t MEIO_PERIODO_I2C_US = 2;
// Sem resultado neste prazo => timeout. 2x a medição mais longa (convergência
// máxima + averaging), para recuperar rápido de uma medição perdida.
constexpr uint32_t TIMEOUT_AMOSTRA_MS = 2 * vl6180x::TEMPO_MAXIMO_MEDICAO_MS;
constexpr uint32_t ESPERA_REINICIALIZACAO_MS = 3000;  // entre tentativas de inicialização
constexpr uint32_t ERROS_I2C_PARA_REINICIAR = 5;      // erros consecutivos antes de reinicializar
constexpr uint32_t TIMEOUTS_PARA_REINICIAR = 3;
constexpr uint32_t INTERVALO_LINHA_MS = 200;          // linhas "[TOF] ..." (5 Hz)
constexpr uint32_t INTERVALO_RESUMO_MS = 10000;       // bloco "===== TOF050C ====="

I2cSoft barramento(static_cast<gpio_num_t>(PIN_SDA), static_cast<gpio_num_t>(PIN_SCL), MEIO_PERIODO_I2C_US);
vl6180x::Sensor sensor(barramento);
bola::Detector detector;  // usado só pela tarefa do sensor

SemaphoreHandle_t mutex_leitura;
Leitura leitura_compartilhada;
volatile bool log_ativo = true;
volatile bool log_rapido = false;  // true: imprime todas as amostras

// Relógio monotônico real (esp_timer, resolução de µs), usado nos timestamps das amostras.
uint64_t agora_ms() { return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL; }

void publicar(const Leitura &leitura)
{
    xSemaphoreTake(mutex_leitura, portMAX_DELAY);
    leitura_compartilhada = leitura;
    xSemaphoreGive(mutex_leitura);
}

const char *nome_erro_i2c(esp_err_t erro)
{
    switch (erro) {
        case ESP_ERR_NOT_FOUND: return "sem ACK no endereco (sensor ausente?)";
        case ESP_FAIL: return "sem ACK nos dados";
        case ESP_ERR_TIMEOUT: return "timeout (SCL preso em nivel baixo)";
        case ESP_ERR_INVALID_STATE: return "barramento travado (SDA baixo)";
        case ESP_ERR_INVALID_RESPONSE: return "ID inesperado";
        default: return esp_err_to_name(erro);
    }
}

void imprimir_resumo(const Leitura &l, float taxa_hz)
{
    char distancia[24];
    if (l.tem_amostra && l.valida) snprintf(distancia, sizeof(distancia), "%u mm", l.distancia_mm);
    else snprintf(distancia, sizeof(distancia), "-- mm");

    const char *status = !l.sensor_ok ? "SEM SENSOR" : !l.tem_amostra ? "AGUARDANDO" : l.valida ? "VALIDA" : "INVALIDA";
    printf("\n========== TOF050C ==========\n"
           "Sensor: VL6180X\n"
           "I2C: 0x%02X (SDA=GPIO%d, SCL=GPIO%d, software)\n"
           "Distancia: %s\n"
           "Status: %s\n"
           "Taxa de medicao: %.1f Hz\n"
           "Bola: %s | Deteccoes: %lu (entrada <= %u mm, saida > %u mm, %u confirmacoes)\n"
           "Amostras: %lu | Invalidas: %lu\n"
           "Erros I2C: %lu | Timeouts: %lu | Falhas init: %lu\n"
           "=============================\n",
           sensor.endereco(), PIN_SDA, PIN_SCL, distancia, status, taxa_hz,
           l.bola_presente ? "BOLA_PRESENTE" : "BOLA_AUSENTE", static_cast<unsigned long>(l.bola_deteccoes),
           bola::LIMITE_ENTRADA_MM, bola::LIMITE_SAIDA_MM, bola::CONFIRMACOES_NECESSARIAS,
           static_cast<unsigned long>(l.amostras), static_cast<unsigned long>(l.invalidas),
           static_cast<unsigned long>(l.erros_i2c), static_cast<unsigned long>(l.timeouts),
           static_cast<unsigned long>(l.falhas_inicializacao));
}

void imprimir_linha(const Leitura &l, const bola::Evento &ev, bool confirmacao_interrompida)
{
    // Estado do detector: "Confirmacao k/N" durante a confirmação (inclusive a
    // que causou a transição para PRESENTE), senão o nome do estado.
    char estado[48];
    if (ev.confirmacoes > 0 && (ev.estado == bola::Estado::Ausente || ev.transicao))
        snprintf(estado, sizeof(estado), "Confirmacao %u/%u", ev.confirmacoes, bola::CONFIRMACOES_NECESSARIAS);
    else
        snprintf(estado, sizeof(estado), "%s%s", bola::nome_estado(ev.estado),
                 confirmacao_interrompida ? " (confirmacao interrompida)" : "");

    // Um único printf por linha para não intercalar com os logs do kicker.
    if (l.valida) {
        printf("[TOF] %3u mm | %s | sinal %.2f Mcps\n", l.distancia_mm, estado, l.sinal_mcps);
    } else if (l.erro_sistema) {
        printf("[TOF]  -- mm | %s | INVALIDA | Codigo: %u (%s) | ERRO SISTEMA %u (%s) | bruto %u | sinal %.2f Mcps\n",
               estado, l.codigo_erro, vl6180x::descrever_codigo(l.codigo_erro), l.erro_sistema,
               l.erro_sistema == 1 ? "laser safety" : "PLL", l.distancia_bruta, l.sinal_mcps);
    } else {
        printf("[TOF]  -- mm | %s | INVALIDA | Codigo: %u (%s) | bruto %u | sinal %.2f Mcps\n",
               estado, l.codigo_erro, vl6180x::descrever_codigo(l.codigo_erro), l.distancia_bruta, l.sinal_mcps);
    }
}

// Atualiza a leitura publicada e registra transições (sempre, mesmo com o log desligado).
void aplicar_evento(Leitura &l, const bola::Evento &ev)
{
    l.bola_presente = detector.estado() == bola::Estado::Presente;
    l.bola_confirmacoes = detector.confirmacoes();
    l.bola_deteccoes = detector.transicoes_para_presente();
    if (!ev.transicao) return;

    if (ev.estado == bola::Estado::Presente) {
        printf("[BOLA] PRESENTE | %u mm | %u leituras validas consecutivas <= %u mm\n", l.distancia_mm,
               bola::CONFIRMACOES_NECESSARIAS, bola::LIMITE_ENTRADA_MM);
        return;
    }
    switch (ev.causa) {
        case bola::Causa::AcimaDoLimiteSaida:
            printf("[BOLA] AUSENTE | %u mm > %u mm (limite de saida)\n", l.distancia_mm, bola::LIMITE_SAIDA_MM);
            break;
        case bola::Causa::LeituraInvalida:
            printf("[BOLA] AUSENTE | leitura invalida (codigo %u: %s%s)\n", l.codigo_erro,
                   vl6180x::descrever_codigo(l.codigo_erro), l.erro_sistema ? ", erro de sistema" : "");
            break;
        case bola::Causa::LeituraDesatualizada:
            printf("[BOLA] AUSENTE | leitura desatualizada (%lu ms > %lu ms)\n",
                   static_cast<unsigned long>(ev.idade_ms), static_cast<unsigned long>(bola::MAX_IDADE_LEITURA_MS));
            break;
        default:
            printf("[BOLA] AUSENTE | %s\n", bola::descrever_causa(ev.causa));
            break;
    }
}

void diagnosticar_barramento()
{
    int sda_up = 0, scl_up = 0, sda_down = 0, scl_down = 0;
    barramento.testar_linhas(&sda_up, &scl_up, &sda_down, &scl_down);
    printf("\n[TOF] ===== Diagnostico do barramento (SDA=GPIO%d, SCL=GPIO%d) =====\n"
           "[TOF] Com pull-up interno : SDA=%d SCL=%d (esperado 1 e 1)\n"
           "[TOF] Com pull-down interno: SDA=%d SCL=%d (1 = ha pull-up externo/modulo alimentado)\n",
           PIN_SDA, PIN_SCL, sda_up, scl_up, sda_down, scl_down);
    if (!sda_up || !scl_up)
        printf("[TOF] -> Linha presa em 0: curto com GND, SDA/SCL trocados com outro sinal ou modulo travando a linha.\n");
    else if (!sda_down && !scl_down)
        printf("[TOF] -> Nenhum pull-up externo: modulo sem alimentacao, fio solto ou modulo sem resistores de pull-up.\n");
    else if (!sda_down || !scl_down)
        printf("[TOF] -> So uma das linhas tem pull-up externo: verifique o fio da linha que leu 0.\n");

    barramento.recuperar();
    char encontrados[128] = "";
    size_t pos = 0;
    int quantidade = 0;
    for (uint8_t endereco = 0x08; endereco <= 0x77; ++endereco) {
        if (barramento.sondar(endereco) == ESP_OK && pos < sizeof(encontrados) - 6) {
            pos += snprintf(encontrados + pos, sizeof(encontrados) - pos, " 0x%02X", endereco);
            ++quantidade;
        }
        if ((endereco & 0x0f) == 0) vTaskDelay(1);  // Não monopoliza a CPU durante a varredura.
    }
    if (quantidade) printf("[TOF] Varredura I2C: %d dispositivo(s):%s\n", quantidade, encontrados);
    else printf("[TOF] Varredura I2C: nenhum dispositivo respondeu (0x08-0x77)\n");
    printf("[TOF] =============================================================\n");
}

bool inicializar_sensor(Leitura &l)
{
    vl6180x::Identificacao id = {};
    esp_err_t erro = barramento.recuperar();
    if (erro == ESP_OK) erro = sensor.iniciar(&id);

    if (erro != ESP_OK) {
        ++l.falhas_inicializacao;
        l.sensor_ok = false;
        if (erro == ESP_ERR_INVALID_RESPONSE)
            printf("[TOF] Falha na inicializacao (tentativa %lu): ID 0x%02X em 0x%02X, esperado 0x%02X (VL6180X)\n",
                   static_cast<unsigned long>(l.falhas_inicializacao), id.model_id, sensor.endereco(),
                   vl6180x::MODEL_ID_ESPERADO);
        else
            printf("[TOF] Falha na inicializacao (tentativa %lu): %s. Nova tentativa em %lu s.\n",
                   static_cast<unsigned long>(l.falhas_inicializacao), nome_erro_i2c(erro),
                   static_cast<unsigned long>(ESPERA_REINICIALIZACAO_MS / 1000));
        // Diagnóstico na 1ª falha e depois a cada 10 tentativas (~30 s), para não poluir o log.
        if (l.falhas_inicializacao % 10 == 1) diagnosticar_barramento();
        return false;
    }

    l.sensor_ok = true;
    l.tem_amostra = false;
    printf("\n[TOF] VL6180X OK em 0x%02X | modelo 0x%02X rev %u.%u modulo %u.%u | %s\n"
           "[TOF] Offset fabrica: %d mm | crosstalk: %.2f Mcps | convergencia max %lu ms\n",
           sensor.endereco(), id.model_id, id.revisao_modelo >> 4, id.revisao_modelo & 0x0f,
           id.revisao_modulo >> 4, id.revisao_modulo & 0x0f,
           id.fresh_out_of_reset ? "configuracao obrigatoria carregada" : "sensor ja estava configurado",
           id.offset_mm, vl6180x::taxa_mcps(id.crosstalk),
           static_cast<unsigned long>(vl6180x::MAX_CONVERGENCIA_MS));
    return true;
}

void tarefa_tof(void *)
{
    Leitura l = {};
    const esp_err_t erro_gpio = barramento.iniciar();
    if (erro_gpio != ESP_OK)
        printf("[TOF] Aviso: barramento SDA=GPIO%d/SCL=GPIO%d nao liberou (%s)\n",
               PIN_SDA, PIN_SCL, nome_erro_i2c(erro_gpio));

    uint32_t erros_consecutivos = 0;
    uint32_t timeouts_consecutivos = 0;
    uint64_t inicio_medicao = 0;
    uint64_t ultima_linha = 0;
    uint64_t ultimo_resumo = 0;
    uint32_t amostras_no_resumo = 0;  // para calcular a taxa real entre resumos
    bool medindo = false;           // aguardando o resultado (ou o sensor ficar pronto)
    bool aguardando_pronto = false;  // START ainda não aceito: sensor não estava pronto
    bool recem_disparada = false;

    while (true) {
        if (!l.sensor_ok) {
            medindo = false;
            aplicar_evento(l, detector.invalidar(bola::Causa::ErroSensor));
            if (inicializar_sensor(l)) {
                erros_consecutivos = timeouts_consecutivos = 0;
                publicar(l);
                imprimir_resumo(l, 0.0f);
                ultimo_resumo = agora_ms();
                amostras_no_resumo = l.amostras;
            } else {
                publicar(l);
                vTaskDelay(pdMS_TO_TICKS(ESPERA_REINICIALIZACAO_MS));
            }
            continue;
        }

        const uint64_t agora = agora_ms();
        esp_err_t erro = ESP_OK;
        bool pronta = false;

        if (!medindo || aguardando_pronto) {
            // Dispara a próxima medição logo após a anterior; a taxa fica limitada
            // pelo tempo real de convergência do sensor. Cada ciclo sempre passa
            // por um vTaskDelay (abaixo), então não há laço ocupado.
            //
            // device_ready é SEMPRE conferido antes do START: o VL6180X ignora um
            // START recebido enquanto não está pronto, e a medição se perderia.
            erro = sensor.disparar_medicao(true);
            if (erro == ESP_OK) {
                medindo = true;
                aguardando_pronto = false;
                recem_disparada = true;
                inicio_medicao = agora;
            } else if (erro == ESP_ERR_INVALID_STATE) {
                // Ainda não pronto (finalizando a medição anterior): nenhuma medição
                // foi pedida; tenta de novo em 1 ms. O prazo de timeout vale também aqui.
                erro = ESP_OK;
                if (!medindo) inicio_medicao = agora;
                medindo = true;
                aguardando_pronto = true;
            }
        } else {
            erro = sensor.amostra_pronta(&pronta);
        }

        if (erro == ESP_OK && pronta) {
            vl6180x::Amostra amostra = {};
            erro = sensor.ler_amostra(&amostra);
            medindo = false;
            if (erro == ESP_OK) {
                const uint64_t t_amostra = agora_ms();  // instante real em que o resultado foi lido
                l.tem_amostra = true;
                l.codigo_erro = amostra.codigo_erro;
                l.erro_sistema = amostra.erro_sistema;
                l.distancia_bruta = amostra.distancia_bruta_mm;
                l.valida = vl6180x::codigo_valido(amostra.codigo_erro) && amostra.erro_sistema == 0;
                // Leitura inválida NÃO vira 0 mm: a distância só é publicada quando válida.
                l.distancia_mm = l.valida ? amostra.distancia_bruta_mm : 0;
                l.sinal_mcps = vl6180x::taxa_mcps(amostra.taxa_retorno);
                l.instante_ms = t_amostra;
                ++l.amostras;
                if (!l.valida) ++l.invalidas;
                erros_consecutivos = timeouts_consecutivos = 0;

                const uint8_t confirmacoes_antes = detector.confirmacoes();
                const bola::Evento ev = detector.processar_amostra(l.valida, l.distancia_mm, t_amostra);
                const bool interrompida = confirmacoes_antes > 0 && ev.confirmacoes == 0 && !ev.transicao;
                // Linhas de confirmação, interrupção e transição são sempre impressas;
                // as demais seguem o limite de INTERVALO_LINHA_MS (ou todas com "tof r").
                const bool forcar = ev.transicao || ev.confirmacoes > 0 || interrompida;
                if (log_ativo && (forcar || log_rapido || t_amostra - ultima_linha >= INTERVALO_LINHA_MS)) {
                    imprimir_linha(l, ev, interrompida);
                    ultima_linha = t_amostra;
                }
                aplicar_evento(l, ev);
                publicar(l);
            }
        }

        if (erro != ESP_OK) {
            medindo = aguardando_pronto = false;
            ++l.erros_i2c;
            ++erros_consecutivos;
            printf("[TOF] Erro I2C: %s (%lu consecutivos)\n", nome_erro_i2c(erro),
                   static_cast<unsigned long>(erros_consecutivos));
            aplicar_evento(l, detector.invalidar(bola::Causa::ErroSensor));
            barramento.recuperar();
            if (erros_consecutivos >= ERROS_I2C_PARA_REINICIAR) {
                printf("[TOF] Muitos erros I2C: reinicializando o sensor\n");
                l.sensor_ok = false;
            }
            publicar(l);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (medindo && agora - inicio_medicao >= TIMEOUT_AMOSTRA_MS) {
            const bool nunca_ficou_pronto = aguardando_pronto;
            medindo = aguardando_pronto = false;
            ++l.timeouts;
            ++timeouts_consecutivos;
            // Invalida a presença antes do diagnóstico/recuperação, que pode demorar.
            aplicar_evento(l, detector.invalidar(bola::Causa::Timeout));
            publicar(l);
            vl6180x::Sensor::Registros r = {};
            if (sensor.ler_registros(&r) == ESP_OK)
                printf("[TOF] TIMEOUT: %s em %lu ms | RANGE_STATUS=0x%02X (ready=%u, erro=%u) "
                       "INT_STATUS=0x%02X START=0x%02X INT_CONFIG=0x%02X FRESH=0x%02X\n",
                       nunca_ficou_pronto ? "sensor nao ficou pronto" : "medicao sem resultado",
                       static_cast<unsigned long>(TIMEOUT_AMOSTRA_MS), r.range_status, r.range_status & 0x01,
                       r.range_status >> 4, r.interrupt_status, r.sysrange_start, r.interrupt_config,
                       r.fresh_out_of_reset);
            else
                printf("[TOF] TIMEOUT: medicao sem resultado em %lu ms (registros ilegiveis)\n",
                       static_cast<unsigned long>(TIMEOUT_AMOSTRA_MS));
            // Um timeout isolado só re-dispara (próxima volta). A tentativa de
            // destravar (que dorme >= 250 ms) fica para timeouts consecutivos.
            if (timeouts_consecutivos >= 2 && sensor.destravar() != ESP_OK)
                printf("[TOF] Sensor continua ocupado apos tentativa de parada\n");
            if (timeouts_consecutivos >= TIMEOUTS_PARA_REINICIAR) {
                printf("[TOF] Timeouts repetidos: reinicializando o sensor\n");
                l.sensor_ok = false;
            }
            publicar(l);
            continue;
        }

        // Leitura desatualizada: verificada a cada volta do laço (no máximo a cada ~5 ms).
        const bola::Evento ev_idade = detector.verificar_idade(agora_ms());
        if (ev_idade.transicao) {
            aplicar_evento(l, ev_idade);
            publicar(l);
        }

        if (agora - ultimo_resumo >= INTERVALO_RESUMO_MS) {
            const float taxa = (l.amostras - amostras_no_resumo) * 1000.0f / static_cast<float>(agora - ultimo_resumo);
            if (log_ativo) imprimir_resumo(l, taxa);
            ultimo_resumo = agora;
            amostras_no_resumo = l.amostras;
        }

        // Logo após o disparo dorme a duração mínima de uma medição; depois
        // consulta a cada INTERVALO_CONSULTA_MS. Após ler uma amostra, volta ao
        // início e dispara a próxima (todo ciclo passa por um vTaskDelay).
        if (medindo) {
            // aguardando_pronto: recem_disparada é false, então tenta de novo em 1 ms.
            vTaskDelay(pdMS_TO_TICKS(recem_disparada ? vl6180x::ESPERA_MINIMA_MEDICAO_MS : INTERVALO_CONSULTA_MS));
            recem_disparada = false;
        }
    }
}

}  // namespace

void iniciar()
{
    if (mutex_leitura) return;
    mutex_leitura = xSemaphoreCreateMutex();
    if (!mutex_leitura) {
        ESP_LOGE(TAG, "Falha ao criar mutex do TOF050C; monitor desativado");
        return;
    }
    if (xTaskCreate(tarefa_tof, "tof050c", PILHA_TAREFA, nullptr, PRIORIDADE_TAREFA, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar tarefa do TOF050C; monitor desativado");
        return;
    }
    printf("TOF050C: monitor de distancia (somente diagnostico), SDA=GPIO%d, SCL=GPIO%d. Comandos: 'tof' liga/desliga o log, 'tof r' mostra todas as amostras.\n",
           PIN_SDA, PIN_SCL);
}

bool obter_leitura(Leitura *destino)
{
    if (!mutex_leitura || !destino) return false;
    xSemaphoreTake(mutex_leitura, portMAX_DELAY);
    *destino = leitura_compartilhada;
    xSemaphoreGive(mutex_leitura);
    return true;
}

bool bola_presente_autorizada()
{
    Leitura l;
    if (!obter_leitura(&l)) return false;
    if (!l.sensor_ok || !l.bola_presente || !l.tem_amostra) return false;
    const uint64_t agora = agora_ms();
    return agora < l.instante_ms || agora - l.instante_ms <= bola::MAX_IDADE_LEITURA_MS;
}

void processar_comando(const char *argumento)
{
    while (*argumento == ' ') ++argumento;
    if (!*argumento) {
        log_ativo = !log_ativo;
        printf("TOF050C: log %s\n", log_ativo ? "LIGADO" : "DESLIGADO");
    } else if ((argumento[0] == 'r' || argumento[0] == 'R') && !argumento[1]) {
        log_rapido = !log_rapido;
        log_ativo = true;
        printf("TOF050C: log %s\n", log_rapido ? "RAPIDO (todas as amostras)" : "NORMAL (5 linhas/s)");
    } else {
        printf("TOF050C: comando invalido. Use 'tof' (liga/desliga) ou 'tof r' (todas as amostras).\n");
    }
}

}  // namespace tof050c
