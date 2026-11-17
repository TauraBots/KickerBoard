#pragma once

// Integração da máquina de estados kicker_sim ao firmware (modo SIMULAÇÃO).
// Só é compilado/chamado quando KICKER_SIMULACAO=1 (ambiente
// esp32-c3-supermini-sim do platformio.ini).
//
// Todas as funções devem ser chamadas pela tarefa principal (app_main), a
// mesma que processa Serial e a fila I2C: a máquina não é acessada em paralelo.

namespace kicker_sim_fw {

enum class ComandoRemoto { Carga, Parar, Chute, Setpoint, Percentual };

void iniciar();

// Chamado a cada volta do loop principal. tensao_adc_real é só monitorada.
void atualizar(float tensao_adc_real, bool fault_real_ativo);

// Comando da Serial (já normalizado). Retorna true se foi tratado aqui.
bool processar_comando_serial(const char *comando);

// Comando já validado vindo do I2C escravo (0x42). Nunca aciona hardware.
void processar_comando_remoto(ComandoRemoto comando, float valor);

}  // namespace kicker_sim_fw
