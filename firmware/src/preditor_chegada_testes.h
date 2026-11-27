#pragma once

// Testes do preditor de chegada (lógica pura). Rodam no ESP32 pelo comando
// serial "teste" e no PC (test/host).

#include "autorizacao_chute_testes.h"

namespace chegada {

// Retorna o número de casos reprovados.
int executar_testes(autorizacao::Escrita escrever);

}  // namespace chegada
