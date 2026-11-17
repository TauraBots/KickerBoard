#pragma once

// Testes automatizados da máquina de estados kicker_sim.
// Código puro: roda no ESP32 (comando serial "sim teste" no firmware de
// simulação) e em qualquer compilador C++17 do PC (test/host/).

namespace kicker_sim {

using Escrita = void (*)(const char *linha);

// Executa todos os casos. Retorna o número de casos reprovados (0 = todos aprovados).
int executar_testes(Escrita escrever);

}  // namespace kicker_sim
