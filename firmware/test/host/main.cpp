// Runner dos testes de lógica pura no PC (sem hardware).
//
// Com g++ ou clang++ (C++17), a partir de firmware/test/host:
//   g++ -std=c++17 -Wall -Wextra -I../../src ../../src/kicker_sim.cpp ../../src/kicker_sim_testes.cpp \
//       ../../src/autorizacao_chute.cpp ../../src/autorizacao_chute_testes.cpp \
//       ../../src/preditor_chegada.cpp ../../src/preditor_chegada_testes.cpp main.cpp -o testes
//   ./testes
//
// Os mesmos casos rodam no ESP32: comando serial "teste" (firmware normal) ou
// "sim teste" (firmware de simulação).

#include <cstdio>

#include "autorizacao_chute_testes.h"
#include "kicker_sim_testes.h"
#include "preditor_chegada_testes.h"

int main()
{
    const auto escrever = [](const char *linha) { std::puts(linha); };
    const int reprovados = kicker_sim::executar_testes(escrever) + autorizacao::executar_testes(escrever) +
                           chegada::executar_testes(escrever);
    return reprovados == 0 ? 0 : 1;
}
