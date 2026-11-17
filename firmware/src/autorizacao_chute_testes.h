#pragma once

// Testes da autorização do chute real (lógica pura). Rodam no ESP32 pelo
// comando serial "teste" e no PC pelo runner em test/host/.

namespace autorizacao {

using Escrita = void (*)(const char *linha);

// Retorna o número de casos reprovados (0 = todos aprovados).
int executar_testes(Escrita escrever);

}  // namespace autorizacao
