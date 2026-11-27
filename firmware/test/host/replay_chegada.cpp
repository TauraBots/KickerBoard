// Reproduz uma captura (tools/captura_chute.py) no preditor de chegada, no PC.
//
//   g++ -std=c++17 -O2 -Wall -Wextra -I../../src ../../src/preditor_chegada.cpp replay_chegada.cpp -o replay_chegada
//   ./replay_chegada ../../tools/capturas/rodada2.csv [d_contato_mm] [latencia_ms] [amostras]
//
// Saída (CSV, uma linha por amostra "C"): t_us,valida,mm,prev,t_contato_us,t_disparo_us,v_mm_ms

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "preditor_chegada.h"

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "uso: %s captura.csv [d_contato_mm] [latencia_ms] [amostras]\n", argv[0]);
        return 2;
    }
    chegada::Config cfg;
    if (argc > 2) cfg.d_contato_mm = static_cast<uint8_t>(std::atoi(argv[2]));
    if (argc > 3) cfg.latencia_kicker_us = static_cast<uint32_t>(std::atof(argv[3]) * 1000.0);
    if (argc > 4) cfg.amostras_regressao = static_cast<uint8_t>(std::atoi(argv[4]));
    chegada::Preditor p;
    p.configurar(cfg);

    FILE *f = std::fopen(argv[1], "r");
    if (!f) {
        std::perror(argv[1]);
        return 1;
    }
    char linha[256];
    std::puts("t_us,valida,mm,prev,t_contato_us,t_disparo_us,v_mm_ms,t_ultima_us");
    while (std::fgets(linha, sizeof(linha), f)) {
        if (std::strncmp(linha, "C,", 2) != 0) continue;
        long long t = 0;
        unsigned valida = 0, mm = 0;
        if (std::sscanf(linha, "C,%lld,%u,%u", &t, &valida, &mm) != 3) continue;
        const chegada::Previsao pr = p.processar_amostra(valida != 0, static_cast<uint8_t>(mm), t);
        std::printf("%lld,%u,%u,%d,%lld,%lld,%.3f,%lld\n", t, valida, mm, pr.valida ? 1 : 0,
                    static_cast<long long>(pr.t_contato_us), static_cast<long long>(pr.t_disparo_us), pr.v_mm_ms,
                    static_cast<long long>(pr.t_ultima_leitura_us));
    }
    std::fclose(f);
    return 0;
}
