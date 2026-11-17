#pragma once

// Mestre I2C por software (bit-banging) em dois GPIOs open-drain.
//
// O ESP32-C3 tem um único controlador I2C de hardware, que já está em uso
// como escravo (0x42, GPIO6/GPIO7). Este barramento é totalmente independente
// dele: usa outros pinos e não toca no periférico I2C.

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

class I2cSoft {
public:
    // meio_periodo_us = 5 resulta em SCL de até ~100 kHz (menos, com o overhead do GPIO).
    I2cSoft(gpio_num_t sda, gpio_num_t scl, uint32_t meio_periodo_us = 5,
            uint32_t timeout_stretch_us = 1000);

    // Configura os pinos como open-drain com pull-up interno e libera o barramento.
    esp_err_t iniciar();

    // Gera até 9 pulsos de clock e um STOP para destravar um escravo que
    // ficou segurando SDA em nível baixo.
    esp_err_t recuperar();

    // Diagnóstico: START + endereço+W + STOP. ESP_OK se algum escravo deu ACK.
    esp_err_t sondar(uint8_t endereco);

    // Diagnóstico elétrico. Lê cada linha solta com o pull-up interno ligado e
    // depois com pull-down interno (~45 kΩ). Se a linha continua alta com o
    // pull-down, existe um pull-up externo (módulo alimentado e conectado).
    void testar_linhas(int *sda_pullup, int *scl_pullup, int *sda_pulldown, int *scl_pulldown);

    // Escrita simples: START, endereço+W, dados, STOP.
    esp_err_t escrever(uint8_t endereco, const uint8_t *dados, size_t tamanho);

    // Escrita seguida de leitura com START repetido.
    esp_err_t escrever_ler(uint8_t endereco, const uint8_t *tx, size_t tamanho_tx,
                           uint8_t *rx, size_t tamanho_rx);

    // Códigos de retorno:
    //   ESP_OK                  transação concluída
    //   ESP_ERR_NOT_FOUND       endereço sem ACK (sensor ausente/desconectado)
    //   ESP_FAIL                byte de dados sem ACK
    //   ESP_ERR_TIMEOUT         SCL preso em nível baixo além do timeout
    //   ESP_ERR_INVALID_STATE   barramento ocupado/travado (SDA baixo no START)

private:
    void atraso() const;
    void soltar_sda() const;
    void puxar_sda() const;
    void puxar_scl() const;
    esp_err_t soltar_scl() const;
    esp_err_t start() const;
    void stop() const;
    esp_err_t escrever_byte(uint8_t byte, bool *ack) const;
    esp_err_t ler_byte(bool enviar_ack, uint8_t *byte) const;
    esp_err_t transacao(uint8_t endereco, const uint8_t *tx, size_t tamanho_tx,
                        uint8_t *rx, size_t tamanho_rx);

    gpio_num_t sda_;
    gpio_num_t scl_;
    uint32_t meio_periodo_us_;
    uint32_t timeout_stretch_us_;
};
