#include "i2c_soft.h"

#include "esp_rom_sys.h"

I2cSoft::I2cSoft(gpio_num_t sda, gpio_num_t scl, uint32_t meio_periodo_us,
                 uint32_t timeout_stretch_us)
    : sda_(sda), scl_(scl), meio_periodo_us_(meio_periodo_us),
      timeout_stretch_us_(timeout_stretch_us)
{
}

esp_err_t I2cSoft::iniciar()
{
    // Nível 1 no modo open-drain = pino solto (o pull-up leva a linha para alto).
    gpio_set_level(sda_, 1);
    gpio_set_level(scl_, 1);

    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << sda_) | (1ULL << scl_);
    config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    config.pull_up_en = GPIO_PULLUP_ENABLE;  // Reforço; o módulo deve ter pull-ups próprios.
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    const esp_err_t erro = gpio_config(&config);
    if (erro != ESP_OK) return erro;

    gpio_set_level(sda_, 1);
    gpio_set_level(scl_, 1);
    esp_rom_delay_us(100);
    return recuperar();
}

void I2cSoft::atraso() const { esp_rom_delay_us(meio_periodo_us_); }
void I2cSoft::soltar_sda() const { gpio_set_level(sda_, 1); }
void I2cSoft::puxar_sda() const { gpio_set_level(sda_, 0); }
void I2cSoft::puxar_scl() const { gpio_set_level(scl_, 0); }

esp_err_t I2cSoft::soltar_scl() const
{
    gpio_set_level(scl_, 1);
    // Aguarda o nível alto real (clock stretching do escravo ou pull-up lento).
    for (uint32_t t = 0; !gpio_get_level(scl_); ++t) {
        if (t >= timeout_stretch_us_) return ESP_ERR_TIMEOUT;
        esp_rom_delay_us(1);
    }
    return ESP_OK;
}

esp_err_t I2cSoft::recuperar()
{
    soltar_sda();
    for (int pulso = 0; pulso < 9 && !gpio_get_level(sda_); ++pulso) {
        puxar_scl();
        atraso();
        soltar_scl();  // Mesmo com timeout, segue tentando os pulsos restantes.
        atraso();
    }
    // STOP: SDA sobe com SCL alto.
    puxar_scl();
    atraso();
    puxar_sda();
    atraso();
    soltar_scl();
    atraso();
    soltar_sda();
    atraso();
    return (gpio_get_level(sda_) && gpio_get_level(scl_)) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t I2cSoft::start() const
{
    // Funciona tanto para START quanto para START repetido (SCL pode estar baixo).
    soltar_sda();
    atraso();
    esp_err_t erro = soltar_scl();
    if (erro != ESP_OK) return erro;
    atraso();
    if (!gpio_get_level(sda_)) return ESP_ERR_INVALID_STATE;
    puxar_sda();
    atraso();
    puxar_scl();
    atraso();
    return ESP_OK;
}

void I2cSoft::stop() const
{
    puxar_scl();
    puxar_sda();
    atraso();
    soltar_scl();
    atraso();
    soltar_sda();
    atraso();
}

esp_err_t I2cSoft::escrever_byte(uint8_t byte, bool *ack) const
{
    for (uint8_t mascara = 0x80; mascara; mascara >>= 1) {
        if (byte & mascara) soltar_sda();
        else puxar_sda();
        atraso();
        const esp_err_t erro = soltar_scl();
        if (erro != ESP_OK) return erro;
        atraso();
        puxar_scl();
    }
    soltar_sda();
    atraso();
    const esp_err_t erro = soltar_scl();
    if (erro != ESP_OK) return erro;
    atraso();
    *ack = gpio_get_level(sda_) == 0;
    puxar_scl();
    return ESP_OK;
}

esp_err_t I2cSoft::ler_byte(bool enviar_ack, uint8_t *byte) const
{
    uint8_t valor = 0;
    soltar_sda();
    for (int bit = 0; bit < 8; ++bit) {
        atraso();
        const esp_err_t erro = soltar_scl();
        if (erro != ESP_OK) return erro;
        atraso();
        valor = static_cast<uint8_t>((valor << 1) | (gpio_get_level(sda_) ? 1 : 0));
        puxar_scl();
    }
    if (enviar_ack) puxar_sda();
    else soltar_sda();
    atraso();
    const esp_err_t erro = soltar_scl();
    if (erro != ESP_OK) return erro;
    atraso();
    puxar_scl();
    soltar_sda();
    *byte = valor;
    return ESP_OK;
}

esp_err_t I2cSoft::transacao(uint8_t endereco, const uint8_t *tx, size_t tamanho_tx,
                             uint8_t *rx, size_t tamanho_rx)
{
    bool ack = false;
    esp_err_t erro = start();
    if (erro != ESP_OK) {
        stop();
        return erro;
    }

    if (tamanho_tx) {
        erro = escrever_byte(static_cast<uint8_t>(endereco << 1), &ack);
        if (erro == ESP_OK && !ack) erro = ESP_ERR_NOT_FOUND;
        for (size_t i = 0; erro == ESP_OK && i < tamanho_tx; ++i) {
            erro = escrever_byte(tx[i], &ack);
            if (erro == ESP_OK && !ack) erro = ESP_FAIL;
        }
        if (erro == ESP_OK && tamanho_rx) erro = start();  // START repetido
    }

    if (erro == ESP_OK && tamanho_rx) {
        erro = escrever_byte(static_cast<uint8_t>((endereco << 1) | 1), &ack);
        if (erro == ESP_OK && !ack) erro = ESP_ERR_NOT_FOUND;
        for (size_t i = 0; erro == ESP_OK && i < tamanho_rx; ++i)
            erro = ler_byte(i + 1 < tamanho_rx, &rx[i]);
    }

    stop();
    return erro;
}

esp_err_t I2cSoft::sondar(uint8_t endereco)
{
    bool ack = false;
    esp_err_t erro = start();
    if (erro == ESP_OK) erro = escrever_byte(static_cast<uint8_t>(endereco << 1), &ack);
    stop();
    if (erro != ESP_OK) return erro;
    return ack ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void I2cSoft::testar_linhas(int *sda_pullup, int *scl_pullup, int *sda_pulldown, int *scl_pulldown)
{
    soltar_sda();
    gpio_set_level(scl_, 1);
    esp_rom_delay_us(50);
    *sda_pullup = gpio_get_level(sda_);
    *scl_pullup = gpio_get_level(scl_);

    gpio_set_pull_mode(sda_, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(scl_, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(200);
    *sda_pulldown = gpio_get_level(sda_);
    *scl_pulldown = gpio_get_level(scl_);

    gpio_set_pull_mode(sda_, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(scl_, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(200);
}

esp_err_t I2cSoft::escrever(uint8_t endereco, const uint8_t *dados, size_t tamanho)
{
    if (!dados || !tamanho) return ESP_ERR_INVALID_ARG;
    return transacao(endereco, dados, tamanho, nullptr, 0);
}

esp_err_t I2cSoft::escrever_ler(uint8_t endereco, const uint8_t *tx, size_t tamanho_tx,
                                uint8_t *rx, size_t tamanho_rx)
{
    if (!tx || !tamanho_tx || !rx || !tamanho_rx) return ESP_ERR_INVALID_ARG;
    return transacao(endereco, tx, tamanho_tx, rx, tamanho_rx);
}
