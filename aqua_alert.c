#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2818b.pio.h"

// ================================================================================================================
// Definições de pinos e constantes
// ================================================================================================================

#define LED_PIN 12        // Pino do LED (GP12)
#define BUZZER_A_PIN 21   // Pino do Buzzer A (GP21)
#define BUZZER_B_PIN 10   // Pino do Buzzer B (GP10)
#define LED_PIN_MATRIZ 7  // Pino da Matriz de LED RGB (GP7)
#define LED_COUNT 25      // Número de LEDs na matriz
#define BUTTON_ALARM_ON 5  // Botão para ligar alarmes (GP5)
#define BUTTON_ALARM_OFF 6 // Botão para desligar alarmes (GP6)
#define VSYS_PIN 29        // Pino para leitura da tensão VSYS (GP29)
const uint I2C_SDA = 14;   // Pino SDA do display OLED (GP14)
const uint I2C_SCL = 15;   // Pino SCL do display OLED (GP15)

// Definições para leitura da tensão VSYS
#define USB_CONNECTED_VOLTAGE 4.0f  // Tensão mínima para considerar USB conectado
#define NUM_AMOSTRAS 10             // Número de leituras para média móvel

// Definições para as faixas de valores do joystick
#define JOYSTICK_MIN 1000  // Valor mínimo para considerar movimento
#define JOYSTICK_MAX 3000  // Valor máximo para considerar movimento

// Definições do Joystick
const int vRx = 26;          // Pino de leitura do eixo X do joystick (conectado ao ADC)
const int vRy = 27;          // Pino de leitura do eixo Y do joystick (conectado ao ADC)
const int ADC_CHANNEL_0 = 0; // Canal ADC para o eixo X do joystick
const int ADC_CHANNEL_1 = 1; // Canal ADC para o eixo Y do joystick
const int SW = 22;           // Pino de leitura do botão do joystick

// Variáveis globais
bool alarme_ativo = false;   // Estado do alarme
bool com_internet = true;    // Estado do Wi-Fi
bool matriz_ligada = false;  // Estado da matriz de LEDs
int tela_atual = 0;          // Tela atual (0 = Status e Tensão, 1 = IP, 2 = Temperatura, 3 = pH)
char ip_address_str[16];     // String para armazenar o endereço IP

// Configurações do Wi-Fi
#define WIFI_SSID "As 3 Marias"  // Nome da rede Wi-Fi
#define WIFI_PASS "DRC290479"    // Senha da rede Wi-Fi

// Buffer para respostas HTTP
#define HTTP_RESPONSE "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n" \
"<!DOCTYPE html>" \
"<html>" \
"<head><title>Status do Alarme</title>" \
"<meta charset='UTF-8'>" \
"<style> body { background-color: black; color: white; } </style></head>" \
"<body>" \
"<center>" \
"<h1 style='font-size: 30px;'> AMBIENTE DE TESTE - NÃO LIGUE PARA APARÊNCIA </h1>" \
"<hr>" \
"<h1>Status do Alarme </h1>" \
"<h2>Alarme: <b>%s</b></h2>" \
"<hr>" \
"<h3>Controles</h3>" \
"<p><a href='/alarme/on'> <b>Ligar Alarme</b></a></p>" \
"<p><a href='/alarme/off'> <b>Desligar Alarme</b></a></p>" \
"</center>" \
"</body>" \
"</html>\r\n"

// ================================================================================================================
// Funções de inicialização e configuração
// ================================================================================================================

/**
 * Inicializa o joystick, configurando os pinos ADC e o botão.
 */
void setup_joystick() {
    adc_gpio_init(vRx); // Configura o pino VRX (eixo X) para entrada ADC
    adc_gpio_init(vRy); // Configura o pino VRY (eixo Y) para entrada ADC
    gpio_init(SW);      // Inicializa o pino do botão
    gpio_set_dir(SW, GPIO_IN); // Configura o pino do botão como entrada
    gpio_pull_up(SW);  // Ativa o pull-up no pino do botão
}

/**
 * Lê os valores dos eixos X e Y do joystick.
 */
void joystick_read_axis(uint16_t *eixo_x, uint16_t *eixo_y) {
    adc_select_input(ADC_CHANNEL_0); // Seleciona o canal ADC para o eixo X
    sleep_us(2);                     // Pequeno delay para estabilidade
    *eixo_x = adc_read();            // Lê o valor do eixo X (0-4095)
    adc_select_input(ADC_CHANNEL_1); // Seleciona o canal ADC para o eixo Y
    sleep_us(2);                     // Pequeno delay para estabilidade
    *eixo_y = adc_read();            // Lê o valor do eixo Y (0-4095)
}

/**
 * Configura o PWM para os buzzers.
 */
void configurar_pwm() {
    gpio_set_function(BUZZER_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER_B_PIN, GPIO_FUNC_PWM);
    uint slice_num_a = pwm_gpio_to_slice_num(BUZZER_A_PIN);
    uint slice_num_b = pwm_gpio_to_slice_num(BUZZER_B_PIN);
    pwm_set_wrap(slice_num_a, 62500);  // Frequência de 2 kHz
    pwm_set_wrap(slice_num_b, 62500);
    pwm_set_chan_level(slice_num_a, pwm_gpio_to_channel(BUZZER_A_PIN), 31250); // Duty cycle de 50%
    pwm_set_chan_level(slice_num_b, pwm_gpio_to_channel(BUZZER_B_PIN), 31250);
}

// ================================================================================================================
// Funções da matriz de LEDs
// ================================================================================================================

// Definição de pixel GRB
struct pixel_t {
    uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

npLED_t leds[LED_COUNT]; // Buffer de pixels que formam a matriz.
PIO np_pio;              // Máquina PIO usada para controlar a matriz de LEDs.
uint sm;                 // State machine da PIO.

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear(); // Limpa a matriz de LEDs
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

/**
 * Calcula o índice de um LED na matriz com base nas coordenadas (x, y).
 */
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linha par (esquerda para direita).
    } else {
        return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
    }
}

/**
 * Exibe o desenho do peixe na matriz de LEDs.
 */
void exibir_matriz_led() {
    int peixe[5][5][3] = {
        // Linha 1
        {{16, 16, 245}, {0, 0, 0}, {16, 16, 245}, {0, 0, 0}, {16, 16, 245}},
        // Linha 2
        {{0, 0, 0}, {16, 16, 245}, {16, 16, 245}, {16, 16, 245}, {16, 16, 245}},
        // Linha 3
        {{16, 16, 245}, {16, 16, 245}, {255, 255, 255}, {16, 16, 245}, {16, 16, 245}},
        // Linha 4
        {{0, 0, 0}, {16, 16, 245}, {16, 16, 245}, {16, 16, 245}, {128, 0, 128}},
        // Linha 5
        {{16, 16, 245}, {16, 16, 245}, {16, 16, 245}, {16, 16, 245}, {0, 0, 0}}
    };

    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int posicao = getIndex(linha, coluna);
            npSetLED(posicao, peixe[linha][coluna][0], peixe[linha][coluna][1], peixe[linha][coluna][2]);
        }
    }
    npWrite();
}

// ================================================================================================================
// Funções do alarme
// ================================================================================================================

/**
 * Emite bipes sonoros e aciona a matriz de LEDs.
 */
void emitir_bipes() {
    exibir_matriz_led();
    npWrite();
    gpio_put(LED_PIN, 1);  // Liga o LED SOLO
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_A_PIN), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_B_PIN), true);
    sleep_ms(500);  // Duração do beep
    npClear();
    npWrite();
    gpio_put(LED_PIN, 0);  // Desliga o LED SOLO
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_A_PIN), false);
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_B_PIN), false);
    sleep_ms(200);  // Intervalo entre bipes
}

/**
 * Ativa o alarme com bipes contínuos.
 */
void iniciar_alerta() {
    alarme_ativo = true;
    printf("Queda de energia detectada!\n");
    while (alarme_ativo) {
        emitir_bipes();
        if (gpio_get(BUTTON_ALARM_OFF) == 0) {
            parar_alerta();
            break;
        }
    }
}

/**
 * Desativa o alarme.
 */
void parar_alerta() {
    npClear();
    npWrite();
    gpio_put(LED_PIN, 0);
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_A_PIN), false);
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_B_PIN), false);
    alarme_ativo = false;
    printf("Alarmes desativados!\n");
}

// ================================================================================================================
// Funções do servidor HTTP
// ================================================================================================================

/**
 * Callback para processar requisições HTTP.
 */
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        tcp_close(tpcb);
        return ERR_OK;
    }
    char *request = (char *)p->payload;
    if (strstr(request, "GET /alarme/on")) {
        alarme_ativo = true;
    } else if (strstr(request, "GET /alarme/off")) {
        alarme_ativo = false;
    }
    char response[512];
    snprintf(response, sizeof(response), HTTP_RESPONSE, alarme_ativo ? "Ligado" : "Desligado");
    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
    pbuf_free(p);
    return ERR_OK;
}

/**
 * Callback de conexão: associa o http_callback à conexão.
 */
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);
    return ERR_OK;
}

/**
 * Inicia o servidor HTTP na porta 80.
 */
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

// ================================================================================================================
// Função principal
// ================================================================================================================

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(VSYS_PIN);
    setup_joystick();
    npInit(LED_PIN_MATRIZ);

    // Inicialização dos pinos
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(BUZZER_A_PIN);
    gpio_set_dir(BUZZER_A_PIN, GPIO_OUT);
    gpio_init(BUZZER_B_PIN);
    gpio_set_dir(BUZZER_B_PIN, GPIO_OUT);
    gpio_init(BUTTON_ALARM_ON);
    gpio_set_dir(BUTTON_ALARM_ON, GPIO_IN);
    gpio_pull_up(BUTTON_ALARM_ON);
    gpio_init(BUTTON_ALARM_OFF);
    gpio_set_dir(BUTTON_ALARM_OFF, GPIO_IN);
    gpio_pull_up(BUTTON_ALARM_OFF);

    // Inicialização do I2C para o display OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init();

    // Configura o PWM nos buzzers
    configurar_pwm();

    // Área de renderização do display OLED
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    // Exibe mensagem de inicialização no display
    char *text[] = {"   INICIANDO   ", "  EQUIPAMENTO   ", "    AGUARDE   "};
    int y = 0;
    for (uint i = 0; i < count_of(text); i++) {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);

    // Inicializa o Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        com_internet = false;
    }
    if (com_internet) {
        cyw43_arch_enable_sta_mode();
        printf("Conectando ao Wi-Fi...\n");
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
            printf("Falha ao conectar ao Wi-Fi\n");
            com_internet = false;
        } else {
            printf("Conectado ao Wi-Fi!\n");
            uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
            snprintf(ip_address_str, sizeof(ip_address_str), "%d.%d.%d.%d", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
            printf("Endereço IP %s\n", ip_address_str);
            com_internet = true;
            start_http_server();
        }
    }

    // Loop principal
    float soma_tensao = 0;
    float leituras[NUM_AMOSTRAS] = {0};
    int indice = 0;
    bool usb_conectado = true;
    int leituras_desconectado = 0;

    while (1) {
        // Leitura do joystick
        uint16_t valor_x, valor_y;
        joystick_read_axis(&valor_x, &valor_y);
        printf("Joystick - X: %d, Y: %d, Botao: %d\n", valor_x, valor_y, gpio_get(SW));

        // Verifica o movimento do joystick para mudar a tela
        if (valor_y < JOYSTICK_MIN) {
            tela_atual = 0;   // Tela padrão (Status e Tensão)
        } else if (valor_y > JOYSTICK_MAX) {
            tela_atual = 1;   // Tela "IP Address"
        } else if (valor_x < JOYSTICK_MIN) {
            tela_atual = 2;   // Tela de Temperatura
        } else if (valor_x > JOYSTICK_MAX) {
            tela_atual = 3;   // Tela de PH
        }

        // Verifica se o botão do joystick foi pressionado
        if (gpio_get(SW) == 0) {
            sleep_ms(50); // Debounce
            if (gpio_get(SW) == 0) {
                matriz_ligada = !matriz_ligada;
                if (matriz_ligada) {
                    exibir_matriz_led();
                    npWrite();
                } else {
                    npClear();
                    npWrite();
                }
                while (gpio_get(SW) == 0) {
                    sleep_ms(10);
                }
            }
        }

        // Verifica a tensão VSYS para detectar desconexão do USB
        float tensao_vsys = ler_tensao_vsys();
        soma_tensao -= leituras[indice];
        leituras[indice] = tensao_vsys;
        soma_tensao += tensao_vsys;
        indice = (indice + 1) % NUM_AMOSTRAS;
        float media_tensao = soma_tensao / NUM_AMOSTRAS;

        if (media_tensao < USB_CONNECTED_VOLTAGE) {
            leituras_desconectado++;
        } else {
            leituras_desconectado = 0;
        }

        // Exibe a tela atual no display OLED
        char linha1[16], linha2[16], linha3[16], linha4[16];
        if (com_internet) {
            if (tela_atual == 0) {
                sprintf(linha1, "  Tensao VSYS ");
                sprintf(linha2, "     %.2f V", media_tensao);
                sprintf(linha3, "   STATUS OK");
            } else if (tela_atual == 1) {
                sprintf(linha1, "   IP Address");
                sprintf(linha2, " ");
                sprintf(linha3, "%s", ip_address_str);
            } else if (tela_atual == 2) {
                sprintf(linha1, "  TEMPERATURA");
                sprintf(linha2, "    26 graus");
                sprintf(linha3, "    celsius");
            } else if (tela_atual == 3) {
                sprintf(linha1, "  nivel de pH");
                sprintf(linha2, "      7.0 ");
                sprintf(linha3, "    alcalino");
            }
        } else {
            if (tela_atual == 0) {
                sprintf(linha1, "  Tensao VSYS ");
                sprintf(linha2, "     %.2f V", media_tensao);
                sprintf(linha3, "   WiFi ERRO ");
            } else if (tela_atual == 1) {
                sprintf(linha1, "   IP Address");
                sprintf(linha2, " ");
                sprintf(linha3, " No Connection");
            } else if (tela_atual == 2) {
                sprintf(linha1, "  TEMPERATURA");
                sprintf(linha2, "    26 graus");
                sprintf(linha3, "    celsius");
            } else if (tela_atual == 3) {
                sprintf(linha1, "  nivel de pH");
                sprintf(linha2, "      7.0 ");
                sprintf(linha3, "    alcalino");
            }
        }

        // Limpa a tela antes de exibir o novo texto
        memset(ssd, 0, sizeof(ssd));
        ssd1306_draw_string(ssd, 5, 0, linha1);
        ssd1306_draw_string(ssd, 5, 10, linha2);
        ssd1306_draw_string(ssd, 5, 20, linha3);
        ssd1306_draw_string(ssd, 5, 30, linha4);
        render_on_display(ssd, &frame_area);

        // Verifica se o botão A foi pressionado ou a energia caiu para ativar o alarme
        if (gpio_get(BUTTON_ALARM_ON) == 0 || alarme_ativo) {
            iniciar_alerta();
            sleep_ms(300);  // Debounce básico
            printf("Média Tensão VSYS: %.2f V\n", media_tensao);
        }

        // Verifica se o USB foi desconectado
        if (leituras_desconectado >= NUM_AMOSTRAS) {
            if (usb_conectado) {
                printf("USB desconectado! Emitindo bip de alerta...\n");
                iniciar_alerta();
            }
            usb_conectado = false;
        } else {
            if (!usb_conectado) {
                parar_alerta();
                printf("USB reconectado!\n");
            }
            usb_conectado = true;
        }

        cyw43_arch_poll();  // Necessário para manter o Wi-Fi ativo
        sleep_ms(100);
    }

    cyw43_arch_deinit();  // Desliga o Wi-Fi (não será chamado, pois o loop é infinito)
    return 0;
}