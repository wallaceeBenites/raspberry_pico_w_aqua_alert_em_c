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

// Definição dos pinos ==========================================================================================================================================================
#define LED_PIN 12        // Pino do LED (GP12)
#define BUZZER_A_PIN 21   // Pino do Buzzer A (GP21)
#define BUZZER_B_PIN 10   // Pino do Buzzer B (GP10)
#define LED_PIN_MATRIZ 7  // Pino da Matriz de LED RGB (GP7)
#define LED_COUNT 25 
#define BUTTON_ALARM_ON 5  // Botão para ligar alarmes (GP5)
#define BUTTON_ALARM_OFF 6 // Botão para desligar alarmes (GP6)
#define VSYS_PIN 29        // Pino para leitura da tensão VSYS (GP29)
const uint I2C_SDA = 14; // DisplayOLED SDA
const uint I2C_SCL = 15; // DisplayOLED SCL 

// Definições para leitura da tensão VSYS
#define USB_CONNECTED_VOLTAGE 4.0f  // Consideramos USB conectado acima de 4.0V
#define NUM_AMOSTRAS 10             // Número de leituras para média móvel


bool alarme_ativo = false; // Estado do alarme
bool com_internet = true; // Estado do WI-FI

// Configurações do Wi-Fi ======================================================================================================================================================
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

// Função de callback para processar requisições HTTP
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Cliente fechou a conexão
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Processa a requisição HTTP
    char *request = (char *)p->payload;

    if (strstr(request, "GET /alarme/on")) {
        alarme_ativo = true;  // Liga o alarme
    } else if (strstr(request, "GET /alarme/off")) {
        alarme_ativo = false; // Desliga o alarme
    }

    // Prepara a resposta HTTP
    char response[512];
    snprintf(response, sizeof(response), HTTP_RESPONSE, alarme_ativo ? "Ligado" : "Desligado");

    // Envia a resposta HTTP
    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);

    // Libera o buffer recebido
    pbuf_free(p);

    return ERR_OK;
}

// Callback de conexão: associa o http_callback à conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);  // Associa o callback HTTP
    return ERR_OK;
}

// Função de setup do servidor TCP 
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }

    // Liga o servidor na porta 80
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);  // Coloca o PCB em modo de escuta
    tcp_accept(pcb, connection_callback);  // Associa o callback de conexão

    printf("Servidor HTTP rodando na porta 80...\n");
}
// Codigo referente a matriz de led =================================================================================================================================================

// Definição de pixel GRB
struct pixel_t {
    uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {
    // Cria programa PIO.
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    // Toma posse de uma máquina PIO.
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
    }

    // Inicia programa na máquina PIO obtida.
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    // Limpa buffer de pixels.
    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
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
    // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

int getIndex(int x, int y) {
    // Se a linha for par (0, 2, 4), percorremos da esquerda para a direita.
    // Se a linha for ímpar (1, 3), percorremos da direita para a esquerda.
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linha par (esquerda para direita).
    } else {
        return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
    }
}

// Variáveis para controle do PWM ========================================================================================================================================
uint slice_num_a;  // Slice do PWM para o Buzzer A
uint channel_a;    // Canal do PWM para o Buzzer A
uint slice_num_b;  // Slice do PWM para o Buzzer B
uint channel_b;    // Canal do PWM para o Buzzer B

// Função para configurar o PWM nos buzzers
void configurar_pwm() {
    // Configura o pino do Buzzer A como saída PWM
    gpio_set_function(BUZZER_A_PIN, GPIO_FUNC_PWM);
    slice_num_a = pwm_gpio_to_slice_num(BUZZER_A_PIN);
    channel_a = pwm_gpio_to_channel(BUZZER_A_PIN);

    // Configura o pino do Buzzer B como saída PWM
    gpio_set_function(BUZZER_B_PIN, GPIO_FUNC_PWM);
    slice_num_b = pwm_gpio_to_slice_num(BUZZER_B_PIN);
    channel_b = pwm_gpio_to_channel(BUZZER_B_PIN);

    // Configura o PWM para ambos os buzzers
    pwm_set_wrap(slice_num_a, 62500);  // Define a frequência para 2 kHz (125000000 / 62500)
    pwm_set_chan_level(slice_num_a, channel_a, 31250);  // Duty cycle de 50% (31250 / 62500)
    pwm_set_wrap(slice_num_b, 62500);  // Define a frequência para 2 kHz (125000000 / 62500)
    pwm_set_chan_level(slice_num_b, channel_b, 31250);  // Duty cycle de 50% (31250 / 62500)
}

// Função para emitir o padrão de bipes ===================================================================================================================================
void emitir_bipes() {
    // Matriz de leds 5x5
    int matriz[5][5][3] = {
        {{0, 0, 0}, {0, 0, 0}, {255, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {255, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{255, 0, 0},{255, 0, 0}, {255, 0, 0}, {255, 0, 0}, {255, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {255, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {255, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    };
    // Desenhando Sprite contido na matriz.c
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int posicao = getIndex(linha, coluna);
            npSetLED(posicao, matriz[coluna][linha][0], matriz[coluna][linha][1], matriz[coluna][linha][2]);
        }
    }

    // Aciona os leds
    npWrite();  // Liga o LED MATRIZ       
    gpio_put(LED_PIN, 1);  // Liga o LED SOLO 

    // Aciona os buzzers
    pwm_set_enabled(slice_num_a, true);
    pwm_set_enabled(slice_num_b, true);
    sleep_ms(500);  // Duração do beep

    // Desliga os leds
    npClear(); // Desliga o LED
    npWrite();
    gpio_put(LED_PIN, 0);  // Desliga o LED SOLO
    // Desliga os buzzers
    pwm_set_enabled(slice_num_a, false);
    pwm_set_enabled(slice_num_b, false);
    sleep_ms(200);  // Intervalo entre bipes
}

void bipes_de_inicializacao() {
    gpio_put(LED_PIN, 1);  // Liga o LED

    // Aciona os buzzers
    pwm_set_enabled(slice_num_a, true);
    pwm_set_enabled(slice_num_b, true);
    sleep_ms(2000);  // Duração do beep

    gpio_put(LED_PIN, 0);  // Desliga o LED
    // Desliga os buzzers
    pwm_set_enabled(slice_num_a, false);
    pwm_set_enabled(slice_num_b, false);
    sleep_ms(200);  // Intervalo entre bipes
}

// Função para ativar o alarme com bipes contínuos ============================================================================================================
void iniciar_alerta() {
    alarme_ativo = true;
    printf("Queda de energia detectada!\n");

    // Loop infinito para manter o alarme ativado
    while (alarme_ativo) {
        emitir_bipes();  // Emitir o padrão de bipes

        // Verificar se o botão B foi pressionado para desligar o alarme
        if (gpio_get(BUTTON_ALARM_OFF) == 0) {
            
            parar_alerta();  // Chama a função para desligar o alarme
            break;  // Sai do loop
        }
    }
}

// Função para desativar o alarme =================================================================================================================================
void parar_alerta() {

    // Desativa os Leds MATRIZ
    npClear();
    npWrite();    
    gpio_put(LED_PIN, 0);  // Desliga o LED SOLO 
    // Desabilita o PWM nos buzzers
    pwm_set_enabled(slice_num_a, false);
    pwm_set_enabled(slice_num_b, false);

    // Atualiza o estado do alarme
    alarme_ativo = false;

    printf("Alarmes desativados!\n");
}

// Função para ler a tensão VSYS =============================================================================================================================================
float ler_tensao_vsys() {
    adc_select_input(2);  // Usa o canal 2
    uint16_t valor_adc = adc_read();
    float tensao = valor_adc * 3.3f / (1 << 12);
    return tensao * 3.0f;
}
// ================================== INICIO DA FO MAIN ============================================================================================================================================
int main() {
    stdio_init_all();
    adc_init();
    
    adc_gpio_init(VSYS_PIN);

// Inicializa matriz de LEDs NeoPixel.
npInit(LED_PIN_MATRIZ);
npClear();
npWrite(); // Escreve os dados nos LEDs.

    

// Inicialização dos pinos =====================================================================================================================================================
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

// Inicialização do i2c==========================================================================================================================================================
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

    // Configura o PWM nos buzzers
    configurar_pwm();

    // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    // zera o display inteiro
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

restart:

// Parte do código para exibir a mensagem INCIANDO EQUIPAMENTO no display 
// /**
    char *text[] = {
        "   INICIANDO   ",
        "  EQUIPAMENTO   ",
        "    AGUARDE   "};

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);

// Inicializa o Wi-Fi ===============================================================================================================================================================
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        com_internet = false;
    }
    if (com_internet){
        
    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        com_internet = false; 
    } else {
        printf("Conectado ao Wi-Fi!\n");
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
        com_internet = true;
    }
     
    }
    if (com_internet) {
          // Inicia o servidor HTTP
        start_http_server();
    }



    float soma_tensao = 0;
    float leituras[NUM_AMOSTRAS] = {0};
    int indice = 0;
    bool usb_conectado = true;
    int leituras_desconectado = 0;
    

    bipes_de_inicializacao();

    while (1) {

// Verifica a tensão VSYS para detectar desconexão do USB ==========================================================================================================================
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

// Parte do código para exibir a mensagem STATUS no display  =========================================================================================================================

    char linha1[16];  
    char linha2[16];  
    char linha3[16];
    char linha4[16]; 

    sprintf(linha1, "  Tensao VSYS ");  
    sprintf(linha2, "     %.2fV", media_tensao); 

    if (com_internet)
    {

        sprintf(linha3, "   STATUS OK");
        com_internet = true;

        

    }else{

        sprintf(linha3, "   WiFi ERRO ");

    }
    
    
    
    
 

// Limpa a tela antes de exibir o novo texto 
    memset(ssd, 0, sizeof(ssd));  

// Exibe cada linha separadamente 
    ssd1306_draw_string(ssd, 5, 0, linha1); // Primeira linha  
    ssd1306_draw_string(ssd, 5, 10, linha2); // Segunda linha 
    ssd1306_draw_string(ssd, 5, 20, linha3); // Segunda linha 
    ssd1306_draw_string(ssd, 5, 30, linha4); // Segunda linha


    render_on_display(ssd, &frame_area);  



// Verifica se o botão A foi pressionado ou a energia caiu para ativar o alarme e atualiza o Status ========================================================================================================================
        if (gpio_get(BUTTON_ALARM_ON) == 0 || alarme_ativo) {

            char linha1[16];  
            char linha2[16];  
            char linha3[16];
            
            sprintf(linha1, "  Tensao VSYS ");  
            sprintf(linha2, "     %.2fV", media_tensao); 
            sprintf(linha3, "    ATENCAO");

        // Limpa a tela antes de exibir o novo texto 
            memset(ssd, 0, sizeof(ssd));  
        
        // Exibe cada linha separadamente 
            ssd1306_draw_string(ssd, 5, 0, linha1); // Primeira linha  
            ssd1306_draw_string(ssd, 5, 10, linha2); // Segunda linha 
            ssd1306_draw_string(ssd, 5, 20, linha3); // Segunda linha 
            ssd1306_draw_string(ssd, 5, 30, linha4); // Segunda linha
        
        
            render_on_display(ssd, &frame_area);

            iniciar_alerta();
            sleep_ms(300);  // Debounce básico
            printf("Média Tensão VSYS: %.2f V\n", media_tensao);
        }

        if (leituras_desconectado >= NUM_AMOSTRAS) {
            if (usb_conectado) {

                char linha1[16];  
                char linha2[16];  
                char linha3[16];
                
                sprintf(linha1, "  Tensao VSYS ");  
                sprintf(linha2, "     %.2fV", media_tensao); 
                sprintf(linha3, "    ATENCAO");

            // Limpa a tela antes de exibir o novo texto 
                memset(ssd, 0, sizeof(ssd));  
            
            // Exibe cada linha separadamente 
                ssd1306_draw_string(ssd, 5, 0, linha1); // Primeira linha  
                ssd1306_draw_string(ssd, 5, 10, linha2); // Segunda linha 
                ssd1306_draw_string(ssd, 5, 20, linha3); // Segunda linha 
                ssd1306_draw_string(ssd, 5, 30, linha4); // Segunda linha
            
            
                render_on_display(ssd, &frame_area);  

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