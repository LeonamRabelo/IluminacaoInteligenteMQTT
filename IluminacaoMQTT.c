#include "pico/stdlib.h"            //Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        //Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         //Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "inc/ssd1306.h"
#include "inc/matriz_leds.h"
#include "lwip/apps/mqtt.h"         //Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    //Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               //Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         //Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

//Definições de GPIO
#define I2C_SDA 14
#define I2C_SCL 15
#define JOYSTICK_Y 27
#define WS2812_PIN 7
#define BUZZER_PIN 21
#define LED_RED 13
#define IS_RGBW false

#define NUM_AREAS 10    //Áreas de 0 a 9

typedef struct{         //Status das Áreas
    int luminosidade;   //0 a 100%
}AreaStatus;
AreaStatus areas[NUM_AREAS];

//Variáveis Globais
uint8_t area_atual = 0;
bool alarme_disparado = false;
bool economia = false;
uint32_t tempo_ultima_atividade = 0;

ssd1306_t ssd;          //Estrutura para o SSD1306
uint buzzer_slice;      //Slice para o buzzer

// Configurações de rede
#define WIFI_SSID "LOGIN_WIFI"                  //Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "SENHA_WIFI"        //Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "192.168.1.7"       //Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "LOGIN_MQTT"             //Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "SENHA_MQTT"             //Substitua pelo Password da host MQTT - credencial de acesso - caso exista

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

//This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

//Definições de Tamanho
#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct{
    mqtt_client_t* mqtt_client_inst;                        //Instância do cliente MQTT
    struct mqtt_connect_client_info_t mqtt_client_info;     //Informações do cliente MQTT
    char data[MQTT_OUTPUT_RINGBUF_SIZE];                    //Dados de entrada
    char topic[MQTT_TOPIC_LEN];                             //Tópico
    uint32_t len;                                           //Tamanho
    ip_addr_t mqtt_server_address;                          //Endereço do servidor MQTT
    bool connect_done;                                      //Conexão concluida
    int subscribe_count;                                    //Contagem de assinaturas
    bool stop_client;                                       //Parar o cliente
}MQTT_CLIENT_DATA_T;

//Definições de Debug
#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

//Definições de Informação
#ifndef INFO_printf
#define INFO_printf printf
#endif

//Definições de Erro
#ifndef ERROR_printf
#define ERROR_printf printf
#endif

//Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

//QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

//Tópico usado para: last will and testament - Verificar se o dispositivo está ativo
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

//Definir o nome do dispositivo
#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

//Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/*                  PROTóTIPOS DE FUNÇÕES                */
//Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);
//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
//Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);
//Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);
//Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);
//Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
//Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
//Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
//Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);
//Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);
//Requisição para publicar
static void publicar_modoeconomia(MQTT_CLIENT_DATA_T* state);

//Função para atualizar o display
static void atualizar_display(){
    char buffer[32];             //Buffer para armazenar as informações
    ssd1306_fill(&ssd, false);  //Limpa o display
    //Bordas
    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
    ssd1306_rect(&ssd, 1, 1, 128 - 2, 64 - 2, true, false);
    ssd1306_rect(&ssd, 2, 2, 128 - 4, 64 - 4, true, false);
    ssd1306_rect(&ssd, 3, 3, 128 - 6, 64 - 6, true, false);
    //Informações como area, luminosidade e modo de economia no display
    sprintf(buffer, "Area %d", area_atual);
    ssd1306_draw_string(&ssd, buffer, 30, 10);
    sprintf(buffer, "Luz: %d%%", areas[area_atual].luminosidade);
    ssd1306_draw_string(&ssd, buffer, 10, 30);
    ssd1306_draw_string(&ssd, economia ? "Modo: Eco" : "Modo: Normal", 10, 50);
    ssd1306_send_data(&ssd);    //Envia os dados para o display
}

//Função para atualizar os LEDs da Matriz WS2812
static void atualizar_leds(){
    if(economia){   //Verificar se o modo de economia está ativado
        set_one_led(0, 0, 0, area_atual);   //Apaga os LEDs, representando luminosidade zero
    }else{  //Modo normal
        uint8_t intensidade = (areas[area_atual].luminosidade * 255) / 100; //Calcular a intensidade baseada na luminosidade da area em porcentagem
        set_one_led(intensidade, intensidade, intensidade, area_atual);     //Liga os LEDs com a intensidade calculada
    }
}

//Função para verificar se há presença na area e mudar o modo de economia conforme necessário
static void verificar_presenca(int eixo_x){
    int distancia = abs(eixo_x - 2048); //Calcular a distancia entre o centro da area e o eixo x
    if(distancia < 500){                //Se a distancia for menor que 500, estiver em um intervalo muito proximo do centro do joystick
        if(to_ms_since_boot(get_absolute_time()) - tempo_ultima_atividade > 2000){  //Verificar se o tempo desde a ultima atividade foi maior que 2 segundos
            if(!economia){              //Verificar se o modo de economia nao esta ativado
                economia = true;        //Ativar o modo de economia
                gpio_put(LED_RED, 1);   //Ativar LED vermelho para o modo de economia
                atualizar_leds();       //Apaga LED ao entrar em economia
            }
        }
    }else{                      //Se nao estiver em um intervalo muito proximo do centro do joystick
        if(economia){           //Verificar se o modo de economia esta ativado
            economia = false;   //Desativar o modo de economia
            gpio_put(LED_RED, 0);   //Desativar LED vermelho
            atualizar_leds();       //Chamar a funcao para atualizar os leds da matriz
        }
        tempo_ultima_atividade = to_ms_since_boot(get_absolute_time()); //Atualizar o tempo da ultima atividade
    }
    atualizar_display();    //Chamar a funcao para atualizar o display com as informacoes atualizadas
}

/* Função para publicar luminosidade de cada area referente, para ter controle de cada area. 
Inicialmente os dados estarão em /pico/luminosidade,logo depois serao publicados 
em /pico/luminosidade/area1, /pico/luminosidade/area2 e assim por diante */
static void publicar_luminosidade(MQTT_CLIENT_DATA_T* state){
    char payload[8];    //Buffer para armazenar o payload
    snprintf(payload, sizeof(payload), "%d", areas[area_atual].luminosidade);    //Converte a luminosidade para string para o payload
    char buffer[32];    //Buffer para armazenar o topico
    sprintf(buffer, "/pico/luminosidade/area%d", area_atual);    //Converte o area atual para string para o topico
    //Publica o payload (luminosidade) no topico correspondente
    mqtt_publish(state->mqtt_client_inst, full_topic(state, buffer), payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

//Função para publicar o modo de economia
static void publicar_modoeconomia(MQTT_CLIENT_DATA_T* state){
    char payload[8];    //Buffer para armazenar o payload
     //Condição ternaria para o payload
    (economia) ? snprintf(payload, sizeof(payload), "%s", "on") : snprintf(payload, sizeof(payload), "%s", "off");
    //Publica o payload (modo de economia) no topico /pico/modoeconomia
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/pico/modoeconomia"), payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

//Inicializa periféricos usados
static void init_perifericos(){
    //Inicializa LED vermelho
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, 0);

    //Inicializa ADC
    adc_init();
    adc_gpio_init(JOYSTICK_Y);

    // Inicializa buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(buzzer_slice, 125.0f);
    pwm_set_wrap(buzzer_slice, 999);
    pwm_set_gpio_level(BUZZER_PIN, 300);
    pwm_set_enabled(buzzer_slice, false);

    //Inicializa I2C para o display SSD1306
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  //Dados
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);  //Clock
    //Define como resistor de pull-up interno
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa display
    ssd1306_init(&ssd, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa WS2812
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}

//Processa dados recebidos MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags){
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;   //Define state como um ponteiro para MQTT_CLIENT_DATA_T

//Pre-processamento definindo se os topicos serao unicos ou nao (/luz ou /pico/luz, por exemplo)
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    //Armazena os dados em state
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';    //Termina a string

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);    //Imprime o tópico e o payload

    //Recebe aqui em /pico/luminosidade para depois publicar em cada topico separado por area por meio da funcao publicar_luminosidade()
    if(strcmp(basic_topic, "/pico/luminosidade") == 0){ //Se o tópico for /pico/luminosidade
        int valor = atoi(state->data);                  //Converte o payload para inteiro
        if (valor >= 0 && valor <= 100){                //Se o valor estiver entre 0 e 100
            areas[area_atual].luminosidade = valor;     //Armazena o valor na area atual
            atualizar_leds();                           //Chama a funcao para atualizar os leds
            publicar_luminosidade(state);               //Chama a funcao para publicar o valor em sua area atual separadamente por tópicos
        }
    }else if(strcmp(basic_topic, "/pico/alarme") == 0){ //Se o tópico for /pico/alarme
        if(strcmp(state->data, "on") == 0 || strcmp(state->data, "On") == 0){    //Se o payload for "on"
            alarme_disparado = true;                    //Define alarme_disparado como true
        }else{  //Se o payload nao for "on"
            alarme_disparado = false;                   //Define alarme_disparado como false
            pwm_set_enabled(buzzer_slice, false);       //Desativa o buzzer
        }
    }else if(strcmp(basic_topic, "/pico/areaprox") == 0){  //Se o tópico for /pico/areaprox
        area_atual = (area_atual + 1) % NUM_AREAS;         //Incrementa a area atual
        atualizar_leds();                                  //Chama a funcao para atualizar os leds para atualizar a area na matriz de LEDs
        //Abaixo publica o valor da nova área no tópico do slider, o atualizando
        char payload[8];
        snprintf(payload, sizeof(payload), "%d", areas[area_atual].luminosidade);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/pico/luminosidade"), payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }else if(strcmp(basic_topic, "/pico/areaanter") == 0){  //Se o tópico for /pico/areaanter
        area_atual = (area_atual == 0) ? NUM_AREAS - 1 : area_atual - 1;    //Decrementa a area atual
        atualizar_leds();                                      //Chama a funcao para atualizar os leds para atualizar a area na matriz de LEDs
        //Publica o valor da nova área no tópico do slider
        char payload[8];
        snprintf(payload, sizeof(payload), "%d", areas[area_atual].luminosidade);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/pico/luminosidade"), payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }else if(strcmp(basic_topic, "/exit") == 0){    //Se o tópico for /exit
        state->stop_client = true;                  //Define stop_client como true
        sub_unsub_topics(state, false);             //Chama a funcao para desassinar os tópicos
    }
}

//Tópicos de assinatura --- SUBSCRIBE
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub){
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/pico/luminosidade"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/pico/alarme"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/pico/areaanter"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/pico/areaprox"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

//Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err){
    if(err != 0){   //Se a publicação falhar
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name){ //Retorna o tópico completo
//Pre-processamento, verificando se os topicos serao unicos ou nao
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

//Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err){
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0){  //Se a assinatura falhar
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;   //Aumenta o contador de assinaturas
}

//Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err){
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if(err != 0){   //Se a assinatura falhar
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;               //Diminui o contador de assinaturas
    assert(state->subscribe_count >= 0);    //Verifica se o contador de assinaturas é negativo

    //Stop if requested
    if(state->subscribe_count <= 0 && state->stop_client){  //Se o contador de assinaturas for 0 e o cliente deve ser parado
        mqtt_disconnect(state->mqtt_client_inst);           //Desconecta o cliente MQTT
    }
}

//Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len){
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic)); //Copia o tópico para o estado
}


//Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED){    //Se a conexão for aceita
        state->connect_done = true;          //Define connect_done como true
        sub_unsub_topics(state, true);       //Assina os tópicos
        //Indica que o dispositivo esta online no servidor - Will
        if(state->mqtt_client_info.will_topic){
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }
    }else if(status == MQTT_CONNECT_DISCONNECTED){  //Se a conexão for desconectada
        if(!state->connect_done){                   //Se connect_done for falso
            panic("Failed to connect to mqtt server");  //Indica que a conexão falhou
        }
    }else{  //Outro status
        panic("Unexpected status"); //Indica um status inesperado
    }
}

//Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state){
//Pre-processamento verificando se o servidor utiliza TLS ou nao
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif
    
    //Inicia o cliente MQTT
    state->mqtt_client_inst = mqtt_client_new();
    if(!state->mqtt_client_inst){   //Se o cliente MQTT nao for criado
        panic("MQTT client instance creation error");    //Indica um erro na criação do cliente MQTT
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));    //Imprime o IP do dispositivo
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));    //Imprime o IP do servidor MQTT

    cyw43_arch_lwip_begin();    //Inicia o LWIP
    if(mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK){
        panic("MQTT broker connection error");  //Indica um erro na conexão com o servidor MQTT
    }

//Pre-processamento verificando se o servidor utiliza TLS ou nao
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);    //Define as funções de callback
    cyw43_arch_lwip_end();  //Finaliza o LWIP
}

//Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg){
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if(ipaddr){ //Se o IP for encontrado
        state->mqtt_server_address = *ipaddr;    //Define o IP do servidor MQTT
        start_client(state);                     //Inicia o cliente MQTT
    }else{
        panic("dns request failed");    //Indica um erro na solicitação de DNS
    }
}

//Função principal
int main(void){
    stdio_init_all();
    init_perifericos(); //Chama a funcao para inicializar os periféricos

    static MQTT_CLIENT_DATA_T state;    //Define state como uma estrutura do tipo MQTT_CLIENT_DATA_T

    //Inicializa a arquitetura do cyw43
    if(cyw43_arch_init()){
        panic("Failed to inizialize CYW43");
    }

    //Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    //Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    //Configura o cliente MQTT
    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    //Configura o will
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

//Verifica se o MQTT usa TLS
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    //Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)){
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    //Se tiver o endereço, inicia o cliente
    if(err == ERR_OK){
        start_client(&state);
    }else if(err != ERR_INPROGRESS){ //ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    bool economia_publicado = false;    //Verifica se o modo de economia foi publicado
    publicar_modoeconomia(&state);      //Publica o modo de economia inicial

    publicar_luminosidade(&state);      //Publica valor inicial da área ativa

    //Loop principal
    while(!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)){ //Enquanto o cliente estiver conectado
        cyw43_arch_poll();      //Em pooling, faz uma verificação de conexão

        adc_select_input(1);                //Canal do joystick X
        int eixo_x = adc_read();            //Leitura do joystick X
        verificar_presenca(eixo_x);         //Verifica se houve presenca
        if(economia != economia_publicado){ //Verifica se o modo de economia mudou
        publicar_modoeconomia(&state);      //Publica o modo de economia
        economia_publicado = economia;      //Atualiza o valor do modo de economia publicado
        }

        if(alarme_disparado){               //Verifica se o alarme foi disparado
            pwm_set_enabled(buzzer_slice, true);
            sleep_ms(200);
            pwm_set_enabled(buzzer_slice, false);
            sleep_ms(800);
        }else{                              //Se nao foi disparado, aguarda 300ms
            sleep_ms(300);
        }
    }
    return 0;
}