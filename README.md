# SIIP - SIstema Inteligente de Iluminação e Presença IoT com MQTT e Raspberry Pi Pico W

## Visão Geral

Este projeto implementa um sistema de controle de iluminação inteligente baseado em IoT utilizando a placa Raspberry Pi Pico W. O sistema é controlado remotamente via MQTT e permite:

* Controle de intensidade de luz (0% a 100%) por área
* Comutação entre até 10 áreas diferentes
* Modo de economia automático por detecção de presença (via joystick analógico)
* Alarme sonoro com buzzer
* Sinalização visual com LEDs WS2812 por área
* Interface visual em display OLED SSD1306
* Integração com o app IoT MQTT Panel (Android)

## Hardware Utilizado

* Raspberry Pi Pico W
* Módulo de display OLED SSD1306 (I2C)
* Módulo WS2812 (matriz de LEDs RGB)
* Joystick analógico (eixo X em ADC)
* Buzzer (PWM)
* LED vermelho (GPIO)
* Broker MQTT (Mosquitto via Termux no Android ou computador)

## Funcionalidades via MQTT

### Tópicos Assinados:

* `/pico/luminosidade`: (slider) Define a intensidade da luz da área atual (0 a 100%)
* `/pico/alarme`: (switch) Liga/desliga o alarme sonoro ("on"/"off")
* `/pico/areaprox`: (botão) Avança para próxima área
* `/pico/areaanter`: (botão) Retorna para área anterior

### Tópicos Publicados:

* `/pico/luminosidade/areaX`: Publica a luminosidade atual da área (0 ≤ X ≤ 9)
* `/pico/modoeconomia`: Indica o estado atual do modo de economia ("on"/"off")
* `/online`: Indica se o dispositivo está ativo ("1" conectado, "0" desconectado)

## Interface Visual

O display OLED mostra:

* Número da área atual
* Intensidade da luz atual
* Estado do modo economia ("Modo: Eco" ou "Modo: Normal")

## Lógica de Presença (Economia)

* Se o joystick estiver parado por >2 segundos, ativa modo economia (apaga LED e ativa LED vermelho)
* Ao detectar movimento, retorna para modo normal e reativa luz conforme configuração MQTT da área

## Instruções de Uso

1. **Configure o broker MQTT** (ex: Termux + Mosquitto no Android)
2. **Ajuste as configurações de Wi-Fi e broker no código:**

```c
#define WIFI_SSID "SEU_SSID"
#define WIFI_PASSWORD "SUA_SENHA"
#define MQTT_SERVER "SEU_IP"
#define MQTT_USERNAME "USUARIO"
#define MQTT_PASSWORD "SENHA"
```

3. **Compile e grave o firmware na Pico W** com SDK do Pico
4. **Abra o IoT MQTT Panel no celular e configure os widgets conforme os tópicos descritos acima**

## Observações

* Toda a comunicação é feita exclusivamente por MQTT (sem HTTP)
* As áreas funcionam como zonas independentes com luminosidade separada
* O display pode ser reconfigurado para exibir outras informações se desejado

## Extensões Futuras

* Inclusão de sensor de temperatura ou luminosidade real
* Armazenamento local da última configuração
* Controle via dashboard web

## Autor

Implementado por Leonam S. Rabelo.