#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef PT_H_
#define PT_H_

struct pt {
  unsigned short lc;
};

#define PT_INIT(pt)   ((pt)->lc = 0)
#define PT_THREAD(name_args) char name_args
#define PT_WAITING 0
#define PT_YIELDED 1
#define PT_EXITED  2
#define PT_ENDED   3

#define PT_BEGIN(pt) { char PT_YIELD_FLAG = 1; switch((pt)->lc) { case 0:
#define PT_END(pt)   } PT_YIELD_FLAG = 0; (pt)->lc = 0; return PT_ENDED; }
#define PT_WAIT_UNTIL(pt, condition) \
  do { (pt)->lc = __LINE__; case __LINE__: \
  if(!(condition)) { return PT_WAITING; } } while(0)
#define PT_WAIT_WHILE(pt, cond)  PT_WAIT_UNTIL((pt), !(cond))
#define PT_YIELD(pt) \
  do { PT_YIELD_FLAG = 0; (pt)->lc = __LINE__; case __LINE__: \
  if(PT_YIELD_FLAG == 0) { return PT_YIELDED; } } while(0)
#define PT_EXIT(pt) \
  do { PT_INIT(pt); return PT_EXITED; } while(0)
#define PT_SCHEDULE(f) ((f) < PT_EXITED)

#endif /* PT_H_ */


// Constantes do Protocolo
#define PKT_SOH (0x02) // Start of Header (Início do Pacote)
#define PKT_EOT (0x03) // End of Transmission (Fim do Pacote)
#define MSG_ACK (0x06) // Mensagem de Acknowledge (Confirmação)
#define DATA_BUFFER_CAPACITY 32 // Capacidade máxima da carga útil
#define ACK_WAIT_PERIOD 100 // Tempo de espera pelo ACK em ticks do sistema

// Ambiente de Teste Simulado
uint8_t g_serial_channel[DATA_BUFFER_CAPACITY + 5]; // Simula o meio físico (ex: UART)
int g_serial_channel_bytes_avail = 0;
bool g_handshake_complete = false; // Flag que indica se o ACK foi recebido
unsigned int g_timer_ticks = 0;    // Simula um contador de tempo do sistema

// Variáveis de Verificação para os Testes 
int g_retry_count = 0; // Conta o número de retransmissões
uint8_t g_decoded_data[DATA_BUFFER_CAPACITY];
int g_decoded_data_len = 0;



#define VERIFY(message, condition) do { if (!(condition)) return message; } while (0)
#define EXEC_TEST(test_func) do { char *err_msg = test_func(); g_tests_run++; \
                                if (err_msg) return err_msg; } while (0)
int g_tests_run = 0;

void initializeTestBed() {
    g_serial_channel_bytes_avail = 0;
    g_handshake_complete = false;
    g_timer_ticks = 0;
    g_retry_count = 0;
    g_decoded_data_len = 0;
    memset(g_serial_channel, 0, sizeof(g_serial_channel));
    memset(g_decoded_data, 0, sizeof(g_decoded_data));
}


// Contextos para as duas protothreads
struct pt pt_encoder, pt_decoder;

// Protótipos
PT_THREAD(decoder_protothread(struct pt *pt));
PT_THREAD(encoder_protothread(struct pt *pt, const uint8_t* payload, uint8_t size));


uint8_t compute_xor_checksum(const uint8_t* data, uint8_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < size; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}


PT_THREAD(decoder_protothread(struct pt *pt)) {
    // Variáveis locais devem ser estáticas para preservar seu valor entre chamadas
    static uint8_t payload_size;
    static uint8_t computed_chk;
    static uint8_t incoming_chk;
    static uint8_t i;
    static uint8_t current_byte;

    PT_BEGIN(pt);

    for (;;) { // Loop infinito para processar pacotes continuamente
        // 1. Espera pelo byte de início do pacote
        PT_WAIT_UNTIL(pt, g_serial_channel_bytes_avail > 0 && g_serial_channel[0] == PKT_SOH);
        g_serial_channel_bytes_avail = 0;

        // 2. Espera pelo byte de tamanho
        PT_WAIT_UNTIL(pt, g_serial_channel_bytes_avail > 0);
        payload_size = g_serial_channel[0];
        g_decoded_data_len = 0;
        computed_chk = 0;
        g_serial_channel_bytes_avail = 0;

        // 3. Espera e processa os bytes da carga útil
        for (i = 0; i < payload_size; ++i) {
            PT_WAIT_UNTIL(pt, g_serial_channel_bytes_avail > 0);
            current_byte = g_serial_channel[0];
            g_decoded_data[i] = current_byte;
            computed_chk ^= current_byte;
            g_decoded_data_len++;
            g_serial_channel_bytes_avail = 0;
        }

        // 4. Espera pelo byte de checksum
        PT_WAIT_UNTIL(pt, g_serial_channel_bytes_avail > 0);
        incoming_chk = g_serial_channel[0];
        g_serial_channel_bytes_avail = 0;

        // 5. Espera pelo byte de fim e valida o pacote
        PT_WAIT_UNTIL(pt, g_serial_channel_bytes_avail > 0);
        current_byte = g_serial_channel[0];
        g_serial_channel_bytes_avail = 0;

        if (current_byte == PKT_EOT && incoming_chk == computed_chk) {
            // Se tudo estiver correto, envia a confirmação (ACK)
            g_serial_channel[0] = MSG_ACK;
            g_serial_channel_bytes_avail = 1;
        }
    }

    PT_END(pt);
}

PT_THREAD(encoder_protothread(struct pt *pt, const uint8_t* payload, uint8_t size)) {
    static unsigned int deadline_tick;
    static uint8_t packet_checksum;
    static uint8_t i;
    
    PT_BEGIN(pt);

    packet_checksum = compute_xor_checksum(payload, size);

    for (;;) { // Loop para suportar retransmissões em caso de falha
        // Envia SOH
        g_serial_channel[0] = PKT_SOH;
        g_serial_channel_bytes_avail = 1;
        PT_YIELD(pt);

        // Envia Tamanho
        g_serial_channel[0] = size;
        g_serial_channel_bytes_avail = 1;
        PT_YIELD(pt);

        // Envia Carga Útil
        for (i = 0; i < size; ++i) {
            g_serial_channel[0] = payload[i];
            g_serial_channel_bytes_avail = 1;
            PT_YIELD(pt);
        }

        // Envia Checksum
        g_serial_channel[0] = packet_checksum;
        g_serial_channel_bytes_avail = 1;
        PT_YIELD(pt);

        // Envia EOT
        g_serial_channel[0] = PKT_EOT;
        g_serial_channel_bytes_avail = 1;
        PT_YIELD(pt);

        // Aguarda pelo ACK até o timeout
        deadline_tick = g_timer_ticks + ACK_WAIT_PERIOD;
        PT_WAIT_UNTIL(pt, g_handshake_complete || g_timer_ticks >= deadline_tick);

        if (g_handshake_complete) {
            // Sucesso! Sai do loop.
            break;
        } else {
            // Timeout! Incrementa o contador de retentativas e o loop continua.
            g_retry_count++;
        }
    }

    PT_END(pt);
}


static char* test_Decoder_HandlesValidPacket() {
    initializeTestBed();
    uint8_t data[] = {'T', 'E', 'S', 'T'};
    uint8_t chk = compute_xor_checksum(data, 4);
    uint8_t packet[] = {PKT_SOH, 4, 'T', 'E', 'S', 'T', chk, PKT_EOT};

    PT_INIT(&pt_decoder);

    for (int i = 0; i < sizeof(packet); ++i) {
        g_serial_channel[0] = packet[i];
        g_serial_channel_bytes_avail = 1;
        decoder_protothread(&pt_decoder);
    }
    
    VERIFY("Decodificador: A carga útil não foi extraída corretamente.", g_decoded_data_len == 4 && memcmp(g_decoded_data, data, 4) == 0);
    VERIFY("Decodificador: O ACK não foi enviado após um pacote válido.", g_serial_channel_bytes_avail == 1 && g_serial_channel[0] == MSG_ACK);
    return 0;
}

static char* test_Encoder_SuccessfulTransmissionWithAck() {
    initializeTestBed();
    uint8_t data_to_send[] = {0xCA, 0xFE};
    
    PT_INIT(&pt_encoder);
    PT_INIT(&pt_decoder);

    // Simula um escalonador cooperativo
    while (PT_SCHEDULE(encoder_protothread(&pt_encoder, data_to_send, 2))) {
        g_timer_ticks++;
        decoder_protothread(&pt_decoder);
        
        // Simula a recepção do ACK
        if(g_serial_channel_bytes_avail == 1 && g_serial_channel[0] == MSG_ACK) {
            g_handshake_complete = true;
            g_serial_channel_bytes_avail = 0;
        }
    }

    VERIFY("Codificador: Retransmissões não deveriam ocorrer em um cenário de sucesso.", g_retry_count == 0);
    VERIFY("Codificador: O handshake de confirmação (ACK) deveria ter sido completado.", g_handshake_complete == true);
    return 0;
}

static char* test_Encoder_RetriesOnTimeout() {
    initializeTestBed();
    uint8_t data_to_send[] = {0xAB, 0xCD};
    
    PT_INIT(&pt_encoder);
    PT_INIT(&pt_decoder); // Decoder é iniciado mas não recebe o ACK

    g_handshake_complete = false; // Força a condição de falha no ACK

    // Roda o escalonador por tempo suficiente para o timeout ocorrer
    while (g_timer_ticks < ACK_WAIT_PERIOD + 50) {
        encoder_protothread(&pt_encoder, data_to_send, 2);
        g_timer_ticks++;
        // Propositalmente não verificamos/setamos o g_handshake_complete
    }
    
    VERIFY("Codificador: Pelo menos uma retentativa deveria ter ocorrido no timeout.", g_retry_count > 0);
    return 0;
}

static char* test_Decoder_RejectsInvalidEOT() {
    initializeTestBed();
    uint8_t data[] = {'E', 'R', 'R'};
    uint8_t chk = compute_xor_checksum(data, 3);
    // Pacote com byte finalizador incorreto (0xFF)
    uint8_t packet[] = {PKT_SOH, 3, 'E', 'R', 'R', chk, 0xFF};

    PT_INIT(&pt_decoder);
    
    g_serial_channel_bytes_avail = 0; // Garante que não há ACK residual

    for (int i = 0; i < sizeof(packet); ++i) {
        g_serial_channel[0] = packet[i];
        g_serial_channel_bytes_avail = 1;
        decoder_protothread(&pt_decoder);
    }
    
    VERIFY("Decodificador: Um ACK foi enviado para um pacote com finalizador inválido.", g_serial_channel_bytes_avail == 0);
    return 0;
}

static char* executeTestSuite() {
    EXEC_TEST(test_Decoder_HandlesValidPacket);
    EXEC_TEST(test_Encoder_SuccessfulTransmissionWithAck);
    EXEC_TEST(test_Encoder_RetriesOnTimeout);
    EXEC_TEST(test_Decoder_RejectsInvalidEOT); // Novo teste adicionado
    return 0;
}

int main() {
    char *result = executeTestSuite();
    
    printf("\n--- Resultados dos testes ---\n");
    if (result != 0) {
        printf("ERRO: %s\n", result);
    } else {
        printf("SUCESSO: Todos os testes passaram.\n");
    }
    printf("Total de testes executados: %d\n", g_tests_run);
    
    return result != 0;
}
