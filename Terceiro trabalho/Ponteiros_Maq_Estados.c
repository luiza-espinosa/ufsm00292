#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// Parâmetros e Constantes
#define PAYLOAD_MAX_SIZE 250
#define FRAME_START 0x02
#define FRAME_END 0x03


// Estruturas e Tipos da Máquina de Estados

typedef enum {
    PHASE_AWAIT_START = 0, // Aguardando o byte de início
    PHASE_READ_LEN,        // Lendo o byte de tamanho
    PHASE_READ_PAYLOAD,    // Lendo os dados da carga útil
    PHASE_VERIFY_CHECKSUM, // Verificando a soma de verificação
    PHASE_AWAIT_END        // Aguardando o byte de fim
} FsmPhase;


typedef void (*StateAction)(unsigned char receivedByte);

struct ProtocolParser {
    FsmPhase currentPhase;
    unsigned char payloadBuffer[PAYLOAD_MAX_SIZE];
    unsigned char computedChecksum;
    int payloadIndex;
    int expectedLength;
    StateAction phaseHandler[5];
} parser; 


// Protótipos das Funções da Lógica Principal
void phase_handleStartByte(unsigned char receivedByte);
void phase_handleLengthByte(unsigned char receivedByte);
void phase_handlePayloadByte(unsigned char receivedByte);
void phase_handleChecksumByte(unsigned char receivedByte);
void phase_handleEndByte(unsigned char receivedByte);

void initializeParser();
void processInputStream(unsigned char* stream, int streamSize);
void onFrameSuccess(const unsigned char* framePayload, int payloadSize);

// Estrutura para validar os resultados dos testes de forma automatizada.
struct VerificationResult {
    bool frameWasValidated;
    unsigned char payload[PAYLOAD_MAX_SIZE];
    int payloadLength;
};
struct VerificationResult g_verifyResult;


#define ASSERT_TRUE(message, condition) do { if (!(condition)) return message; } while (0)
#define RUN_TEST_CASE(test_function) do { char *msg = test_function(); g_testsRun++; \
                                          if (msg) return msg; } while (0)
int g_testsRun = 0;

void resetTestEnvironment() {
    g_verifyResult.frameWasValidated = false;
    g_verifyResult.payloadLength = 0;
    memset(g_verifyResult.payload, 0, PAYLOAD_MAX_SIZE);
    initializeParser();
}

// Casos de testes

static char* test_FullValidFrame() {
    resetTestEnvironment();
    unsigned char frame[] = {FRAME_START, 0x03, 'O', 'L', 'A', ('O'^'L'^'A'), FRAME_END};
    
    processInputStream(frame, sizeof(frame));
    
    ASSERT_TRUE("FALHA: Um quadro válido não foi processado corretamente.", g_verifyResult.frameWasValidated == true);
    ASSERT_TRUE("FALHA: O tamanho da carga útil está incorreto.", g_verifyResult.payloadLength == 3);
    ASSERT_TRUE("FALHA: O conteúdo da carga útil está incorreto.", g_verifyResult.payload[2] == 'A');
    ASSERT_TRUE("FALHA: O parser não retornou ao estado inicial.", parser.currentPhase == PHASE_AWAIT_START);
    return 0; // Sucesso
}

static char* test_FrameWithInvalidChecksum() {
    resetTestEnvironment();
    unsigned char frame[] = {FRAME_START, 0x02, 0x10, 0x20, 0xFF /*checksum incorreto*/, FRAME_END};
    
    processInputStream(frame, sizeof(frame));
    
    ASSERT_TRUE("FALHA: Um quadro com checksum inválido foi aceito.", g_verifyResult.frameWasValidated == false);
    ASSERT_TRUE("FALHA: O parser não resetou após o erro de checksum.", parser.currentPhase == PHASE_AWAIT_START);
    return 0;
}

static char* test_FramePrecededByGarbage() {
    resetTestEnvironment();
    unsigned char frame[] = {0xDE, 0xAD, 0xBE, 0xEF, FRAME_START, 0x01, 0x42, 0x42, FRAME_END};
    
    processInputStream(frame, sizeof(frame));
    
    ASSERT_TRUE("FALHA: Um quadro válido precedido por lixo não foi aceito.", g_verifyResult.frameWasValidated == true);
    ASSERT_TRUE("FALHA: O tamanho da carga útil (após lixo) está incorreto.", g_verifyResult.payloadLength == 1);
    ASSERT_TRUE("FALHA: O conteúdo (após lixo) está incorreto.", g_verifyResult.payload[0] == 0x42);
    return 0;
}

// Novo teste: valida o tratamento de um quadro sem dados (tamanho 0).
static char* test_FrameWithZeroPayload() {
    resetTestEnvironment();
    unsigned char frame[] = {FRAME_START, 0x00, 0x00 /*checksum de 0 bytes*/, FRAME_END};

    processInputStream(frame, sizeof(frame));

    ASSERT_TRUE("FALHA: Quadro com carga útil zero não foi validado.", g_verifyResult.frameWasValidated == true);
    ASSERT_TRUE("FALHA: O tamanho da carga útil deveria ser 0.", g_verifyResult.payloadLength == 0);
    return 0;
}

// Novo teste: valida a rejeição de um quadro com byte de finalização incorreto.
static char* test_FrameWithInvalidEndByte() {
    resetTestEnvironment();
    unsigned char frame[] = {FRAME_START, 0x01, 'Z', 'Z', 0xFF /*fim inválido*/};

    processInputStream(frame, sizeof(frame));

    ASSERT_TRUE("FALHA: Quadro com byte finalizador inválido foi aceito.", g_verifyResult.frameWasValidated == false);
    return 0;
}


// Executa Testes e Função Principal

static char* runAllTests() {
    RUN_TEST_CASE(test_FullValidFrame);
    RUN_TEST_CASE(test_FrameWithInvalidChecksum);
    RUN_TEST_CASE(test_FramePrecededByGarbage);
    RUN_TEST_CASE(test_FrameWithZeroPayload);
    RUN_TEST_CASE(test_FrameWithInvalidEndByte);
    return 0;
}

int main() {
    char *result = runAllTests();
    
    printf("\n--- Verificação Final ---\n");
    if (result != 0) {
        printf("UM TESTE FALHOU: %s\n", result);
    } else {
        printf("STATUS: Todos os testes foram bem-sucedidos.\n");
    }
    printf("Total de casos de teste executados: %d\n", g_testsRun);
    
    return result != 0;
}


// Implementação da Lógica da Máquina de Estados

void onFrameSuccess(const unsigned char* framePayload, int payloadSize) {
    g_verifyResult.frameWasValidated = true;
    g_verifyResult.payloadLength = payloadSize;
    memcpy(g_verifyResult.payload, framePayload, payloadSize);
}

// Fase 1: Aguardando o byte de início (STX).
void phase_handleStartByte(unsigned char receivedByte) {
    if (receivedByte == FRAME_START) {
        parser.payloadIndex = 0;
        parser.expectedLength = 0;
        parser.computedChecksum = 0;
        parser.currentPhase = PHASE_READ_LEN;
    }
}

// Fase 2: Lendo o byte de tamanho da carga útil.
void phase_handleLengthByte(unsigned char receivedByte) {
    parser.expectedLength = receivedByte;
    if (parser.expectedLength > 0) {
        parser.currentPhase = PHASE_READ_PAYLOAD;
    } else {
        // Pula para a fase de checksum se não houver dados.
        parser.currentPhase = PHASE_VERIFY_CHECKSUM;
    }
}

// Fase 3: Lendo os bytes da carga útil e calculando o checksum.
void phase_handlePayloadByte(unsigned char receivedByte) {
    parser.payloadBuffer[parser.payloadIndex++] = receivedByte;
    // Acumula o checksum usando a operação XOR.
    parser.computedChecksum ^= receivedByte;

    // Se todos os bytes esperados foram lidos, avança a fase.
    if (parser.payloadIndex == parser.expectedLength) {
        parser.currentPhase = PHASE_VERIFY_CHECKSUM;
    }
}

// Fase 4: Comparando o checksum recebido com o calculado.
void phase_handleChecksumByte(unsigned char receivedByte) {
    if (receivedByte == parser.computedChecksum) {
        parser.currentPhase = PHASE_AWAIT_END;
    } else {
        // Em caso de falha, retorna à fase inicial.
        parser.currentPhase = PHASE_AWAIT_START;
    }
}

// Fase 5: Aguardando o byte de fim (ETX).
void phase_handleEndByte(unsigned char receivedByte) {
    if (receivedByte == FRAME_END) {
        // O quadro está completo e válido, executa a ação de sucesso.
        onFrameSuccess(parser.payloadBuffer, parser.payloadIndex);
    }
    // Retorna à fase inicial, pronto para o próximo quadro.
    parser.currentPhase = PHASE_AWAIT_START;
}

void processInputStream(unsigned char* stream, int streamSize) {
    for (int i = 0; i < streamSize; i++) {
        // Invoca a função de tratamento da fase atual via ponteiro.
        parser.phaseHandler[parser.currentPhase](stream[i]);
    }
}

void initializeParser() {
    parser.currentPhase = PHASE_AWAIT_START;
    parser.payloadIndex = 0;
    parser.expectedLength = 0;
    parser.computedChecksum = 0;
    
    // Conecta cada fase da FSM à sua respectiva função de tratamento.
    parser.phaseHandler[PHASE_AWAIT_START]     = phase_handleStartByte;
    parser.phaseHandler[PHASE_READ_LEN]        = phase_handleLengthByte;
    parser.phaseHandler[PHASE_READ_PAYLOAD]    = phase_handlePayloadByte;
    parser.phaseHandler[PHASE_VERIFY_CHECKSUM] = phase_handleChecksumByte;
    parser.phaseHandler[PHASE_AWAIT_END]       = phase_handleEndByte;
}
