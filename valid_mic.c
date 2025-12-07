#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "i2s_mic.pio.h"
#include "ff.h"
#include "pico/bootrom.h"

// Definição de pinos
#define botaoB  6
#define PIN_SCK 8
#define PIN_WS  9
#define PIN_SD  20
#define LED_PIN 12

#define REC_DURATION 60      // Duração de cada arquivo (segundos)
#define MIN_FREE_SPACE_MB 50     // Mínimo de espaço livre para começar a apagar antigos
#define MAX_FILES_SCAN 99999     // Limite de numeração

#define SAMPLE_RATE 24000
#define CHANNELS 1
#define BIT_DEPTH 24
#define BYTES_PER_SAMPLE (BIT_DEPTH / 8)
#define PIO_CLK_HZ (SAMPLE_RATE * 132) // Ajuste: 132 ciclos reais de instrução PIO por frame
#define BUFFER_SIZE 4096 

int32_t buffer_A[BUFFER_SIZE];
int32_t buffer_B[BUFFER_SIZE];
uint8_t stage_buffer[BUFFER_SIZE * BYTES_PER_SAMPLE];

// Variáveis de Controle de Arquivo
uint32_t current_file_idx = 0;
uint32_t oldest_file_idx = 0;

int dma_chan_A;
int dma_chan_B;

volatile bool buffer_A_full = false;
volatile bool buffer_B_full = false;

// CABEÇALHO WAV 
typedef struct {
    char riff[4];           // RIFF
    uint32_t total_size;    // Tamanho do arquivo - 8 bytes
    char wave[4];           // WAVE
    char fmt[4];            // fmt 
    uint32_t fmt_size;      // 16
    uint16_t audio_format;  // 1 (PCM)
    uint16_t channels;      // 1
    uint32_t sample_rate;   // 24000
    uint32_t byte_rate;     // sample_rate * channels * bytes_per_sample
    uint16_t block_align;   // channels * bytes_per_sample
    uint16_t bits_per_sample;// 24
    char data[4];           // data
    uint32_t data_size;     // Tamanho dos dados de áudio
} __attribute__((packed)) WavHeader;

// Chamada automaticamente quando um buffer enche
void dma_handler() {
    if (dma_hw->ints1 & (1u << dma_chan_A)) {
        dma_hw->ints1 = (1u << dma_chan_A); // Limpa IRQ
        buffer_A_full = true;
        dma_channel_set_write_addr(dma_chan_A, buffer_A, false); // Rearma para o futuro
    }
    
    if (dma_hw->ints1 & (1u << dma_chan_B)) {
        dma_hw->ints1 = (1u << dma_chan_B); // Limpa IRQ
        buffer_B_full = true;
        dma_channel_set_write_addr(dma_chan_B, buffer_B, false); // Rearma para o futuro
    }
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0); // entra em bootsel
}

void write_wav_header(FIL *fil, uint32_t audio_data_size) {
    WavHeader h;
    
    // Identificadores
    memcpy(h.riff, "RIFF", 4);
    memcpy(h.wave, "WAVE", 4);
    memcpy(h.fmt, "fmt ", 4);
    memcpy(h.data, "data", 4);
    
    // Tamanhos e Formatos
    h.fmt_size = 16;
    h.audio_format = 1; // PCM
    h.channels = CHANNELS;
    h.sample_rate = SAMPLE_RATE;
    h.bits_per_sample = BIT_DEPTH;
    
    // Cálculos Importantes para 24-bit
    h.block_align = CHANNELS * BYTES_PER_SAMPLE;     // 1 * 3 = 3 bytes
    h.byte_rate = SAMPLE_RATE * h.block_align;       // 24000 * 3 = 72000 bytes/s
    
    h.data_size = audio_data_size;
    h.total_size = audio_data_size + sizeof(WavHeader) - 8;
    
    UINT bw;
    f_write(fil, &h, sizeof(WavHeader), &bw);
}

void erro_fatal(const char* msg, int codigo) {
    printf("ERRO FATAL: %s (%d)\n", msg, codigo);
    while(true) { gpio_put(LED_PIN, 1); sleep_ms(100); gpio_put(LED_PIN, 0); sleep_ms(100); }
}

void scan_files(void) {
    printf("Escaneando arquivos existentes...\n");
    FILINFO fno;
    FRESULT fr;
    char fname[20];

    sprintf(fname, "REC_00000.WAV");
    fr = f_stat(fname, &fno);
    
    if (fr == FR_NO_FILE) {
        printf("Nenhum arquivo inicial encontrado. Comecando do zero.\n");
        current_file_idx = 0;
        oldest_file_idx = 0;

        for (int i = 0; i < 100; i++) {
             sprintf(fname, "REC_%05d.WAV", i);
             if (f_stat(fname, &fno) == FR_OK) {
                 // Opa, achamos um arquivo perdido! Vamos cair na lógica completa.
                 goto full_scan; 
             }
        }
        return;
    }

    full_scan:   

    printf("Arquivos detectados. Buscando sequencia...\n");
    
    bool found_oldest = false;
    for (int i = 0; i < MAX_FILES_SCAN; i++) {
        sprintf(fname, "REC_%05d.WAV", i);
        fr = f_stat(fname, &fno);
        
        if (fr == FR_OK) {
            oldest_file_idx = i;
            printf("Mais antigo encontrado: %s\n", fname);
            found_oldest = true;
            break; 
        }
    }
    
    if (!found_oldest) {
        current_file_idx = 0;
        oldest_file_idx = 0;
        return;
    }

    for (int i = oldest_file_idx; i < MAX_FILES_SCAN; i++) {
        sprintf(fname, "REC_%05d.WAV", i);
        fr = f_stat(fname, &fno);
        
        if (fr == FR_NO_FILE) {
            current_file_idx = i;
            printf("Proximo arquivo sera: %s\n", fname);
            return;
        }
    }
    
    printf("Limite de arquivos atingido! Resetando contador.\n");
    current_file_idx = 0; 
}

// Garante espaço livre apagando arquivos antigos
void manage_space(FATFS *fs) {
    FATFS *pfs = fs;
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT fr;

    // Calcula espaço livre
    fr = f_getfree("0:", &fre_clust, &pfs);
    if (fr != FR_OK) return;

    // Conversão para MB (Assumindo setor de 512 bytes)
    uint32_t free_mb = (fre_clust * pfs->csize) / 2048; // (Cluster * SetoresPorCluster * 512) / 1024 / 1024

    printf("Espaco Livre: %lu MB\n", free_mb);

    while (free_mb < MIN_FREE_SPACE_MB) {
        // Tenta apagar o mais antigo
        char old_fname[20];
        sprintf(old_fname, "REC_%05d.WAV", oldest_file_idx);
        
        printf("Espaco baixo! Apagando %s...\n", old_fname);
        fr = f_unlink(old_fname);
        
        if (fr == FR_OK) {
            printf("Apagado.\n");
            oldest_file_idx++; // Avança o índice do mais antigo
        } else if (fr == FR_NO_FILE) {
            // Se o arquivo não existe (buraco na sequencia), pula ele
            oldest_file_idx++;
        } else {
            printf("Erro ao apagar: %d\n", fr);
            break; // Evita loop infinito se der erro de hardware
        }

        // Recalcula espaço
        f_getfree("0:", &fre_clust, &pfs);
        free_mb = (fre_clust * pfs->csize) / 2048;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(4000);

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(botaoB); gpio_set_dir(botaoB, GPIO_IN); gpio_pull_up(botaoB);
     
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    FATFS fs; 
    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) erro_fatal("Mount Falhou", fr);

    scan_files();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &i2s_mic_program);
    uint sm = pio_claim_unused_sm(pio, true);

    i2s_mic_program_init(pio, sm, offset, PIN_SD, PIN_SCK);

    float div = (float)clock_get_hz(clk_sys) / PIO_CLK_HZ;
    pio_sm_set_clkdiv(pio, sm, div);
    
    // Configura FIFO para avisar o DMA quando tiver dados
    uint dreq = pio_get_dreq(pio, sm, false);

    // Configuração DMA
    dma_chan_A = dma_claim_unused_channel(true);
    dma_chan_B = dma_claim_unused_channel(true);
    
    dma_channel_config cfg_A = dma_channel_get_default_config(dma_chan_A);
    channel_config_set_transfer_data_size(&cfg_A, DMA_SIZE_32); 
    channel_config_set_read_increment(&cfg_A, false);
    channel_config_set_write_increment(&cfg_A, true);
    channel_config_set_dreq(&cfg_A, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&cfg_A, dma_chan_B); 

    dma_channel_config cfg_B = dma_channel_get_default_config(dma_chan_B);
    channel_config_set_transfer_data_size(&cfg_B, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_B, false);
    channel_config_set_write_increment(&cfg_B, true);
    channel_config_set_dreq(&cfg_B, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&cfg_B, dma_chan_A); // OTIMIZAÇÃO 2 (Chaining B->A)

    dma_channel_configure(dma_chan_A, &cfg_A, buffer_A, &pio->rxf[sm], BUFFER_SIZE, false);
    dma_channel_configure(dma_chan_B, &cfg_B, buffer_B, &pio->rxf[sm], BUFFER_SIZE, false);

    dma_channel_set_irq1_enabled(dma_chan_A, true);
    dma_channel_set_irq1_enabled(dma_chan_B, true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    dma_channel_start(dma_chan_A);
    pio_sm_set_enabled(pio, sm, true);

    while (true) {
        // Gerencia Espaço 
        manage_space(&fs);

        // Abre Novo Arquivo
        FIL fil;
        char filename[20];
        sprintf(filename, "REC_%05d.WAV", current_file_idx);
        printf("Iniciando arquivo: %s\n", filename);
        
        fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) erro_fatal("Erro ao criar arquivo", fr);
        write_wav_header(&fil, 0); // Header placeholder

        // Grava por X segundos
        uint32_t current_file_bytes = 0;
        uint32_t max_file_bytes = SAMPLE_RATE * 3 * REC_DURATION;
        
        gpio_put(LED_PIN, 1);
        
        // Loop de Captura do Arquivo Atual
        while (current_file_bytes < max_file_bytes) {
            int32_t *src = NULL;
            if (buffer_A_full) { src = buffer_A; buffer_A_full = false; }
            else if (buffer_B_full) { src = buffer_B; buffer_B_full = false; }

            if (src) {
                // Empacota
                for (int i = 0; i < BUFFER_SIZE; i++) {
                    int32_t s = src[i];
                    stage_buffer[i*3 + 0] = (s >> 0) & 0xFF;
                    stage_buffer[i*3 + 1] = (s >> 8) & 0xFF;
                    stage_buffer[i*3 + 2] = (s >> 16) & 0xFF;
                }
                // Escreve
                UINT bw;
                f_write(&fil, stage_buffer, BUFFER_SIZE * 3, &bw);
                current_file_bytes += bw;
            }
        }

        // Finaliza Arquivo Atual
        f_lseek(&fil, 0);
        write_wav_header(&fil, current_file_bytes);
        f_close(&fil);
        f_sync(&fil); // Garante que foi pro disco
        
        printf("Arquivo salvo. Trocando...\n");
        gpio_put(LED_PIN, 0);
        
        // Incrementa contador para o próximo
        current_file_idx++;
    }
}