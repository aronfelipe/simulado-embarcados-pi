

/************************************************************************/
/* includes                                                             */
/************************************************************************/
#include <asf.h>
#include "oled/gfx_mono_ug_2832hsweg04.h"
#include "oled/gfx_mono_text.h"
#include "sysfont.h"
#include "asf.h"
#include "musics.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

// Buzzer
#define BUZZ_PIO PIOC
#define BUZZ_PIO_ID ID_PIOC
#define BUZZ_PIO_IDX 19
#define BUZZ_PIO_IDX_MASK  (1 << BUZZ_PIO_IDX)

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Button
#define BUT_PIO_1      PIOD
#define BUT_PIO_ID_1   ID_PIOD
#define BUT_IDX_1  28
#define BUT_IDX_MASK_1 (1 << BUT_IDX_1)

// Button
#define BUT_PIO_2      PIOC
#define BUT_PIO_ID_2   ID_PIOC
#define BUT_IDX_2  31
#define BUT_IDX_MASK_2 (1 << BUT_IDX_2)

// Button
#define BUT_PIO_3      PIOA
#define BUT_PIO_ID_3   ID_PIOA
#define BUT_IDX_3  19
#define BUT_IDX_MASK_3 (1 << BUT_IDX_3)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* Variables                                                            */
/************************************************************************/
volatile char but_flag;

volatile int size;

int currentNote = 0;
int currentSong = 0;

typedef struct {
	int size;
	int melody[];
	int temp[];
	char name;
} song;

/************************************************************************/
/* Prototypes                                                           */
/************************************************************************/
void init(void);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void but_callback_1(void)
{
	if (but_flag != 10){
		but_flag = 10;
	}
	else {
		but_flag = currentSong;
	}
}

void but_callback_2(void)
{
	but_flag = 2;
}

void but_callback_3(void)
{
	but_flag = 9;
}

/************************************************************************/
/* Functions                                                            */
/************************************************************************/

void init(void){
	
	sysclk_init();
	
	// Disable watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Configure LED
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Configure BUZZER
	pmc_enable_periph_clk(BUZZ_PIO_ID);
	pio_configure(BUZZ_PIO, PIO_OUTPUT_0, BUZZ_PIO_IDX_MASK, PIO_DEFAULT);
	
	// Initialize clock of PIO responsible for buttons 1,2 and 3
	pmc_enable_periph_clk(BUT_PIO_ID_1);
	pmc_enable_periph_clk(BUT_PIO_ID_2);
	pmc_enable_periph_clk(BUT_PIO_ID_3);
	
	// Configure PIO for buttons 1,2 and 3
	pio_configure(BUT_PIO_1, PIO_INPUT, BUT_IDX_MASK_1, PIO_PULLUP);
	pio_configure(BUT_PIO_2, PIO_INPUT, BUT_IDX_MASK_2, PIO_PULLUP);
	pio_configure(BUT_PIO_3, PIO_INPUT, BUT_IDX_MASK_3, PIO_PULLUP);
	
	// PIO 1 handler
	pio_handler_set(BUT_PIO_1,
	BUT_PIO_ID_1,
	BUT_IDX_MASK_1,
	PIO_IT_RISE_EDGE,
	but_callback_1);
	
	// Set PIO 2 handler
	pio_handler_set(BUT_PIO_2,
	BUT_PIO_ID_2,
	BUT_IDX_MASK_2,
	PIO_IT_RISE_EDGE,
	but_callback_2);
	
	// Set PIO 3 handler
	pio_handler_set(BUT_PIO_3,
	BUT_PIO_ID_3,
	BUT_IDX_MASK_3,
	PIO_IT_RISE_EDGE,
	but_callback_3);
	
	// Enable interruption
	pio_enable_interrupt(BUT_PIO_1, BUT_IDX_MASK_1);
	pio_enable_interrupt(BUT_PIO_2, BUT_IDX_MASK_2);
	pio_enable_interrupt(BUT_PIO_3, BUT_IDX_MASK_3);

	// Configure interruption button PIO 1
	NVIC_EnableIRQ(BUT_PIO_ID_1);
	NVIC_SetPriority(BUT_PIO_ID_1, 4); // Prioridade 4
	
	// Configure interruption button PIO 2
	NVIC_EnableIRQ(BUT_PIO_ID_2);
	NVIC_SetPriority(BUT_PIO_ID_2, 4); // Prioridade 4
	
	// Configure interruption button PIO 3
	NVIC_EnableIRQ(BUT_PIO_ID_3);
	NVIC_SetPriority(BUT_PIO_ID_3, 4); // Prioridade 4
}

void playNote(long frequency, int length){
	if (frequency != 0 ) {
		long delayNote = 1000000 / (frequency / 2);
		long numCycles = (frequency * length) / 1000;
		for (long i=0; i < numCycles; i++){
			pio_set(BUZZ_PIO, BUZZ_PIO_IDX_MASK);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_us(delayNote);
			pio_clear(BUZZ_PIO, BUZZ_PIO_IDX_MASK);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_us(delayNote);
		}
	} else {
		delay_ms(length);
	}
}

void createSong(int size, int[] melody, int[] temp, char name) {
	song _song;
	_song.size = size;
	_song.melody = melody;
	_song.temp = temp;
	_song.name;
	return _song
}

void createSongs() {
	
}



int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();

  // Init OLED
	gfx_mono_ssd1306_init();
  
  // Escreve na tela um circulo e um texto
  // gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (but_flag == 2) {
			int size = sizeof(underworld_melody)/sizeof(int);
			for (int currentNote = 0; currentNote < size; currentNote++){
				gfx_mono_draw_string("Underworld", 10,16, &sysfont);
				int noteDuration = 1000 / underworld_tempo[currentNote];
				playNote(underworld_melody[currentNote], noteDuration);
				if (but_flag == 10){
					currentSong = 2;
					while (but_flag == 10) {
						delay_ms(1);
					}
				}
				if (but_flag == 9){
					but_flag = currentSong + 1;
					break;
				}
			}
		}
		
		if (but_flag == 3) {
			int size = sizeof(melody)/sizeof(int);
			for (int currentNote = 0; currentNote < size; currentNote++){
				gfx_mono_draw_string("            ", 10,16, &sysfont);
				gfx_mono_draw_string("Mario", 10,16, &sysfont);
				int noteDuration = 1000 / tempo[currentNote];
				playNote(melody[currentNote], noteDuration);
				if (but_flag == 10){
					currentSong = 3;
					while (but_flag == 10) {
						delay_ms(1);
					}
					if (but_flag == 9){
						but_flag = currentSong + 1;
						break;
					}
				}
			}
		}

		if (but_flag == 4) {
			int size = sizeof(pirates_melody)/sizeof(int);
			for (int currentNote = 0; currentNote < size; currentNote++){
				gfx_mono_draw_string("            ", 10,16, &sysfont);
				gfx_mono_draw_string("Pirates", 10,16, &sysfont);
				int noteDuration = (1000 / pirates_tempo[currentNote]) + 10;
				playNote(melody[currentNote], noteDuration);
				if (but_flag == 10){
					currentSong = 4;
					while (but_flag == 10) {
						delay_ms(1);
					}
				if (but_flag == 9){
					but_flag = currentSong - 2;
					break;
				}
			}
		}
	}
	}
}
