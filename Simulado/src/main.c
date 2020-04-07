

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

// RTC Calendar struct
typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;


// Buzzer
#define BUZZ_PIO PIOC
#define BUZZ_PIO_ID ID_PIOC
#define BUZZ_PIO_IDX 19
#define BUZZ_PIO_IDX_MASK  (1 << BUZZ_PIO_IDX)

// LED PLACA
#define LED_PIO_PLACA      PIOC
#define LED_PIO_ID_PLACA   ID_PIOC
#define LED_IDX_PLACA      8
#define LED_IDX_MASK_PLACA (1 << LED_IDX_PLACA)

// LED1 OLED
#define LED_PIO_1 PIOA
#define LED_PIO_ID_1 ID_PIOA
#define LED_IDX_1 0
#define LED_IDX_MASK_1 (1 << LED_IDX_1)

// LED2 OLED
#define LED_PIO_2 PIOC
#define LED_PIO_ID_2 ID_PIOC
#define LED_IDX_2 30
#define LED_IDX_MASK_2 (1 << LED_IDX_2)

// LED3 OLED
#define LED_PIO_3 PIOB
#define LED_PIO_ID_3 ID_PIOB
#define LED_IDX_3 2
#define LED_IDX_MASK_3 (1 << LED_IDX_3)

// Button 1 OLED
#define BUT_PIO_1      PIOD
#define BUT_PIO_ID_1   ID_PIOD
#define BUT_IDX_1  28
#define BUT_IDX_MASK_1 (1 << BUT_IDX_1)

// Button 2 OLED
#define BUT_PIO_2      PIOC
#define BUT_PIO_ID_2   ID_PIOC
#define BUT_IDX_2  31
#define BUT_IDX_MASK_2 (1 << BUT_IDX_2)

// Button 3 OLED
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
volatile char but_flag_1;
volatile char but_flag_2;
volatile char but_flag_3;

volatile int size;

// TC
volatile char flag_tc_led1 = 0;
volatile char flag_tc_led2 = 0;
volatile char flag_tc_led3 = 0;

// RTT
volatile Bool f_rtt_alarme = false;
volatile Bool led_status = false;

// RTC
volatile char flag_rtc = 0;

/************************************************************************/
/* Prototypes                                                           */
/************************************************************************/
void init(void);


// TC
void TC_init_led1(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void TC_init_led2(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void TC_init_led3(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

// RTT
void pin_toggle(Pio *pio, uint32_t mask);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

// RTC
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void but_callback_1(void)
{
	if (but_flag_1 == 1) {
		but_flag_1 = 0;
		} else {
		but_flag_1 = 1;
	}
}

void but_callback_2(void)
{
	if (but_flag_2 == 1) {
		but_flag_2 = 0;
		} else {
		but_flag_2 = 1;
	}
}

void but_callback_3(void)
{
	if (but_flag_3 == 1) {
		but_flag_3 = 0;
	} else {
		but_flag_3 = 1;
	}
}


// TC

/**
*  Handle Interrupcao botao 1
*/
static void Button1_Handler(uint32_t id, uint32_t mask)
{
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);


	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc_led1 = 1;
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC3_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc_led2 = 1;
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC6_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC2, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc_led3 = 1;
}

// RTT
void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

// RTC
/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}



/************************************************************************/
/* Functions                                                            */
/************************************************************************/

void init(void){
	
	sysclk_init();
	
	// Disable watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Configure LED1
	pmc_enable_periph_clk(LED_PIO_ID_1);
	pio_configure(LED_PIO_1, PIO_OUTPUT_1, LED_IDX_MASK_1, PIO_DEFAULT);
	
	// Configure LED2
	pmc_enable_periph_clk(LED_PIO_ID_2);
	pio_configure(LED_PIO_2, PIO_OUTPUT_0, LED_IDX_MASK_2, PIO_DEFAULT);
		
	// Configure LED3
	pmc_enable_periph_clk(LED_PIO_ID_3);
	pio_configure(LED_PIO_3, PIO_OUTPUT_0, LED_IDX_MASK_3, PIO_DEFAULT);

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
	PIO_IT_FALL_EDGE,
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

// TC
/*
 * @Brief Pisca LED placa
 */
void pisca_TC_1(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED_PIO_1, LED_IDX_MASK_1);
    delay_ms(t);
    pio_set(LED_PIO_1, LED_IDX_MASK_1);
    delay_ms(t);
  }
}

void pisca_TC_2(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_PIO_2, LED_IDX_MASK_2);
		delay_ms(t);
		pio_set(LED_PIO_2, LED_IDX_MASK_2);
		delay_ms(t);
	}
}

void pisca_TC_3(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_PIO_3, LED_IDX_MASK_3);
		delay_ms(t);
		pio_set(LED_PIO_3, LED_IDX_MASK_3);
		delay_ms(t);
	}
}

// RTT
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
	return ul_previous_time;
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

// RTC
/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();

	// Init OLED
	gfx_mono_ssd1306_init();
	
	
	// TC INIT
	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC1, ID_TC3, 0, 10);
	TC_init(TC2, ID_TC6, 0, 1);
	
	f_rtt_alarme = true;

	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
	
	int hour;
	int minute;
	int second = 1;
	
  /* Insert application code here, after the board has been initialized. */
	while(1) {
// 		rtc_get_time(RTC, hour, minute, second);
// 		second = 1;
// 		char result[50];
// 		sprintf(result, "%f", second);
// 
//  	gfx_mono_draw_string(result, 10, 0, &sysfont);

// 		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

// 		gfx_mono_draw_char(get_time_rtt(), 10, 0, &sysfont);

		gfx_mono_draw_string("5   10   1", 10, 0, &sysfont);

		if (get_time_rtt() == 1) {
			gfx_mono_draw_filled_circle(5, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(15, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(25, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
		}
		
		if (get_time_rtt() == 9) {
			gfx_mono_draw_filled_circle(5, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(15, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(25, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(35, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(45, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(55, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);

		}
		
		if (get_time_rtt() == 17) {
			gfx_mono_draw_filled_circle(5, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(15, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(25, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(35, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(45, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(55, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(65, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(75, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(85, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(95, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
			gfx_mono_draw_filled_circle(105, 23, 2, GFX_PIXEL_SET, GFX_WHOLE);
		}
		
		if (get_time_rtt() == 18) {
			gfx_mono_draw_filled_circle(5, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(15, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(25, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(35, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(45, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(55, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(65, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(75, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(85, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(95, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);
			gfx_mono_draw_filled_circle(105, 23, 2, GFX_PIXEL_CLR, GFX_WHOLE);

		}
				
		if (led_status) {
			
			if(flag_tc_led1){
				if (but_flag_1 == 1) {
					pisca_TC_1(1,5);
					flag_tc_led1 = 0;
				}
			}
			
			if(flag_tc_led2){
				if (but_flag_2 == 1) {
					pisca_TC_2(1,10);
					flag_tc_led2 = 0;
				}
			}
			
			if(flag_tc_led3){
				if (but_flag_3 == 1) {
					pisca_TC_3(1,1);
					flag_tc_led3 = 0;
				}
			}
			
		}
		
		if (f_rtt_alarme){
			
			/*
			* IRQ apos 4s -> 8*0.5
			*/
		  
		  			
			/*
			* IRQ apos 5s -> 10*0.5
			*/
			
// 			gfx_mono_draw_string("5   10   1", 10, 0, &sysfont);
			
// 			pin_toggle(LED_PIO_1, LED_IDX_MASK_1);    // BLINK Led

			uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
			uint32_t irqRTTvalue = 18;
      
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);         
      		 						
			f_rtt_alarme = false;
			
			if (led_status == true) {
				led_status = false;
			} else {
				led_status = true;
			}
		}
	}
}
