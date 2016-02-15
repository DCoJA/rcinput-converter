#include <stdint.h>
#include <stdbool.h>

extern void _start (void);
static void stc_handler (void);

static void __attribute__ ((naked))
reset (void)
{
  asm volatile ("cpsid	i\n\t"		/* Mask all interrupts. */
		"ldr	r0, 0f\n\t"	/* Go to entry. */
		"bx	r0\n\t"
		".align	2\n"
	"0:	.word	_start"
		: /* no output */ : /* no input */ : "memory");
  /* Never reach here. */
}

static void
unexpected (void)
{
  for (;;);
}

typedef void (*handler)(void);
extern uint8_t __ram_end__;

#if defined (ENABLE_ISP)
uint32_t crp __attribute__ ((section(".crp"))) = 0;
#else
// Code Red protection: NO_ISP
uint32_t crp __attribute__ ((section(".crp"))) = 0x4E697370;
#endif

handler vector[48] __attribute__ ((section(".vectors"))) = {
  (handler)&__ram_end__,
  reset,
  unexpected,	/*  2: NMI */
  unexpected,	/*  3: Hard fault */
  unexpected, unexpected, unexpected, unexpected, /* 4-7 */
  unexpected, unexpected, unexpected, /* 8-10 */
  unexpected,	/* 11: SVC */
  unexpected, unexpected, /* 12-13 */
  unexpected,	/* 14: Pend SV */
  unexpected,	/* 15: SysTick */
  unexpected,	/* 16: SPI0 */
  unexpected,	/* 17: SPI1 */
  unexpected,	/* 18: Reserve */
  unexpected,	/* 19: UART0 */
  unexpected,	/* 20: UART1 */
  unexpected,	/* 21: UART2 */
  unexpected,	/* 22: Reserve */
  unexpected,	/* 23: Reserve */
  unexpected,	/* 24: I2C */
  stc_handler,	/* 25: SCT */
  unexpected,	/* 26: MRT */
  unexpected,	/* 27: CMP */
  unexpected,	/* 28: WDT */
  unexpected,	/* 29: BOD */
  unexpected,	/* 30: Reserve */
  unexpected,	/* 31: WKT */
  unexpected, unexpected, unexpected, unexpected, /* 32-35 */
  unexpected, unexpected, unexpected, unexpected, /* 36-39 */
  unexpected, unexpected, unexpected, unexpected, /* 40-43 PININT0-3 */
  unexpected, unexpected, unexpected, unexpected, /* 44-47 PININT4-7 */
};

#include "lpc8xx.h"

#define STC_IRQn (25-16)

// STC counter prescale 1/15 which means 30/15=2MHz clock
#define STC_CTRL_PRESCALE	(14 << 5)

#define UART_STAT_TXRDY (0x1 << 2)
#define UART_STAT_TXIDLE (0x1 << 3)

static void main_loop (void);

extern char _edata[], _data[], _textdata[];
extern char _bss_start[], _bss_end[];

void
_start (void)
{
  // Copy .data section from flash.  All are word aligned.  See linker script.
  uint32_t *p = (uint32_t *) _data;
  uint32_t *q = (uint32_t *) _textdata;
  uint32_t size = (uint32_t *) _edata - (uint32_t *) _data;

  while (size--)
    *p++ = *q++;

  // Clear .bss.  Also word aligned.
  p = (uint32_t *) _bss_start;
  size = (uint32_t *) _bss_end - (uint32_t *) _bss_start;
  while (size--)
    *p++ = 0;

  // 30Mhz: main 60Mhz, P=2, M=(4+1), 60*2*2=240Mhz 240/(2*2)/5=12Mz
  LPC_SYSCON->SYSPLLCTRL = 0x24;
  LPC_SYSCON->SYSAHBCLKDIV = 0x02;

  LPC_SYSCON->PDRUNCFG &= ~(0x80);
  while (!(LPC_SYSCON->SYSPLLSTAT & 0x01))
    ;

  LPC_SYSCON->MAINCLKSEL = 0x03;
  LPC_SYSCON->MAINCLKUEN = 0x01;
  while (!(LPC_SYSCON->MAINCLKUEN & 0x01))
    ;

  // 1 wait state for flash
  LPC_FLASHCTRL->FLASHCFG &= ~(0x03);

  // PIO0_1 is an output
  LPC_GPIO_PORT->DIR0 |= 0x02;

  // Enable SCT and UART0 clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8)|(1 << 14);

  // Reset SCT and UART0
  LPC_SYSCON->PRESETCTRL &= ~((1 << 8)|(1 << 3));
  LPC_SYSCON->PRESETCTRL |= ((1 << 8)|(1 << 3));

  // Enable U0-TXD on PIO0_4 (Pin 2) PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN0 = 0xffffff04;
  // Enable SCIN_0 on PIO0_0 (Pin 8) PINASSIGN5(31:24)
  LPC_SWM->PINASSIGN5 = 0x00ffffff;

  // UART0 baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 115200->207, 250000->0,  500000->128
  // BRG        115200->17,  250000->14, 500000->4
#if defined (DT_ULB128)
  // 500000 for ulb128
  LPC_SYSCON->UARTFRGMULT = 128;
  LPC_USART0->BRG = 4;
#else
  // 250000 for XBUS
  LPC_SYSCON->UARTFRGMULT = 0;
  LPC_USART0->BRG = 14;
#endif

  // UART0 enabled, 8 bit, no parity, 1 stop bit, no flow control
  LPC_USART0->CFG = 0x05;

  // [0]=1:32-bit op, [2:1]=0:Prescaled bus clock, [16:9]=1:INSYNC, other 0
  LPC_SCT->CONFIG = 0x1|(1 << 9);
  // As capture registers
  LPC_SCT->REGMODE_L = 0x1f;
  // Enable event 0 in state 0
  LPC_SCT->EVENT[0].STATE = 0x1;
  // [5]=0:Input, [9:6]=0:CTIN_0, [11:10]=1: Rise, [13:12]=2: IO
  LPC_SCT->EVENT[0].CTRL = (1 << 10)|(2 << 12);
  // Enable event 0 in state 0
  LPC_SCT->EVENT[1].STATE = 0x1;
  // [5]=0:Input, [9:6]=0:CTIN_0, [11:10]=2: Fall, [13:12]=2: IO
  LPC_SCT->EVENT[1].CTRL = (2 << 10)|(2 << 12);
  // Capture 0 for event 0
  LPC_SCT->CAPCTRL[0].U = 1;
  // Capture 1 for event 1
  LPC_SCT->CAPCTRL[1].U = 2;
  // Start, clear counter, 1/15 prescale
  LPC_SCT->CTRL_U = (1 << 3)|STC_CTRL_PRESCALE;

  // Enable STC intr with NVIC
  NVIC_ISER = 1 << STC_IRQn;

  // Wait 20ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  // Enable SCT event 0 and 1 to request interrupt.
  LPC_SCT->EVEN = 0x1|0x2;

  main_loop ();
}

#define FIFOSZ 256
static uint16_t dtfifo[FIFOSZ];
static volatile uint32_t dtbegin;
static volatile uint32_t dtsize;
static uint32_t tlast;

// Ignore too narrow pulse as noise.
#define DT_MIN 3

static void
dtoverflow (void)
{
#if 1
  // This is unexpected and should be treated as fatal error.
  for(;;);
#endif
}

static void
stc_handler (void)
{
  uint32_t dt, tnow;
  uint32_t ev;

  if (dtsize == FIFOSZ)
    dtoverflow ();
  else
    {
      /* Get capture reg and compute DT.  Encode rise/fall to 0th bit.  */
      ev = LPC_SCT->EVFLAG;
      if ((ev & 3) == 3)
	{
	  LPC_SCT->EVFLAG = 1|2;
	  if (LPC_SCT->CAP[0].U > LPC_SCT->CAP[1].U)
	    dt = LPC_SCT->CAP[0].U - LPC_SCT->CAP[1].U;
	  else
	    dt = LPC_SCT->CAP[1].U > LPC_SCT->CAP[0].U;
	  if (dt >= DT_MIN)
	    // This is unexpected and should be treated as fatal error.
	    for(;;);
	  return;
	}
      if (ev & 1)
	{
	  LPC_SCT->EVFLAG = 1; // Clear interrupt request for this event.
	  tnow = LPC_SCT->CAP[0].U;
	  dt = tnow - tlast;
#if 1
	  // Not to wrap up counter
	  if (tnow >> 24)
	    {
	      // Halt SCT, reset counter and run it again
	      LPC_SCT->CTRL_U = (1 << 2)|STC_CTRL_PRESCALE;
	      tnow = LPC_SCT->COUNT_U - tnow;
	      LPC_SCT->COUNT_U = tnow;
	      LPC_SCT->CTRL_U = STC_CTRL_PRESCALE;
	    }
#endif
	  tlast = tnow;
	  if (dt & 1)
	    ++dt;
	  dt &= ~1;  // Encode "off" to 0-th bit
	}
      else if (ev & 2)
	{
	  LPC_SCT->EVFLAG = 2; // Clear interrupt request for this event.
	  tnow = LPC_SCT->CAP[1].U;
	  dt = tnow - tlast;
	  tlast = tnow;
	  if (dt & 1)
	    ++dt;
	  dt |= 1;  // Encode "on" to 0-th bit
	}
      else
	// Can be ignored as sprious interrupt?
	return;
      // Saturate when dt > ~32ms
      if (dt > 0xffff)
	dt = 0xffff;
      dtfifo[(dtbegin + dtsize) % FIFOSZ] = (uint16_t) dt;
      ++dtsize;
    }
}

static void
send_uart (uint8_t b)
{
  while(~LPC_USART0->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART0->TXDATA = b;
  while(~LPC_USART0->STAT & UART_STAT_TXIDLE)
    ;
}

#if defined (DT_ULB128)
/* Send DT with ulb128 format.  */
static inline void
send_dt_ulb128 (uint16_t d)
{
  uint8_t b;

  b = d & 0x7f;
  d >>= 7;
  if (d == 0)
    send_uart (b);
  else
    {
      send_uart (b | 0x80);
      b = d & 0x7f;
      d >>= 7;
      if (d == 0)
	send_uart (b);
      else
	{
	  send_uart (b | 0x80);
	  send_uart (d);
	}
    }
}
#else

#define SBUS_NUM_CHANNELS 16
#define SBUS_FRAME_SIZE	25
#define SBUS_FLAGS_BYTE	23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

struct {
  uint16_t bytes[SBUS_FRAME_SIZE]; // including start bit, parity and stop bits
  uint16_t bit_ofs;
} sbus_state;

static uint16_t xpwm_values[SBUS_NUM_CHANNELS];

static uint8_t crc8_array[256] =
{
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
  0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
  0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
  0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
  0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
  0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
  0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
  0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
  0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
  0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
  0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
  0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
  0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
  0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
  0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
  0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
  0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/* XBUS packet is a byte sequence which looks like:
  command(0xa4), length(2+4*(1-50)), key(0), type(0),
  ch-id(1-50), ch-func(0), ch-data-high, ch-data-low,
  repeated ch-* stuff
  crc8 (x^8+x^5+x^4+1)
*/

static void
xbus (int num_channels)
{
  int i;
  uint8_t d;
  uint8_t crc = 0;
  uint16_t *q = &xpwm_values[0];

  crc = crc8_array[(crc ^ 0xa4) & 0xff];
  crc = crc8_array[(crc ^ (2+4*num_channels)) & 0xff];
  send_uart (0xa4);
  send_uart (2+4*num_channels);
  crc = crc8_array[(crc ^ 0) & 0xff];
  crc = crc8_array[(crc ^ 0) & 0xff];
  send_uart (0);
  send_uart (0);

  for (i = 1; i <= num_channels; i++)
    {
      crc = crc8_array[(crc ^ i) & 0xff];
      send_uart (i);
      crc = crc8_array[(crc ^ 0) & 0xff];
      send_uart (0);
      d = (uint8_t) (*q >> 8);
      crc = crc8_array[(crc ^ d) & 0xff];
      send_uart (d);
      d = (uint8_t) (*q++ & 0xff);
      crc = crc8_array[(crc ^ d) & 0xff];
      send_uart (d);
    }
  send_uart (crc);
}

/* S.BUS decoder matrix based on src/modules/px4iofirmware/sbus.c from
   PX4Firmware.

   Each channel value can come from up to 3 input bytes. Each row in the
   matrix describes up to three bytes, and each entry gives:

   - byte offset in the data portion of the frame
   - right shift applied to the data byte
   - mask for the data byte
   - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};

static const struct sbus_bit_pick sbus_decoder[SBUS_NUM_CHANNELS][3] = {
  /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
  /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
  /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
  /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
  /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
  /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static bool
sbus_decode (const uint8_t frame[SBUS_FRAME_SIZE])
{
  /* check frame boundary markers to avoid out-of-sync cases */
  if (frame[0] != 0x0f)
    return false;

  /* use the decoder matrix to extract channel data */
  for (int ch = 0; ch < SBUS_NUM_CHANNELS; ch++)
    {
      unsigned value = 0;

      for (int pick = 0; pick < 3; pick++)
	{
	  const struct sbus_bit_pick *decode = &sbus_decoder[ch][pick];

	  if (decode->mask != 0)
	    {
	      unsigned piece = frame[1 + decode->byte];
	      piece >>= decode->rshift;
	      piece &= decode->mask;
	      piece <<= decode->lshift;
	      value |= piece;
	    }
	}

      /* Map SBUS range to XBUS range.
	 200 -> 1000micro -> 0x2492
	 1800 -> 2000micro -> 0xdb6d
      */
      xpwm_values[ch] = ((value * 29959) >> 10) + 0xdb7;
    }

  return true;
}

/* Process S.BUS pulse to XBUS pwm values, based on the implementation
   of ardupilot/libraries/AP_HAL_Linux/RCInput.cpp.  */
static void
sbus_pulse (uint16_t width_s0, uint16_t width_s1)
{
  /* Precision of the internal clock generator is < 1.5%.  */
#define CLK_ADJ 3
  /* uint16_t bits_s0 = (width_s0+1) / 10;
     uint16_t bits_s1 = (width_s1+1) / 10;
     Approximate 1/10 to 205/2048  */
  uint16_t bits_s0 = ((width_s0+1) * (205 + CLK_ADJ)) >> 11;
  uint16_t bits_s1 = ((width_s1+1) * (205 + CLK_ADJ)) >> 11;
  uint16_t nlow;

  /* uint8_t byte_ofs = sbus_state.bit_ofs/12;
     uint8_t bit_ofs = sbus_state.bit_ofs%12;
     Approximate 1/12 to 683/8192  */
  uint8_t byte_ofs = (sbus_state.bit_ofs * 683) >> 13;
  uint8_t bit_ofs = sbus_state.bit_ofs - (byte_ofs * 12);

  if (bits_s0 == 0 || bits_s1 == 0)
    // invalid data
    goto reset;

  if (bits_s0+bit_ofs > 10)
    // invalid data as last two bits must be stop bits
    goto reset;

  // pull in the high bits
  sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
  sbus_state.bit_ofs += bits_s0;
  bit_ofs += bits_s0;

  // pull in the low bits
  nlow = bits_s1;
  if (nlow + bit_ofs > 12)
    nlow = 12 - bit_ofs;

  bits_s1 -= nlow;
  sbus_state.bit_ofs += nlow;

  if (sbus_state.bit_ofs == SBUS_FRAME_SIZE*12 && bits_s1 > 12)
    {
      // we have a full frame
      static uint8_t bytes[SBUS_FRAME_SIZE];
      uint8_t i;

      for (i=0; i<SBUS_FRAME_SIZE; i++)
	{
	  // get inverted data
	  uint16_t v = ~sbus_state.bytes[i];
	  // check start bit
	  if ((v & 1) != 0)
	    goto reset;
	  // check stop bits
	  if ((v & 0xC00) != 0xC00)
	    goto reset;

	  // check parity
	  uint8_t parity = 0, j;

	  for (j=1; j<=8; j++)
	    parity ^= (v & (1U<<j))?1:0;

	  if (parity != (v&0x200)>>9)
	    goto reset;

	  bytes[i] = ((v>>1) & 0xFF);
        }

      if (sbus_decode (bytes))
	{
	  // Assemble xbus frame and send it
	  xbus (SBUS_NUM_CHANNELS);
	}
      goto reset;
    }
  else if (bits_s1 > 12)
    {
      // break
      goto reset;
    }
  return;
 reset:
  {
    for (int i = 0; i <= byte_ofs; i++)
      sbus_state.bytes[i] = 0;
    sbus_state.bit_ofs = 0;
  }
}

# if defined (PROCESS_CPPM)

#define CPPM_NUM_CHANNELS SBUS_NUM_CHANNELS
struct {
  int _channel_counter;
  uint16_t _pulse_capt[CPPM_NUM_CHANNELS];
} ppm_state;

/* Process PPM-sum pulse to XBUS pwm values, based on the implementation
   of ardupilot/libraries/AP_HAL_Linux/RCInput.cpp.  */
static void
ppmsum_pulse (uint16_t width_usec)
{
  if (width_usec >= 2700)
    {
      /* A long pulse indicates the end of a frame. Reset the channel
	 counter so next pulse is channel 0.  */
      if (ppm_state._channel_counter >= 5)
	{
	  for (int i = 0; i < ppm_state._channel_counter; i++)
	    xpwm_values[i] = ((ppm_state._pulse_capt[i] - 800) * 47934) >> 10;
	  xbus (ppm_state._channel_counter);
        }
      ppm_state._channel_counter = 0;
      return;
    }
  if (ppm_state._channel_counter == -1)
    {
      // we are not synchronised
      return;
    }

  /* We limit inputs to between 800usec and 2200usec for XBUS. This
     also allows us to decode SBUS on the same pin, as SBUS will have
     a maximum pulse width of 100usec.  */
  if (width_usec >= 800 && width_usec <= 2200)
    {
      // take a reading for the current channel buffer these
      ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

      // move to next channel
      ppm_state._channel_counter++;
    }

  /* If we have reached the maximum supported channels then mark
     as unsynchronised, so we wait for a wide pulse.  */
  if (ppm_state._channel_counter == CPPM_NUM_CHANNELS)
    {
      for (int i = 0; i < CPPM_NUM_CHANNELS; i++)
	xpwm_values[i] = ((ppm_state._pulse_capt[i] - 800) * 47934) >> 10;

      xbus (CPPM_NUM_CHANNELS);
      ppm_state._channel_counter = -1;
    }
}
# endif
#endif

static void
main_loop (void)
{
  uint16_t dt;
  uint16_t dt_high, dt_low;

  dtbegin = 0;
  dtsize = 0;
  dt_high = dt_low = 0;

  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
      asm volatile ("cpsid	i" : : : "memory");
      if (dtsize)
	{
	  dt = dtfifo[dtbegin];
	  dtbegin = (dtbegin + 1) % FIFOSZ;
	  --dtsize;
	  asm volatile ("cpsie      i" : : : "memory");
	}
      else
	{
	  asm volatile ("cpsie      i" : : : "memory");
	  // asm volatile ("wfi" : : : "memory");
	  continue;
	}

#if defined (DT_ULB128)
      send_dt_ulb128 (dt);
#else
      if (dt & 1)
	dt_high = dt >> 1;
      else
	dt_low = dt >> 1;
      if (dt_low && dt_high)
	{
# if defined (PROCESS_CPPM)
	  ppmsum_pulse (dt_high + dt_low);
# endif
	  sbus_pulse (dt_high, dt_low);

	  dt_high = dt_low = 0;
	}
#endif
    }
}
