#pragma once

// Configuration switches; remove/add a double-slash at line-start to enable/disable a feature; to save space disable e.g. CAT, DIAG, KEYER
#define DIAG             1   // Hardware diagnostics on startup (only disable when your rig is working)
#define KEYER            1   // CW keyer
//#define CAT              1   // CAT-interface
#define F_XTAL    27005000   // 27MHz SI5351 crystal
//#define F_XTAL  25004000   // 25MHz SI5351 crystal  (enable for WB2CBA-uSDX, SI5351 break-out board or uSDXDuO)
//#define F_XTAL  25000000   // 25MHz SI5351 crystal  (enable for 25MHz TCXO)
//#define SWAP_ROTARY    1   // Swap rotary direction (enable for WB2CBA-uSDX)
//#define QCX            1   // Supports older (non-SDR) QCX HW modifications (QCX, QCX-SSB, QCX-DSP with I/Q alignment-feature)


//#define LCD
#define OLED           1   // OLED display, connect SDA (PD2), SCL (PD3)
#define BITBANG_OLED //bit bang OLED on SDA=PD2 (header 13) SCL=PD3 (header 14)
//#define BLIND

#define LPF_SWITCHING_DL2MAN_USDX_REV3           1   // Enable 8-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH 1   // Enable 8-band filter bank switching: non-latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m. Enable this if you are using 8-band non-latching version for the relays, the radio will draw extra 15mA current but will work ity any relay (Tnx OH2UDS/TA7W Baris)
//#define LPF_SWITCHING_DL2MAN_USDX_REV2         1   // Enable 5-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV2_BETA    1   // Enable 5-band filter bank switching:     latching relays wired to a PCA9539PW   GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV1         1   // Enable 3-band filter bank switching:     latching relays wired to a PCA9536D    GPIO extender on the PC4/PC5 I2C bus; relays are using IO0 as common (ground), IO1-IO3 used by the individual latches K1-3 switching respectively LPFs for 20m, 40m, 80m
//#define LPF_SWITCHING_WB2CBA_USDX_QUADBAND     1   // Enable 4-band filter bank switching: non-latching relays wired to a MCP23008    GPIO extender on the PC4/PC5 I2C bus; relays are using GND as common (ground), GP2..5 used by the individual latches K1-4 switching respectively LPFs for 40m, 30m, 20m, 17m
//#define LPF_SWITCHING_WB2CBA_USDX_OCTOBAND     1   // Enable 8-band filter bank switching: non-latching relays wired to a MCP23008    GPIO extender on the PC4/PC5 I2C bus; relays are using GND as common (ground), GP0..7 used by the individual latches K1-8 switching respectively LPFs for 80m, 60m, 40m, 30m, 20m, 17m, 15m, 10m
//#define LPF_SWITCHING_PE1DDA_USDXDUO           14  // Enable 2-band filter bank switching: non-latching relay  wired to pin PD5 (pin 11); specify as value the frequency in MHz for which (and above) the relay should be altered (e.g. put 14 to enable the relay at 14MHz and above to use the 20m LPF).
#define SI5351_ADDR   0x60   // SI5351A I2C address: 0x60 for SI5351A-B-GT, Si5351A-B04771-GT, MS5351M; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum

// Advanced configuration switches
//#define CAT_EXT        1   // Extended CAT support: remote button and screen control commands over CAT
//#define CAT_STREAMING  1   // Extended CAT support: audio streaming over CAT, once enabled and triggered with CAT cmd, 7.812ksps 8-bit unsigned audio is sent over UART. The ";" is omited in the data-stream, and only sent to indicate the beginning and end of a CAT cmd.
#define CW_DECODER       1   // CW decoder
#define TX_ENABLE        1   // Disable this for RX only (no transmit), e.g. to support uSDX for kids idea: https://groups.io/g/ucx/topic/81030243#6276
#define KEY_CLICK        1   // Reduce key clicks by envelope shaping
#define SEMI_QSK         1   // Just after keying the transmitter, keeps the RX muted for a short amount of time in the anticipation for continued keying
#define RIT_ENABLE       1   // Receive-In-Transit alternates the receiving frequency with an user-defined offset to compensate for any necessary tuning needed on receive
#define VOX_ENABLE       1   // Voice-On-Xmit which is switching the transceiver into transmit as soon audio is detected (above noise gate level)
//#define MOX_ENABLE     1   // Monitor-On-Xmit which is audio monitoring on speaker during transmit
#define FAST_AGC         1   // Adds fast AGC option (good for CW)
//#define VSS_METER      1   // Supports Vss measurement (as s-meter option)
//#define SWR_METER      1   // Supports SWR meter with bridge on A6/A7 (LQPF ATMEGA328P) by Alain, K1FM, see: https://groups.io/g/ucx/message/6262 and https://groups.io/g/ucx/message/6361
//#define ONEBUTTON      1   // Use single (encoder) button to control full the rig; optionally use L/R buttons to completely replace rotory encoder function
//#define F_MCU   16000000   // 16MHz ATMEGA328P crystal (enable for unmodified Arduino Uno/Nano boards with 16MHz crystal). You may change this value to any other crystal frequency (up to 28MHz may work)
//#define DEBUG          1   // for development purposes only (adds debugging features such as CPU, sample-rate measurement, additional parameters)
//#define TESTBENCH      1   // Tests RX chain by injection of sine wave, measurements results are sent over serial
//#define CW_FREQS_QRP   1   // Defaults to CW QRP   frequencies when changing bands
//#define CW_FREQS_FISTS 1   // Defaults to CW FISTS frequencies when changing bands
#define CW_MESSAGE       1   // Transmits pre-defined CW messages on-demand (left-click menu item 4.2)
//#define CW_MESSAGE_EXT 1   // Additional CW messages
//#define TX_DELAY       1   // Enables a delay in the actual transmission to allow relay-switching to be completed before the power is applied (see also NTX, PTX definitions below for GPIO that can switch relay/PA)
//#define NTX            11  // Enables LOW  on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define PTX            11  // Enables HIGH on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define CLOCK          1   // Enables clock
//#define TX_CLK0_CLK1   1   // Use CLK0, CLK1 for TX instead of CLK2 (enable this for uSDXDuO); NTX pin may be used for enabling the TX path (this is like RX pin, except that RX may also be used as attenuator)
#define CW_INTERMEDIATE  1   // CW decoder shows intermediate characters (only for LCD and F_MCU at 20M), sequences like:  EIS[HV] EIUF EAW[JP] EARL TMO TMG[ZQ] TND[BX] TNK[YC], may be good to learn CW; a full list of possible sequences:  EISH5 EISV3 EIUF EIUU2 EAWJ1 EAWP EARL TMOO0 TMOO9 TMOO8 TMGZ7 TMGQ TNDB6 TNDX TNKY TNKC

// QCX pin defintions
#define LCD_D4  0         //PD0    (pin 2)
#define LCD_D5  1         //PD1    (pin 3)
#define LCD_D6  2         //PD2    (pin 4)
#define LCD_D7  3         //PD3    (pin 5)
#define LCD_EN  4         //PD4    (pin 6)
#define FREQCNT 5         //PD5    (pin 11)
#define ROT_A   6         //PD6    (pin 12)
#define ROT_B   7         //PD7    (pin 13)
#define RX      8         //PB0    (pin 14)
#define SIDETONE 9        //PB1    (pin 15)
#define KEY_OUT 10        //PB2    (pin 16)
#define SIG_OUT 11        //PB3    (pin 17)
#define DAH     12        //PB4    (pin 18)
#define DIT     13        //PB5    (pin 19)
#define AUDIO1  14        //PC0/A0 (pin 23)
#define AUDIO2  15        //PC1/A1 (pin 24)
#define DVM     16        //PC2/A2 (pin 25)
#define BUTTONS 17        //PC3/A3 (pin 26)
#define LCD_RS  18        //PC4    (pin 27)
#define SDA     18        //PC4    (pin 27)
#define SCL     19        //PC5    (pin 28)
//#define NTX   11        //PB3    (pin 17)
//#define PTX   11        //PB3    (pin 17)

#ifdef SWAP_ROTARY
#undef ROT_A
#undef ROT_B
#define ROT_A   7         //PD7    (pin 13)
#define ROT_B   6         //PD6    (pin 12)
#endif

#if (defined(CAT) || defined(TESTBENCH)) && !(OLED)
#define _SERIAL  1       // Coexistence support for serial port and LCD on the same pins
#endif

#ifdef LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH
#define LPF_SWITCHING_DL2MAN_USDX_REV3         1
#endif

#ifndef TX_ENABLE
#undef KEYER
#undef TX_DELAY
#undef SEMI_QSK
#undef RIT_ENABLE
#undef VOX_ENABLE
#undef MOX_ENABLE
#endif //!TX_ENABLE


#define MORE_MIC_GAIN // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)

#ifdef SWR_METER
float FWD;
float SWR;
float ref_V = 5 * 1.15;
static uint32_t stimer;
#define vin_FWD  A6
#define vin_REF  A7
#endif