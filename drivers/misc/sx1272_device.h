#ifndef __SX1272_H__
#define __SX1272_H__

/* LoRa Mode register */
/* definition */								/* default value          description. 					*/
#define REGFIFO 					0x00		/* 0x00 FIFO read/write access 							*/
#define REGOPMODE					0x01		/* 0x01 Operating mode & LoRaTM / FSK selection 		*/
#define REGFRFMSB 					0x06 		/* 0xE4 RF Carrier Frequency, Most Significant Bits		*/
#define REGFRFMID 					0x07 		/* 0xC0 RF Carrier Frequency, Intermediate Bits         */
#define REGFRFLSB 					0x08 		/* 0x00 RF Carrier Frequency, Least Significant Bits    */
#define REGPACONFIG 				0x09 		/* 0x0F PA selection and Output Power control           */
#define REGPARAMP 					0x0A 		/* 0x19 Control of PA ramp time, low phase noise PLL    */
#define REGOCP 					    0x0B 		/* 0x2B Over Current Protection control                 */
#define REGLNA 					    0x0C 		/* 0x20 LNA settings                                    */
#define REGFIFOADDRPTR 			    0x0D 		/* 0x08 0x1E FIFO SPI pointer                           */
#define REGFIFOTXBASEADDR			0x0E 		/* 0x02 Start Tx data                                   */
#define REGFIFORXBASEADDR			0x0F 		/* 0x0A Start Rx data                                   */
#define FIFORXCURRENTADDR			0x10 		/* 0xFF Start address of last packet received           */
#define REGIRQFLAGSMASK 			0x11 		/* n/a n/a Optional IRQ flag mask                       */
#define REGIRQFLAGS 				0x12 		/* 0x15 IRQ flags                                       */
#define REGRXNBBYTES 				0x13 		/* 0x0B Number of received bytes                        */
#define REGRXHEADERCNTVALUEMSB		0x14 		/* 0x28 Number of valid headers received                */
#define REGRXHEADERCNTVALUELSB 	    0x15 		/* 0x0C Number of valid headers received                */
#define REGRXPACKETCNTVALUEMSB		0x16 		/* 0x12 Number of valid packets received                */
#define REGRXPACKETCNTVALUELSB		0x17 		/* 0x47 Number of valid packets received                */
#define REGMODEMSTAT 				0x18 		/* 0x32 - Live LoRaTM modem status                      */
#define REGPKTSNRVALUE 			    0x19 		/* 0x3E - Espimation of last packet SNR                 */
#define REGPKTRSSIVALUE 			0x1A 		/* 0x00 RSSI of last packet                             */
#define REGRSSIVALUE_LORA			0x1B 		/* 0x00 Current RSSI                                    */
#define REGHOPCHANNEL 				0x1C 		/* 0x00 FHSS start channel                              */
#define REGMODEMCONFIG1			    0x1D 		/* 0x00 Modem PHY config 1                              */
#define REGMODEMCONFIG2			    0x1E 		/* 0x00 Modem PHY config 2                              */
#define REGSYMBTIMEOUTLSB			0x1F 		/* 0xAA Receiver timeout value                          */
#define REGPREAMBLEMSB_LORA		    0x20 		/* 0x00 Size of preamble                                */
#define REGPREAMBLELSB_LORA		    0x21 		/* 0x00 Size of preamble                                */
#define REGPAYLOADLENGTH_LORA		0x22 		/* 0x00 LoRa payload length                             */
#define REGMAXPAYLOADLENGTH		    0x23 		/* 0x00 LoRa maximum payload length                     */
#define REGHOPPERIOD 				0x24 		/* 0x05 FHSS Hop period                                 */
#define REGFIFORXBYTEADDR			0x25 		/* 0x00 Address of last byte written in FIFO            */
#define REGFEIMSB_LORA				0x28 		/* 0x55 Estimated frequency error                       */
#define REGFEIMIB_LORA				0x29 		/* 0x55                                                 */
#define REGFEILSB_LORA				0x2A 		/* 0x55                                                 */
#define REGRSSIWIDEBAND			    0x2C 		/* 0x55 Wideband RSSI measurement                       */
#define REGDETECTOPTIMIZE			0x31 		/* 0x40 LoRa detection Optimize for SF6                 */

#define REGINVERTIQ 				0x33 		/* 0x00 Invert LoRa I and Q signals						*/
#define REGDETECTIONTHRESHOLD		0x37 		/* 0x00 Change the LoRa Detection threshold for SF6     */
#define REGSYNCWORD 				0x39 		/* 0xF5 LoRa Sync Word                                  */
/* Common registers */
#define REGDIOMAPPING1 			    0x40 		/* 0x00 Mapping of pins DIO0 to DIO3					*/
#define REGDIOMAPPING2 			    0x41 		/* 0x00 Mapping of pins DIO4 and DIO5, ClkOut frequency */
#define REGVERSION 				    0x42 		/* 0x22 Semtech ID relating the silicon revision        */
#define REGAGCREF 					0x43 		/* 0x13 Adjustment of the AGC thresholds                */
#define REGAGCTHRESH1 				0x44 		/* 0x0E                                                 */
#define REGAGCTHRESH2 				0x45 		/* 0x5B                                                 */
#define REGAGCTHRESH3 				0x46 		/* 0xDB                                                 */
#define REGPLLHOP 					0x4B 		/* 0x2E Control the fast frequency hopping mode         */
#define REGTCXO 					0x58 		/* 0x09 TCXO or XTAL input setting                      */
#define REGPADAC 					0x5A 		/* 0x84 Higher power settings of the PA                 */
#define REGPLL 					    0x5C 		/* 0xD0 Control of the PLL bandwidth                    */
#define REGPLLLOWPN 				0x5E 		/* 0xD0 Control of the Low Phase Noise PLL bandwidth    */
#define REGFORMERTEMP 				0x6C 		/* - Stored temperature during the former IQ Calibration*/
#define REGBITRATEFRAC 			    0x70 		/* 0x00 Fractional part in the Bit Rate division ratio  */

/* FSK/ODK mode register */
#define REGBITRATEMSB				0x02		/* 0x1A Bit Rate setting, Most Significant Bits          	*/
#define REGBITRATELSB 				0x03		/* 0x0B Bit Rate setting, Least Significant Bits            */
#define REGFDEVMSB 					0x04		/* 0x00 Frequency Deviation setting, Most Significant Bits  */
#define REGFDEVLSB 					0x05		/* 0x52 Frequency Deviation setting, Least Significant Bits */
        
#define REGRXCONFIG 				0x0D 		/* 0x08 0x1E AFC, AGC, ctrl									*/                
#define REGRSSICONFIG 				0x0E 		/* 0x02 RSSI                                                */
#define REGRSSICOLLISION			0x0F 		/* 0x0A RSSI Collision detector                             */
#define REGRSSITHRESH				0x10 		/* 0xFF RSSI Threshold control                              */
#define REGRSSIVALUE				0x11 		/* n/a n/a RSSI value in dBm                                */
#define REGRXBW					    0x12 		/* 0x15 Channel Filter BW Control                           */
#define REGAFCBW					0x13 		/* 0x0B AFC Channel Filter BW                               */
#define REGOOKPEAK					0x14 		/* 0x28 OOK demodulator                                     */
#define REGOOKFIX					0x15 		/* 0x0C Threshold of the OOK demod                          */
#define REGOOKAVG					0x16 		/* 0x12 Average of the OOK demod                            */
#define RESERVED17					0x17 		/* 0x47 -                                                   */
#define RESERVED18 				    0x18 		/* 0x32 -                                                   */
#define RESERVED19					0x19 		/* 0x3E -                                                   */
#define REGAFCFEI					0x1A 		/* 0x00 AFC and FEI control                                 */
#define REGAFCMSB					0x1B 		/* 0x00 Frequency correction value of the AFC               */
#define REGAFCLSB					0x1C 		/* 0x00                                                     */
#define REGFEIMSB					0x1D 		/* 0x00 Value of the calculated frequency error             */
#define REGFEILSB					0x1E 		/* 0x00                                                     */
#define REGPREAMBLEDETECT			0x1F 		/* 0x40 Settings of the Preamble Detector                   */
#define REGRXTIMEOUT1				0x20 		/* 0x00 Timeout Rx request and RSSI                         */
#define REGRXTIMEOUT2				0x21 		/* 0x00 Timeout RSSI and Pay-loadReady                      */
#define REGRXTIMEOUT3				0x22 		/* 0x00 Timeout RSSI and SyncAddress                        */
#define REGRXDELAY					0x23 		/* 0x00 Delay between Rx cycles                             */
#define REGOSC						0x24 		/* 0x05 RC Oscillators Settings, CLKOUT frequency           */
#define REGPREAMBLEMSB				0x25 		/* 0x00 Preamble length, MSB                                */
#define REGPREAMBLELSB				0x26 		/* 0x03 Preamble length, LSB                                */
#define REGSYNCCONFIG				0x27 		/* 0x93 Sync Word Recognition control RESERVED              */
#define REGSYNCVALUE1				0x28 		/* 0x55 Sync Word bytes, 1 through 8                        */
#define REGSYNCVALUE2				0x29 		/* 0x55                                                     */
#define REGSYNCVALUE3				0x2A 		/* 0x55                                                     */
#define REGSYNCVALUE4				0x2B 		/* 0x55                                                     */
#define REGSYNCVALUE5				0x2C 		/* 0x55                                                     */
#define REGSYNCVALUE6				0x2D 		/* 0x55                                                     */
#define REGSYNCVALUE7				0x2E 		/* 0x55                                                     */
#define REGSYNCVALUE8				0x2F 		/* 0x55                                                     */
#define REGPACKETCONFIG1			0x30 		/* 0x90 Packet mode settings                                */
#define REGPACKETCONFIG2			0x31 		/* 0x40 Packet mode settings                                */
#define REGPAYLOADLENGTH			0x32 		/* 0x40 Payload length setting RESERVED                     */
#define REGNODEADRS				    0x33 		/* 0x00 Node address Invert LoRa I and Q signals            */
#define REGBROADCASTADRS			0x34 		/* 0x00 Broadcast address                                   */
#define REGFIFOTHRESH				0x35 		/* 0x0F 0x8F Fifo threshold, Tx start condition             */
#define REGSEQCONFIG1				0x36 		/* 0x00 Top level Sequencer settings                        */
#define REGSEQCONFIG2				0x37 		/* 0x00 Top level Sequencer settings Change                 */
#define REGTIMERRESOL				0x38 		/* 0x00 Timer 1 and 2 resolution control                    */
#define REGTIMER1COEF				0x39 		/* 0xF5 0x12 Timer 1 setting                                */
#define REGTIMER2COEF				0x3A 		/* 0x20 Timer 2 setting                                     */
#define REGIMAGECAL				    0x3B 		/* 0x82 0x02 Image calibration engine control               */
#define REGTEMP					    0x3C 		/* - Temperature Sensor value                               */
#define REGLOWBAT					0x3D 		/* 0x02 Low Battery Indicator Settings                      */
#define REGIRQFLAGS1				0x3E 		/* 0x80 n/a Status register: PLL Lock state,Timeout, RSSI   */
#define REGIRQFLAGS2				0x3F 		/* 0x40 n/a Status register: FIFO handling flags, Low Battery */

/*-----------------------------------------------------------------------------
 *	IOCTL CODE
 */
typedef struct _sx1272_rw_struc {
	unsigned char reg;
	unsigned char data;
	unsigned char status;
	unsigned char reserved;
} sx1272_rw_struc_t;

typedef struct _sx1272_data_struc {
	unsigned int size;
	unsigned char data[0];
} sx1272_data_struc_t;

typedef struct _sx1272_freq_chan_struc {
	unsigned int base_freq;
	unsigned int chan;
} sx1272_freq_chan_t;

enum {
	IOCTL_SX1272_INIT			= 	_IOR('L',100, int),
	IOCTL_SX1272_GET_STATUS		= 	_IOR('L',101, int),
	/* Basic functions */
	IOCTL_SX1272_REG_WRITE		= 	_IOW('L',102, sx1272_rw_struc_t),
	IOCTL_SX1272_REG_READ_DELTA	= 	_IOWR('L',103, sx1272_rw_struc_t),
	IOCTL_SX1272_RESET			=   _IOW('L',104, int),
	IOCTL_SX1272_RF_SWITCH		=	_IOW('L',105, int),
	/* Advanced functions */
	IOCTL_SX1272_TX_DATA		=	_IOW('L',106, sx1272_data_struc_t ),
	IOCTL_SX1272_WAIT_RX		=	_IOR('L',107, int),
	/* Set frequency or channel */
	IOCTL_SX1272_SET_FREQ_N_CHAN=	_IOW('L',110, sx1272_freq_chan_t ),
};

#endif