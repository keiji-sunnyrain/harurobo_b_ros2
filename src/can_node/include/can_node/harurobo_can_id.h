// コマンド定義
#define RESET 0x00
#define READ 0x03
#define WRITE 0x02

// アドレス定義
#define C1CON 0x00
#define OSC 0x0E00
#define C1NBTCFG 0x0004
// FIFO1送信用
#define C1FIFOCON1 0x005C
#define C1FIFOSTA1 0x0060
#define C1FIFOUA1 0x0064
// FIFO2受信用
#define C1FIFOCON2 0x0068
#define C1FIFOSTA2 0x006C
#define C1FIFOUA2 0x0070
// フィルタ,マスク0
#define C1FLTOBJ0 0x01F0
#define C1MASK0 0x01F4
#define C1FLTCON1 0x01D0

//配列構成
//send_data[3]=31-24
//send_data[2]=23-16
//send_data[1]=15-8
//send_data[0]=7-0

// データ定義
#define DUMMY 0x00

// CAN_ID定義 11bit(16bit定義)
#define MOTORDRIVER4_LONCH 0b0000010111100001
#define MOTORDRIVER4_RUN 0b0000010111100010
#define SERVO 0b0000010111110001
#define ROBOMAS 0b0000010111000001
#define LED 0b0000010111010001
#define RASPI5 0b0000011111110001
#define I2C 0b0000010010010001
#define ANALOG 0b0000010011110001