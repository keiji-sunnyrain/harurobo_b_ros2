#include "can_node/harurobo_can_id.h"
#include "can_node/can_node.hpp"
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "std_msgs/msg/u_int16.hpp"

Can_Node::Can_Node() : Node("can_node"){
  RCLCPP_INFO(this->get_logger(),"Can node start");
  running_ = true;
  publisher_ =
        this->create_publisher<std_msgs::msg::UInt16>("can_data",10);
  subscription_ =
        this->create_subscription<std_msgs::msg::UInt16>("opencv_data",10,std::bind(&Can_Node::callback,this,std::placeholders::_1));
  rpi_spi_set();
  RCLCPP_INFO(this->get_logger(),"spi done");
  worker_ = std::thread(&Can_Node::can_thread, this);
}

Can_Node::~Can_Node(){
  RCLCPP_INFO(this->get_logger(),"Can node end");
  running_ = false;
  if (worker_.joinable()){
    worker_.join();
  }
  RCLCPP_INFO(this->get_logger(),"Thread end");
}

void Can_Node::can_thread(){
  printf("thred start\r\n");
  MCP2517FD_set();
  printf("setted!\r\n");
  std_msgs::msg::UInt16 tx_msg;
  publisher_->publish(tx_msg);
  while (1){
    // 受信:撮影,最も左の色送信0x01
    // 送信:青0x01 橙0x02 黄0x03
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
    tx_msg.data=0x00FF;
    can_read_data[0]=0x00;
    cv_flag = 0;
    can_R(1);
    if ((can_read_data[0]&0b00000001)==0b00000001){
      printf("read!\r\n");
      publisher_->publish(tx_msg);
      while (cv_flag != 1){}
      can_send_data[1] = 0x05;
      can_send_data[0] = cal_data_send;
      can_T(MOTORDRIVER4_RUN,2);
      can_read_data[0]=0x00;
      cv_flag = 0;
    }
    // if (((can_read_data[0]>>1)&0x01)==0x01){
    //   while (cv_flag != 1){}
    //   can_send_data[1] = 0x05;
    //   can_send_data[0] = cal_data;
    //   can_T(MOTORDRIVER4_RUN,2);
    //   can_read_data[0]=0x00;
    //   cv_flag = 0;
    // }
  }
}

void Can_Node::callback(const std_msgs::msg::UInt16::SharedPtr msg){
  uint16_t rx_data = msg->data;
  int ball_count = (rx_data&0b0000000000000111);
  cal_data = (rx_data>>14)&0b11;
  switch (cal_data&0b11){
  case 0b01:
    printf("B\r\n");
    cal_data_send = 0x01;
    break;
  case 0b10:
    cal_data_send = 0x02;
    printf("O\r\n");
    break;
  case 0b11:
    cal_data_send = 0x03;
    printf("Y\r\n");
    break;
  default:
    cal_data_send = 0x02;
    printf("NO\r\n");
    break;
  }
  cv_flag = 1;
}

void Can_Node::rpi_spi_set(){
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
      std::cerr << "SPI open failed\n";
      return;
    }
    // SPIモード
    uint8_t mode = SPI_MODE_0;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    // ビット幅
    uint8_t bits = 8;
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    // SPIクロック
    uint32_t speed = 10000000;
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    // SPI転送構造体の設定
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx_data;
    tr.rx_buf = (unsigned long)rx_data;
    // tr.len = sizeof(tx_data);
    tr.speed_hz = 10000000;
    tr.bits_per_word = 8;
}

void Can_Node::MCP2517FD_spi(uint8_t comand,uint16_t address,int data_langh){
  // MCP2517FD通信用関数 グローバル変数更新
  int send_langh = data_langh + 2; //扱うbyte数
  tx_data[0] = ((comand << 4) & 0b11110000);
  tx_data[0] |= ((address >> 8) & 0b00001111);
  tx_data[1] = address & 0xFF;
  for (int i = 0; i < data_langh; i++){
    tx_data[2 + i] = send_data[i];
  }
  tr.len = send_langh;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

void Can_Node::MCP2517FD_spi_write(uint16_t address,int data_langh){
  // send_data書き込み
  MCP2517FD_spi(WRITE, address, data_langh);
}

void Can_Node::MCP2517FD_spi_read(uint16_t address,int data_langh){
  // レジスタ全体読み込み data_langh:バイト形式
  for (int i = 0; i < data_langh; i++){
    send_data[i] = DUMMY;
  }
  MCP2517FD_spi(READ, address, data_langh);
  for (int i = 0; i < data_langh; i++){
    read_data[i] = rx_data[i+2];
  }
}

void Can_Node::copy_data(int data_langh){
  //データコピー
  for (int i = 0; i < data_langh; i++){
    send_data[i] = read_data[i];
  }
}

void Can_Node::MCP2517FD_set(){
  // MCP2517FD初期設定
  tx_data[1] = 0x00;
  tx_data[0] = 0x00;
  tr.len = 2;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);//リセット
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printf("set_start\r\n");
  do {
    MCP2517FD_spi_read(C1CON,4);
    print_byte_binary(read_data[3]);
    print_byte_binary(read_data[2]);
    print_byte_binary(read_data[1]);
    print_byte_binary(read_data[0]);
    printf("\r\n");
  }while (((read_data[2] >> 5) & 0x07) != 0b100); //OPMOD確認
  printf("done1\r\n");
  MCP2517FD_spi_read(OSC,4);//クロック設定読む
  copy_data(4);
  send_data[0] &= ~(1 << 0);//PLLEN=0
  send_data[0] &= ~(1 << 4);//SCLKDIV=0 
  send_data[0] &= ~(1 << 2);//OSCDIS=0
  MCP2517FD_spi_write(OSC,4);//クロック設定変える
  do{
    MCP2517FD_spi_read(OSC,4);
  }while (((read_data[1] >> 2) & 0x01) != 1);//OSCRDY確認
  MCP2517FD_spi_read(C1NBTCFG,4);//ビットタイミング設定読む
  copy_data(4);

  send_data[3] = 0;//BRP(brp-1)
  send_data[2] = 62;//TSEG1=63
  send_data[1] &= 0b10000000;
  send_data[0] &= 0b10000000;
  send_data[1] |= 15 & 0x7F;//TSEG2=16
  send_data[0] |= 15 & 0x7F;//SJW=16
  MCP2517FD_spi_write(C1NBTCFG,4);//ビットタイミング設定変える
  MCP2517FD_spi_read(C1CON,4);//C1CON設定
  copy_data(4);
  send_data[2] &= ~(1 << 4);//TXQEN無効
  send_data[2] &= ~(1 << 3);//STEF無効
  MCP2517FD_spi_write(C1CON,4);//C1CON設定

  MCP2517FD_spi_read(C1FIFOCON1,4);//FIFO1設定
  copy_data(4);
  send_data[0] |= (1 << 7);//TXEN(送信)
  send_data[3] &= ~(0b11111);//FSIZE(1)=0b00000
  send_data[3] &= ~(0b111 << 5);//PLSIZE(8byt)0b000
  send_data[1] &= ~(1 << 1);//TXREQ(0)
  MCP2517FD_spi_write(C1FIFOCON1,4);//FIFO1設定
  printf("done2\r\n");

  MCP2517FD_spi_read(C1FIFOCON2,4);//FIFO2設定
  copy_data(4);
  send_data[0] &= ~(1 << 7);//TXEN(受信)
  send_data[3] &= ~(0b11111);//FSIZE(1)=0b00000
  send_data[3] &= ~(0b111 << 5);//PLSIZE(8byt)0b000
  MCP2517FD_spi_write(C1FIFOCON2,4);//FIFO2設定
  printf("done3\r\n");

  //マスク,フィルタ設定
  for (int i = 0; i < 4; i++){
    send_data[i] = 0x00;//send_data初期化
  }
  send_data[3] &= (0 << 6);//MIDE=0(比較しない)
  send_data[1] |= 0b00000111;
  send_data[0] |= 0b11111111;//全ビット比較
  MCP2517FD_spi_write(C1MASK0,4);//マスク設定
  for (int i = 0; i < 4; i++){
    send_data[i] = 0x00;//send_data初期化
  }
  send_data[3] &= ~(1 << 6);//EXIDE=0(標準)
  send_data[1] |= ((RASPI5 >> 8)&0b00000111);
  send_data[0] |= ((RASPI5 >> 0)&0b11111111);//フィルタID定義11111111111
  MCP2517FD_spi_write(C1FLTOBJ0,4);//フィルタ設定
  for (int i = 0; i < 4; i++){
    send_data[i] = 0x00;//send_data初期化
  }
  send_data[0] = 0b10000000;//フィルタ0有効化FLTN0
  send_data[0] |= 0x02;//FIFO2割り当て
  MCP2517FD_spi_write(C1FLTCON1,4);//フィルタ有効化


  MCP2517FD_spi_read(C1CON,4);//Mode変更
  copy_data(4);
  // send_data[3] &= ~(0b00000111);//REQOP(000)Normal mode
  send_data[3] &= ~(0b00000111);//REQOP(110)CAN2.0
  send_data[3] |= 0b00000110  ;
  MCP2517FD_spi_write(C1CON,4);//Mode変更
  do{
    MCP2517FD_spi_read(C1CON,4);
  }while (((read_data[2] >> 5) & 0x07) != 0b110);
  printf("set_done\r\n");
}

void Can_Node::can_T(uint16_t can_id,int can_data_langh){
  // can送信
  do{
    MCP2517FD_spi_read(C1FIFOSTA1,4);
    printf("no_space\r\n");
  }while (((read_data[0] >> 0) & 0x01) != 0x01);//TFNRFNIF=1(空きあり確認)
  do{
    MCP2517FD_spi_read(C1FIFOCON1,4);
  }while (((read_data[1] >> 1) & 0x01) != 0x00);//TXREQ=0(送信中でない)
  MCP2517FD_spi_read(C1FIFOUA1, 4);//ユーザーアドレス確認
  fifo1_emp_address = 0x0000;
  fifo1_emp_address |= ((0x04+read_data[1]) & 0b00001111) << 8;
  fifo1_emp_address |= read_data[0];
  for (int i = 0; i < 16; i++){
    send_data[i] = 0x00;//send_data初期化
  }
  send_data[1] |= ((can_id >> 8)&0b00000111);
  send_data[0] |= ((can_id >> 0)&0b11111111);//ID定義
  //T1設定 FDF=0(クラシック) BRS=0 RTR=0 IDE=0(標準ID) DLC=bite数
  send_data[4] = (can_data_langh & 0b00000111);
  // //T1設定 FDF=0(クラシック) BRS=0 RTR=0 IDE=0(標準ID) DLC=bite数2
  // send_data[4] = (0b00000010 & 0xFF);
  for (int i = 0; i < can_data_langh; i++){
    send_data[8+i] = can_send_data[i];
  }

  MCP2517FD_spi_write(fifo1_emp_address, 12);//実データ送信
  MCP2517FD_spi_read(C1FIFOCON1,4);//FIFOポインタ進める
  copy_data(4);
  send_data[1] |= 0x01;//UINC立てる
  MCP2517FD_spi_write(C1FIFOCON1, 4);//FIFOポインタ進める
  MCP2517FD_spi_read(C1FIFOCON1,4);//送信命令
  copy_data(4);
  send_data[1] |= (0x01 << 1);//TXREQ立てる
  MCP2517FD_spi_write(C1FIFOCON1,4);//送信命令
  do{
    MCP2517FD_spi_read(C1FIFOCON1,4);
    printf("not_sended\r\n");
  }while (((read_data[1] >> 1) & 0x01) != 0x00);//TXREQ=0(送信完了確認)
  printf("Sended!\r\n");
}

void Can_Node::can_R(int read_data_lengh){
  // can受信
  MCP2517FD_spi_read(C1FIFOSTA2,4);
  if (((read_data[0] & 0b00000001) == 0x01) || ((read_data[0] & 0b00000100) == 0x04)){//FIFO埋まってるか確認TFNRFNIF=1 or TFERFNIF=1
    printf("read_stare!\r\n");
    MCP2517FD_spi_read(C1FIFOUA2,4);
    // 0x0400+
    fifo2_readable_address = 0x0000;
    fifo2_readable_address |= (((0x04+read_data[1]) & 0b00001111) << 8);
    fifo2_readable_address |= read_data[0];
    // MCP2517FD_spi_read(fifo2_readable_address,12+read_data_lengh);
    MCP2517FD_spi_read(fifo2_readable_address,12);
    for (int i = 0; i < read_data_lengh; i++){
      can_read_data[i] = read_data[i+8];
    }
    printf("Reseved!:\r\n");
    MCP2517FD_spi_read(C1FIFOCON2,4);//FIFOポインタ進める
    copy_data(4);
    send_data[1] |= 0x01;//UINC立てる
    MCP2517FD_spi_write(C1FIFOCON2, 4);//FIFOポインタ進める
    printf("DATA:\r\n");
    print_byte_binary(can_read_data[0]);
  }
  else{
    // printf("no_data\r\n");
  }
}

void Can_Node::print_byte_binary(uint8_t value){
  //bit表示
  for(int i = 7; i >= 0; i--){
    printf("%d", (value >> i) & 0x01);
  }
  printf("\r\n");
}