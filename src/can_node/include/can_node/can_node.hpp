#pragma once
#include "rclcpp/rclcpp.hpp"

class Can_Node : public rclcpp::Node{
public:
    Can_Node();
    ~Can_Node();
private:
    void rpi_spi_set();
    void MCP2517FD_spi(uint8_t comand,uint16_t address,int data_langh);
    void MCP2517FD_spi_write(uint16_t address,int data_langh);
    void MCP2517FD_spi_read(uint16_t address,int data_langh);
    void copy_data(int data_langh);
    void MCP2517FD_set();
    void can_T(uint16_t can_id,int can_data_langh);
    void can_R(int read_data_lengh);
    void print_byte_binary(uint8_t value);

    uint8_t rx_data[20];
    uint8_t tx_data[20];
    uint8_t send_data[22];//書き込みデータ
    uint8_t read_data[22];//読み込み込みデータ

    uint16_t fifo1_emp_address;//FIFO1空アドレス 12bit
    uint8_t can_send_data[10];//CAN送信データ 8byte
    uint16_t fifo2_readable_address;//FIFO2読むべきアドレス 12bit
    uint8_t can_read_data[10];//CAN受信データ 8byte;

    struct spi_ioc_transfer tr;
    int spi_fd;
};