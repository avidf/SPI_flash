module spi_master(
//由外部控制器提供给master
 input               I_clk       , // 全局时钟
 input               I_rst_n     , // 复位信号，低电平有效
 input               I_rx_en     , //主机给从机接收数据的使能信号，当I_rx_en为1时主机才能从从机接收数据
 input               I_tx_en     , //主机给从机发送数据的使能信号，当I_tx_en为1时主机才能给从机发送数据
 input        [7:0]  I_data_in   , // 要发送的数据

 output  reg  [7:0]  O_data_out  , // 接收到的数据在master中的存储

 output  reg         O_tx_done   , // 是主机给从机发送数据完成的标志位，发送完成后会产生一个高脉冲；
 output  reg         O_rx_done   , // 是主机从从机接收数据完成的标志位，接收完成后会产生一个高脉冲；
 // 四线标准SPI信号定义
 input               I_spi_miso  , // SPI串行输入，用来接收从机的数据
 output  reg         O_spi_sck   , //SPI时钟，实际是由全局时钟产生；之所以要产生这个时钟，是因为全局时钟接在主模块上，
											  //从模块不一定有，从模块的编程需要时钟来确定来的数据，所以由主模块产生一个时钟                                      
 output  reg         O_spi_cs    , // SPI片选信号
 output  reg         O_spi_mosi    // SPI输出，用来给从机发送数据          
);

reg [3:0]   R_tx_state; //用于进行各个状态的辨识
reg [3:0]   R_rx_state;

always @(posedge I_clk or negedge I_rst_n)//经过16个系统时钟发送或者接收八位有效数据
begin
    if(!I_rst_n)
        begin
            R_tx_state  <=  4'd0    ;
            R_rx_state  <=  4'd0    ;
            O_spi_cs    <=  1'b1    ;//片选0有效
            O_spi_sck   <=  1'b0    ;
            O_spi_mosi  <=  1'b0    ;
            O_tx_done   <=  1'b0    ;
            O_rx_done   <=  1'b0    ;
            O_data_out  <=  8'd0    ;//没有I_spi_miso，因为它是从机控制的
        end 
    else if(I_tx_en) // 发送使能信号打开的情况下
        begin
            O_spi_cs    <=  1'b0    ; // 把片选CS拉低
            case(R_tx_state)
                4'd1, 4'd3 , 4'd5 , 4'd7, 4'd9, 4'd11, 4'd13, 4'd15 : //整合奇数状态
                    begin
                        O_spi_sck   <=  1'b1                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end
                4'd0:    // 发送第7位
                    begin
                        O_spi_mosi  <=  I_data_in[7]        ;//主机要发送的数据，把数据发送到mosi线上
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;//没有发送完所以标志为0
                    end
                4'd2:    // 发送第6位
                    begin
                        O_spi_mosi  <=  I_data_in[6]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end
                4'd4:    // 发送第5位
                    begin
                        O_spi_mosi  <=  I_data_in[5]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end 
                4'd6:    // 发送第4位
                    begin
                        O_spi_mosi  <=  I_data_in[4]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end 
                4'd8:    // 发送第3位
                    begin
                        O_spi_mosi  <=  I_data_in[3]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end                            
                4'd10:    // 发送第2位
                    begin
                        O_spi_mosi  <=  I_data_in[2]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end 
                4'd12:    // 发送第1位
                    begin
                        O_spi_mosi  <=  I_data_in[1]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b0                ;
                    end 
                4'd14:    // 发送第0位
                    begin
                        O_spi_mosi  <=  I_data_in[0]        ;
                        O_spi_sck   <=  1'b0                ;
                        R_tx_state  <=  R_tx_state + 1'b1   ;
                        O_tx_done   <=  1'b1                ;//发送完了，发送标志为1
                    end
                default:R_tx_state  <=  4'd0                ;//?????????????
            endcase 
        end
    else if(I_rx_en) // 接收使能信号打开的情况下
        begin
            O_spi_cs    <=  1'b0        ; // 拉低片选信号CS
            case(R_rx_state)
                4'd0, 4'd2 , 4'd4 , 4'd6, 4'd8, 4'd10, 4'd12, 4'd14: //整合偶数状态
                    begin
								O_spi_sck		 <=	1'b0;
								R_rx_state		 <=	R_rx_state+1'b1;
								O_rx_done		 <=	1'b0;
                    end
                4'd1:    // 接收第7位
                    begin                       
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;//没有发送完，所以数据为0
                        O_data_out[7]   <=  I_spi_miso       ; //要从丛机读取的信号，从miso线上接受数据
                    end
                4'd3:    // 接收第6位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[6]   <=  I_spi_miso       ; 
                    end
                4'd5:    // 接收第5位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[5]   <=  I_spi_miso       ; 
                    end 
                4'd7:    // 接收第4位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[4]   <=  I_spi_miso       ; 
                    end 
                4'd9:    // 接收第3位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[3]   <=  I_spi_miso       ; 
                    end                            
                4'd11:    // 接收第2位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[2]   <=  I_spi_miso       ; 
                    end 
                4'd13:    // 接收第1位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b0             ;
                        O_data_out[1]   <=  I_spi_miso       ; 
                    end 
                4'd15:    // 接收第0位
                    begin
                        O_spi_sck       <=  1'b1             ;
                        R_rx_state      <=  R_rx_state + 1'b1;
                        O_rx_done       <=  1'b1             ;//发送完了，数据为1
                        O_data_out[0]   <=  I_spi_miso       ; 
                    end
                default:R_rx_state  <=  4'd0                 ;   
            endcase 
        end    
    else
        begin
            R_tx_state  <=  4'd0;
            R_rx_state  <=  4'd0;
            O_tx_done   <=  1'b0;
            O_rx_done   <=  1'b0;
            O_spi_cs    <=  1'b1;
            O_spi_sck   <=  1'b0;
            O_spi_mosi  <=  1'b0;
            O_data_out  <=  8'd0;
        end      
end

endmodule
