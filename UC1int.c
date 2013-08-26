#include <msp430.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <i2c.h>

//buffers for UCA1 UART
struct UART_Tx UCA1_TxBuf;
struct UART_Rx UCA1_RxBuf;

//I2C structure for data passing
I2C_STAT I2C_stat;

//UART TX ISR called to transmit UART data
void UC1_TX(void) __ctl_interrupt[USCIAB1TX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
//=============[UART Transmit Handler]===============
  if(flags&UCA1TXIFG){
    unsigned char c;
    if (ctl_byte_queue_receive_nb(&UCA1_TxBuf.queue,&c)==0){
      //buffer empty disable TX
      UCA1_TxBuf.done=1;
      UC1IFG&=~UCA1TXIFG;
    }else{
      //send char to UART
      UCA1TXBUF=c;
    }
  }
//==============================================================
//===============[I2C data Tx/Rx handler]================
  //check if data was received
  if(flags&UCB1RXIFG){
    //receive data
    I2C_stat.rx.ptr[I2C_stat.rx.idx]=UCB1RXBUF;
    //increment index
    I2C_stat.rx.idx++;
    //check if this is the 2nd to last byte
    if(I2C_stat.rx.len==(I2C_stat.rx.idx+1)){
      if(I2C_stat.mode==I2C_RXTX){
        //set transmit mode
        UCB1CTL1|=UCTR;
        //generate repeated start condition
        UCB1CTL1|=UCTXSTT;
        //enable Tx interrupt
        UC1IE|= UCB1TXIE;
      }else{
        //generate stop condition
        UCB1CTL1|=UCTXSTP;
      }
      //one more interrupt to go
    }
    //check if this was the last byte
    if(I2C_stat.rx.idx>=I2C_stat.rx.len && I2C_stat.mode!=I2C_RXTX){
      //set complete event
      ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
    }
  }
  //check if data needs to be transmitted
  if(flags&UCB1TXIFG){
    //check if there are more bytes
    if(I2C_stat.tx.len>I2C_stat.tx.idx){
      //transmit data
      UCB1TXBUF=I2C_stat.tx.ptr[I2C_stat.tx.idx++];
    }else{
      if(I2C_stat.mode==I2C_TXRX){
        //set receive mode
        UCB1CTL1&=~UCTR;
        //generate start condition
        UCB1CTL1|=UCTXSTT;
        //clear interrupt flag
        UC1IFG&= ~UCB1TXIFG;
        //clear Tx enable
        UC1IE&= ~UCB1TXIE;
        //enable Rx interrupt
        UC1IE|= UCB1RXIE;
        //one byte receive needs special treatment
        if(I2C_stat.rx.len==1){
          //set complete event
          ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
        }
      }else{
        //generate stop condition
        UCB1CTL1|=UCTXSTP;
        //clear interrupt flag
        UC1IFG&= ~UCB1TXIFG;
        //set complete event
        ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
      }
    }
  }
//==============================================================
}

// receive UART ISR
void UC1_rx(void) __ctl_interrupt[USCIAB1RX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
  unsigned char I2Cstate=UCB1STAT;
//==============[UART Receive Handler]==========================
  if(flags&UCA1RXIFG){
    //read a byte from UART
    unsigned char c=UCA1RXBUF;
    //put byte in queue, if no room too darn bad
    ctl_byte_queue_post_nb(&UCA1_RxBuf.queue,c);
    //TODO: raise error if no room
  }
//==============================================================
//=================[I2C Status Handler]=============================
  if(I2Cstate&UCNACKIFG){
    //Not-Acknowledge received
#ifndef SCCB
    //clear Tx interrupt flag see USCI25 in "MSP430F261x, MSP430F241x Device Erratasheet (Rev. J)"
    UC1IFG&= ~UCB1TXIFG;
    //set NACK error event
    ctl_events_set_clear(&I2C_stat.events,I2C_EV_ERR_NACK,0);
#endif
    //clear interrupt flag
    UCB1STAT&=~UCNACKIFG;
#ifndef SCCB
    //disable I2C Tx and Rx Interrupts
    UC1IE&=~(UCB1TXIE|UCB1RXIE);
    //generate stop condition
    UCB1CTL1|=UCTXSTP;
#endif
  }
  if(I2Cstate&UCSTPIFG){
    //Stop condition received, not used in master mode
  }
  if(I2Cstate&UCSTTIFG){
    //Start condition received, not used in master mode
  }
  if(I2Cstate&UCALIFG){
    //Arbitration Lost, not used in master mode
  }
//==================================================================
}
