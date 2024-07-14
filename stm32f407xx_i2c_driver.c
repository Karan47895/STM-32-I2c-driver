/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 16, 2024
 *      Author: Karan Patel
 */


#include "stm32f407xx_i2c_driver.h"


static  void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static  void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static  void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static  void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1|=(1<<I2C_CR1_START);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr<<1;
	SlaveAddr&=~(1);
	pI2Cx->DR=SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr<<1;
	SlaveAddr|=1;
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummyRead;
	 /// check for the device mode
	if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
		//device is in master mode
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize==1){
			   // first disable the acking.
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				//clear the ADDR flag.
				dummyRead=pI2CHandle->pI2Cx->SR1;
				dummyRead=pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else {
			dummyRead=pI2CHandle->pI2Cx->SR1;
			dummyRead=pI2CHandle->pI2Cx->SR2;
		   (void)dummyRead;
		}
	}
	else {
		dummyRead=pI2CHandle->pI2Cx->SR1;
		dummyRead=pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1|=(1<<I2C_CR1_STOP);
}



void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2&=~(1<<I2C_CR2_ITBUFEN);
	// Implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2&=~(1<<I2C_CR2_ITEVTEN);

   pI2CHandle->RxLen=0;
   pI2CHandle->RxSize=0;
   pI2CHandle->TxRxState=I2C_READY;
   pI2CHandle->pTxBuffer=NULL;
   if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE){
   I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
   }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2&=~(1<<I2C_CR2_ITBUFEN);
	// Implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2&=~(1<<I2C_CR2_ITEVTEN);

   pI2CHandle->TxLen=0;
   pI2CHandle->TxRxState=I2C_READY;
   pI2CHandle->pTxBuffer=NULL;

}






void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi==ENABLE){
	if(pI2Cx==I2C1){
	     I2C1_PCLK_EN();
	}
	else if(pI2Cx==I2C2){
		I2C2_PCLK_EN();
	}
	else if(pI2Cx==I2C3){
		I2C3_PCLK_EN();
	}
	}

	else{
		if(pI2Cx==I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx==I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx==I2C3){
			I2C3_PCLK_DI();
		}
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack.
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


void I2C_Init(I2C_Handle_t *pI2CHandle){

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
   uint32_t temp=0;
   //ack control bit
   temp|=pI2CHandle->I2C_Config.I2C_ACKControl<<10;
   pI2CHandle->pI2Cx->CR1=temp;

   //COnfigure the freq	field of cr2

   temp=0;
   temp|=RCC_GetPCLK1Value()/1000000U;
   pI2CHandle->pI2Cx->CR2=(temp&0x3F);

   // program the device own address
    temp=0;
   temp|=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
   temp|=(1<<14);
   pI2CHandle->pI2Cx->OAR1=(temp&0x3F);

   //CCR calculations.
   uint16_t ccr_value=0;
   temp=0;

   if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM){
	   //mode is standard mode
	   ccr_value=(RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	   temp|=(ccr_value&0xFFF);
   }
   else {
	   //mode is fast mode
	   temp|=(1<<15);
	   temp|=(pI2CHandle->I2C_Config.I2C_FMDutyCycle<<14);
	   if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_2){
		   ccr_value=(RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	   }
	   else {
		   ccr_value=(RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	   }
	   temp|=(ccr_value&0xFFF);
   }

   pI2CHandle->pI2Cx->CCR=temp;
   //TRISE configuration.
   if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM){
  	   //mode is standard mode
  	   temp=(RCC_GetPCLK1Value()/1000000U)+1;
     }
   else {
	   temp=((RCC_GetPCLK1Value()*300)/1000000000U)+1;
   }
   pI2CHandle->pI2Cx->TRISE=(temp&0x3f);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx){




}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t flagname){
	if(pI2Cx->SR1&flagname){
		return 1;
	}
	else return 0;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	   if(EnorDi==ENABLE){
		   pI2Cx->CR1=(1<<0);	   }
	   else {
		   pI2Cx->CR1&=~(1<<0);	;
	   }
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){
	//1_Genrqate the start condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2)Confirm that start  genration is completed by checking the SB flag in ths SR!
	//Note:Until SB is cleared the clock will be stretched to low.
	 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG)));
	//3)Send the address of the slave with r/nw bit  set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);
	//4)Confirem the adddress phase is completed by checking the ADDR flag in the SR!
	 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)));
	 //5)Clear the ADDR flag according to its software sequence.
	 I2C_ClearADDRFlag(pI2CHandle);
	 // Send data until the len becomes zero.
	 while(Len>0){
	   while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG)));
	   pI2CHandle->pI2Cx->DR=*pTxbuffer;
	   pTxbuffer++;
	   Len--;
	 }
	//7)when len becomes zero wait for TXE=1 and BTF=1 before genrating the STOP condition
	 //Note:TXE=1,BTF=1	,means that both SR and DR are empty and next transmission should begin
	 //when BTF=1,SCL will be stretched to LOW
	 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG)));
	 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG)));
	 //8) Genrate STOP condition and master need not to wait for the completion of stop condition.
	    //Note:genrating STOP,automatically clears the BTF.
	 if(Sr==0){
      I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	 }
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr){
	//1)Genrate the start condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2)confirm that start condition is completed by checking the SB flag in the SR!.
	//note:until SB is cleared SCL will be stretched to low.
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG)));
	//3) Send the address of the slave with r/nw bit set to 1.
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);
	//4)wait until the address phase is completed by checking the ADDR flag in SR!.
	 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)));

	 if(Len==1){
		 //Disable Acking
		 I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

	    //CLEAR THE ADDR flag
		 I2C_ClearADDRFlag(pI2CHandle);
		 // wait unitl RXNE becomes 1.
		 while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG)));
		 // genrate the stop condition.
		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		 *pRxBuffer=pI2CHandle->pI2Cx->DR;
	 }
	 if(Len>1){
		 //CLEAR THE ADDR flag
	    I2C_ClearADDRFlag(pI2CHandle);

      while(Len>0){
    	  // wait until RXNE becomes 1.
    	  while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG)));

    	  if(Len==2){
    		  // Disable acking
    		 I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
    		 // Genertae stop condition.
    		 if(Sr==0){
    		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    		 }
    	  }

    	  *pRxBuffer=pI2CHandle->pI2Cx->DR;
    	  pRxBuffer++;
    	  Len--;
      }
	 }
	 if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE){
	 I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	 }
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	 if(EnorDi==ENABLE){
			 if(IRQNumber<=31){
				//Program ISER0 register.
	           *NVIC_ISER0|=(1<<IRQNumber);
			 }
			 else if(IRQNumber>31 &&IRQNumber<64){
				 //Program ISER1 register.
			  *NVIC_ISER1|=(1<<(IRQNumber)%32);
			 }
			 else if(IRQNumber>=64 &&IRQNumber<96){
				 //Program ISER2 register
				 *NVIC_ISER2|=(1<<(IRQNumber)%64);
			 }
		 }
		 else
		 {
			 if(IRQNumber<=31){
				 *NVIC_ICER0|=(1<<IRQNumber);
			   }
	         else if(IRQNumber>31 &&IRQNumber<64){
	        	 *NVIC_ICER1|=(1<<(IRQNumber)%32);
			   }
			 else if(IRQNumber>=64 &&IRQNumber<96){
				 *NVIC_ICER2|=(1<<(IRQNumber)%64);
			   }
		 }

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data){
	pI2Cx->DR=data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return pI2Cx->DR;
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//First found the ipr register.
		// Lower 4 bits are not applicable for every register.
	     uint8_t iprx=IRQNumber/4;
	     uint8_t iprx_section=IRQNumber%4;
	     uint8_t shift_amount=(8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	     *(NVIC_PR_BASE_ADDR+(4*iprx))|=(IRQPriority<<shift_amount);
}

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(I2C1);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//interrupt handling for both master and slave mode of a device.
	uint8_t temp1,temp2,temp3;
	temp1=pI2CHandle->pI2Cx->CR2&(1<<I2C_CR2_ITEVTEN);
	temp2=pI2CHandle->pI2Cx->CR2&(1<<I2C_CR2_ITBUFEN);

	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_SB);
	//1) Handle for interrupt generated by SB event
	//Note:SB flag applicable only in master mode
	if(temp1&&temp3){
		//the interrupt is genrated by SB event
		//this block will not be executed if the device is in the slave mode
		// in this block lets execute the address phase.
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2) Handle for interrupt generated by ADDR event

	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_ADDR);
	if(temp1&&temp3){
			//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	//3) Handle for interrupt generated by BTF event
	//Note:SB flag applicable only in master mode
	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_BTF);
		if(temp1&&temp3){
		//BTF flag is set
		//make sure that TXE is also set.
		  if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			  //BTF=1,TXE=1
			  //Generate stop condition
			  if(pI2CHandle->TxLen==0){
			  if(pI2CHandle->Sr==0){
			  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			  }
			  //reset all the member elements of the handle structure.
			  I2C_CloseSendData(pI2CHandle);
			  I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
		  }
		  }
		  else if(pI2CHandle->TxRxState==SPI_BUSY_IN_RX){}
		  }
	//3) Handle for interrupt generated by STOPF event.
	//Note:SB flag applicable only in master mode
	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_STOPF);
	if(temp1&&temp3){
	//STOPF flag is set
	//clear the stopf(i.read SR1 2) write to CR1)
		pI2CHandle->pI2Cx->CR1|=0x0000;
		//notify the application that STOP is generated by the master.
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}
	//3) Handle for interrupt generated by TXE event
		//Note:SB flag applicable only in master mode
	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_TXE);
	if(temp2&&temp3&&temp1){
	//TXE flag is set
	// We have to do the data transmission.
		if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			if(pI2CHandle->TxLen>0){
			   //1) load the data into the DR.
				//2)decrement the Txlen
				 pI2CHandle->pI2Cx->DR=*(pI2CHandle->pTxBuffer);
				 pI2CHandle->TxLen--;
				 pI2CHandle->pTxBuffer++;
			}
		}
	 }
		else {
			//slave has to send the data.
			if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_TRA))
			{
			I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}
	//3) Handle for interrupt generated by RXNE event
		//Note:SB flag applicable only in master mode
	temp3=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_RXNE);
	if(temp2&&temp3&&temp1){
	    //RXNE flag is set
		if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			//We have to do the data reception.
			if(pI2CHandle->RxSize==1){
				*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxLen--;
			}

			if(pI2CHandle->RxSize>1){
				if(pI2CHandle->RxLen==2){
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}
				// read DR
				*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;
			}
			if(pI2CHandle->RxLen==0){
				//close the i2c data reception.

				//1)generate the stop condition.
				if(pI2CHandle->Sr==0){
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
				//2)close the i2c rx.
				I2C_CloseReceiveData(pI2CHandle);
				//3)notify the application.
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
			}
		}
	}
		else {
			if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_TRA))
			{
			I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==ENABLE){
		//Implement the code to enable ITBUFEN Control Bit
				pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
				pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
			    pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
	else {
		//Implement the code to enable ITBUFEN Control Bit
				pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
				pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
				pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1&=~(1<<I2C_SR1_ARLO);

		//Implement the code to notify the application about the error

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1&=~(1<<I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1&=~(1<<I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1&=~(1<<I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

}
