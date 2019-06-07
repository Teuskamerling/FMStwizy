/**
  ******************************************************************************
  * @file    stm32f4x7_eth_bsp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   STM32F4x7 Ethernet hardware configuration.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright 2019 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "stm32f4xx_conf.h"
#include "stm32f4x7_eth.h"
#include "stm32f4x7_eth_bsp.h"
#include "lwip/netif.h"
#include "enet.h"
#include "lwip/dhcp.h"
#include "arch/sys_arch.h"
#include "lwip/sys.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define ETH_LINK_TASK_STACK_SIZE		      (configMINIMAL_STACK_SIZE)
#define ETH_LINK_TASK_PRIORITY		        (tskIDLE_PRIORITY + 2)
#define emacBLOCK_TIME_WAITING_ETH_LINK_IT	( ( CTL_TIME_t ) 100 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ETH_InitTypeDef ETH_InitStructure;
__IO uint32_t  EthStatus = 0;
extern struct netif xnetif;
#ifdef USE_DHCP
  extern __IO uint8_t DHCP_state;
#endif /* LWIP_DHCP */

sys_sem_t ETH_link_xSemaphore = SYS_SEM_NULL;

/* Global Variables ----------------------------------------------------------*/
uint16_t lan8710a_phy_addr=0;

/* Private function prototypes -----------------------------------------------*/
static void ETH_GPIO_Config(void);
static void ETH_NVIC_Config(void);
static void ETH_MACDMA_Config(void);
static int ETH_Find_Phy_Address(uint16_t *addr);


/* Private functions ---------------------------------------------------------*/
/* Forward declarations ------------------------------------------------------*/
extern void Eth_Link_IT_task( void * pvParameters );
extern sys_sem_t s_xSemaphore;

/**
  * @brief  ETH_BSP_Config
  * @param  None
  * @retval None
  */
void ETH_BSP_Config(void)
{

  /***************************************************************************
    NOTE: 
         When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt 
         priority should be the highest one.
  *****************************************************************************/
  
  /* Configure the GPIO ports for ethernet pins */
  ETH_GPIO_Config();

  /* Config NVIC for Ethernet */
  ETH_NVIC_Config();
  
  /* Configure the Ethernet MAC/DMA */
  ETH_MACDMA_Config();

  /* Get Ethernet link status*/
  if(ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_SR) & PHY_Linked_Status)
  {
    EthStatus |= ETH_LINK_FLAG;
  }

  /* Configure the PHY to generate an interrupt on change of link status */
  Eth_Link_PHYITConfig(lan8710a_phy_addr);

  /* Configure the EXTI for Ethernet link status. */
  Eth_Link_EXTIConfig(); 

  if (ETH_link_xSemaphore == SYS_SEM_NULL)
  {
    /* create binary semaphore used for ETH_link handling */
    sys_sem_new(&ETH_link_xSemaphore, 0);
  }

  /* create the task that handles the ETH_link */
  sys_thread_new("E_link", Eth_Link_IT_task , (void *)((uint32_t)lan8710a_phy_addr), ETH_LINK_TASK_STACK_SIZE, ETH_LINK_TASK_PRIORITY);
}


/**
  * @brief  Configures the Ethernet Interface
  * @param  None
  * @retval None
  */
static void ETH_MACDMA_Config(void)
{  
  /* Enable ETHERNET clock  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |
                        RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);

  /* Reset ETHERNET on AHB Bus */
  ETH_DeInit();

  /* Software reset */
  ETH_SoftwareReset();

  /* Wait for software reset */
  while (ETH_GetSoftwareResetStatus() == SET);

  /* ETHERNET Configuration --------------------------------------------------*/
  /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
  ETH_StructInit(&ETH_InitStructure);

  /* find the Phy address */
  ETH_Find_Phy_Address(&lan8710a_phy_addr);



  /* Fill ETH_InitStructure parametrs */
  /*------------------------   MAC   -----------------------------------*/
  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
// Does not work with smsc 8710a
//  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Disable;
//  ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
//  ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;

  ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
  ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
  ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
  ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
  ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
  ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/  
  
  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode: 
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum, 
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
  ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
  ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;

  ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
  ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
  ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
  ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
  ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

  /* Configure Ethernet */
  EthStatus = ETH_Init(&ETH_InitStructure, lan8710a_phy_addr);
  
  /* Enable the Ethernet Rx Interrupt */
  ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void ETH_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |	RCC_AHB1Periph_GPIOB | 
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG, ENABLE);

  


  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  

    SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII);
  
  /* Only Mode RMII with Olimex STM32-E407 */
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
  
  /* ETHERNET pins configuration */
  /* PA
    ETH_RMII_REF_CLK: PA1
    ETH_RMII_MDIO: PA2
    ETH_RMII_MDINT: PA3
    ETH_RMII_CRS_DV: PA7
   */

  /* Configure PA1, PA2, PA3 and PA7*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect PA1, PA2, PA3 and PA7 to ethernet module*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

  
// ETH_RMII_TX_EN: PG11

  /* Configure PG11*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Connect PG11 to ethernet module*/
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_ETH);

  /* PC
    ETH_RMII_MDC: PC1
    ETH_RMII_RXD0: PC4
    ETH_RMII_RXD1: PC5
  */

  /* Configure PC1, PC4 and PC5*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect PC1, PC4 and PC5 to ethernet module*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

  /* PG
    ETH_RMII_TXD0: PG13
    ETH_RMII_TXD1: PG14
  */
/* Ethernet pins configuration ************************************************/
   /*
        ETH_MII_REF_CLK ------------------> PA1
        ETH_MDIO -------------------------> PA2
        ETH_MII_MDINT --------------------> PA3
        ETH_MDC --------------------------> PC1
        ETH_PPS_OUT ----------------------> 
        ETH_MII_CRS ----------------------> PA7
        ETH_MII_COL ----------------------> 
        ETH_MII_RX_ER --------------------> 
        ETH_MII_RXD2 ---------------------> 
        ETH_MII_RXD3 ---------------------> 
        ETH_MII_TX_CLK -------------------> 
        ETH_MII_TXD2 ---------------------> 
        ETH_MII_TXD3 ---------------------> 
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> 
        ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> 
        ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PG11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
   */

  /* Configure PG13 and PG14*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Connect PG13 and PG14 to ethernet module*/
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_ETH);
}

/**
  * @brief  Configures and enable the Ethernet global interrupt.
  * @param  None
  * @retval None
  */

void ETH_NVIC_Config(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure; 
  
  /* Enable the Ethernet global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12 ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the PHY to generate an interrupt on change of link status.
  * @param PHYAddress: external PHY address  
  * @retval None
  */
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress)
{
  uint16_t tmpreg = 0;


  /* Read Interrupt Flag register */
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_MIFR);

  /* clear ALTINT */ 
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_MCSR);
  tmpreg &= ~(uint16_t)(PHY_MCSR_ALTINT);
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_MCSR, tmpreg)))
  {
    /* Return ERROR in case of write timeout */
    return ETH_ERROR;
  }


  /* Read Mask register */
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_MIMR);

  /* Enable Interrupt on change of link and energy status */
  tmpreg |= (uint16_t)(PHY_MIMR_LINK_INT_EN |  PHY_MIMR_AUTON_INT_EN);
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_MIMR, tmpreg)))
  {
    /* Return ERROR in case of write timeout */
    return ETH_ERROR;
  }
  /* Return SUCCESS */
  return ETH_SUCCESS;   
}

/**
  * @brief  EXTI configuration for Ethernet link status.
  * @param PHYAddress: external PHY address  
  * @retval None
  */
void Eth_Link_EXTIConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the INT (PA3) Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure INT pin as input floating*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line to INT Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set the EXTI interrupt to priority 1*/
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
* @brief  This function handles Ethernet link status.
* @param  None
* @retval None
*/
void Eth_Link_IT_task( void * pvParameters )
{
  for(;;) {
    // RHB chng: Wait w/o timeout, because Link Up and Down are Signaled by ISR 
    // if (sys_arch_sem_wait( &ETH_link_xSemaphore, emacBLOCK_TIME_WAITING_ETH_LINK_IT))
    sys_arch_sem_wait( &ETH_link_xSemaphore, 0);

    /* Get Ethernet link status*/
    uint16_t sr = ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_SR);
    if( sr & PHY_Linked_Status) {
        /* Set link up */
        netif_set_link_up(&xnetif);
    } else {
        /* Set link down */
        netif_set_link_down(&xnetif);
    }
  } // for
}


/**
  * @brief  Link callback function, this function is called on change of link status.
  * @param  The network interface
  * @retval None
  */
void ETH_link_callback(struct netif *netif)
{
  __IO uint32_t timeout = 0;
  uint32_t tmpreg,RegValue;
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
  uint8_t addr1, addr2, addr3, addr4;


  if(netif_is_link_up(netif))
  {
    /* Restart the autonegotiation */
    if(ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
    {
      /* Reset Timeout counter */
      timeout = 0;

      /* Enable Auto-Negotiation */
      ETH_WritePHYRegister(lan8710a_phy_addr, PHY_BCR, PHY_AutoNegotiation);

      /* Wait until the auto-negotiation will be completed */
      do
      {
        timeout++;
      } while (!(ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));

      /* Reset Timeout counter */
      timeout = 0;

      /* Read the result of the auto-negotiation */
      RegValue = ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_SR);
    
      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
      if((RegValue & PHY_FULL_DUPLEX) != (uint32_t)RESET)
      {
        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
        ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;  
      }
      else
      {
        /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
        ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
      }
      /* Configure the MAC with the speed fixed by the auto-negotiation process */
      if(RegValue & PHY_SPEED_100)
      {
        /* Set Ethernet speed to 100M following the auto-negotiation */ 
        ETH_InitStructure.ETH_Speed = ETH_Speed_100M;      
      }
      else
      {
        /* Set Ethernet speed to 10M following the auto-negotiation */
        ETH_InitStructure.ETH_Speed = ETH_Speed_10M; 
      }

      /*------------------------ ETHERNET MACCR Re-Configuration --------------------*/
      /* Get the ETHERNET MACCR value */  
      tmpreg = ETH->MACCR;

      /* Set the FES bit according to ETH_Speed value */ 
      /* Set the DM bit according to ETH_Mode value */ 
      tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Speed | ETH_InitStructure.ETH_Mode);

      /* Write to ETHERNET MACCR */
      ETH->MACCR = (uint32_t)tmpreg;

      _eth_delay_(ETH_REG_WRITE_DELAY);
      tmpreg = ETH->MACCR;
      ETH->MACCR = tmpreg;
    }

    /* Restart MAC interface */
    ETH_Start();

    EnetGetIpAddress(&addr1, &addr2, &addr3, &addr4);
    IP4_ADDR(&ipaddr, addr1, addr2, addr3, addr4);
    EnetGetNetMask(&addr1, &addr2, &addr3, &addr4);
    IP4_ADDR(&netmask, addr1, addr2, addr3, addr4);
    EnetGetGateway(&addr1, &addr2, &addr3, &addr4);
    IP4_ADDR(&gw, addr1, addr2, addr3, addr4);

    netif_set_addr(&xnetif, &ipaddr , &netmask, &gw);
    
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&xnetif);    

    /* invoke hook function */
    EnetLinkUpHook();
  }
  else
  {
    ETH_Stop();

    /*  When the netif link is down this function must be called.*/
    netif_set_down(&xnetif);

    /* invoke hook function */
    EnetLinkDownHook();

  }
}

static int ETH_Find_Phy_Address(uint16_t *addr)
{
    union {
      uint32_t    HI_LO;
      struct
      {
        uint16_t  LO;
        uint16_t  HI;
      };
    } PHYID;
  for(*addr = 0; 32 > *addr; (*addr)++)
  { 
      // datasheet for the SMSC LAN8710A 
      // page 54/55 --> PHY Identifier 1 and 2
      PHYID.HI = ETH_ReadPHYRegister(*addr,2);  // 0x0007
      PHYID.LO = ETH_ReadPHYRegister(*addr,3) & 0xFFF0;  // 0xC0F0  , 4 LSB vary with silicon revision
      if (0x0007C0F0 == PHYID.HI_LO)
        break;
  }
   

  return (32 >= *addr);
}

/* iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii */   
/**
  * @brief  This function handles Ethernet link status.
  * @param  None
  * @retval None
  */

void EXTI3_IRQHandler(void){
  
  /* Clear the EXTI line 3 pending bit */
  EXTI_ClearITPendingBit(EXTI_Line3);

  uint16_t tmpreg=ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_MIFR);

  /* Check interrupt source : Autonegotiation Complete Flag acts as link up    */
  /* and link down acts as link down                      */
  if(tmpreg & PHY_MIFR_LINK_FLAG) {
    // Reading the Base status register will clear interrupt flag
    ETH_ReadPHYRegister(lan8710a_phy_addr, PHY_SR);
 
  } else if(tmpreg & PHY_MIFR_AUTON_FLAG) {
    // Reading the flag register ( done above ) has already cleared the flag bit
  }

  /* Give the semaphore to wakeup LwIP task */
  sys_sem_signal( &ETH_link_xSemaphore); 
}


/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  /* Frame received */
  if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) {
  /* Clear the interrupt flags. */
  /* Clear the Eth DMA Rx IT pending bits */
    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
    ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
    /* Give the semaphore to wakeup LwIP task */
    sys_sem_signal( &s_xSemaphore);
  }

  if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_NIS) == SET) {
   ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
  }

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
