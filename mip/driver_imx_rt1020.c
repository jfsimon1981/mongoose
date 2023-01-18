// TODO Work in progress:
// Functions: TX, RX, IRQ
// Migrate to full descriptor so we can check frame status, errors, ...

/*
TODO
Check Constraints for MDC:
  Frequency <= 2.5 MHz
  Holdtime  >=  10 ns
*/

/*
Measurements (scope)
  PHY
    Clock pins OK: RMII / 50MHz: ENET_TX_CLK at PHY:XI
    TXEN: route OK.
    RXD0: Signal OK.
*/

/*
FIXME
Trace back TP15 silent with MIP (should rcv outputs at DHCP requests).
*/

/*
References
  SDK
    ENET_SetMacController()
    CLOCK_EnableClock(); // ENET: CCGR1:CG5 (CCM_CCGR1_CG5_SHIFT)
*/

#include "mip.h"

#if MG_ENABLE_MIP && defined(MG_ENABLE_DRIVER_IMXRT1020)
struct imx_rt1020_enet {
volatile uint32_t RESERVED0;          // 0x0
volatile uint32_t EIR;                // 0x4 Interrupt_Event_Register
volatile uint32_t EIMR;               // 0x8 Interrupt_Mask_Register
volatile uint32_t RESERVED1;          // 0xC
volatile uint32_t RDAR;               // 0x10 Receive_Descriptor_Active_Register_Ring_0
volatile uint32_t TDAR;               // 0x14 Transmit_Descriptor_Active_Register_Ring_0
volatile uint32_t RESERVED2[3];       // 0x18
volatile uint32_t ECR;                // 0x24 Ethernet_Control_Register
volatile uint32_t RESERVED3[6];       // 0x28
volatile uint32_t MMFR;               // 0x40 MII_Management_Frame_Register
volatile uint32_t MSCR;               // 0x44 MII_Speed_Control_Register
volatile uint32_t RESERVED4[7];       // 0x48
volatile uint32_t MIBC;               // 0x64
volatile uint32_t RESERVED5[7];       // 0x68
volatile uint32_t RCR;                // 0x84 Receive_control_register
volatile uint32_t RESERVED6[15];      // 0x88
volatile uint32_t TCR;                // 0xC4 Transmit_control_register
volatile uint32_t RESERVED7[7];       // 0xC8
volatile uint32_t PALR;               // 0xE4 Physical_Address_Lower_Register
volatile uint32_t PAUR;               // 0xE8 Physical_Address_Upper_Register
volatile uint32_t OPD;                // 0xEC
volatile uint32_t TXIC0;              // 0xF0
volatile uint32_t TXIC1;              // 0xF4
volatile uint32_t TXIC2;              // 0xF8
volatile uint32_t RESERVED8;          // 0xFC
volatile uint32_t RXIC0;              // 0x100
volatile uint32_t RXIC1;              // 0x104
volatile uint32_t RXIC2;              // 0x108
volatile uint32_t RESERVED9[3];       // 0x10C
volatile uint32_t IAUR;               // 0x118
volatile uint32_t IALR;               // 0x11C
volatile uint32_t GAUR;               // 0x120
volatile uint32_t GALR;               // 0x124
volatile uint32_t RESERVED10[7];      // 0x128
volatile uint32_t TFWR;               // 0x144 Transmit_FIFO_Watermark_Register
volatile uint32_t RESERVED11[14];     // 0x148
volatile uint32_t RDSR;               // 0x180 Receive_Descriptor_Ring_0_Start_Register
volatile uint32_t TDSR;               // 0x184 Transmit_Buffer_Descriptor_Ring_0_Start_Register
volatile uint32_t MRBR[2];            // 0x188 Maximum_Receive_Buffer_Size_Register_Ring_0
volatile uint32_t RSFL;               // 0x190
volatile uint32_t RSEM;               // 0x194
volatile uint32_t RAEM;               // 0x198
volatile uint32_t RAFL;               // 0x19C
volatile uint32_t TSEM;               // 0x1A0
volatile uint32_t TAEM;               // 0x1A4
volatile uint32_t TAFL;               // 0x1A8
volatile uint32_t TIPG;               // 0x1AC
volatile uint32_t FTRL;               // 0x1B0
volatile uint32_t RESERVED12[3];      // 0x1B4
volatile uint32_t TACC;               // 0x1C0
volatile uint32_t RACC;               // 0x1C4
volatile uint32_t RESERVED13[15];     // 0x1C8
volatile uint32_t RMON_T_PACKETS;     // 0x204
volatile uint32_t RMON_T_BC_PKT;      // 0x208
volatile uint32_t RMON_T_MC_PKT;      // 0x20C
volatile uint32_t RMON_T_CRC_ALIGN;   // 0x210
volatile uint32_t RMON_T_UNDERSIZE;   // 0x214
volatile uint32_t RMON_T_OVERSIZE;    // 0x218
volatile uint32_t RMON_T_FRAG;        // 0x21C
volatile uint32_t RMON_T_JAB;         // 0x220
volatile uint32_t RMON_T_COL;         // 0x224
volatile uint32_t RMON_T_P64;         // 0x228
volatile uint32_t RMON_T_P65TO127;    // 0x22C
volatile uint32_t RMON_T_P128TO255;   // 0x230
volatile uint32_t RMON_T_P256TO511;   // 0x234
volatile uint32_t RMON_T_P512TO1023;  // 0x238
volatile uint32_t RMON_T_P1024TO2048; // 0x23C
volatile uint32_t RMON_T_GTE2048;     // 0x240
volatile uint32_t RMON_T_OCTETS;      // 0x244
volatile uint32_t IEEE_T_DROP;        // 0x248
volatile uint32_t IEEE_T_FRAME_OK;    // 0x24C
volatile uint32_t IEEE_T_1COL;        // 0x250
volatile uint32_t IEEE_T_MCOL;        // 0x254
volatile uint32_t IEEE_T_DEF;         // 0x258
volatile uint32_t IEEE_T_LCOL;        // 0x25C
volatile uint32_t IEEE_T_EXCOL;       // 0x260
volatile uint32_t IEEE_T_MACERR;      // 0x264
volatile uint32_t IEEE_T_CSERR;       // 0x268
volatile uint32_t IEEE_T_SQE;         // 0x26C
volatile uint32_t IEEE_T_FDXFC;       // 0x270
volatile uint32_t IEEE_T_OCTETS_OK;   // 0x274
volatile uint32_t RESERVED14[3];      // 0x278
volatile uint32_t RMON_R_PACKETS;     // 0x284
volatile uint32_t RMON_R_BC_PKT;      // 0x288
volatile uint32_t RMON_R_MC_PKT;      // 0x28C
volatile uint32_t RMON_R_CRC_ALIGN;   // 0x290
volatile uint32_t RMON_R_UNDERSIZE;   // 0x294
volatile uint32_t RMON_R_OVERSIZE;    // 0x298
volatile uint32_t RMON_R_FRAG;        // 0x29C
volatile uint32_t RMON_R_JAB;         // 0x2A0
volatile uint32_t RESERVED15;         // 0x2A4
volatile uint32_t RMON_R_P64;         // 0x2A8
volatile uint32_t RMON_R_P65TO127;    // 0x2AC
volatile uint32_t RMON_R_P128TO255;   // 0x2B0
volatile uint32_t RMON_R_P256TO511;   // 0x2B4
volatile uint32_t RMON_R_P512TO1023;  // 0x2B8
volatile uint32_t RMON_R_P1024TO2047; // 0x2BC
volatile uint32_t RMON_R_GTE2048;     // 0x2C0
volatile uint32_t RMON_R_OCTETS;      // 0x2C4
volatile uint32_t IEEE_R_DROP;        // 0x2C8
volatile uint32_t IEEE_R_FRAME_OK;    // 0x2CC
volatile uint32_t IEEE_R_CRC;         // 0x2D0
volatile uint32_t IEEE_R_ALIGN;       // 0x2D4
volatile uint32_t IEEE_R_MACERR;      // 0x2D8
volatile uint32_t IEEE_R_FDXFC;       // 0x2DC
volatile uint32_t IEEE_R_OCTETS_OK;   // 0x2E0
volatile uint32_t RESERVED16[71];     // 0x2E4
volatile uint32_t ATCR;               // 0x400
volatile uint32_t ATVR;               // 0x404
volatile uint32_t ATOFF;              // 0x408
volatile uint32_t ATPER;              // 0x40C
volatile uint32_t ATCOR;              // 0x410
volatile uint32_t ATINC;              // 0x414
volatile uint32_t ATSTMP;             // 0x418
volatile uint32_t RESERVED17[122];    // 0x41C
volatile uint32_t TGSR;               // 0x604
volatile uint32_t TCSR0;              // 0x608
volatile uint32_t TCCR0;              // 0x60C
volatile uint32_t TCSR1;              // 0x610
volatile uint32_t TCCR1;              // 0x614
volatile uint32_t TCSR2;              // 0x618
volatile uint32_t TCCR2;              // 0x61C
volatile uint32_t TCSR3;              // 0x620
};

#undef ENET
#define ENET ((struct imx_rt1020_enet *) (uintptr_t) 0x402D8000u)

#undef BIT_SET
#define BIT_SET(x) ((uint32_t) 1 << (x))

#undef BIT_CLEAR
#define BIT_CLEAR(x) ((uint32_t) ~(1U << (x)))

#define ENET_RXBUFF_SIZE 1536 // 1522 Buffer must be 64bits aligned
#define ENET_TXBUFF_SIZE 1536 // 1522 hence set to 0x600 (1536)
#define ENET_RXBD_NUM          (4)
#define ENET_TXBD_NUM          (4)

static void (*s_rx)(void *, size_t, void *);         // Recv callback
static void *s_rxdata;                               // Recv callback data

 // 1522: max size without preamble nor gap
 // but CRC and possible 802.1Q (VLAN) tag 

 /*
  * Data path
  * DMA -> buffer pointed to by descriptor -> IRQ -> queue -> rx.ptr -> process
  * application -> tx.ptr -> buffer pointed to by descriptor -> DMA -> wire
  */

// ************* Prototypes *************
// *** Utilities
void clock_speed_test(void); // Clock speed test

// ************* PHY *************

enum { PHY_ADDR = 0x02, PHY_BCR = 0, PHY_BSR = 1 };     // PHY constants

void delay(uint32_t);
void delay (uint32_t di) {
  volatile int dno = 0; // Prevent optimization
  for (uint32_t i = 0; i < di; i++)
    for (int j=0; j<20; j++) // PLLx20 (500 MHz/24MHz)
      dno++;
}

void wait_phy_complete(void);
void wait_phy_complete(void) {
  const uint32_t delay_min = 0x00010000;
  const uint32_t delay_max = 0x00100000;
  uint32_t delay_cnt = 0;
  while (!(ENET->EIR & BIT_SET(23)) && \
        (delay_cnt > delay_min) && (delay_cnt < delay_max))
  {delay_cnt++;}

  ENET->EIR |= BIT_SET(23); // MII interrupt clear
}

// ************* PHY read *************

static uint32_t eth_read_phy(uint8_t addr, uint8_t reg) {
  ENET->EIR |= BIT_SET(23); // MII interrupt clear
  uint32_t mask_phy_adr_reg = 0x1f; // 0b00011111: Ensure we write 5 bits (Phy address & register)
  uint32_t phy_transaction = 0x00;
  phy_transaction = (0x1 << 30) \
                  | (0x2 << 28) \
                  | ((uint32_t)(addr & mask_phy_adr_reg) << 23) \
                  | ((uint32_t)(reg & mask_phy_adr_reg)  << 18) \
                  | (0x2 << 16);

  //MG_INFO(("phy_transaction %x", phy_transaction));
  ENET->MMFR = phy_transaction;
  wait_phy_complete();

  // volatile uint32_t phy_read_out = ENET->MMFR;
  //MG_INFO(("phy_read_out 0x%x", phy_read_out));
  return (ENET->MMFR & 0x0000ffff);
}

// ************* PHY write *************

static void eth_write_phy(uint8_t addr, uint8_t reg, uint32_t val) {
  ENET->EIR |= BIT_SET(23); // MII interrupt clear
  uint8_t mask_phy_adr_reg = 0x1f; // 0b00011111: Ensure we write 5 bits (Phy address & register)
  uint32_t mask_phy_data = 0x0000ffff; // Ensure we write 16 bits (data)
  addr &= mask_phy_adr_reg;
  reg &= mask_phy_adr_reg;
  val &= mask_phy_data;
  uint32_t phy_transaction = 0x00;
  phy_transaction = (uint32_t)(0x1 << 30) \
                  | (uint32_t)(0x1 << 28) \
                  | (uint32_t)(addr << 23) \
                  | (uint32_t)(reg  << 18) \
                  | (uint32_t)(0x2 << 16) \
                  | (uint32_t)(val);
  ENET->MMFR = phy_transaction;
  wait_phy_complete();
}

// ************* Global *************

// FEC RX/TX descriptors (Enhanced descriptor not enabled)
// Descriptor buffer structure, little endian

typedef struct enet_bd_struct_def
{
    uint16_t length;  // Data length
    uint16_t control; // Control and status
    uint32_t *buffer; // Data ptr
} enet_bd_struct_t;

__attribute__((section("NonCacheable,\"aw\",%nobits @"))) enet_bd_struct_t rx_buffer_descriptor[(ENET_RXBD_NUM)] __attribute__((aligned((64U))));
__attribute__((section("NonCacheable,\"aw\",%nobits @"))) enet_bd_struct_t tx_buffer_descriptor[(ENET_TXBD_NUM)] __attribute__((aligned((64U))));

uint8_t rx_data_buffer[(ENET_RXBD_NUM)][((unsigned int)(((ENET_RXBUFF_SIZE)) + (((64U))-1U)) & (unsigned int)(~(unsigned int)(((64U))-1U)))] __attribute__((aligned((64U))));
uint8_t tx_data_buffer[(ENET_TXBD_NUM)][((unsigned int)(((ENET_TXBUFF_SIZE)) + (((64U))-1U)) & (unsigned int)(~(unsigned int)(((64U))-1U)))] __attribute__((aligned((64U))));

// Unused vars
void unused(void);
void unused() {
/*
  (void)s_rxdesc;
  (void)s_txdesc;
  (void)s_rxbuf;
  (void)s_txbuf;
  (*s_rx)((void *)0, (size_t)0, (void *)0);
  (void)s_rxdata;
*/
}

void display_registers(void);
static bool mip_driver_imx_rt1020_up(void *userdata);

// ************* Initialization *************

// Initialise driver
// Driver name: imx_rt1020
static bool mip_driver_imx_rt1020_init(uint8_t *mac, void *data) {
  /*
   * TODO
   * Prevent program halt if user's clocks misconfiguration:
   * Before ENET register access attempt, check Clock state
   * - ENET PLL: 500 MHz
   * - PLL6_BYPASS: source from ENET PLL
   * Check ENET Clock (CCM CCGR1 CG5)
   * Enable if necessary
   * Print msg about clock configuration
   * At the moment, done at main program level.
   */

  MG_INFO(("Entered MG MIP driver: i.IMRT1020"));

  /*
    ENET Reset, wait complete
    If software reset (Register 0.15) is used to exit
    power-down mode (Register 0.11 = 1), two
    software reset writes (Register 0.15 = 1) are
    required. The first write clears power-down
    mode; the second write resets the chip and re-
    latches the pin strapping pin values.
  */

  ENET->ECR |= BIT_SET(0);
  while((ENET->ECR & BIT_SET(0)) != 0) {}
  ENET->ECR |= BIT_SET(0);
  while((ENET->ECR & BIT_SET(0)) != 0) {}

  MG_INFO(("Reset complete"));

  // Setup MII/RMII MDC clock divider (<= 2.5MHz).
//  ENET->MSCR = 0x118; // HOLDTIME 2 clk, Preamble enable, MDC MII_Speed Div 0x18
  ENET->MSCR = 0x130; // HOLDTIME 2 clk, Preamble enable, MDC MII_Speed Div 0x18
  eth_write_phy(PHY_ADDR, PHY_BCR, 0x8000); // PHY W @0x00 D=0x8000 Soft reset
  while (eth_read_phy(PHY_ADDR, PHY_BSR) & BIT_SET(15)) {delay(0x5000);} // Wait finished poll 10ms

  // PHY: Start Link
  {
    // Reset and set clock
    eth_write_phy(PHY_ADDR, PHY_BCR, 0x1200); // PHY W @0x00 D=0x1200 Autonego enable + start
    eth_write_phy(PHY_ADDR, 0x1f, 0x8180);    // PHY W @0x1f D=0x8180 Ref clock 50 MHz at XI input
    // Auto configuration
    {
      MG_INFO(("Wait for link up (Autoconf)"));
      uint32_t linkup = 0;
      int linkup_tentatives = 5;
      while (!linkup && linkup_tentatives-- > 0) {
        linkup = mip_driver_imx_rt1020_up(NULL);
        delay(0x500000); // Approx 1s
      }

      if (!linkup) {
        MG_ERROR(("Error: Link didn't come up"));
      }
      else {
          MG_INFO(("Link up"));
        }
        uint32_t bsr = eth_read_phy(PHY_ADDR, PHY_BSR);
        uint32_t bcr = eth_read_phy(PHY_ADDR, PHY_BCR);
        MG_INFO(("bsr: 0x%x", bsr));
        MG_INFO(("bcr: 0x%x", bcr));
    }

    // PHY MII configuration
    {
      {
        uint32_t bcr = eth_read_phy(PHY_ADDR, PHY_BCR);
        bcr &= BIT_CLEAR(10); // Isolation -> Normal
        eth_write_phy(PHY_ADDR, PHY_BCR, bcr);
      }

      MG_INFO(("Link configuration"));
      uint32_t linkup = 0;
      int linkup_tentatives = 5;
      while (!linkup && linkup_tentatives-- > 0) {
        linkup = mip_driver_imx_rt1020_up(NULL);
        delay(0x500000); // Approx 1s
      }

      if (!linkup) {
        MG_ERROR(("Error: Link not ready"));
      }
      else {
          MG_INFO(("Link ready"));


      // PHY INTR
      {
        /*
          15 jabber
          14 receive error
          13 page received
          12 parallel detect fault
          11 link partner acknowledge
          10 link-down
          9 remote fault
          8 link-up
        */
        //eth_write_phy(PHY_ADDR, 0x1b, 0xff00); // Test enable all interrupts
        // uint16_t phy_intr_filter = 0;
        // phy_intr_filter |= 1<<14; // Error many rcvd ?
        // phy_intr_filter |= 1<<14; // Up
        // eth_write_phy(PHY_ADDR, 0x1b, 0x00);   

        /*
          Can't get a filter properly ?
          Check & assert we can filter through required INT mask.
          This may be the sign there's a fault in the Write_Phy function.
          Update:
          Even a read at 1B will trigger many intrp.
        */

        if (1) {
          uint32_t rd=eth_read_phy(PHY_ADDR, 0x1b);
          MG_INFO(("PHY read: 0x%x", rd));
        }

      }

// got: solve the phy rd / set INTR reg.

      }
      uint32_t bsr = eth_read_phy(PHY_ADDR, PHY_BSR);
      uint32_t bcr = eth_read_phy(PHY_ADDR, PHY_BCR);
      MG_INFO(("bsr: 0x%x", bsr));
      MG_INFO(("bcr: 0x%x", bcr));
      // Show actual configuration
      uint32_t phy_1b = eth_read_phy(PHY_ADDR, 0x1b);
      uint32_t phy_1e = eth_read_phy(PHY_ADDR, 0x1e);
      uint32_t phy_1f = eth_read_phy(PHY_ADDR, 0x1f);
      MG_INFO(("phy_1b: 0x%x", phy_1b));
      MG_INFO(("phy_1e: 0x%x", phy_1e));
      MG_INFO(("phy_1f: 0x%x", phy_1f));
    }
  }

  // Link UP, 100BaseTX Full-duplex
  MG_INFO(("PHY init OK"));

  // Disable ENET
  ENET->ECR = 0x0; //  Disable before configuration

  // Configure ENET
  ENET->RCR = 0x05ee4104; // CRCFWD=1 (CRC stripped from frame) + RMII + MII Enable
  
  ENET->TCR = BIT_SET(8) | BIT_SET(2); // Addins (MAC address from PAUR+PALR) + Full duplex enable
//ENET->TFWR = BIT_SET(8); // Acc/to SDK // Store And Forward Enable, 64 bytes (minimize tx latency)

  // ******* *******

  // Configure descriptors and buffers
  // RX

  for (int i = 0; i < ENET_RXBD_NUM; i++) {
    // Wrap last descriptor buffer ptr
    rx_buffer_descriptor[i].control = (i<(ENET_RXBD_NUM-1)?0:BIT_SET(13)) | BIT_SET(11); // (W?)+L
    rx_buffer_descriptor[i].buffer = (uint32_t *)rx_data_buffer[i];
  }

  // ******* *******

  // TX
  // FIXME
  for (int i = 0; i < ENET_TXBD_NUM; i++) {
    // Wrap last descriptor buffer ptr
    tx_buffer_descriptor[i].control = (i<(ENET_RXBD_NUM-1)?0:BIT_SET(13)) \
                                    | BIT_SET(11) | BIT_SET(10); // (W?)+L+TC
    tx_buffer_descriptor[i].buffer = (uint32_t *)tx_data_buffer[i];
  }

  // ******* *******

  // Continue ENET configuration
  ENET->RDSR = (uint32_t)(uintptr_t)rx_buffer_descriptor;
  ENET->TDSR = (uint32_t)(uintptr_t)tx_buffer_descriptor;
  ENET->MRBR[0] = ENET_RXBUFF_SIZE; // Same size for RX/TX buffers

  /*
  // I>?
  //Clear any pending interrupts
  ENET->EIR = 0xFFFFFFFF;
  //Enable desired interrupts
  ENET->EIMR = BIT_SET(27) | BIT_SET(25) | BIT_SET(23); // TXF, RXF, EBERR
  // ENET->EIMR = ENET_EIMR_TXF_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_EBERR_MASK;
  */

  // MAC address filtering
  // (nota order revered / STM32)
  ENET->PAUR = ((uint32_t) mac[4] << 24U) | (uint32_t) mac[5] << 16U;
  ENET->PALR = (uint32_t) (mac[0] << 24U) | ((uint32_t) mac[1] << 16U) |
                 ((uint32_t) mac[2] << 8U) | mac[3];

  // Init Hash tables (mac filtering)
  ENET->IAUR = 0; // Unicast
  ENET->IALR = 0;
  ENET->GAUR = 0; // Multicast
  ENET->GALR = 0;

  // Set ENET Online
  ENET->ECR |= BIT_SET(8); // ENET Set Little-endian + (FEC buffer desc.)
  ENET->ECR |= BIT_SET(1); // Enable

  // RX Descriptor activation
  ENET->RDAR = BIT_SET(24); // Activate Receive Descriptor

  ENET->EIMR = 0xffffffff; // Enable all interrupts

  // MG_INFO(("ENET init OK"));
  // display_registers();

//(void) mac;
  (void) data;
  return true;
}

// ************* IF TX *************

// Transmit frame
static uint32_t s_txno;

static size_t mip_driver_imx_rt1020_tx(const void *buf, size_t len, void *userdata) {

  if (len > sizeof(tx_data_buffer[ENET_TXBD_NUM])) {
//  MG_ERROR(("Frame too big, %ld", (long) len));
    len = 0;  // Frame is too big
  } else if ((tx_buffer_descriptor[s_txno].control & BIT_SET(15))) {
  MG_ERROR(("No free descriptors"));
    // printf("D0 %lx SR %lx\n", (long) s_txdesc[0][0], (long) ETH->DMASR);
    len = 0;  // All descriptors are busy, fail
  } else {
    memcpy(tx_data_buffer[s_txno], buf, len);     // Copy data
    tx_buffer_descriptor[s_txno].length = (uint16_t) len;  // Set data len
    tx_buffer_descriptor[s_txno].control |= (uint16_t)(BIT_SET(10)); // TC (transmit CRC)
//  tx_buffer_descriptor[s_txno].control &= (uint16_t)(BIT_SET(14) | BIT_SET(12)); // Own doesn't affect HW operation
    tx_buffer_descriptor[s_txno].control |= (uint16_t)(BIT_SET(15) | BIT_SET(11)); // R+L (ready+last)
    ENET->TDAR = BIT_SET(24); // Descriptor updated
    // Relevant Descriptor bits: 15(R): Ready, 14/12(TO1/2) Soft own,
    //                            11(L): last in frame, 10(TC): transmis CRC
    // FIXME Frames aresn't process by uDMA engine at this point.
    // TODO
    // __DSB(); // ARM errata 838869 Cortex-M4, M4F, M7, M7F: "store immediate overlapping
                // exception" return might vector to incorrect interrupt.
    if (++s_txno >= ENET_TXBD_NUM) s_txno = 0;
  }
  while (ENET->TDAR) {
/**/
//  Test interrupt handler is indeed called
//  delay(0x100000); // Approx 200ms
//  delay(0x300000); // Approx 600ms
//  delay(0x500000); // Approx 1s
    delay(0xa00000); // Approx 2s
//  delay(0x5000000); // Approx 16s
    static cnt = 0;
    MG_INFO(("cnt == %d", cnt++));
    //
//  eth_write_phy(PHY_ADDR, 0x1f, 0x8180); // Dummy write to trigger an interrupt
//  eth_read_phy(PHY_ADDR, PHY_BSR);
/*
    uint32_t phy_1b = eth_read_phy(PHY_ADDR, 0x1b);
    uint32_t phy_1e = eth_read_phy(PHY_ADDR, 0x1e);
    uint32_t phy_1f = eth_read_phy(PHY_ADDR, 0x1f);
    MG_INFO(("phy_1b: 0x%x", phy_1b));
    MG_INFO(("phy_1e: 0x%x", phy_1e));
    MG_INFO(("phy_1f: 0x%x", phy_1f));
*/
/**/
  }

  // INFO PHY: Green light did'nt come up initially -> Can start/negociate 10M line
  static int i=0;

  MG_INFO(("Passing %d", i++));
  /*(void) buf, (void) len,*/ (void) userdata;
  return len;
}

// ************* RX *************

void ENET_IRQHandler(void);
static uint32_t s_rxno;

void ENET_IRQHandler(void) {

  // Read EIR
  uint32_t eir = ENET->EIR;
  // Display
  MG_INFO(("irq: 0x%x", eir));
  // Clear
  ENET->EIR = 0xffffffff; // Clear interrupts
  return;

  MG_INFO(("irq:rx "));
  if (rx_buffer_descriptor[s_rxno].control & BIT_SET(15)) return;  // Empty? -> exit.
  // Read inframes
  else { // Frame received, loop
  MG_INFO(("irq:rx->data "));
    for (uint32_t i = 0; i < 10; i++) {  // read as they arrive but not forever
      if (rx_buffer_descriptor[s_rxno].control & BIT_SET(15)) break;  // exit when done
      uint32_t len = (rx_buffer_descriptor[s_rxno].length);
      if (s_rx != NULL)
        s_rx(rx_buffer_descriptor[s_rxno].buffer, len > 4 ? len - 4 : len, s_rxdata);
      rx_buffer_descriptor[s_rxno].control |= BIT_SET(15); // Inform DMA RX is empty
      if (++s_rxno >= ENET_RXBD_NUM) s_rxno = 0;
    }
  }
}

// ************* IF Up *************
// >> Implementation OK

// Up/down status
static bool mip_driver_imx_rt1020_up(void *userdata) {
  uint32_t bsr = eth_read_phy(PHY_ADDR, PHY_BSR);

  (void) userdata;
  return bsr & BIT_SET(2) ? 1 : 0;
}

// ************* Set RX CB *************
// Migration? _setrx() -> _mip_rxcb()
// >> Temporary: setrx -> rx (polling)
// >> TODO IRQ interrupt driven

// Set receive callback for interrupt-driven drivers
static void mip_driver_imx_rt1020_setrx(void (*rx)(void *, size_t, void *),
                                   void *rxdata) {
  s_rx = rx;
  s_rxdata = rxdata;
}

// ************* Utilities *************

// Utility to display Actual ENET registers content
void display_registers() {
  // Register read test
  MG_INFO(("ENET->RAFL == %x", ENET->RAFL));
  MG_INFO(("ENET->TAEM == %x", ENET->TAEM));
  MG_INFO(("ENET->TAFL == %x", ENET->TAFL));
  MG_INFO(("ENET->ECR == %x", ENET->ECR));
  MG_INFO(("ENET->RCR == %x", ENET->RCR));
  MG_INFO(("ENET->MMFR == %x", ENET->MMFR));
  MG_INFO(("ENET->PAUR == %x", ENET->PAUR));
  MG_INFO(("ENET->PALR == %x", ENET->PALR));
  MG_INFO(("ENET->OPD == %x", ENET->OPD));
  MG_INFO(("ENET->TIPG == %x", ENET->TIPG));
  MG_INFO(("ENET->FTRL == %x", ENET->FTRL));
  MG_INFO(("ENET->RAEM == %x", ENET->RAEM));
  MG_INFO(("ENET->RAFL == %x", ENET->RAFL));
  MG_INFO(("ENET->ATPER == %x", ENET->ATPER));
  MG_INFO(("ENET->MIBC == %x", ENET->MIBC));
}

bool imx_rt1020_register_changed(void);
bool imx1020_des_changed(void);
bool imx_rt1020_buf_changed(void);
bool imx_rt1020_register_changed(void);

bool imx1020_des_changed(void) {
//if state changed in desc -> notify & print
  return 0;
}
bool imx_rt1020_buf_changed(void) {
//if state changed buff -> notify & print
  return 0;
}
bool imx_rt1020_register_changed(void) {
//if state changed in ref -> notify & print
  return 0;
}

// Clock speed test
void clock_speed_test() {
  while (1){
    delay(0x500000); // Approx 1s
    MG_INFO(("c"));
  }
}

// ************* API *************

// Interrupt driven call-back
// mip_rxcb: New API

struct mip_driver mip_driver_imx_rt1020 = {
  mip_driver_imx_rt1020_init, mip_driver_imx_rt1020_tx, NULL,
  mip_driver_imx_rt1020_up, mip_driver_imx_rt1020_setrx};

// *************  *************

#endif
