/*
  Register definitions for slave core: Golden bitstream WB Slave

  * File           : golden_regs.h
  * Author         : auto-generated by wbgen2 from golden_wb.wb
  * Created        : Thu May 16 10:48:36 2013
  * Standard       : ANSI C

    THIS FILE WAS GENERATED BY wbgen2 FROM SOURCE FILE golden_wb.wb
    DO NOT HAND-EDIT UNLESS IT'S ABSOLUTELY NECESSARY!

*/

#ifndef __WBGEN2_REGDEFS_GOLDEN_WB_WB
#define __WBGEN2_REGDEFS_GOLDEN_WB_WB

#if defined( __GNUC__)
#define PACKED __attribute__ ((packed))
#else
#error "Unsupported compiler?"
#endif

#ifndef __WBGEN2_MACROS_DEFINED__
#define __WBGEN2_MACROS_DEFINED__
#define WBGEN2_GEN_MASK(offset, size) (((1<<(size))-1) << (offset))
#define WBGEN2_GEN_WRITE(value, offset, size) (((value) & ((1<<(size))-1)) << (offset))
#define WBGEN2_GEN_READ(reg, offset, size) (((reg) >> (offset)) & ((1<<(size))-1))
#define WBGEN2_SIGN_EXTEND(value, bits) (((value) & (1<<bits) ? ~((1<<(bits))-1): 0 ) | (value))
#endif


/* definitions for register: Control/Status reg */

/* definitions for field: Number of FMC slots in reg: Control/Status reg */
#define GLD_CSR_SLOT_COUNT_MASK               WBGEN2_GEN_MASK(0, 4)
#define GLD_CSR_SLOT_COUNT_SHIFT              0
#define GLD_CSR_SLOT_COUNT_W(value)           WBGEN2_GEN_WRITE(value, 0, 4)
#define GLD_CSR_SLOT_COUNT_R(reg)             WBGEN2_GEN_READ(reg, 0, 4)

/* definitions for field: FMC presence line status in reg: Control/Status reg */
#define GLD_CSR_FMC_PRESENT_MASK              WBGEN2_GEN_MASK(4, 4)
#define GLD_CSR_FMC_PRESENT_SHIFT             4
#define GLD_CSR_FMC_PRESENT_W(value)          WBGEN2_GEN_WRITE(value, 4, 4)
#define GLD_CSR_FMC_PRESENT_R(reg)            WBGEN2_GEN_READ(reg, 4, 4)

/* definitions for register: I2C bitbanged IO register 0 */

/* definitions for field: SCL Line out in reg: I2C bitbanged IO register 0 */
#define GLD_I2CR0_SCL_OUT                     WBGEN2_GEN_MASK(0, 1)

/* definitions for field: SDA Line out in reg: I2C bitbanged IO register 0 */
#define GLD_I2CR0_SDA_OUT                     WBGEN2_GEN_MASK(1, 1)

/* definitions for field: SCL Line in in reg: I2C bitbanged IO register 0 */
#define GLD_I2CR0_SCL_IN                      WBGEN2_GEN_MASK(2, 1)

/* definitions for field: SDA Line in in reg: I2C bitbanged IO register 0 */
#define GLD_I2CR0_SDA_IN                      WBGEN2_GEN_MASK(3, 1)

/* definitions for register: I2C bitbanged IO register 1 */

/* definitions for field: SCL Line out in reg: I2C bitbanged IO register 1 */
#define GLD_I2CR1_SCL_OUT                     WBGEN2_GEN_MASK(0, 1)

/* definitions for field: SDA Line out in reg: I2C bitbanged IO register 1 */
#define GLD_I2CR1_SDA_OUT                     WBGEN2_GEN_MASK(1, 1)

/* definitions for field: SCL Line in in reg: I2C bitbanged IO register 1 */
#define GLD_I2CR1_SCL_IN                      WBGEN2_GEN_MASK(2, 1)

/* definitions for field: SDA Line in in reg: I2C bitbanged IO register 1 */
#define GLD_I2CR1_SDA_IN                      WBGEN2_GEN_MASK(3, 1)

/* definitions for register: I2C bitbanged IO register 2 */

/* definitions for field: SCL Line out in reg: I2C bitbanged IO register 2 */
#define GLD_I2CR2_SCL_OUT                     WBGEN2_GEN_MASK(0, 1)

/* definitions for field: SDA Line out in reg: I2C bitbanged IO register 2 */
#define GLD_I2CR2_SDA_OUT                     WBGEN2_GEN_MASK(1, 1)

/* definitions for field: SCL Line in in reg: I2C bitbanged IO register 2 */
#define GLD_I2CR2_SCL_IN                      WBGEN2_GEN_MASK(2, 1)

/* definitions for field: SDA Line in in reg: I2C bitbanged IO register 2 */
#define GLD_I2CR2_SDA_IN                      WBGEN2_GEN_MASK(3, 1)

/* definitions for register: I2C bitbanged IO register 3 */

/* definitions for field: SCL Line out in reg: I2C bitbanged IO register 3 */
#define GLD_I2CR3_SCL_OUT                     WBGEN2_GEN_MASK(0, 1)

/* definitions for field: SDA Line out in reg: I2C bitbanged IO register 3 */
#define GLD_I2CR3_SDA_OUT                     WBGEN2_GEN_MASK(1, 1)

/* definitions for field: SCL Line in in reg: I2C bitbanged IO register 3 */
#define GLD_I2CR3_SCL_IN                      WBGEN2_GEN_MASK(2, 1)

/* definitions for field: SDA Line in in reg: I2C bitbanged IO register 3 */
#define GLD_I2CR3_SDA_IN                      WBGEN2_GEN_MASK(3, 1)
/* [0x0]: REG Control/Status reg */
#define GLD_REG_CSR 0x00000000
/* [0x4]: REG I2C bitbanged IO register 0 */
#define GLD_REG_I2CR0 0x00000004
/* [0x8]: REG I2C bitbanged IO register 1 */
#define GLD_REG_I2CR1 0x00000008
/* [0xc]: REG I2C bitbanged IO register 2 */
#define GLD_REG_I2CR2 0x0000000c
/* [0x10]: REG I2C bitbanged IO register 3 */
#define GLD_REG_I2CR3 0x00000010
#endif
