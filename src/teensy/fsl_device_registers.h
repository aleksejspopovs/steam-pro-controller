#pragma once

// TinyUSB's official i.MX RT DCD uses MCUX SDK names. Teensyduino already
// provides the same CMSIS core and peripheral registers; map only the names
// required by ci_hs_imxrt.h instead of bringing in a second MCU runtime.
#include <imxrt.h>

typedef enum IRQ_NUMBER_t IRQn_Type;
#define NVIC_EnableIRQ(irq) NVIC_ENABLE_IRQ((int)(irq))
#define NVIC_DisableIRQ(irq) NVIC_DISABLE_IRQ((int)(irq))

#define USB1_BASE IMXRT_USB1_ADDRESS
#define USB2_BASE IMXRT_USB2_ADDRESS
#define USB_OTG1_IRQn IRQ_USB1
#define USB_OTG2_IRQn IRQ_USB2
#define FSL_FEATURE_SOC_USBHS_COUNT 2
