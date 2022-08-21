// Board and hardware specific configuration
#define MICROPY_HW_BOARD_NAME                   "Raspberry Pi Pico"
#define MICROPY_HW_FLASH_STORAGE_BYTES          (1408 * 1024)

// Enable USB Mass Storage with FatFS filesystem.
// #define MICROPY_HW_USB_MSC  (1)

void init_cdc_uart(void);

#define MICROPY_PORT_INIT_FUNC init_cdc_uart()