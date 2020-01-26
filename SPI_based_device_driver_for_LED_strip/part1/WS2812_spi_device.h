#define RESET _IO(81, 1)

#define ZERO 0x30 /* At 5Mhz we need to send 6bits-110000 for zero on spi device */
#define ONE 0x78 /* At 5Mhz we need to send 7bits-11110000 for One on spi device */

/* Each colour need 8 bit */
#define LOW ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO
#define HIGH ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE

/* To light one LED we need 8*3=24 bits */
#define GREEN HIGH, LOW, LOW
#define RED LOW, HIGH, LOW
#define BLUE LOW, LOW, HIGH
#define NONE LOW, LOW, LOW


#define BUF_LENGTH 384 /*  16*8*3 */
