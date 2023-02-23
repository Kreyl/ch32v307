/*
 * kl_init.c
 *
 *  Created on: 16 февр. 2022 г.
 *      Author: laurelindo
 */

//extern void (*__preinit_array_start []) (void);
//extern void (*__preinit_array_end []) (void);
extern void (*__init_array_start []) (void);
extern void (*__init_array_end []) (void);

/* Iterate over all the init routines.  */
void __libc_init_array(void) {
    unsigned long count;
    unsigned long i;

    // Preinit array is not used, so commented out
//    count = __preinit_array_end - __preinit_array_start;
//    for (i = 0; i < count; i++) {
//        __preinit_array_start[i] ();
//    }

    count = __init_array_end - __init_array_start;
    for(i = 0; i < count; i++) {
        __init_array_start[i] ();
    }
}
