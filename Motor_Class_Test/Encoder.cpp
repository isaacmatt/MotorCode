
#include "Encoder.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

Encoder_internal_state_t * Encoder::interruptArgs[];

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
// Add similar ISR definitions for other vectors as needed
#endif
#endif
