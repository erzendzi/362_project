#include "stm32f0xx.h"
#include <string.h> // for memmove()
#include <stdio.h> // for memmove()

// Define a larger queue size
#define QUEUE_SIZE 16

// Initialize the queue with QUEUE_SIZE
char queue[QUEUE_SIZE];  // Queue to hold key events
int qin = 0;              // Index for the next input
int qout = 0;             // Index for the next output

const char keymap[] = "DCBA#9630852*741";

const char font[] = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, // 32: space
    0x86, // 33: exclamation
    0x22, // 34: double quote
    0x76, // 35: octothorpe
    0x00, // dollar
    0x00, // percent
    0x00, // ampersand
    0x20, // 39: single quote
    0x39, // 40: open paren
    0x0f, // 41: close paren
    0x49, // 42: asterisk
    0x00, // plus
    0x10, // 44: comma
    0x40, // 45: minus
    0x80, // 46: period
    0x00, // slash
    // digits
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
    // seven unknown
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // Uppercase
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x6f, 0x76, 0x30, 0x1e, 0x00, 0x38, 0x00,
    0x37, 0x3f, 0x73, 0x7b, 0x31, 0x6d, 0x78, 0x3e, 0x00, 0x00, 0x00, 0x6e, 0x00,
    0x39, // 91: open square bracket
    0x00, // backslash
    0x0f, // 93: close square bracket
    0x00, // circumflex
    0x08, // 95: underscore
    0x20, // 96: backquote
    // Lowercase
    0x5f, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x6f, 0x74, 0x10, 0x0e, 0x00, 0x30, 0x00,
    0x54, 0x5c, 0x73, 0x7b, 0x50, 0x6d, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x6e, 0x00
};

// Existing variables
uint8_t hist[16];
uint8_t debounced_state[16] = {0}; // Initialize to 0

// Function to check if the queue is full
int is_queue_full() {
    return ((qin + 1) % QUEUE_SIZE) == qout;
}

// Function to check if the queue is empty
int is_queue_empty() {
    return qin == qout;
}

// Function to push an event to the queue
void push_queue(char n) {
    if (!is_queue_full()) {
        queue[qin] = n;
        qin = (qin + 1) % QUEUE_SIZE;

        if (n & 0x80) { // Key press
            printf("Key Pressed: %c\n", n & 0x7F);
        } else { // Key release
            printf("Key Released: %c\n", n);
        }
    } else {
        // Handle queue overflow
        // For simplicity, we'll ignore the new event.
        // Alternatively, you can implement flags or counters to track lost events.
    }
}

// Function to pop an event from the queue
char pop_queue() {
    if (!is_queue_empty()) {
        char tmp = queue[qout];
        qout = (qout + 1) % QUEUE_SIZE;
        return tmp;
    }
    return 0; // Or some sentinel value indicating the queue is empty
}

void update_history(int c, int rows)
{
    for (int i = 0; i < 4; i++) {
        int idx = 4 * c + i;

        // Update history for each key independently
        hist[idx] = (hist[idx] << 1) | ((rows >> i) & 0x1);

        // Check if key has been consistently pressed for 8 scans (debounced)
        if ((hist[idx] & 0xFF) == 0xFF && debounced_state[idx] == 0) {
            debounced_state[idx] = 1;
            push_queue(0x80 | keymap[idx]);
        }

        // Check if key has been consistently released for 8 scans (debounced)
        if ((hist[idx] & 0xFF) == 0x00 && debounced_state[idx] == 1) {
            debounced_state[idx] = 0;
            push_queue(keymap[idx]);
        }
    }
}

//void update_history(int c, int rows)
//{
    // We used to make students do this in assembly language.
//    for(int i = 0; i < 4; i++) {
//        hist[4*c+i] = (hist[4*c+i]<<1) + ((rows>>i)&1);
//        if (hist[4*c+i] == 0x01)
//            push_queue(0x80 | keymap[4*c+i]);
//        if (hist[4*c+i] == 0xfe)
//            push_queue(keymap[4*c+i]);
//    }
//}

void drive_column(int c)
{
    GPIOC->BSRR = 0xf00000 | ~(1 << (c + 4));
}

int read_rows()
{
    return (~GPIOC->IDR) & 0xf;
}

char get_key_event(void) {
    for(;;) {
        asm volatile ("wfi");   // wait for an interrupt
        if (!is_queue_empty()) {
            return pop_queue();
        }
    }
}

char get_keypress() {
    char event;
    for(;;) {
        // Wait for every button event...
        event = get_key_event();
        // ...but ignore if it's a release.
        if (event & 0x80)
            break;
    }
    return event & 0x7f;
}

//////////////////////////////////////////////////////////
// REMOVE + EDIT FOR PROJECT

// Read an entire floating-point number.
float getfloat(void)
{
    int num = 0;
    int digits = 0;
    int decimal = 0;
    int enter = 0;
    //clear_display();
    //set_digit_segments(7, font['0']);
    while(!enter) {
        int key = get_keypress();
        if (digits == 8) {
            if (key != '#')
                continue;
        }
        switch(key) {
        case '0':
            if (digits == 0)
                continue;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            num = num*10 + key-'0';
            decimal <<= 1;
            digits += 1;
            break;
        case '*':
            if (decimal == 0) {
                decimal = 1;
                //dot();
            }
            break;
        case '#':
            enter = 1;
            break;
        default: continue; // ABCD
        }
    }
    float f = num;
    while (decimal) {
        decimal >>= 1;
        if (decimal)
            f = f/10.0;
    }
    return f;
}

//===========================================================================
// Part 4: Create an analog sine wave of a specified frequency
//===========================================================================
void set_freq(int chan, float freq);