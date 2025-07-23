#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"

#define L1_PIN      PD5
#define L2_PIN      PD4
#define L3_PIN      PD3
#define L4_PIN      PD2
#define LATCH_PIN   PB2  // alternate function SPI
#define DATA_PIN    PB3  // alternate function SPI 
#define CLK_PIN     PB5  // alternate function SPI
 
#define CUBE_A_PIN  PD6

#define NR_LAYERS       4
#define NR_CUBES        2
#define BAM_RESOLUTION  4
#define MAX_ANIMATIONS  1

#define BT_BUFFER_SIZE  15

void init(void);
void shiftOut_spi(const uint8_t data);
void load_data(const int cube_id, const uint16_t data);
void enable_layer(const int layer);
void disable_layer(const int layer);
void select_cube(const int cube_id);

void display_led(const int cube_id, const int layer, const int id, const uint8_t brightness);
void display_led(const int cube_id, const int id, const uint8_t brightness);
void display_cube(const int start_point, const uint8_t brightness);

void clear(const int cube_id);

void Rotate_90(const uint16_t layer);

void startup_test_leds();

const uint8_t LAYER_PINS[NR_LAYERS] = {L1_PIN, L2_PIN, L3_PIN, L4_PIN};

/*
For each layer:

    LED-15                        LED-0
      |                             |
      v                             v
      x x x x x x x x x x x x x x x x   (uint16_t)  --> BAM cycle 1
      x x x x x x x x x x x x x x x x
      x x x x x x x x x x x x x x x x
      x x x x x x x x x x x x x x x x               --> BAM cycle 4
*/
uint16_t data[5][NR_LAYERS][BAM_RESOLUTION] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
int current_layer = -1;
int current_cube = 0;
int current_bam_cycle = 0;
uint8_t bt_receive_buffer[BT_BUFFER_SIZE] = {0};

void Rotate_90(const uint16_t layer)
{
    uint8_t rot90[16] = {12,8,4,0,13,9,5,1,14,10,6,2,15,11,7,3};
    uint16_t temp[NR_LAYERS][BAM_RESOLUTION];
    memset(temp, 0, NR_LAYERS*BAM_RESOLUTION*sizeof(uint16_t));

    for (int p=0; p<16; p++)
    {
        for (int cycle=0; cycle<4; cycle++)
        {
            if (data[0][layer][cycle] & (1<<p))
            {
                temp[layer][cycle] |= (1 << rot90[p]); 
            }
        }
    }

    memcpy(data[layer], temp[layer], BAM_RESOLUTION*sizeof(uint16_t));
}

void animation_3()
{ 
    uint8_t rot[4][16] = {
        {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
        {12,8,4,0,13,9,5,1,14,10,6,2,15,11,7,3},
        {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
        {3,7,11,15,2,6,10,14,1,5,9,13,0,4,8,12}
    };

    for (int rot_index=0; rot_index<4; rot_index++)
    {        
        display_led(0, rot[rot_index][8], 15);    
        display_led(0, rot[rot_index][12], 15);       
        display_led(0, rot[rot_index][9], 0);    
        display_led(0, rot[rot_index][13], 0);
        display_led(0, rot[rot_index][10], 0);    
        display_led(0, rot[rot_index][14], 0);
        display_led(0, rot[rot_index][11], 0);    
        display_led(0, rot[rot_index][15], 0);
        display_led(1, rot[rot_index][8], 15);    
        display_led(1, rot[rot_index][12], 15);       
        display_led(1, rot[rot_index][9], 0);    
        display_led(1, rot[rot_index][13], 0);
        display_led(1, rot[rot_index][10], 0);    
        display_led(1, rot[rot_index][14], 0);
        display_led(1, rot[rot_index][11], 0);    
        display_led(1, rot[rot_index][15], 0);
        
        _delay_ms(100);

        display_led(0, rot[rot_index][8], 15);    
        display_led(0, rot[rot_index][12], 15);
        display_led(0, rot[rot_index][9], 3);    
        display_led(0, rot[rot_index][13], 3);
        display_led(1, rot[rot_index][8], 15);    
        display_led(1, rot[rot_index][12], 15);
        display_led(1, rot[rot_index][9], 3);    
        display_led(1, rot[rot_index][13], 3);
        
        _delay_ms(20);

        display_led(0, rot[rot_index][8], 15);    
        display_led(0, rot[rot_index][12], 15);
        display_led(0, rot[rot_index][9], 3);    
        display_led(0, rot[rot_index][13], 3);
        display_led(0, rot[rot_index][10], 3);    
        display_led(0, rot[rot_index][14], 3);
        display_led(1, rot[rot_index][8], 15);    
        display_led(1, rot[rot_index][12], 15);
        display_led(1, rot[rot_index][9], 3);    
        display_led(1, rot[rot_index][13], 3);
        display_led(1, rot[rot_index][10], 3);    
        display_led(1, rot[rot_index][14], 3);

        _delay_ms(20);

        display_led(0, rot[rot_index][8], 15);    
        display_led(0, rot[rot_index][12], 15);
        display_led(0, rot[rot_index][9], 3);    
        display_led(0, rot[rot_index][13], 3);
        display_led(0, rot[rot_index][10], 3);    
        display_led(0, rot[rot_index][14], 3);
        display_led(0, rot[rot_index][11], 3);    
        display_led(0, rot[rot_index][15], 3);
        display_led(1, rot[rot_index][8], 15);    
        display_led(1, rot[rot_index][12], 15);
        display_led(1, rot[rot_index][9], 3);    
        display_led(1, rot[rot_index][13], 3);
        display_led(1, rot[rot_index][10], 3);    
        display_led(1, rot[rot_index][14], 3);
        display_led(1, rot[rot_index][11], 3);    
        display_led(1, rot[rot_index][15], 3);

        _delay_ms(20);

        display_led(0, rot[rot_index][8], 3);    
        display_led(0, rot[rot_index][12], 3);
        display_led(0, rot[rot_index][9], 3);    
        display_led(0, rot[rot_index][13], 3);
        display_led(0, rot[rot_index][10], 3);    
        display_led(0, rot[rot_index][14], 3);
        display_led(0, rot[rot_index][11], 15);    
        display_led(0, rot[rot_index][15], 15);
        display_led(1, rot[rot_index][8], 3);    
        display_led(1, rot[rot_index][12], 3);
        display_led(1, rot[rot_index][9], 3);    
        display_led(1, rot[rot_index][13], 3);
        display_led(1, rot[rot_index][10], 3);    
        display_led(1, rot[rot_index][14], 3);
        display_led(1, rot[rot_index][11], 15);    
        display_led(1, rot[rot_index][15], 15);

        _delay_ms(20);

        display_led(0, rot[rot_index][8], 0);    
        display_led(0, rot[rot_index][12], 0);
        display_led(0, rot[rot_index][9], 0);    
        display_led(0, rot[rot_index][13], 0);
        display_led(0, rot[rot_index][10], 0);    
        display_led(0, rot[rot_index][14], 0);
        display_led(0, rot[rot_index][11], 15);    
        display_led(0, rot[rot_index][15], 15);
        display_led(1, rot[rot_index][8], 0);    
        display_led(1, rot[rot_index][12], 0);
        display_led(1, rot[rot_index][9], 0);    
        display_led(1, rot[rot_index][13], 0);
        display_led(1, rot[rot_index][10], 0);    
        display_led(1, rot[rot_index][14], 0);
        display_led(1, rot[rot_index][11], 15);    
        display_led(1, rot[rot_index][15], 15);

        _delay_ms(200);
    }
    //rot_index = (rot_index + 1) & 0x3;
}
/*
void animation_4()
{
    int path[28] = {
        0,1,2,6,
        10,14,30,46,
        62,61,60,56,
        52,48,49,50,
        51,55,39,23,
        22,21,20,24,
        28,12,8,4
    };
    int prv3=25, prv2=26, prv1=27;
    int step=0;
    while(1)
    {           
        display_led(path[prv3], 0);         
        display_led(path[prv2], 2);                      
        display_led(path[prv1], 6);
        display_led(path[step], 15);
        _delay_ms(75);
        
        step++;         
        if (step == 28)
        {
            step = 0;
            prv3=24, prv2=25, prv1=26;
        }
       
        prv3++; prv2++; prv1++;
        if (prv3 == 28)
            prv3 = 0;
        if (prv2 == 28)
            prv2 = 0;
        if (prv1 == 28)
            prv1 = 0;

       
    }    
}
*/
void animation_5()
{  
    display_cube(0, 15); 

    _delay_ms(125);

    display_cube(0, 5);
    display_cube(21, 15); 

    _delay_ms(125);

    display_cube(0, 0);
    display_cube(21, 5); 
    display_cube(42, 15);

    _delay_ms(125);

    display_cube(21, 0); 
    display_cube(42, 5);
    display_cube(41, 15);

    _delay_ms(125);
    
    display_cube(42, 0);
    display_cube(41, 5);
    display_cube(40, 15);

    _delay_ms(125);
    
    display_cube(41, 0);
    display_cube(40, 5);
    display_cube(21, 15);

    _delay_ms(125);
    
    display_cube(40, 0);
    display_cube(21, 5);
    display_cube(2, 15);

    _delay_ms(125);
       
    display_cube(21, 0);
    display_cube(2, 5);
    display_cube(1, 15);

    _delay_ms(125);
    
    display_cube(2, 0);
    display_cube(1, 5);    
    display_cube(0, 15);  
 
    _delay_ms(125);

    display_cube(1, 0);    

}

void animation_6()
{
    clear(0);
    //clear(1);

    int s=0;
    enum DIRECTION {X=0, Y, Z}; // X = row, Y = col, Z = layer
          
    int path[10];
    int nr_x = 0, nr_y = 0, nr_z = 0;
    path[0] = s;

    // generate path: 3x X-direction, 3x Y-direction, 3x Z-direction
    for (int i=1; i<10; i++)
    {     
        DIRECTION d;
        do {
            d = (DIRECTION)(rand()%3); 
        } while ((d == X && nr_x == 3) || (d == Y && nr_y == 3) || (d == Z && nr_z == 3));
        
        switch(d)
        {
            case X: path[i] = path[i-1] + 1; nr_x++; break;
            case Y: path[i] = path[i-1] + 4; nr_y++; break;
            case Z: path[i] = path[i-1] + 16; nr_z++; break;
        }
    }

    // walk the path 
    int step=0;       
    for (step=0; step<10; step++)
    {
        display_led(0, path[step], 15);
        if (step > 0)
            display_led(0, path[step-1], 6);
        if (step > 1)
            display_led(0, path[step-2], 2);
        if (step > 2)
            display_led(0, path[step-3], 0);

        _delay_ms(75);
    }
    display_led(0, path[step-3], 0);
    _delay_ms(75);
    display_led(0, path[step-2], 2);
    display_led(0, path[step-1], 6);
    _delay_ms(75);
    display_led(0, path[step-2], 0);
    display_led(0, path[step-1], 2);
    _delay_ms(75);
    display_led(0, path[step-1], 0);

    _delay_ms(250);
}

void animation_multicube()
{
    clear(0);
    clear(1);

    _delay_ms(250);
    display_led(0, 0, 15);
    _delay_ms(250);
    display_led(0, 1, 15);
    _delay_ms(250);
    display_led(0, 2, 15);
    _delay_ms(250);
    display_led(0, 3, 15);
    _delay_ms(250);
    display_led(1, 0, 15);
    _delay_ms(250);
    display_led(1, 1, 15);
    _delay_ms(250);
    display_led(1, 2, 15);
    _delay_ms(250);
    display_led(1, 3, 15);
    
    _delay_ms(500);

    display_led(0, 4, 15);
    display_led(1, 4, 15);
    _delay_ms(250);
    display_led(0, 5, 15);
    display_led(1, 5, 15);
    _delay_ms(250);
    display_led(0, 6, 15);
    display_led(1, 6, 15);
    _delay_ms(250);
    display_led(0, 7, 15);
    display_led(1, 7, 15);

    _delay_ms(500);

}

void startup_test_leds()
{
    clear(0);
    //clear(1);
    for (int i=0; i<64; i++)
    {
        display_led(0, i, 15);
        //display_led(1, 63-i, 15);
        _delay_ms(50);
    }

    _delay_ms(500);

    clear(0);
    //clear(1);
    
}

void animation_complete_cube()
{
    for (int i=0; i<64; i++)
    {
        display_led(0, i, 15);
        display_led(1, i, 15);
    }
}


typedef void (*anim_f)();
typedef struct animation
{
    uint8_t nr_iterations;
    anim_f  display_function;
} animation_t;

bool bt_data_collected(void)
{   
   if(UART_available())
   {
     UART_readBytesUntil('\n', bt_receive_buffer, BT_BUFFER_SIZE);
   
     return true;
   }
   return false;
}

int main(void)
{        
    cli();
    init();    
    sei();   

    animation_t animations[1] = {   
        {//1, animation_complete_cube}};    
        1, animation_6}};
        //{1, animation_multicube}};

    uint8_t current_animation_idx = 0;
    
    startup_test_leds();
    
    while(1)
    {
        animation_t current_animation = animations[current_animation_idx];
        for (int i=0; i<current_animation.nr_iterations; i++)
        {
/*
            if (bt_data_collected())
            { 
                //uart_puts(bt_receive_buffer);             
                if (strcmp((char*)bt_receive_buffer, "pauze") == 0)
                {
                          
                }
                else if (strcmp((char*)bt_receive_buffer, "continue") == 0)
                {
                   
                }
                else if (strcmp((char*)bt_receive_buffer, "help") == 0)
                {
                    char s[20];
                    
                    strncpy(s, "pauze-continue-help\0", 20);
                    UART_sendString(s);
                }   

                // other commands:
                //  - home: show info of current running animation
                //  - disable cube
                //  - select animation
                //  - list available animations
                //  - set timer (run animations for certain period, [start at certain time]) 

            }
*/
            current_animation.display_function();
        }

        current_animation_idx++;
        if (current_animation_idx == MAX_ANIMATIONS)
            current_animation_idx = 0;
    }
}

void init()
{
    // init watchdog
    // wdt_reset();
    // wdt_enable(WDTO_2S);
    // WDTCSR = (1<<WDIE);
    
    // init GPIO pins
    // output pins    
    DDRD |= ((1 << L1_PIN) | (1 << L2_PIN) | (1 << L3_PIN) | (1 << L4_PIN) | (1 << CUBE_A_PIN));    
    // layer pins are initially 'high' because they drive a pnp transistor
    PORTD |= ((1 << L1_PIN) | (1 << L2_PIN) | (1 << L3_PIN) | (1 << L4_PIN));    
   
    // builtin led
    // DDRB |= (1 << BUILTIN_LED);
    // PORTB &= ~(1 << BUILTIN_LED);

    // init 8bit timer - BAM modulation
    TCCR0A |= ((1<<WGM01) | (0<<WGM00));
    TCCR0B |= ((0<<WGM02));
    TIMSK0 = (1<<OCIE0A); 
    TCNT0 = 0;
    OCR0A = 1;
    // 1-0-0  =  prescaler 256 --> 1 tick = 0.016ms
    TCCR0B |= ((1<<CS02) | (0<<CS01) | (1<<CS00));  
        
    // init SPI for data transfer to shift registers (MSB)
    DDRB |= ((1 << DATA_PIN) | (1 << CLK_PIN) | (1 << LATCH_PIN));
    PORTB &= ~((1 << DATA_PIN) | (1 << CLK_PIN) | (1 << LATCH_PIN));
    SPCR |= ((1 << SPE) | (0 << DORD) | (1 << MSTR));
    SPSR |= ((1 << SPI2X));

    // init UART for bluetooth (BT) communication
    //UART_begin(9600, UART_ASYNC_NORMAL, UART_NO_PARITY, UART_8_BIT);
    
}

void shiftOut_spi(const uint8_t data)
{
    SPDR = data;
    // wait until SPI transfer is finished
    while(!(SPSR & (1<<SPIF)));	
}

void load_data(const int cube_id, const uint16_t data)
{
    // TO BE:
    //  5 latch_pins (for each cube)
    //  only do shiftout if cube is connected (is_connected pin)
    //  no multiplexer, so no select_cube() needed

    select_cube(cube_id);

    shiftOut_spi(data >> 8);
    shiftOut_spi(data);

    PORTB |= (1 << LATCH_PIN);
    PORTB &= ~(1 << LATCH_PIN);  
}

void display_led(const int cube_id, const int layer, const int id, const uint8_t brightness)
{
    for (int bam_cycle=0; bam_cycle<BAM_RESOLUTION; bam_cycle++)
    {
        if ((brightness & (1<<bam_cycle)) == 0)
            data[cube_id][layer][bam_cycle] &= ~(1<<id);
        else
            data[cube_id][layer][bam_cycle] |= (1<<id);
    }
}

void display_led(const int cube_id, const int id, const uint8_t brightness)
{
    display_led(cube_id, id/16, id%16, brightness);
}

// void display_layer(const int cube_id, const int layer, const uint8_t brightness)
// {
//     for (int bam_cycle=0; bam_cycle<BAM_RESOLUTION; bam_cycle++)
//     {
//         if ((brightness & (1<<bam_cycle)) == 0)
//             data[cube_id][layer][bam_cycle] &= ~(0xFFFF);
//         else
//             data[cube_id][layer][bam_cycle] |= 0xFFFF;
//     }
// }

void display_cube(const int upper_left_point, const uint8_t brightness)
{   
    display_led(0, upper_left_point, brightness);
    display_led(0, upper_left_point+1, brightness);
    display_led(0, upper_left_point+4, brightness);
    display_led(0, upper_left_point+5, brightness);
    display_led(0, upper_left_point+16, brightness);
    display_led(0, upper_left_point+17, brightness);
    display_led(0, upper_left_point+20, brightness);
    display_led(0, upper_left_point+21, brightness);
}

void clear(const int cube_id)
{
    memset(data[cube_id], 0, NR_LAYERS*BAM_RESOLUTION * (sizeof(uint16_t)));
}

void enable_layer(const int layer)
{    
    PORTD &= ~(1 << LAYER_PINS[layer]);     
}

void disable_layer(const int layer)
{
    PORTD |= (1 << LAYER_PINS[layer]);
}

void select_cube(const int cube_id)
{        
    // multiplexer: 
    //  SPI CS-pin is input to multiplexer
    //  CUBE-A-pin is selector of multiplexer

    PORTD &= ~(1 << CUBE_A_PIN);
    PORTD |= (cube_id << CUBE_A_PIN);
}

// ISR(WDT_vect)
// {
    
// }

ISR(TIMER0_COMPA_vect)
{  
    current_layer++;

    if (current_layer == NR_LAYERS)
    {
        current_layer = 0;       
        current_bam_cycle++;
        if (current_bam_cycle == BAM_RESOLUTION)
        {        
            current_bam_cycle = 0; 
        }
    }

    disable_layer((current_layer-1) & 0x3);
    
    // ONLY load data for a cube if the cube is effectively connected 
    //  -> use extra/last wire on ribbon cable to detect this
    load_data(0, data[0][current_layer][current_bam_cycle]);      
    //load_data(1, data[1][current_layer][current_bam_cycle]);  
    
    enable_layer(current_layer);
    
    TCNT0 = 0;
    OCR0A = 1 << current_bam_cycle;
}
