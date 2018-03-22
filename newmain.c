// PIC18F4525 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HS      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ 40000000


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#define E_pulse_with 50 //aeeoaeuiinou oaeoiauo eiioeunia LCD a ien
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // eieoeaeecaoey ii?oia D4-D7 LCD

void motor_init() 
{ 
    TRISDbits.RD0=0; 
    TRISDbits.RD1=0; 
    TRISBbits.RB1=0; 
    TRISBbits.RB2=0; 
    TRISCbits.TRISC1=0; 
    TRISCbits.TRISC2=0; 
    CCP1CONbits.CCP1M=0b1100; //çàäà¸ì ðåæèì ðàáîòû ìîäóëÿ CCP1 (ØÈÌ) CCP1CONbits.P1M=0b00; //çàäåéñòâîâàí òîëüêî îäèí âûâîä P1A 
    CCP2CONbits.CCP2M=0b1111; // çàäà¸ì ðåæèì ðàáîòû ìîäóëÿ CCP2(ØÈÌ) 
    CCPR1L=0; // çàäà¸ì íóëåâóþ ñêâàæíîñòü CCPR2L=0; // çàäà¸ì íóëåâóþ ñêâàæíîñòü 
    PR2=124;// çàäà¸ì ïåðèîä ØÈÌ 
    T2CONbits.T2CKPS=0b00; //çàäà¸ì ïðåääåëèòåëü ìîäóëÿ Timer2 ðàâíûì 1 
    T2CONbits.TMR2ON=1;// âêëþ÷åíèå ìîäóëÿ Timer2 
} 


void motor_a_change_Speed_A (signed char speed) 
 { 
    if (speed>0) // äâèæåíèå âïåð¸ä 
    { 
        CCPR1L=speed; 
        PORTDbits.RD0=1; 
        PORTDbits.RD1=0; 
    } 
    else if (speed<0) // äâèæåíèå íàçàä 
    { 
        CCPR1L =-speed; 
        PORTDbits.RD0=0; 
        PORTDbits.RD1=1; 
    } 
    else //îñòàíîâêà 
    { 
        CCPR1L=0; 
        PORTDbits.RD0=0;         
        PORTDbits.RD1=0; 
    }
} 
void motor_a_change_Speed_B (signed char speed) 
 { 
    if (speed>0) // äâèæåíèå âïåð¸ä 
    { 
        CCPR2L=speed; 
        PORTBbits.RB1=1; 
        PORTBbits.RB2=0; 
    } 
    else if (speed<0) // äâèæåíèå íàçàä 
    { 
        CCPR2L =-speed; 
        PORTBbits.RB1=0; 
        PORTBbits.RB2=1;  
    } 
    else //îñòàíîâêà 
    { 
        CCPR2L=0; 
        PORTBbits.RB1=0; 
        PORTBbits.RB2=0;  
    }
}

void Adc_init() 
{ 
    TRISA|=0b00101111; //ïåðåâîä âûâîäîâ RA0, RA1, RA2, RA3, RA5 â ðåæèì âõîäîâ 
    TRISE|=0b00000111; //ïåðåâîä âûâîäîâ RE0, RE1, RE2 â ðåæèì âõîäîâ 
    ADCON1bits.PCFG=0b0111; // êîíôèãóðàöèÿ àíàëîãî-öèôðîâûõ ïîðòîâ 
    ADCON1bits.VCFG=0b00; // îïîðíîå íàïðÿæåíèå Vss Vdd 
    ADCON2bits.ACQT=0b111;// âðåìÿ ïðåîáðàçîâàíèÿ 20 Tad 
    ADCON2bits.ADCS=0b110;// ÷àñòîòà ïðåîáðàçîâàíèÿ Fosc/64     
    ADCON2bits.ADFM=0;//ëåâîå ñìåùåíèå 
    ADCON0bits.ADON=1;   // ìîäóëü ÀÖÏ âêëþ÷åí 
} 
int read_Adc() 
{  
    ADCON0bits.GO_DONE=1; // çàïóñê ïðåîáðàçîâàíèÿ 
    while(ADCON0bits.GO_DONE==1);       
    return (ADRESH<<2)+(ADRESL>>6);//çàïèñü ðåçóëüòàòà ïðåîáðàçîâàíèÿ 
} 


void main(void) { 
    int i=0;
    Adc_init();
    motor_init();
    motor_a_change_Speed_A(100);
    motor_a_change_Speed_B(100);
    while(1)
    {
       
        ADCON0bits.CHS=i; // âûáîð àíàëîãîâîãî êàíàëà  
        while(read_Adc()>600)
        {
            motor_a_change_Speed_B(-100);
            motor_a_change_Speed_A(100);
        }
        motor_a_change_Speed_A(100);
        motor_a_change_Speed_B(100);
        __delay_ms(50);
        i++;
        ADCON0bits.CHS=i; // âûáîð àíàëîãîâîãî êàíàëà  
        while(read_Adc()>600)
        {
            motor_a_change_Speed_B(100);
            motor_a_change_Speed_A(-100);
        }
        motor_a_change_Speed_A(100);
        motor_a_change_Speed_B(100);
        __delay_ms(50);
        i=0;
         
    } 
}    