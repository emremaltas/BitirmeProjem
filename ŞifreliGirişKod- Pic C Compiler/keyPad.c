#include<16f877.h>
#use delay(clock=4000000)
#fuses XT,NOLVP,NOBROWNOUT,NOPROTECT,NODEBUG,NOWDT
#define use_portb_lcd TRUE
#include<lcd.c>

#use fast_io(d)
#use fast_io(c)
//EMRE MALTAS 17 TEMMUZ 2021
int girilen[7]={1,1,1,1,1,1};
int sifre[7]={1,7,8,5,0,9};
int sayac=0;
int dogruluk=0;
int yanlis=0;

void tusKontrol();

void sorgu()
{
 lcd_putc("\f");
  for(int i=0;i<6;i++)
  {
     if(sifre[i]==girilen[i+1])
     {
        dogruluk++;
     }
  }
  if(dogruluk==6){ printf(lcd_putc,"Sifre Dogru!!");output_high(pin_c1);delay_ms(1000); output_low(pin_c1); dogruluk=0;  yanlis=0;}
  else{ printf(lcd_putc,"Sifre Yanlis!!\nKalan:%d",2-yanlis); output_high(pin_c0);delay_ms(1000); output_low(pin_c0);dogruluk=0; yanlis++;if(yanlis==3){
  printf(lcd_putc,"\f  3 Kez Yanlis \n  30Sn bekle!!");yanlis=0;delay_ms(1000);}}
  sayac=0;
  lcd_putc("\fSifreyi girmek\nicin * basiniz!");

}

void tusKontrol()
{
  output_high(pin_d3);
  if(input(pin_d4))
  { 
     sayac++;
     delay_ms(100);
     output_low(pin_d3);
     printf(lcd_putc,"\fSifre:");
     while(sayac<8 && sayac>=1)
     {
        output_high(pin_d0);
        if(input(pin_d4)){delay_ms(100); girilen[sayac]=1; printf(lcd_putc,"*"); sayac++;  while(input(pin_d4));}
        if(input(pin_d5)){delay_ms(100); girilen[sayac]=2; printf(lcd_putc,"*"); sayac++;  while(input(pin_d5));}
        if(input(pin_d6)){delay_ms(100); girilen[sayac]=3; printf(lcd_putc,"*"); sayac++;  while(input(pin_d6));}
        output_low(pin_d0);
  
  
        output_high(pin_d1);
        if(input(pin_d4)){delay_ms(100); girilen[sayac]=4; printf(lcd_putc,"*"); sayac++; while(input(pin_d4));}
        if(input(pin_d5)){delay_ms(100); girilen[sayac]=5; printf(lcd_putc,"*"); sayac++; while(input(pin_d5));}
        if(input(pin_d6)){delay_ms(100); girilen[sayac]=6; printf(lcd_putc,"*"); sayac++; while(input(pin_d6));}
        output_low(pin_d1);
  
        output_high(pin_d2);
        if(input(pin_d4)){delay_ms(100); girilen[sayac]=7; printf(lcd_putc,"*"); sayac++; while(input(pin_d4));}
        if(input(pin_d5)){delay_ms(100); girilen[sayac]=8; printf(lcd_putc,"*"); sayac++; while(input(pin_d5));}
        if(input(pin_d6)){delay_ms(100); girilen[sayac]=9; printf(lcd_putc,"*"); sayac++; while(input(pin_d6));}
        output_low(pin_d2);
  
        output_high(pin_d3);
        if(input(pin_d5)){delay_ms(100); girilen[sayac]=0; printf(lcd_putc,"*"); sayac++; while(input(pin_d5));}
        if(input(pin_d6)){delay_ms(100);sayac++ ; while(input(pin_d6));sorgu();}
        output_low(pin_d3);
        
     }
  
   }
    
   output_low(pin_d3);
}


void main()
{
   setup_psp(PSP_DISABLED);
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   setup_CCP1(CCP_OFF);
   setup_CCP2(CCP_OFF);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DISABLED,0,1);
   
   set_tris_d(0xF0);
   set_tris_c(0x00);
   set_tris_b(0x00);
   output_c(0x00);
   output_d(0x00);
   lcd_init();
   
   lcd_putc("\fSifreyi girmek\nicin * basiniz!");
   while(true)
   {
      tusKontrol();
   }

}
   
   


