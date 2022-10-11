#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include<string.h>

//6.05.2022 Emre Malta�


uint32_t adc_degerleri_dma[6] = {0};

uint32_t pwm_degerleri_dma[6] = {0};

uint32_t bekle=0;

char gelen_veri[100]= {0};

char gelen_karakter=0;
uint8_t sayac=0;

uint8_t pc_gelenveri_sonuc = 0,pic16dan_gelen_sonuc=0,veri_geldi=0;



//tim2-5 kullan�labilir 32

//tim3-4 kullan�labilir 16 bit  (T�M4 -> D12,13,14,15)

/*
 * 	  adc_kanal0_dma= adc_degerleri_dma[0];
	  adc_kanal1_dma= adc_degerleri_dma[1];
	  adc_kanal2_dma= adc_degerleri_dma[2];
	  adc_kanal3_dma= adc_degerleri_dma[3];
	  adc_kanal4_dma= adc_degerleri_dma[4];
	  adc_kanal5_dma= adc_degerleri_dma[5];

	  pwm_cikis0= pwm_degerleri_dma[0];
	  pwm_cikis1= pwm_degerleri_dma[1];
	  pwm_cikis2= pwm_degerleri_dma[2];
	  pwm_cikis3= pwm_degerleri_dma[3];
	  pwm_cikis4= pwm_degerleri_dma[4];
	  pwm_cikis5= pwm_degerleri_dma[5];
 */

uint32_t adc_kanal0_dma=0;
uint32_t adc_kanal1_dma=0;
uint32_t adc_kanal2_dma=0;
uint32_t adc_kanal3_dma=0;
uint32_t adc_kanal4_dma=0;
uint32_t adc_kanal5_dma=0;


uint32_t pwm_cikis0=0;
uint32_t pwm_cikis1=0;
uint32_t pwm_cikis2=0;
uint32_t pwm_cikis3=0;
uint32_t pwm_cikis4=0;
uint32_t pwm_cikis5=0;


uint32_t map(uint32_t adc_degeri)
{
	volatile uint32_t okunacak_max = 4095;
	volatile uint32_t cevrilecek_min = 500;
	volatile uint32_t cevrilecek_max = 2500;
	return cevrilecek_min + ((cevrilecek_max - cevrilecek_min) /  (okunacak_max / adc_degeri));
}

void SysTick_Conf()
{
	SysTick->VAL = 0;
	SysTick->LOAD = 167999;
	SysTick->CTRL &= ~SysTick_CLKSource_HCLK;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


void SysTick_Handler()
{
	if(bekle>0)
		bekle--;
}

void delay_ms(uint32_t time)
{
	bekle = time;
	while(bekle);
}

void GPIO_Config()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //analog okuma i�in a portu clock  hatt� aktif

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
	//pinler analog olarak ayarland�

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //D port clock aktif

	GPIOD->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;//12,13,14,15 AF olarak ayarland�
	//af2

	GPIOD->AFR[1] |= (GPIO_AF_TIM4 << 16) | (GPIO_AF_TIM4 << 20) | (GPIO_AF_TIM4 << 24) | (GPIO_AF_TIM4 << 28); //hangi af belirtildi

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //b clock hatt� aktif edildi.

	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1; //b0,b1 af modda secildi
	GPIOB->AFR[0] |= (GPIO_AF_TIM3 << 0)  | (GPIO_AF_TIM3 << 4 ); //af2

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //c portunun clock hatt� aktif

	GPIOC->MODER |= GPIO_MODER_MODER11_1; //af olarak ayarland�
	GPIOC->AFR[1] |= GPIO_AF_UART4 << 12;//hangi af bildirildi

	GPIOC->PUPDR |= (1<<3);

}

void DMA_Config()
{
	//DMA2 Stream0 channel 0

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL; //DMA i�lemi channel 0 ayarland�
	DMA2_Stream0->CR |= DMA_SxCR_PL; //y�ksek �ncelikli yap�ld�
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_1; //g�nderilen veri boyutu 32 bit olarak ayarland�-haf�zada yer ay�r�ld�.
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_1; //g�nderilecek verinin 32 bit oldu�u ayarland�
	DMA2_Stream0->CR |= DMA_SxCR_MINC; //her veri al�m� sonras�nda memorydeki adres de�i�ecek
	DMA2_Stream0->CR &= ~DMA_SxCR_PINC; //verinin al�naca�� adres sabit b�rak�ld� ADC->DR den veriler �ekilecek
	DMA2_Stream0->CR |= DMA_SxCR_CIRC; //veri g�nderimi sonras� baslang�c adresine d�n�lecek
	DMA2_Stream0->CR &= ~DMA_SxCR_DIR; //veri cevresel birimimden haf�zaya
	DMA2_Stream0->CR |=DMA_SxCR_TCIE; //transfer tamamland� int. aktif
	DMA2_Stream0->NDTR = 6; //6 adet veri g�nderimi olucak
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR; //adc1->dr registerinin adresi verildi
	DMA2_Stream0->M0AR = (uint32_t) adc_degerleri_dma; //dizinin baslangic adresi

//	DMA2_Stream0->CR |= DMA_SxCR_EN; //dma aktif edildi
}

void DMA2_Stream0_IRQHandler()
{
	if(DMA2->LISR & DMA_LISR_TCIF0) //stream 0 �n t�m bilgileri g�nderildi�i kontrol ediliyor
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0; //int. bayra�� siliniyor

		for(uint8_t i=0;i<=5;i++)
		{
			pwm_degerleri_dma[i]= map(adc_degerleri_dma[i]);
		}
	}
}

void ADC_Config()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	ADC->CCR |= ADC_CCR_ADCPRE_0; //168 MHz kulland���m�z i�in

	NVIC_EnableIRQ(ADC_IRQn); //adc kesmesine izin verildi
	ADC1->CR1 |= ADC_CR1_OVRIE; //veri kayb� hatas�.
	ADC1->CR1 &= ~ADC_CR1_RES; //��z�n�rl�k 12 bit.
	ADC1->CR1 |= ADC_CR1_SCAN; //tarama modu aktif edildi.

	ADC1->CR2 &= ~ADC_CR2_ALIGN; //okuma sa�a dayal�

	ADC1->CR2 |= ADC_CR2_CONT; //s�rekli cevrim modu aktif

	ADC1->CR2 |= ADC_CR2_DDS; //her zaman adc dma 1 oldugu s�rece dma ya verilecek
	ADC1->CR2 |=ADC_CR2_DMA; //adc dma ile kullan�lacak;

	ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5;
	//480 cycles ek + 12 cycles cevirim s�resi

	ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_2; //6 adet okuma olacag� bildirildi.

	ADC1->SQR3 &= ~ADC_SQR3_SQ1; //1.s�ra  kanal 0
	ADC1->SQR3 |= ADC_SQR3_SQ2_0; //2. s�ra kanal 1
	ADC1->SQR3 |= ADC_SQR3_SQ3_1; //3. s�ra kanal 2
	ADC1->SQR3 |= ADC_SQR3_SQ4_0 | ADC_SQR3_SQ4_1; //4. s�ra kanal 3
	ADC1->SQR3 |= ADC_SQR3_SQ5_2; //5. s�ra kanal 4
	ADC1->SQR3 |= ADC_SQR3_SQ6_0 | ADC_SQR3_SQ6_2; //6. s�ra kanal 5
	ADC1->CR2 |= ADC_CR2_ADON; //adc birimi ba�lat�ld�
}

void ADC_IRQHandler()
{
	if(ADC1->SR & ADC_SR_OVR)
	{
		ADC1->SR &= ~ADC_SR_OVR;
		ADC1->CR2 |= ADC_CR2_SWSTART;
	}
}

void TIM4_Config()
{
	//4 servo buradan kontrol edilecek

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	//	NVIC_EnableIRQ(TIM4_IRQn);

	TIM4->CR1 &= ~TIM_CR1_CKD; //clock hatt� b�l�nmedi
	TIM4->CR1 |= TIM_CR1_ARPE;//direkt de�i�im istenMEdi
	TIM4->CR1 &= ~TIM_CR1_CMS; // say�m y�n� d�r biti ile belirlenicek
	TIM4->CR1 &= ~TIM_CR1_DIR; //say�m yukar� y�nl� ayarland�
	TIM4->CR1 &= ~TIM_CR1_OPM; // bir kez say dur kapal�
	TIM4->CR1 &= ~TIM_CR1_URS; // say�m s�f�rland�g�nda da int girer
	TIM4->CR1 &= ~TIM_CR1_UDIS; //olay g�ncellemesi aktif

	//	TIM4->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE; //capture compare 1,2,3,4 kesmeleri aktif
	TIM4->EGR |= TIM_EGR_CC1G | TIM_EGR_CC2G | TIM_EGR_CC3G | TIM_EGR_CC4G; // sayaclar s�f�rland�

	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S; //1. kanal c�k�s yap�ld�.
	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S; //2. kanal c�k�s yap�ld�.
	TIM4->CCMR2 &= ~TIM_CCMR2_CC3S; //3. kanal  ��k�s yap�ld�.
	TIM4->CCMR2 &= ~TIM_CCMR2_CC4S; //4. kanal c�k�s yap�ld�.

	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //1.c�k�s pwm ayarland�.
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; //2. c�k�s pwm i�in ayarland�.
	TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; //3. kanal pwm i�in ayarland�.
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; //4. kanal pwm i�in ayarland�.


	TIM4->CCER |= TIM_CCER_CC1E; //1. kanal aktif.
	TIM4->CCER |= TIM_CCER_CC2E; //2. kanal aktif.
	TIM4->CCER |= TIM_CCER_CC3E; //3. kanal aktif.
	TIM4->CCER |= TIM_CCER_CC4E; // 4. kanal aktif.

	TIM4->PSC = 83;
	TIM4->ARR = 19999;

	TIM4->CCR1 = 500;
	TIM4->CCR2 = 500;
	TIM4->CCR3 = 500;
	TIM4->CCR4 = 500;

	TIM4->CR1 |= TIM_CR1_CEN; //tim birimi ba�lat�ld�
}

void TIM3_Config()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //clock hatt� aktif edildi.

	TIM3->CR1 &= ~TIM_CR1_CKD; //clock hatt� b�l�nmedi
	TIM3->CR1 |= TIM_CR1_ARPE; //direkt de�i�im istenmedi.
	TIM3->CR1 &= ~TIM_CR1_CMS; //say�m y�n�n� dir biti belirleyecek.
	TIM3->CR1 &= ~TIM_CR1_DIR; //say�m yukar� y�nl�
	TIM3->CR1 &= TIM_CR1_OPM; //tek say�m modu kapal�.
	TIM3->CR1 &= ~TIM_CR1_URS; //s�f�rlama isleminde de int girilebilir sekilde
	TIM3->CR1 &= ~TIM_CR1_UDIS; //olay g�ncelle kesmeler�ne onay verilir.

	TIM3->EGR |= TIM_EGR_CC3G | TIM_EGR_CC4G; //sayaclar s�f�rland�

	TIM3->CCMR2 &= ~TIM_CCMR2_CC3S; // kanal 3 c�k�s yap�ld�.
	TIM3->CCMR2 &= ~TIM_CCMR2_CC4S; //kanal 4 c�k�s yap�ld�

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // kanal 3 pwm1 modu secildi.
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // kanal 4 pwm1 modu secildi.

	TIM3->CCER |= TIM_CCER_CC3E; //ccp3 aktif edildi.
	TIM3->CCER |= TIM_CCER_CC4E; //ccp4 aktif edildi.

	TIM3->PSC = 83;
	TIM3->ARR = 19999;

	TIM3->CCR3 = 500; //baslangic pulse degeri
	TIM3->CCR4 = 500; // baslangic pulse degeri

	TIM3->CR1 |= TIM_CR1_CEN; // sayma islemi baslat�ld�.

}

void UART_Config()
{
	//uart4 rx -> pin c 11 af8

	RCC->APB1ENR |= RCC_APB1ENR_UART4EN; //uart3 clock hatt� aktif

	NVIC_EnableIRQ(UART4_IRQn);
	UART4->BRR = 0x1117; //baud degeri 9600

	UART4->CR1 &= ~USART_CR1_OVER8; //16 �rnekleme
	UART4->CR1 &= ~USART_CR1_M; // 1 start bit,8 data bit ,n stop bit
	UART4->CR1 |= USART_CR1_RXNEIE; //read data register not empty interrrupt aktif
	UART4->CR1 |= USART_CR1_RE; //uart biriminin veri alma i�levi aktif edildi
	UART4->CR2 |= USART_CR2_STOP; // stop bit 1 cycles

	UART4->CR1 |= USART_CR1_UE; //uart birimi aktif hale getirildi
}

void UART4_IRQHandler()
{
	if(UART4->SR & USART_SR_RXNE)
	{
		gelen_karakter = (char) UART4->DR;
		gelen_veri[sayac] = gelen_karakter;
		sayac++;

		if(gelen_karakter == '\n' || gelen_karakter == '\r')
		{
			gelen_veri[sayac-1] = '\0';

			if(strcmp(gelen_veri,"aktif")==0)
			{
				pc_gelenveri_sonuc = 1;
				veri_geldi++;
			}
			else
			{
				pc_gelenveri_sonuc = 0;

			}
			sayac=0;
		}
	}
}

void EXTI_Config()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	NVIC_EnableIRQ(EXTI1_IRQn);
	SYSCFG->EXTICR[0] |= 1 << 5;

	EXTI->IMR |= EXTI_IMR_MR1; //o knal in.
	EXTI->RTSR |= EXTI_RTSR_TR1; //Y�KSELEN kenar tetiklemesi yap�lacak

}

void EXTI1_IRQHandler()
{
	if(EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR |= EXTI_PR_PR1;
		pic16dan_gelen_sonuc = 1;
	}

}

int main(void)
{
	SysTick_Conf();
	GPIO_Config();
	UART_Config();
	DMA_Config();
	ADC_Config();
	TIM4_Config();
	TIM3_Config();
	EXTI_Config();

	while (1)
	{
		if(pc_gelenveri_sonuc == 1 && pic16dan_gelen_sonuc == 1)
		{
			ADC1->CR2 &= ~ADC_CR2_SWSTART;
			delay_ms(5);
			ADC1->CR2 |= ADC_CR2_SWSTART; //adc cevrimi baslat�ld�
			DMA2_Stream0->CR |= DMA_SxCR_EN; //dma aktif edildi

			delay_ms(500);

			TIM4->CCR1 =  pwm_degerleri_dma[0];
			delay_ms(100);

			TIM4->CCR2 = pwm_degerleri_dma[1];
			delay_ms(100);

			TIM4->CCR3 = pwm_degerleri_dma[2];
			delay_ms(100);

			TIM4->CCR4 = pwm_degerleri_dma[3];
			delay_ms(100);

			TIM3->CCR3 = pwm_degerleri_dma[4];
			delay_ms(100);

			TIM3->CCR4 = pwm_degerleri_dma[5];
			delay_ms(100);


			  adc_kanal0_dma= adc_degerleri_dma[0];
			  adc_kanal1_dma= adc_degerleri_dma[1];
			  adc_kanal2_dma= adc_degerleri_dma[2];
			  adc_kanal3_dma= adc_degerleri_dma[3];
			  adc_kanal4_dma= adc_degerleri_dma[4];
			  adc_kanal5_dma= adc_degerleri_dma[5];

			  pwm_cikis0= pwm_degerleri_dma[0];
			  pwm_cikis1= pwm_degerleri_dma[1];
			  pwm_cikis2= pwm_degerleri_dma[2];
			  pwm_cikis3= pwm_degerleri_dma[3];
			  pwm_cikis4= pwm_degerleri_dma[4];
			  pwm_cikis5= pwm_degerleri_dma[5];
		}
		else
		{
			ADC1->CR2 &= ~ADC_CR2_SWSTART;
			DMA2_Stream0->CR &= ~DMA_SxCR_EN; //dma aktif edildi
		}
	}
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

	return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){

	return -1;
}
