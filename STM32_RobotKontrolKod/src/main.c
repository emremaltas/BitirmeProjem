#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include<string.h>

//6.05.2022 Emre Maltaþ


uint32_t adc_degerleri_dma[6] = {0};

uint32_t pwm_degerleri_dma[6] = {0};

uint32_t bekle=0;

char gelen_veri[100]= {0};

char gelen_karakter=0;
uint8_t sayac=0;

uint8_t pc_gelenveri_sonuc = 0,pic16dan_gelen_sonuc=0,veri_geldi=0;



//tim2-5 kullanýlabilir 32

//tim3-4 kullanýlabilir 16 bit  (TÝM4 -> D12,13,14,15)

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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //analog okuma için a portu clock  hattý aktif

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
	//pinler analog olarak ayarlandý

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //D port clock aktif

	GPIOD->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;//12,13,14,15 AF olarak ayarlandý
	//af2

	GPIOD->AFR[1] |= (GPIO_AF_TIM4 << 16) | (GPIO_AF_TIM4 << 20) | (GPIO_AF_TIM4 << 24) | (GPIO_AF_TIM4 << 28); //hangi af belirtildi

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //b clock hattý aktif edildi.

	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1; //b0,b1 af modda secildi
	GPIOB->AFR[0] |= (GPIO_AF_TIM3 << 0)  | (GPIO_AF_TIM3 << 4 ); //af2

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //c portunun clock hattý aktif

	GPIOC->MODER |= GPIO_MODER_MODER11_1; //af olarak ayarlandý
	GPIOC->AFR[1] |= GPIO_AF_UART4 << 12;//hangi af bildirildi

	GPIOC->PUPDR |= (1<<3);

}

void DMA_Config()
{
	//DMA2 Stream0 channel 0

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL; //DMA iþlemi channel 0 ayarlandý
	DMA2_Stream0->CR |= DMA_SxCR_PL; //yüksek öncelikli yapýldý
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_1; //gönderilen veri boyutu 32 bit olarak ayarlandý-hafýzada yer ayýrýldý.
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_1; //gönderilecek verinin 32 bit olduðu ayarlandý
	DMA2_Stream0->CR |= DMA_SxCR_MINC; //her veri alýmý sonrasýnda memorydeki adres deðiþecek
	DMA2_Stream0->CR &= ~DMA_SxCR_PINC; //verinin alýnacaðý adres sabit býrakýldý ADC->DR den veriler çekilecek
	DMA2_Stream0->CR |= DMA_SxCR_CIRC; //veri gönderimi sonrasý baslangýc adresine dönülecek
	DMA2_Stream0->CR &= ~DMA_SxCR_DIR; //veri cevresel birimimden hafýzaya
	DMA2_Stream0->CR |=DMA_SxCR_TCIE; //transfer tamamlandý int. aktif
	DMA2_Stream0->NDTR = 6; //6 adet veri gönderimi olucak
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR; //adc1->dr registerinin adresi verildi
	DMA2_Stream0->M0AR = (uint32_t) adc_degerleri_dma; //dizinin baslangic adresi

//	DMA2_Stream0->CR |= DMA_SxCR_EN; //dma aktif edildi
}

void DMA2_Stream0_IRQHandler()
{
	if(DMA2->LISR & DMA_LISR_TCIF0) //stream 0 ýn tüm bilgileri gönderildiði kontrol ediliyor
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0; //int. bayraðý siliniyor

		for(uint8_t i=0;i<=5;i++)
		{
			pwm_degerleri_dma[i]= map(adc_degerleri_dma[i]);
		}
	}
}

void ADC_Config()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	ADC->CCR |= ADC_CCR_ADCPRE_0; //168 MHz kullandýðýmýz için

	NVIC_EnableIRQ(ADC_IRQn); //adc kesmesine izin verildi
	ADC1->CR1 |= ADC_CR1_OVRIE; //veri kaybý hatasý.
	ADC1->CR1 &= ~ADC_CR1_RES; //çözünürlük 12 bit.
	ADC1->CR1 |= ADC_CR1_SCAN; //tarama modu aktif edildi.

	ADC1->CR2 &= ~ADC_CR2_ALIGN; //okuma saða dayalý

	ADC1->CR2 |= ADC_CR2_CONT; //sürekli cevrim modu aktif

	ADC1->CR2 |= ADC_CR2_DDS; //her zaman adc dma 1 oldugu sürece dma ya verilecek
	ADC1->CR2 |=ADC_CR2_DMA; //adc dma ile kullanýlacak;

	ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5;
	//480 cycles ek + 12 cycles cevirim süresi

	ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_2; //6 adet okuma olacagý bildirildi.

	ADC1->SQR3 &= ~ADC_SQR3_SQ1; //1.sýra  kanal 0
	ADC1->SQR3 |= ADC_SQR3_SQ2_0; //2. sýra kanal 1
	ADC1->SQR3 |= ADC_SQR3_SQ3_1; //3. sýra kanal 2
	ADC1->SQR3 |= ADC_SQR3_SQ4_0 | ADC_SQR3_SQ4_1; //4. sýra kanal 3
	ADC1->SQR3 |= ADC_SQR3_SQ5_2; //5. sýra kanal 4
	ADC1->SQR3 |= ADC_SQR3_SQ6_0 | ADC_SQR3_SQ6_2; //6. sýra kanal 5
	ADC1->CR2 |= ADC_CR2_ADON; //adc birimi baþlatýldý
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

	TIM4->CR1 &= ~TIM_CR1_CKD; //clock hattý bölünmedi
	TIM4->CR1 |= TIM_CR1_ARPE;//direkt deðiþim istenMEdi
	TIM4->CR1 &= ~TIM_CR1_CMS; // sayým yönü dýr biti ile belirlenicek
	TIM4->CR1 &= ~TIM_CR1_DIR; //sayým yukarý yönlü ayarlandý
	TIM4->CR1 &= ~TIM_CR1_OPM; // bir kez say dur kapalý
	TIM4->CR1 &= ~TIM_CR1_URS; // sayým sýfýrlandýgýnda da int girer
	TIM4->CR1 &= ~TIM_CR1_UDIS; //olay güncellemesi aktif

	//	TIM4->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE; //capture compare 1,2,3,4 kesmeleri aktif
	TIM4->EGR |= TIM_EGR_CC1G | TIM_EGR_CC2G | TIM_EGR_CC3G | TIM_EGR_CC4G; // sayaclar sýfýrlandý

	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S; //1. kanal cýkýs yapýldý.
	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S; //2. kanal cýkýs yapýldý.
	TIM4->CCMR2 &= ~TIM_CCMR2_CC3S; //3. kanal  çýkýs yapýldý.
	TIM4->CCMR2 &= ~TIM_CCMR2_CC4S; //4. kanal cýkýs yapýldý.

	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //1.cýkýs pwm ayarlandý.
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; //2. cýkýs pwm için ayarlandý.
	TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; //3. kanal pwm için ayarlandý.
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; //4. kanal pwm için ayarlandý.


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

	TIM4->CR1 |= TIM_CR1_CEN; //tim birimi baþlatýldý
}

void TIM3_Config()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //clock hattý aktif edildi.

	TIM3->CR1 &= ~TIM_CR1_CKD; //clock hattý bölünmedi
	TIM3->CR1 |= TIM_CR1_ARPE; //direkt deðiþim istenmedi.
	TIM3->CR1 &= ~TIM_CR1_CMS; //sayým yönünü dir biti belirleyecek.
	TIM3->CR1 &= ~TIM_CR1_DIR; //sayým yukarý yönlü
	TIM3->CR1 &= TIM_CR1_OPM; //tek sayým modu kapalý.
	TIM3->CR1 &= ~TIM_CR1_URS; //sýfýrlama isleminde de int girilebilir sekilde
	TIM3->CR1 &= ~TIM_CR1_UDIS; //olay güncelle kesmelerþne onay verilir.

	TIM3->EGR |= TIM_EGR_CC3G | TIM_EGR_CC4G; //sayaclar sýfýrlandý

	TIM3->CCMR2 &= ~TIM_CCMR2_CC3S; // kanal 3 cýkýs yapýldý.
	TIM3->CCMR2 &= ~TIM_CCMR2_CC4S; //kanal 4 cýkýs yapýldý

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // kanal 3 pwm1 modu secildi.
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // kanal 4 pwm1 modu secildi.

	TIM3->CCER |= TIM_CCER_CC3E; //ccp3 aktif edildi.
	TIM3->CCER |= TIM_CCER_CC4E; //ccp4 aktif edildi.

	TIM3->PSC = 83;
	TIM3->ARR = 19999;

	TIM3->CCR3 = 500; //baslangic pulse degeri
	TIM3->CCR4 = 500; // baslangic pulse degeri

	TIM3->CR1 |= TIM_CR1_CEN; // sayma islemi baslatýldý.

}

void UART_Config()
{
	//uart4 rx -> pin c 11 af8

	RCC->APB1ENR |= RCC_APB1ENR_UART4EN; //uart3 clock hattý aktif

	NVIC_EnableIRQ(UART4_IRQn);
	UART4->BRR = 0x1117; //baud degeri 9600

	UART4->CR1 &= ~USART_CR1_OVER8; //16 örnekleme
	UART4->CR1 &= ~USART_CR1_M; // 1 start bit,8 data bit ,n stop bit
	UART4->CR1 |= USART_CR1_RXNEIE; //read data register not empty interrrupt aktif
	UART4->CR1 |= USART_CR1_RE; //uart biriminin veri alma iþlevi aktif edildi
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
	EXTI->RTSR |= EXTI_RTSR_TR1; //YÐKSELEN kenar tetiklemesi yapýlacak

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
			ADC1->CR2 |= ADC_CR2_SWSTART; //adc cevrimi baslatýldý
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
