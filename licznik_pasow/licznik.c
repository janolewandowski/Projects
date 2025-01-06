#include "main.h"
#include "licznik.h"
#include "i2c-lcd.h"
#include "UI.h"
#include "button.h"
#include "bluetooth.h"
#include "eeprom.h"
#include "math.h"
#include "stdbool.h"
#include <stdio.h>

//#define fedon	// INNE CEWKI DLA ADAMA
#define BTDEBUG	// WYSYLAJ DEBUG NA BT
#define TESTENC //TESOWE KOLKO
//States of relays
bool K1, K2, K3, K4;
// Sprayers widths
bool SprayerWidth[4] = {WIDTH24CM, WIDTH12CM, WIDTH24CM, WIDTH12CM};

//Variable for user encoder
int UserEncoder;

//Variables for measurements of distance
//From start to stop
uint32_t Steps;
//Variable for saving steps when relays switched off
uint32_t LastSteps;

//Steps for updating
int EncSteps;

//Variables for measurements of painting and pause of each sprayer
int Pause[4];
int Paint[4];
int PauseSteps[4];
int PaintSteps[4];

LineTypes RecognizedLineType[4] = {L_NOLINE, L_NOLINE, L_NOLINE, L_NOLINE};
MarkingTypes RecognizedMarkingType = M_NOMARKING;

// Array for memory of lines in steps to save/read EEPROM
uint32_t MarkingsSteps[13];
// Local array for memory of lines in steps for each sprayer
uint32_t LinesSteps[4][7];

const float PaintCalculationsMap[] = {0, 0.12, 0.12, 0.24, 0.06, 0.04, 0.08, 0.04, 0.12, 0.24, 0.18, 0.2};

// TABLICA Z DLUGOSCIAMI MALOWANIA
const int PaintMap[6] = {STEPSPERMETER*0.5, STEPSPERMETER*1, STEPSPERMETER*2, STEPSPERMETER*2, STEPSPERMETER*4, STEPSPERMETER*4};

// TABLICA Z DLUGOSCIAMI PAUZY
const int PauseMap[6] = {STEPSPERMETER*0.5, STEPSPERMETER*1, STEPSPERMETER*2, STEPSPERMETER*4, STEPSPERMETER*2, STEPSPERMETER*8};

// Variables line banks
int selectedBank = 0;
// Variables for menu purposes
AllMainMenuItems SelectedMenuItem = BANK;
int SelectedDisplayBank = 0;

// Variables for button purposes
struct BUTTON Start, Stop, Ok;

//Flag - painting in process
bool Painting = 0;

// Extern timers handlers
extern TIM_HandleTypeDef htim4, htim3, htim2;
extern I2C_HandleTypeDef hi2c1;;


// ************** INIT **************
void LicznikInit(){

	// Power On
	HAL_GPIO_WritePin(POWER_TFT_GPIO_Port, POWER_TFT_Pin, 1);
	HAL_GPIO_WritePin(POWER_EEPROM_GPIO_Port, POWER_EEPROM_Pin, 1);
	HAL_GPIO_WritePin(POWER_ENCODER_GPIO_Port, POWER_ENCODER_Pin, 1);
	HAL_GPIO_WritePin(POWER_HC_05_GPIO_Port, POWER_HC_05_Pin, 1);

	// Timer for start/stop

	Ok.GPIO_Pin = ENC_SW_Pin;
	Ok.GPIO_Port = ENC_SW_GPIO_Port;

	Start.GPIO_Pin = StartButton_Pin;
	Start.GPIO_Port = StartButton_GPIO_Port;

	Stop.GPIO_Pin = StopButton_Pin;
	Stop.GPIO_Port = StopButton_GPIO_Port;

	Ok.Inertion = 50;
	Stop.Inertion = 50;
	Start.Inertion = 50;

	Ok.lastState = 1;
	Start.lastState = 1;
	Stop.lastState = 1;
}
// ************** START MAIN APP **************
void StartLicznikApp(){
    // Enable the backup domain access
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN; // Power and Backup clocks
    PWR->CR |= PWR_CR_DBP; // Enable write access to backup domain

    HAL_Delay(1000);

    if (BKP->DR1 == 0x1234) {
           // Clear the backup register flag
           BKP->DR1 = 0;
           // Skip reset
       } else {
           // Trigger a reset
    	    BKP->DR1 = 0x1234; // Write a flag to Backup Register 1
    	    NVIC_SystemReset(); // Perform a software reset
       }


	LicznikInit();
//	HAL_Delay(200);
//
//	for (int i = 0; i < 5; i++){
//		lcd_init();
//		HAL_Delay(10);
//	}
	lcd_init();
	HAL_Delay(10);
	lcd_clear();
	SelectedMenuItem = BANK;
	lcd_write("  PaintingLines.pl", 1, 0);
	lcd_write("     882-676-758   ", 2, 0);
#ifdef fedon
	lcd_write("v1.12 FEDON", 3, 0);
#else
	lcd_write("v1.12", 3, 0);
#endif
	EEPROM_ReadBank(); // read all banks
	HAL_Delay(2000);
	// ************** DEBUG **************
	if(!HAL_GPIO_ReadPin(StopButton_GPIO_Port, StopButton_Pin)) debug();
	// ************** DEBUG **************
	lcd_clear();
		while(1){
			while (!Painting){
				NoPaintingInProcess();
			}

			if (Painting){
				PaintingInProcess();
			}
		}
}

// ************** PROCES MALOWANIA **************
void PaintingInProcess(){
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	ClearPaintingSteps();
	lcd_clear();
	LCD_PaintingInProcess();
	while (Painting){
		EncoderUpdate();
		//LCD_PaintingLines();
		if (DIN_Process(&Stop) == LONG_PRESS){
			RelaysUpdate();
			LineRecognize();
			MarkingRecognize();
			Painting = 0;
		}
	}
#ifdef BTDEBUG
		debugBT();
#endif
	EEPROM_SaveBank();
	lcd_clear();
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
// ************** PROCES NIE MALOWANIA **************
void NoPaintingInProcess(){
				LCD_Menu();
				BT_CheckRX();
				UserEncoderMenu_Process();
				RelaysUpdate();
				if (DIN_Process(&Ok) == SINGLE_PRESS){
						switch (SelectedMenuItem){
							case BANK:
								selectedBank++;
								if (selectedBank > 3) selectedBank = 0;
								break;
							case READ_DATA:
								EEPROM_ReadBank();
								LCD_DisplayBank(SelectedDisplayBank, 1);
								while (DIN_Process(&Ok) != SINGLE_PRESS && !Painting){
									LCD_DisplayBank(SelectedDisplayBank, 0);
									UserEncoderBank_Process();
									}
								lcd_clear();
								break;
							case SEND_DATA:
								EEPROM_ReadBank();
								LCD_SendingData();
								SendDataBT();
								lcd_clear();
								break;
							case RESET_DATA:
								LCD_ResetBank();
								while(1){
									int process = DIN_Process(&Ok);
									switch (process){
										case SINGLE_PRESS:
											goto afterReset;
											break;
										case LONG_PRESS:
											EEPROM_PageErase(selectedBank);
											EEPROM_ReadBank();
											LCD_ResetOKMsg();
											HAL_Delay(1500);
											goto afterReset;
											break;
									}
								}
								afterReset:
								lcd_clear();
								break;
							}
				}
}
// ************** ROZPOZNAWANIE RODZAJU LINII NA PODSTAWIE DANYCH W TABELACH PAINTSTEPS ORAZ PAUSESTEPS	**************
void LineRecognize(){
	//Array for calculations
	double CalculationResults[6];

	for (int sprayer = 0; sprayer < 4; sprayer++){

		// Painting without Pause  -> C (6)
				if (PaintSteps[sprayer] > 0 && PauseSteps[sprayer] == 0 ){
					RecognizedLineType[sprayer] = L_C;
					LinesSteps[sprayer][L_C] = PaintSteps[sprayer];
					continue;
				}

		// When no steps for recognize
		if (!(PauseSteps[sprayer] != 0 && PaintSteps[sprayer] != 0)) continue;;

			// (PauseSteps > 0 && PaintSteps > 0)
			// It can be LineType 0 - 5
			// Divide PauseSteps/PauseMap
			// Absolute value
			for (int LineType = L_05x05; LineType < L_C; LineType++){
				CalculationResults[LineType]=(double)PaintSteps[sprayer]/(double)PaintMap[LineType];
				CalculationResults[LineType]=CalculationResults[LineType]-1;
				CalculationResults[LineType]=fabs(CalculationResults[LineType]);
			}

			// Find the smallest
			// Zakladamy, że to jest najmniejsze
			double smallest = CalculationResults[0];
			RecognizedLineType[sprayer] = L_05x05;

			for (int LineType = L_05x05; LineType < L_C; LineType++) {
				 if (CalculationResults[LineType] < smallest) {
					 smallest = CalculationResults[LineType];
					 RecognizedLineType[sprayer] = LineType;
				  }
			}
			// JESLI MALOWANA 05x05 NIE WEJDZIE W ZADNEGO IFa
			// JESLI MALOWANA 1x1   NIE WEJDZIE W ZADNEGO IFa

			// JESLI MALOWANO DWA METRY - MOZLIWE 2x2 LUB 2x4
			if (RecognizedLineType[sprayer] == L_2x2 || RecognizedLineType[sprayer] == L_2x4){
				CalculationResults[2]=(double)PauseSteps[sprayer]/(double)PauseMap[2];
				CalculationResults[4]=(double)PauseSteps[sprayer]/(double)PauseMap[4];

				CalculationResults[2]=CalculationResults[2]-1;
				CalculationResults[4]=CalculationResults[4]-1;

				CalculationResults[2]=fabs(CalculationResults[2]);
				CalculationResults[4]=fabs(CalculationResults[4]);

				if (CalculationResults[2] < CalculationResults[4]){
					RecognizedLineType[sprayer] = L_2x2;
				}else{
					RecognizedLineType[sprayer] = L_2x4;
				}
			}

			// JESLI MALOWANO CZTERY METRY - MOZLIWE 4x2 LUB 4x8
			if (RecognizedLineType[sprayer] == L_4x2 || RecognizedLineType[sprayer] == L_4x8){
				CalculationResults[2]=(double)PauseSteps[sprayer]/(double)PauseMap[2];
				CalculationResults[4]=(double)PauseSteps[sprayer]/(double)PauseMap[4];

				CalculationResults[2]=CalculationResults[2]-1;
				CalculationResults[4]=CalculationResults[4]-1;

				CalculationResults[2]=fabs(CalculationResults[2]);
				CalculationResults[4]=fabs(CalculationResults[4]);

				if (CalculationResults[2] < CalculationResults[4]){
					RecognizedLineType[sprayer] = L_4x2;
				}else{
					RecognizedLineType[sprayer] = L_4x8;
				}
			}

			// Save steps to 2D array of RecognizedLineType (0 - 5) for each sprayer
			LinesSteps[sprayer][RecognizedLineType[sprayer]] += Steps;
	}
}
// ************** ROZPOZNAWANIE RODZAJU MALOWANIA NA PODSTAWIE WYKRYTYCH LINII	**************
void MarkingRecognize(){
	RecognizedMarkingType = M_NOMARKING;

	// Check how many LineType was recognized on each sprayer
	int ActiveLines = 0;
	int SprayerIndex = 0;
	int LineTypeIndex = 0;

	for (int Sprayer = 0; Sprayer < 4; Sprayer++){
		for (int LineType = 0; LineType <= 6; LineType++){
			if 	(LinesSteps[Sprayer][LineType] > 0){
				SprayerIndex = Sprayer;
				LineTypeIndex = LineType;
				ActiveLines++;
			}
		}
	}


// ######################## SINGLE LINE TYPE ########################
	// If only one LineType was detected -> BankMemory (0-8)
	if (ActiveLines == 1){
		// If SprayerWidth = 12CM  -> BankMemory (4-8)
		// 12CM 	 1/1 (1) , 2/4 (3) , 4/2 (4)  , 4/8 (5) , C (6)
		if (SprayerWidth[SprayerIndex] == WIDTH12CM){
			switch(LineTypeIndex){
			case 1:
				MarkingsSteps[4] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 4;
				return; //Save to Bank 4
				break;
			case 3:
				MarkingsSteps[5] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 5;
				return; //Save to Bank 5
				break;
			case 4:
				MarkingsSteps[6] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 6;
				return; //Save to Bank 6
				break;
			case 5:
				MarkingsSteps[7] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 7;
				return; //Save to Bank 7
				break;
			case 6:
				MarkingsSteps[8] += LastSteps;
				RecognizedMarkingType = 8;
				return; //Save to Bank 8
				break;
			}
		}



		// If SprayerWidth = 24CM  -> BankMemory (0-3)
		// 24CM - 	0,5/0,5  (0), 1/1  (1), 2/2 (2),  C (6)
		if (SprayerWidth[SprayerIndex] == WIDTH24CM){
			switch(LineTypeIndex){
			case 0:
				MarkingsSteps[0] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 0;
				return; //Save to Bank 0
				break;
			case 1:
				MarkingsSteps[1] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 1;
				return; //Save to Bank 1
				break;
			case 2:
				MarkingsSteps[2] += LastSteps + PauseSteps[SprayerIndex];
				RecognizedMarkingType = 2;
				return; //Save to Bank 2
				break;
			case 6:
				MarkingsSteps[3] += LastSteps;
				RecognizedMarkingType = 3;
				return; //Save to Bank 3
				break;
			}
		}
	}
// ######################## DOUBLE LINE TYPE ########################
	// If two LineType was detected -> BankMemory (9-12)
	// Available BankMemory - C|C, 4/2|4/2, C|1/1, C|4/2

	if (ActiveLines == 2){
		for (int sprayer = 0; sprayer < 4; sprayer++){
			// Only 12 cm sprayers for DOUBLE LINES
			if (SprayerWidth[sprayer] == WIDTH24CM) continue;
				// Find C|C, C|1/2, C|4/2
				if 	(LinesSteps[sprayer][6] > 0){

					for (int Sprayer = 0; Sprayer < 4; Sprayer++){
						// Only 12 cm sprayers for DOUBLE LINES
						if (SprayerWidth[Sprayer] == WIDTH24CM) continue;
						// One sprayer can't  paint two lines in one painting
						if (Sprayer == sprayer) continue;
						// (6) 	C | C	(6)
						if (LinesSteps[Sprayer][6] > 0){
							MarkingsSteps[9] += LastSteps;
							RecognizedMarkingType = 9;
							return; //Save to Bank 9
						}
						// (6) 	C | 1/1	(1)
						if (LinesSteps[Sprayer][1] > 0){
							MarkingsSteps[11] += LastSteps;
							RecognizedMarkingType = 11;
							return; //Save to Bank 11
						}
						// (6) 	C | 4/2	(4)
						if (LinesSteps[Sprayer][4] > 0){
							MarkingsSteps[12] += LastSteps;
							RecognizedMarkingType = 12;
							return;	//Save to Bank 12
						}
					}

				}
				// Find 4/2 | 4/2 -- NIEUZYWANA
//				if 	(LinesSteps[sprayer][4] > 0){
//					for (int Sprayer = 0; Sprayer < 4; Sprayer++){
//						// One sprayer can't  paint two lines in one painting
//						if (Sprayer == sprayer) continue;
//						// (4) 	4/2 | 4/2 	(4)
//						if (LinesSteps[Sprayer][4] > 0){
//							MarkingsSteps[10] = MarkingsSteps[10] + Steps + 2*STEPSPERMETER;
//							RecognizedMarking = 10;
//							return;	//Save to Bank 10
//						}
//					}
//				}
		}
	}

	// ######################## DOUBLE LINE TYPE RE-PAINTING ########################
	// Repainting LineType 2/2 24cm , 2/4 , 4/2, 4/8
//	if (ActiveLines == 2){
//		for (int sprayer = 0; sprayer < 4; sprayer++){
//			// Re-paint 2/2 && 2/4
//			if (LinesSteps[sprayer][2] > 0){
//				if (LinesSteps[sprayer][3] > 0){
//
//					// Last Recogized Line was 2/2 so save unrecognized steps to it
//					if (RecognizedLineType[sprayer] == 2){
//						LinesSteps[sprayer][2] += PaintSteps[sprayer];
//					}else if (RecognizedLineType[sprayer] == 3){
//						LinesSteps[sprayer][3] += PaintSteps[sprayer];
//					}
//					//RecognizedMarking =
//				}
//			}
//
//			// Re-paint 4/2 && 4/8
//			if (LinesSteps[sprayer][4] > 0){
//				if (LinesSteps[sprayer][5] > 0){
//
//					// Last RecogizedLineType was 4/2 so save unrecognized steps to it
//					if (RecognizedLineType == 4){
//						LinesSteps[sprayer][4] += PaintSteps[sprayer];
//						// DODANE
//						LinesSteps[sprayer][4] += PauseSteps[sprayer];
//
//					}else if (RecognizedLineType == 5){
//						LinesSteps[sprayer][5] += PaintSteps[sprayer];
//
//						// DODANE
//						LinesSteps[sprayer][5] += PauseSteps[sprayer];
//					}
//
//					MarkingsSteps[6] = MarkingsSteps[6] + (LinesSteps[sprayer][4]);
//					MarkingsSteps[7] = MarkingsSteps[7] + (LinesSteps[sprayer][5]);
//					RecognizedMarkingType = 13;
//				}
//			}
//		}
//	}
// ######################## DOUBLE LINE TYPE RE-PAINTING ########################

return;
}

// ************** PROCESOWANIE KROKÓW Z ENKODERA **************
void EncoderUpdate(){
    RelaysUpdate();

    if (EncSteps == 0) return;

    // Array of relay states for each sprayer
    bool relayStates[] = {K1, K2, K3, K4};

    for (int i = 0; i < 4; ++i) {
        if (!relayStates[i]) {  // Paint mode
            Paint[i] += EncSteps;
            if (Pause[i] == 0) {
                PaintSteps[i] = Paint[i];
            }
            else if (PauseSteps[i] == 0) {
                PauseSteps[i] = Pause[i];
            }
        } else {  // Pause mode
            if (Paint[i] > 0) {
                Pause[i] += EncSteps;
            }
        }
    }

    EncSteps = 0;
}
// ************** PROCESOWANIE KROKÓW Z ENKODERA	**************
void EncoderUpdateOLD(){

	RelaysUpdate();

		//Main variable for distance
	if (EncSteps == 0) return;


		if (!K1){
			Paint[0]+= EncSteps;
			if (Pause[0] == 0) PaintSteps[0] = Paint[0];
			if (Pause[0] > 0 && PauseSteps[0] == 0) PauseSteps[0] = Pause[0];
		}else{
			if (Paint[0] > 0) Pause[0]+= EncSteps;
		}

		if (!K2){
			Paint[1]+= EncSteps;
			if (Pause[1] == 0) PaintSteps[1] = Paint[1];
			if (Pause[1] > 0 && PauseSteps[1] == 0) PauseSteps[1] = Pause[1];
		}else{
			if (Paint[1] > 0) Pause[1]+= EncSteps;
		}

		if (!K3){
			Paint[2]+= EncSteps;
			if (Pause[2] == 0) PaintSteps[2] = Paint[2];
			if (Pause[2] > 0 && PauseSteps[2] == 0) PauseSteps[2] = Pause[2];
		}else{
			if (Paint[2] > 0) Pause[2]+= EncSteps;
		}

		if (!K4){
			Paint[3]+= EncSteps;
			if (Pause[3] == 0) PaintSteps[3] = Paint[3];
			if (Pause[3] > 0 && PauseSteps[3] == 0) PauseSteps[3] = Pause[3];
		}else{
			if (Paint[3] > 0) Pause[3]+= EncSteps;
		}

		EncSteps = 0;
}
// ************** CZYSZCZENIE PaintSteps/PauseSteps/Paint/Pause	**************
void ClearPaintingSteps(){
	Steps = 0;
	EncSteps = 0;
	RecognizedMarkingType = M_NOMARKING;
	for (int Sprayer = 0; Sprayer < 4; Sprayer++){

		// Clearing memory for each sprayer of each recognized line type
		for (int LineType = 0; LineType <= 6; LineType++){
			LinesSteps[Sprayer][LineType] = 0;
		}
		RecognizedLineType[Sprayer] = L_NOLINE;
		// Clearing steps variable for sprayers
		Paint[Sprayer] = 0;
		PaintSteps[Sprayer] = 0;
		Pause[Sprayer] = 0;
		PauseSteps[Sprayer] = 0;
	}
}
// ************** UPDATE RELAYS STATE **************
void RelaysUpdateUPDATED() {
    static bool LastState = 0;

    #ifdef fedon
        // FEDON MALOWARKA
        K1 = HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin); // K1
        K2 = HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin); // K2
        K3 = HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin); // K3
        K4 = HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin); // K4
    #else
        // POZOSTALE MALOWARKI
        K1 = HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin);
        K2 = HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin);
        K3 = HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin);
        K4 = HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin);
    #endif

	// Use bitwise operations to check relay states
	 uint8_t relayState_OR = K1 | K2 | K3 | K4;   // Checks if any relay is low (0)
	 uint8_t relayState_AND = K1 & K2 & K3 & K4;  // Checks if all relays are high (1)

	 // Check if any relay is low
	 if (relayState_OR == 0) {
		 Painting = 1;
		 LastState = 1;
	 }

	 // Check if all relays are high and LastState was 1
	 if (relayState_AND == 1 && LastState == 1) {
		 LastState = 0;
		 LastSteps = Steps;
	 }
}
void RelaysUpdate(){

static bool LastState = 0;

#ifdef fedon
	// FEDON MALOWARKA
	K1 = HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin); // K1
	K2 = HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin); // K2
	K3 = HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin); // K3
	K4 = HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin); // K4
#else
	// POZOSTALE MALOWARKI
	K1 = HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin);
	K2 = HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin);
	K3 = HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin);
	K4 = HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin);
#endif

	if (!K1 | !K2 | !K3 | !K4){
		Painting = 1;
		LastState = 1;
	}

	if (K1 && K2 && K3 && K4 && LastState == 1){
		LastState = 0;
		LastSteps = Steps;
	}


}
// ************** READING BANK ENCODER **************
int UserEncoderBank_Process(void)
{
	//Update time encoder
	static uint32_t LastDelayTicks = 0;
	if (HAL_GetTick() - LastDelayTicks < 500){
		return 0;
	}
	LastDelayTicks = HAL_GetTick();

	if (UserEncoder == 1){
		UserEncoder = 0;
		SelectedDisplayBank++;
		if (SelectedDisplayBank == 7) SelectedDisplayBank = 0;
	}

	if (UserEncoder == -1){
		UserEncoder = 0;
		SelectedDisplayBank--;
		if (SelectedDisplayBank == -1) SelectedDisplayBank = 6;
	}

	//HAL_Delay(100);
	return 0;
}
// ************** MENU USER ENCODER **************
int UserEncoderMenu_Process(void)
{
	//Update time encoder
	static uint32_t LastDelayTicks = 0;
	if (HAL_GetTick() - LastDelayTicks < 500){
		return 0;
	}
	LastDelayTicks = HAL_GetTick();

	if (UserEncoder == 1){
		UserEncoder = 0;
		SelectedMenuItem++;
		if (SelectedMenuItem == 4) SelectedMenuItem = 3;
	}

	if (UserEncoder == -1){
		UserEncoder = 0;
		SelectedMenuItem--;
		if (SelectedMenuItem == -1) SelectedMenuItem = 0;
	}

	//HAL_Delay(300);
	return 0;
}
// ************** TIMER START/STOP BUTTON **************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim2){
		//lcd_init();
	}

}
// ************** PRZERWANIA ENKODER ORAZ USER ENCODER **************
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

#ifndef TESTENC
	//DZIALAJCE KOLKO
	if(GPIO_Pin == ENC_WHEEL2_Pin){
		if (HAL_GPIO_ReadPin(ENC_WHEEL_GPIO_Port, ENC_WHEEL_Pin)){
			//RelaysUpdate();
			EncSteps++;
			Steps++;
		}
		return;
	}
#else
	//TESTOWE KOLKO
	if(GPIO_Pin == ENC_WHEEL_Pin){
		//if (HAL_GPIO_ReadPin(ENC_WHEEL_GPIO_Port, ENC_WHEEL_Pin)){
			EncSteps++;
			Steps++;
		//}
		return;
	}
#endif
	// OLD
//	//DZIALAJCE KOLKO
//	if(GPIO_Pin == ENC_WHEEL2_Pin){
//		if (HAL_GPIO_ReadPin(ENC_WHEEL_GPIO_Port, ENC_WHEEL_Pin)){
//			RelaysUpdate();
//			EncSteps++;
//			Steps++;
//		}
//		return;
//	}

//	//TESTOWE KOLKO
//	if(GPIO_Pin == ENC_WHEEL_Pin){
//		//if (HAL_GPIO_ReadPin(ENC_WHEEL_GPIO_Port, ENC_WHEEL_Pin)){
//			//EncSteps++;
//			EncoderUpdate();
//		//}
//		return;
//	}

	  if (GPIO_Pin == ENC_A_Pin && UserEncoder == 0) {
		if (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)) {
			UserEncoder = 1;
			return;
		}

		if (!HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)) {
			UserEncoder = -1;
			return;
		}
	  }
}
