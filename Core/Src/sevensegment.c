#include "sevensegment.h"
#include "main.h"

void ssSetDigit(int digit) {
	switch (digit) {
	case 1:
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D1_GPIO_Port, D2_Pin | D3_Pin | D4_Pin, GPIO_PIN_SET);
		break;

	case 2:
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin | D3_Pin | D4_Pin, GPIO_PIN_SET);
		break;

	case 3:
		HAL_GPIO_WritePin(D2_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin | D2_Pin | D4_Pin, GPIO_PIN_SET);
		break;

	case 4:
		HAL_GPIO_WritePin(D2_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin | D2_Pin | D3_Pin, GPIO_PIN_SET);
		break;
	}
}

void ssDisplayZero() {
	HAL_GPIO_WritePin(A_GPIO_Port,
	A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin, GPIO_PIN_SET);
}

void ssDisplayOne() {
	HAL_GPIO_WritePin(A_GPIO_Port, B_Pin | C_Pin, GPIO_PIN_SET);
}

void ssDisplayTwo() {
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin | B_Pin | G_Pin | E_Pin | D_Pin,
			GPIO_PIN_SET);
}

void ssDisplayThree() {
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin | B_Pin | C_Pin | D_Pin | G_Pin,
			GPIO_PIN_SET);
}

void ssDisplayFour() {
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin | C_Pin | G_Pin | F_Pin, GPIO_PIN_SET);
}

void ssDisplayFive() {
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin | C_Pin | D_Pin | G_Pin | F_Pin,
			GPIO_PIN_SET);
}

void ssDisplaySix() {
	HAL_GPIO_WritePin(A_GPIO_Port,
	A_Pin | G_Pin | C_Pin | D_Pin | E_Pin | F_Pin, GPIO_PIN_SET);
}

void ssDisplaySeven() {
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin | B_Pin | C_Pin, GPIO_PIN_SET);
}

void ssDisplayEight() {
	HAL_GPIO_WritePin(A_GPIO_Port,
	A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
}

void ssDisplayNine() {
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin | B_Pin | C_Pin | D_Pin | G_Pin | F_Pin,
			GPIO_PIN_SET);
}

void ssDisplay(int num) {

	for (int i = 1; i <= 4; i++) {
		int digit = num % 10;
		num /= 10;

		ssSetDigit(i);

		switch (digit) {
		case 0:
			ssDisplayZero();
			break;
		case 1:
			ssDisplayOne();
			break;
		case 2:
			ssDisplayTwo();
			break;
		case 3:
			ssDisplayThree();
			break;
		case 4:
			ssDisplayFour();
			break;
		case 5:
			ssDisplayFive();
			break;
		case 6:
			ssDisplaySix();
			break;
		case 7:
			ssDisplaySeven();
			break;
		case 8:
			ssDisplayEight();
			break;
		case 9:
			ssDisplayNine();
			break;
		}

		HAL_GPIO_WritePin(A_GPIO_Port,
		A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin, GPIO_PIN_RESET);
	}
}
