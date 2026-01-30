/**
 * ******************************************************************************
 * @file    : uart.h
 * @brief   : UART module header file
 * @details : UART initialization and interaction
 * 
 * @author Garrett Geyer 
 * @date 1/23/26
 * ******************************************************************************
*/

#ifndef _UART_H_
#define _UART_H_

/**
 * @brief Initialize UART0
*/
void UART0_init(void);


/**
 * @brief Put a character over UART0
 * @param[in] ch - Character to print
*/
void UART0_putchar(char ch);


/**
 * @brief Retrieve a single character from UART0
*/
char UART0_getchar(void);


/**
 * @brief Send a full character string over UART0
 * @param[in] ptr_str - Pointer to the string to print
*/
void UART0_put(char *ptr_str);

/**
* @brief checks if data is available
*/
int UART0_dataAvailable(void);

/**
 * @brief Initialize UART1
*/
void UART1_init(void);


/**
 * @brief Put a character over UART1
 * @param[in] ch - Character to print
*/
void UART1_putchar(char ch);


/**
 * @brief Retrieve a single character from UART1
*/
char UART1_getchar(void);


/**
 * @brief Send a full character string over UART1
 * @param[in] ptr_str - Pointer to the string to print
*/
void UART1_put(char *ptr_str);

/**
* @brief checks if data is available
*/
int UART1_dataAvailable(void);


#endif // _UART_H_
