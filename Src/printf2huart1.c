#include <errno.h>
#include <sys/unistd.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

// These functions are implemented in the GCC C library as
// stub routines with "weak" linkage so just re-define it
// to write to UART1
int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
      errno = EBADF;
      return -1;
   }

   HAL_StatusTypeDef res;
   res = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

   return (res == HAL_OK ? len : 0);
}
