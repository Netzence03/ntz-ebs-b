#include "utils.h"

void reverseBuffer(uint16_t arr[], int size)
{
    for (int i = 0; i < size / 2; ++i)
    {
        // Swap elements from the beginning and end of the array
        int temp = arr[i];
        arr[i] = arr[size - 1 - i];
        arr[size - 1 - i] = temp;
    }
}